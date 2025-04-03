#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_set>
#include <signal.h>
#include <future>
#include <thread>
#include "LNSSchedulerRND.h"
#include "flow.h"

// using namespace SchedulerUtils;
namespace LNSSchedulerRND
{
    std::mt19937 rng(0); // 乱数生成器
    // エージェントごとのタスク完了時間を追跡
    std::unordered_map<int, int> agent_completion_times;
    int MANHATTAN_DISTANCE_THRESHOLD = 0;
    std::vector<std::vector<unsigned short>> manhattan_distance_table;
    std::vector<int> agent_task_switches;
    bool include_waypoints = true;

    // スレッド数を制御するための定数を追加
    const int NUM_THREADS = std::thread::hardware_concurrency(); // 利用可能なCPUコア数
    const int TASKS_PER_THREAD = 150;                            // 1スレッドあたりの処理タスク数
    int assinged_task_additional_cost = 5;
    // 　今のタスクと同一の場合差し引く値
    int same_task_additional_cost = -5;
    // 同時にタスクを割り当て可能なagent数
    int MAX_ASSIGNED_TASKS = INT_MAX;
    std::vector<int> location_counts;
    const int MAX_FAILED_ATTEMPTS = 500;
    std::unordered_set<unsigned short> deadends;
    const int TASK_SWITCH_THRESHOLD = 10;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // manhattan_distance_table.resize(SchedulerUtils::passableCells.size(), std::vector<unsigned short>(SchedulerUtils::passableCells.size(), 0));
        // for (int i : SchedulerUtils::passableCells)
        // {
        //     for (int j : SchedulerUtils::passableCells)
        //     {
        //         manhattan_distance_table[cellToIndex[i]][cellToIndex[j]] = static_cast<unsigned short>(calculateManhattanDistance(i, j, env));
        //     }
        // }
        agent_task_switches.resize(env->num_of_agents, 0);
        MANHATTAN_DISTANCE_THRESHOLD = std::max(env->rows, env->cols) / 3;
        if (env->map_name == "sortation_large.map" || env->map_name == "warehouse_large.map")
        {
            include_waypoints = false;
        }
        if (env->map_name == "random-32-32-20.map")
        {
            assinged_task_additional_cost = 5;
            same_task_additional_cost = -5;
            // MAX_ASSIGNED_TASKS = 50;
        }
        else if (env->map_name == "brc202d.map")
        {
            assinged_task_additional_cost = 15;
            same_task_additional_cost = -15;
            MAX_ASSIGNED_TASKS = 2500;
        }
        location_counts.resize(env->rows * env->cols, 0);
        deadends = SchedulerUtils::findAllDeadEnds(env);
        std::cout << "deadends.size(): " << deadends.size() << std::endl;
        for (int loc : deadends)
        {
            std::cout << loc << " ";
        }
        std::cout << std::endl;
    }
    // タスクの完了時間を計算
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
        {
            // std::cerr << "task_id is -1" << std::endl;
            return 0;
        }

        const Task &task = env->task_pool[task_id];
        int total_cost = 0;

        // エージェントの現在位置と向きから最初の位置までのコスト
        int current_loc = env->curr_states[agent_id].location;
        int current_dir = env->curr_states[agent_id].orientation;

        // 最初の位置までのコスト
        total_cost = SchedulerUtils::getCostToLocation(current_loc, current_dir, task.locations[0]);
        // auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
        //     current_loc, current_dir, task.locations[0]);
        // 各経由地点間のコストを含める場合のみ計算
        if (include_waypoints)
        {
            total_cost += SchedulerUtils::task_total_cost_table[task_id];
        }
        return total_cost;
    }

    int calculateOppositeFlowCost(int agent_id, int task_id, SharedEnvironment *env)
    {
        // パスを構築
        std::vector<SchedulerUtils::State> path = SchedulerUtils::constructPath(
            env->curr_states[agent_id].location,
            env->curr_states[agent_id].orientation,
            env->task_pool[task_id].locations[0]);

        // パスに沿って対向流コストを計算
        int total_cost = 0;
        for (size_t i = 0; i < path.size(); i++)
        {
            const SchedulerUtils::State &state = path[i];
            // 対向流コスト
            int opposite_dir = (state.direction + 2) % 4;
            total_cost += DefaultPlanner::trajLNS.flow[state.location].d[opposite_dir];
        }

        return total_cost;
    }
    // スケジュールの一部を破壊する関数
    std::vector<int> destroy(double destroy_ratio,
                             const std::unordered_set<int> &critical_agents,
                             const std::unordered_set<int> &nearby_agents)
    {
        std::vector<int> free_agents(SchedulerUtils::global_available_agents.begin(), SchedulerUtils::global_available_agents.end());
        int num_destroy = static_cast<int>(free_agents.size() * destroy_ratio);
        int additional_needed = num_destroy - static_cast<int>(critical_agents.size());
        // std::cout << "critical_agents.size(): " << critical_agents.size() << std::endl;
        // std::cout << "additional_needed: " << additional_needed << std::endl;

        // critical_agentsは必ず含める
        std::vector<int> result(critical_agents.begin(), critical_agents.end());
        std::shuffle(result.begin(), result.end(), rng);

        if (additional_needed <= 0)
        {
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "additional_needed <= 0" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            return result;
        }

        // nearby_agentsがnum_destroyを超える場合は、nearby_agentsをシャッフルして追加
        // isOverTaskSwitchesを考慮
        std::vector<int> remaining_agents;
        std::vector<int> nearby_agents_vector(nearby_agents.begin(), nearby_agents.end());
        std::shuffle(nearby_agents_vector.begin(), nearby_agents_vector.end(), rng);
        for (int agent : nearby_agents_vector)
        {
            if (remaining_agents.size() >= additional_needed / 2)
            {
                break;
            }
            if (!isOverTaskSwitches(agent))
            {
                remaining_agents.push_back(agent);
            }
        }
        if (remaining_agents.size() >= additional_needed)
        {
            std::shuffle(remaining_agents.begin(), remaining_agents.end(), rng);
            result.insert(result.end(), remaining_agents.begin(), remaining_agents.begin() + additional_needed);
        }
        else
        {
            additional_needed -= remaining_agents.size();

            // critical_agents以外のエージェントから重複なくadditional_needed分を選ぶ
            std::vector<int> non_critical_agents;
            std::shuffle(free_agents.begin(), free_agents.end(), rng);
            while (non_critical_agents.size() < additional_needed && free_agents.size() > 0)
            {
                int agent = free_agents.back();
                free_agents.pop_back();
                if (critical_agents.find(agent) == critical_agents.end() &&
                    nearby_agents.find(agent) == nearby_agents.end() &&
                    !isOverTaskSwitches(agent))
                {
                    non_critical_agents.push_back(agent);
                }
            }
            // result.insert(result.end(),
            //               non_critical_agents.begin(),
            //               non_critical_agents.begin() + std::min(additional_needed,
            //                                                      static_cast<int>(non_critical_agents.size())));

            remaining_agents.insert(remaining_agents.end(),
                                    non_critical_agents.begin(),
                                    non_critical_agents.begin() + std::min(additional_needed,
                                                                           static_cast<int>(non_critical_agents.size())));
            // std::cout << "remaining_agents.size(): " << remaining_agents.size() << std::endl;

            std::shuffle(remaining_agents.begin(), remaining_agents.end(), rng);
            result.insert(result.end(),
                          remaining_agents.begin(),
                          remaining_agents.begin() + std::min(additional_needed,
                                                              static_cast<int>(remaining_agents.size())));
        }

        return result;
    }

    // 破壊された部分を修復する関数
    void repair(std::unordered_map<int, int> &schedule, const std::unordered_map<int, int> &task_agent_map,
                const std::vector<int> &destroyed_agents, SharedEnvironment *env,
                std::chrono::steady_clock::time_point end_time,
                std::unordered_map<unsigned short, std::vector<TaskCost>> &agent_task_costs)
    {
        // 現在割り当て済みのタスクを記録
        std::unordered_set<int> assigned_tasks;
        // std::unordered_set<int> destroyed_agents_set(destroyed_agents.begin(), destroyed_agents.end());

        // 破壊されたエージェントに対して、最も近い未割り当てタスクを割り当てる
        for (int agent_id : destroyed_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
            {
                break;
            }
            int current_task_id = env->curr_task_schedule[agent_id];

            // 既存のタスクコストがある場合はそれを使用、ない場合は新規計算
            std::vector<TaskCost> task_costs;
            if (agent_task_costs.find(agent_id) != agent_task_costs.end())
            {
                task_costs = std::ref(agent_task_costs[agent_id]);
            }
            else
            {
                task_costs.reserve(SchedulerUtils::global_available_tasks.size());
                remove_traj(DefaultPlanner::trajLNS, agent_id);
                for (int task_id : SchedulerUtils::global_available_tasks)
                {
                    // int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    int cost;
                    if (env->curr_timestep != 0)
                    {
                        cost = SchedulerUtils::calculateOppositeFlowCost(agent_id, task_id, env);
                    }
                    else
                    {
                        cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    }
                    if (task_id == current_task_id)
                    {
                        cost += same_task_additional_cost;
                    }
                    else if (env->task_pool[task_id].agent_assigned != -1)
                    {
                        cost += assinged_task_additional_cost;
                    }
                    cost += countDeadEndsInErrand(task_id, env) * 200;
                    task_costs.push_back({task_id, cost});
                }

                std::sort(task_costs.begin(), task_costs.end(),
                          [](const TaskCost &a, const TaskCost &b)
                          {
                              return a.cost < b.cost;
                          });

                agent_task_costs[agent_id] = task_costs;
                add_traj(DefaultPlanner::trajLNS, agent_id);
            }

            bool found_task = false;
            for (const TaskCost &task_cost : task_costs)
            {
                if (task_agent_map.find(task_cost.task_id) == task_agent_map.end() &&
                    assigned_tasks.find(task_cost.task_id) == assigned_tasks.end())
                {
                    schedule[agent_id] = task_cost.task_id;
                    assigned_tasks.insert(task_cost.task_id);
                    // agent_completion_times_local[agent_id] = task_cost.cost;
                    found_task = true;
                    break;
                }
            }

            if (!found_task)
            {
                std::cout << "best_task is -1 at repair" << std::endl;
            }
        }
    }

    void optimizeAssignment(std::vector<int> &proposed_schedule, InitialScheduleResult &initial_result, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        std::vector<int> &best_schedule = proposed_schedule;
        // auto [best_makespan, critical_agents] = evaluateSchedule(best_schedule, env, agent_completion_times);
        // auto &[best_makespan, critical_agents, agent_task_costs] = initial_result;
        std::unordered_map<unsigned short, std::vector<TaskCost>> agent_task_costs = initial_result.agent_task_costs;

        // まず、現在の全エージェントの経路をカウント
        std::fill(location_counts.begin(), location_counts.end(), 0);
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            int task_id = env->curr_task_schedule[agent_id];
            if (env->task_pool[task_id].idx_next_loc > 0)
            {
                for (int loc : DefaultPlanner::trajLNS.trajs[agent_id])
                {
                    location_counts[loc]++;
                }
            }
        }

        auto [best_makespan, critical_agents] = evaluateSchedule(best_schedule, env);
        // return;
        std::cout << "initial_makespan: " << best_makespan << std::endl;
        if (std::chrono::steady_clock::now() >= end_time)
        {
            return;
        }

        // int initial_soc = 0;
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     initial_soc += agent_completion_times[agent_id];
        // }
        // std::cout << "initial_soc: " << initial_soc << std::endl;

        std::unordered_set<int> nearby_agents;
        for (int agent_id : critical_agents)
        {
            std::vector<int> tmp_nearby_agents = getNearbyAgents(agent_id, MANHATTAN_DISTANCE_THRESHOLD, env);
            nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
        }
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            task_agent_map[best_schedule[agent_id]] = agent_id;
        }

        // LNSのパラメータ
        const double initial_destroy_ratio = 0.35;
        const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        // const int MAX_FAILED_ATTEMPTS = 500;
        std::unordered_map<int, int> tmp_schedule;
        std::unordered_map<int, int> tmp_schedule_completion_times;
        // bestが更新されたフラグ
        bool best_updated = false;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        // std::vector<std::pair<int, int>> destroyed_schedule(SchedulerUtils::global_available_agents.size());
        std::vector<std::pair<int, int>> destroyed_schedule;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = destroy(current_destroy_ratio, critical_agents, nearby_agents);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                task_agent_map.erase(best_schedule[agent_id]);
                tmp_schedule[agent_id] = best_schedule[agent_id];
                // tmp_schedule_completion_times[agent_id] = agent_completion_times[agent_id];
            }

            // 修復
            repair(tmp_schedule, task_agent_map, destroyed_agents, env, end_time, agent_task_costs);
            if (std::chrono::steady_clock::now() >= end_time)
            {
                break;
            }
            // 評価
            auto [new_makespan, new_critical_agents] = evaluateSchedule(tmp_schedule, env, best_makespan);

            // より良い解が見つかった場合は更新
            if (new_makespan < best_makespan)
            {
                for (const auto &[agent_id, task_id] : tmp_schedule)
                {
                    best_schedule[agent_id] = task_id;
                }
                for (int agent_id : destroyed_agents)
                {
                    task_agent_map[best_schedule[agent_id]] = agent_id;
                    // agent_completion_times[agent_id] = tmp_schedule_completion_times[agent_id];
                }
                std::tie(best_makespan, critical_agents) = evaluateSchedule(best_schedule, env);
                nearby_agents.clear();
                for (int agent_id : critical_agents)
                {
                    std::vector<int> tmp_nearby_agents = getNearbyAgents(agent_id, MANHATTAN_DISTANCE_THRESHOLD, env);
                    nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
                }
                std::cout << "Iteration " << iteration << ": New best makespan = " << best_makespan << std::endl;
                failed_attempts = 0;
            }
            else
            {
                // 破壊したスケジュールを元に戻す
                for (const auto &pair : destroyed_schedule)
                {
                    task_agent_map[pair.second] = pair.first;
                }
                failed_attempts++;
            }
            destroyed_schedule.clear();
            tmp_schedule.clear();
            tmp_schedule_completion_times.clear();
            iteration++;
        }
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     std::cout << best_schedule[agent_id] << " ";
        // }
        // std::cout << std::endl;
        std::cout << "best_makespan: " << best_makespan << std::endl;
        std::cout << "iteration: " << iteration << std::endl;
        // int final_soc = 0;
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     final_soc += agent_completion_times[agent_id];
        // }
        // std::cout << "final_soc: " << final_soc << std::endl;
    }

    void optimizeAssignmentSoC(std::vector<int> &proposed_schedule, InitialScheduleResult &initial_result, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        std::vector<int> &best_schedule = proposed_schedule;
        // auto [best_makespan, critical_agents] = evaluateSchedule(best_schedule, env, agent_completion_times);
        // auto &[best_makespan, critical_agents, agent_task_costs] = initial_result;
        std::unordered_map<unsigned short, std::vector<TaskCost>> agent_task_costs = initial_result.agent_task_costs;

        // まず、現在の全エージェントの経路をカウント
        std::fill(location_counts.begin(), location_counts.end(), 0);
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            int task_id = env->curr_task_schedule[agent_id];
            if (env->task_pool[task_id].idx_next_loc > 0)
            {
                for (int loc : DefaultPlanner::trajLNS.trajs[agent_id])
                {
                    location_counts[loc]++;
                }
            }
        }

        auto [best_soc, critical_agents] = evaluateScheduleSoC(best_schedule, env);
        // return;
        std::cout << "initial_soc: " << best_soc << std::endl;
        if (std::chrono::steady_clock::now() >= end_time)
        {
            return;
        }

        // int initial_soc = 0;
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     initial_soc += agent_completion_times[agent_id];
        // }
        // std::cout << "initial_soc: " << initial_soc << std::endl;

        std::unordered_set<int> nearby_agents;
        for (int agent_id : critical_agents)
        {
            std::vector<int> tmp_nearby_agents = getNearbyAgents(agent_id, MANHATTAN_DISTANCE_THRESHOLD, env);
            nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
        }
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            task_agent_map[best_schedule[agent_id]] = agent_id;
        }

        // LNSのパラメータ
        const double initial_destroy_ratio = 0.25;
        const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        // const int MAX_FAILED_ATTEMPTS = 200;
        std::unordered_map<int, int> tmp_schedule;
        std::unordered_map<int, int> current_schedule;
        std::unordered_map<int, int> tmp_schedule_completion_times;
        // bestが更新されたフラグ
        bool best_updated = false;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        // std::vector<std::pair<int, int>> destroyed_schedule(SchedulerUtils::global_available_agents.size());
        std::vector<std::pair<int, int>> destroyed_schedule;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = destroy(current_destroy_ratio, critical_agents, nearby_agents);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                task_agent_map.erase(best_schedule[agent_id]);
                tmp_schedule[agent_id] = best_schedule[agent_id];
                current_schedule[agent_id] = best_schedule[agent_id];
                // tmp_schedule_completion_times[agent_id] = agent_completion_times[agent_id];
            }

            // 修復
            repair(tmp_schedule, task_agent_map, destroyed_agents, env, end_time, agent_task_costs);
            if (std::chrono::steady_clock::now() >= end_time)
            {
                break;
            }
            // 評価
            auto [new_soc, new_critical_agents] = evaluateScheduleSoC(tmp_schedule, env);
            auto [current_soc, current_critical_agents] = evaluateScheduleSoC(current_schedule, env);

            // より良い解が見つかった場合は更新
            if (new_soc < current_soc)
            {
                for (const auto &[agent_id, task_id] : tmp_schedule)
                {
                    best_schedule[agent_id] = task_id;
                }
                for (int agent_id : destroyed_agents)
                {
                    task_agent_map[best_schedule[agent_id]] = agent_id;
                    // agent_completion_times[agent_id] = tmp_schedule_completion_times[agent_id];
                }
                std::tie(best_soc, critical_agents) = evaluateScheduleSoC(best_schedule, env);
                nearby_agents.clear();
                for (int agent_id : critical_agents)
                {
                    std::vector<int> tmp_nearby_agents = getNearbyAgents(agent_id, MANHATTAN_DISTANCE_THRESHOLD, env);
                    nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
                }
                std::cout << "Iteration " << iteration << ": New best soc = " << best_soc << std::endl;
                failed_attempts = 0;
            }
            else
            {
                // 破壊したスケジュールを元に戻す
                for (const auto &pair : destroyed_schedule)
                {
                    task_agent_map[pair.second] = pair.first;
                }
                failed_attempts++;
            }
            destroyed_schedule.clear();
            tmp_schedule.clear();
            tmp_schedule_completion_times.clear();
            current_schedule.clear();
            iteration++;
        }
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     std::cout << best_schedule[agent_id] << " ";
        // }
        // std::cout << std::endl;
        std::cout << "best_soc: " << best_soc << std::endl;
        std::cout << "iteration: " << iteration << std::endl;
        // int final_soc = 0;
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     final_soc += agent_completion_times[agent_id];
        // }
        // std::cout << "final_soc: " << final_soc << std::endl;
    }

    // タスクの次のerrandが最後のindexとなっているタスクを持つエージェント数を返す関数
    int countAgentsWithLastErrand(SharedEnvironment *env)
    {
        int count = 0;
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            int task_id = env->curr_task_schedule[agent_id];
            if (task_id != -1)
            {
                const Task &task = env->task_pool[task_id];
                if (task.idx_next_loc == task.locations.size() - 1)
                {
                    count++;
                }
            }
        }
        return count;
    }

    InitialScheduleResult generateInitialSchedule(std::vector<int> &proposed_schedule,
                                                  SharedEnvironment *env,
                                                  std::chrono::steady_clock::time_point end_time)
    {
        const std::unordered_set<unsigned short> &global_free_agents = SchedulerUtils::global_available_agents;
        const std::unordered_set<int> &global_available_tasks = SchedulerUtils::global_available_tasks;
        std::unordered_set<int> assigned_tasks;
        int makespan = 0;
        std::unordered_set<int> critical_agents;
        std::unordered_map<unsigned short, std::vector<TaskCost>> agent_task_costs;
        std::unordered_map<int, int> agent_completion_times_local;
        int count = SchedulerUtils::current_task_assignments.size();

        for (int agent_id : global_free_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;

            if (proposed_schedule[agent_id] != -1)
            {
                // agent_completion_times_local[agent_id] = calculateTaskCompletionTime(agent_id, proposed_schedule[agent_id], env);
                agent_completion_times_local[agent_id] = SchedulerUtils::calculateOppositeFlowCost(agent_id, proposed_schedule[agent_id], env);
            }
            else if (count >= MAX_ASSIGNED_TASKS)
            {
                continue;
            }
            else
            {

                std::vector<TaskCost> task_costs;
                task_costs.reserve(global_available_tasks.size());

                std::vector<std::future<std::vector<TaskCost>>> futures;

                // タスクをバッチに分割
                std::vector<std::vector<int>> task_batches;
                std::vector<int> current_batch;
                for (int task_id : global_available_tasks)
                {
                    current_batch.push_back(task_id);
                    if (current_batch.size() >= TASKS_PER_THREAD)
                    {
                        task_batches.push_back(std::move(current_batch));
                        current_batch.clear();
                    }
                }
                if (!current_batch.empty())
                {
                    task_batches.push_back(std::move(current_batch));
                }
                remove_traj(DefaultPlanner::trajLNS, agent_id);

                // バッチごとに並列処理
                for (const auto &batch : task_batches)
                {
                    futures.push_back(std::async(std::launch::async,
                                                 [agent_id, batch, env]() -> std::vector<TaskCost>
                                                 {
                                                     std::vector<TaskCost> batch_costs;
                                                     batch_costs.reserve(batch.size());
                                                     for (int task_id : batch)
                                                     {
                                                         // int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                                                         int cost;
                                                         if (env->curr_timestep != 0)
                                                         {
                                                             cost = SchedulerUtils::calculateOppositeFlowCost(agent_id, task_id, env);
                                                         }
                                                         else
                                                         {
                                                             cost = calculateTaskCompletionTime(agent_id, task_id, env);
                                                         }
                                                         if (env->task_pool[task_id].agent_assigned != -1)
                                                         {
                                                             cost += assinged_task_additional_cost;
                                                         }
                                                         int deadend_count = countDeadEndsInErrand(task_id, env);
                                                         if (deadend_count > 0)
                                                         {
                                                             cost += deadend_count * 200;
                                                         }
                                                         batch_costs.push_back({task_id, cost});
                                                     }
                                                     return batch_costs;
                                                 }));
                }

                // 結果の収集
                for (auto &future : futures)
                {
                    auto batch_results = future.get();
                    task_costs.insert(task_costs.end(),
                                      batch_results.begin(),
                                      batch_results.end());
                }
                add_traj(DefaultPlanner::trajLNS, agent_id);
                std::sort(task_costs.begin(), task_costs.end(),
                          [](const TaskCost &a, const TaskCost &b)
                          {
                              return a.cost < b.cost;
                          });

                // エージェントのタスクコストを保存
                agent_task_costs[agent_id] = std::move(task_costs);

                bool found_task = false;
                for (const TaskCost &task_cost : agent_task_costs[agent_id])
                {
                    if (env->task_pool[task_cost.task_id].agent_assigned == -1 &&
                        assigned_tasks.find(task_cost.task_id) == assigned_tasks.end())
                    {
                        proposed_schedule[static_cast<size_t>(agent_id)] = task_cost.task_id;
                        agent_completion_times_local[agent_id] = task_cost.cost;
                        if (env->task_pool[task_cost.task_id].agent_assigned != -1)
                        {
                            agent_completion_times_local[agent_id] -= assinged_task_additional_cost;
                        }
                        assigned_tasks.insert(task_cost.task_id);
                        found_task = true;
                        count++;
                        break;
                    }
                }

                if (!found_task)
                {
                    std::cerr << "No available tasks for agent " << agent_id << std::endl;
                }
            }

            if (agent_completion_times_local[agent_id] > makespan)
            {
                makespan = agent_completion_times_local[agent_id];
                critical_agents.clear();
                critical_agents.insert(agent_id);
            }
            else if (agent_completion_times_local[agent_id] == makespan)
            {
                critical_agents.insert(agent_id);
            }
        }

        return {makespan, critical_agents, std::move(agent_task_costs)};
    }

    void resetTaskSwitches(SharedEnvironment *env)
    {
        for (int agent_id : env->new_freeagents)
        {
            agent_task_switches[agent_id] = 0;
        }
    }
    bool isOverTaskSwitches(int agent_id)
    {
        return (agent_task_switches[agent_id] + 9) / TASK_SWITCH_THRESHOLD > TASK_SWITCH_THRESHOLD;
    }
    void decreaseTaskSwitches(std::vector<int> &proposed_schedule)
    {
        // global_free_agentsのタスクスイッチカウントのみを1減らす（0未満にはならない）
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            if (proposed_schedule[agent_id] != -1)
                agent_task_switches[agent_id] = std::max(0, agent_task_switches[agent_id] - 2);
        }
    }
    void updateTaskSwitches(std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        // 10ステップごとにタスクスイッチカウントを減らす
        if (env->curr_timestep % 4 == 0)
        {
            decreaseTaskSwitches(proposed_schedule);
        }

        // タスクスイッチの更新
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            if (env->curr_task_schedule[agent_id] != -1 && env->curr_task_schedule[agent_id] != proposed_schedule[agent_id])
            {
                agent_task_switches[agent_id] += 10;
            }
        }
    }
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        resetTaskSwitches(env);
        // std::vector<Int4> flowの内容を表示
        // std::cout << "flow: ";
        // for (const auto &f : DefaultPlanner::trajLNS.flow)
        // {
        //     std::cout << f.d[0] << " " << f.d[1] << " " << f.d[2] << " " << f.d[3] << " ";
        // }
        // std::cout << std::endl;

        InitialScheduleResult initial_result = generateInitialSchedule(proposed_schedule, env, end_time);
        if (std::chrono::steady_clock::now() < end_time)
        {
            // std::pair<int, std::unordered_set<int>> opt_input = {initial_result.makespan, initial_result.critical_agents};
            optimizeAssignment(proposed_schedule, initial_result, env, end_time);
        }
        updateTaskSwitches(proposed_schedule, env);
    }

    // 座標ベースのManhattan距離計算
    int calculateManhattanDistance(int from_x, int from_y, int to_x, int to_y)
    {
        return std::abs(from_x - to_x) + std::abs(from_y - to_y);
    }

    // ロケーションIDベースのManhattan距離計算
    int calculateManhattanDistance(int from_location, int to_location, SharedEnvironment *env)
    {
        int from_x = from_location / env->cols;
        int from_y = from_location % env->cols;
        int to_x = to_location / env->cols;
        int to_y = to_location % env->cols;
        return calculateManhattanDistance(from_x, from_y, to_x, to_y);
    }
    int getManhattanDistance(int from_location, int to_location, SharedEnvironment *env)
    {
        return static_cast<int>(manhattan_distance_table[SchedulerUtils::cellToIndex[from_location]][SchedulerUtils::cellToIndex[to_location]]);
    }

    std::vector<int> getNearbyAgents(int agent_id, int distance_threshold, SharedEnvironment *env)
    {
        std::vector<int> nearby_agents;
        int agent_location = env->curr_states[agent_id].location;
        int agent_dir = env->curr_states[agent_id].orientation;

        // global_free_agentsの中から、指定された距離以内にいるエージェントを探す
        for (int other_agent : SchedulerUtils::global_available_agents)
        {
            if (other_agent == agent_id)
                continue;

            int other_location = env->curr_states[other_agent].location;
            // int distance = calculateManhattanDistance(agent_location, other_location, env);
            int distance = SchedulerUtils::getCostToLocation(agent_location, agent_dir, other_location);

            if (distance <= distance_threshold)
            {
                nearby_agents.push_back(other_agent);
            }
        }

        return nearby_agents;
    }

    // 指定された位置がdeadendかどうかを確認する関数
    bool isDeadEndLocation(unsigned short location)
    {
        return deadends.find(location) != deadends.end();
    }

    // タスクのerrandsにデッドエンドが含まれているかを確認する関数
    bool hasDeadEndInErrand(int task_id, SharedEnvironment *env)
    {
        const Task &task = env->task_pool[task_id];

        // タスクの各位置についてdeadendかどうかをチェック
        for (unsigned short location : task.locations)
        {
            if (isDeadEndLocation(location))
            {
                return true;
            }
        }

        return false;
    }

    // タスクのerrandsに含まれるdeadendの数を返す関数
    int countDeadEndsInErrand(int task_id, SharedEnvironment *env)
    {
        const Task &task = env->task_pool[task_id];
        int deadend_count = 0;

        // タスクの各位置についてdeadendの数をカウント
        for (unsigned short location : task.locations)
        {
            if (isDeadEndLocation(location))
            {
                deadend_count++;
            }
        }

        return deadend_count;
    }

}