#include "ParallelLNSScheduler.h"
#include <thread>
#include <algorithm>

namespace ParallelLNSScheduler
{
    unsigned int num_threads;
    std::vector<ThreadData> thread_data;
    int max_failed_attempts = 100;
    double initial_destroy_ratio = 0.2;
    const double final_destroy_ratio = 0.1;
    OptimizationType optimization_type = OptimizationType::SumOfCosts;
    const bool USE_EARLY_RETURN = false;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // スレッド数を取得（ハードウェアの同時実行可能なスレッド数）
        num_threads = std::thread::hardware_concurrency();
        // num_threads = 1;
        // マップに基づいて最適化タイプを設定
        if (SchedulerUtils::map_name == "random-32-32-20.map")
        {
            max_failed_attempts = 2000;
            optimization_type = OptimizationType::SumOfCosts;
            initial_destroy_ratio = 0.3;
        }
        else if (env->map_name == "Paris_1_256.map")
        {
            optimization_type = OptimizationType::SumOfCosts;
            max_failed_attempts = 200;
        }
        else if (env->map_name == "warehouse_large.map" || env->map_name == "sortation_large.map")
        {
            optimization_type = OptimizationType::SumOfCosts;
            initial_destroy_ratio = 0.2;
        }
        else
        {
            optimization_type = OptimizationType::Makespan;
        }

        // スレッドごとのデータ構造を初期化
        thread_data.clear();
        thread_data.reserve(num_threads);
    }

    void repair(std::unordered_map<int, int> &schedule,
                const std::unordered_map<int, int> &task_agent_map,
                std::vector<int> &destroyed_agents,
                SharedEnvironment *env,
                std::chrono::steady_clock::time_point end_time,
                SharedData &shared_data)
    {
        std::unordered_set<int> assigned_tasks;
        int count = 0;
        // 打ち切りのカウント
        // int max_count = SchedulerUtils::map_name == "brc202d.map" || SchedulerUtils::map_name == "random-32-32-20.map" ? schedule.size() : INT_MAX;
        int max_count = LNSScheduler::disable_agents ? schedule.size() : INT_MAX;
        // std::cout << "max_count: " << max_count << std::endl;
        schedule.clear();
        // destroyed_agentsの重複チェック

        // int remaining_time = LNSScheduler::estimated_sim_time - env->curr_timestep;
        // if (remaining_time <= LNSScheduler::estimated_sim_time / 10)
        // {
        //     std::shuffle(destroyed_agents.begin(), destroyed_agents.end(), LNSScheduler::rng);
        // }
        // std::shuffle(destroyed_agents.begin(), destroyed_agents.end(), LNSScheduler::rng);
        // std::unordered_set<int> must_assign_agents;
        // if (LNSScheduler::disable_agents)
        // {
        //     for (int agent_id : destroyed_agents)
        //     {
        //         if (SchedulerUtils::tabu_locs.find(env->curr_states[agent_id].location) != SchedulerUtils::tabu_locs.end())
        //         {
        //             must_assign_agents.insert(agent_id);
        //         }
        //     }
        // }

        for (int agent_id : destroyed_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            // if (LNSScheduler::disable_agents && must_assign_agents.find(agent_id) != must_assign_agents.end())
            // {
            //     // continue;
            // }
            if (count >= max_count)
            {
                break;
            }
            count++;

            int current_task_id = env->curr_task_schedule[agent_id];

            std::vector<LNSScheduler::TaskCost> task_costs;
            {
                std::shared_lock<std::shared_mutex> lock(shared_data.mutex);
                if (shared_data.agent_task_costs.find(agent_id) != shared_data.agent_task_costs.end())
                {
                    task_costs = shared_data.agent_task_costs[agent_id];
                }
            }

            if (task_costs.empty())
            {
                if (LNSScheduler::INCLUDE_FLOW_COST)
                {
                    // std::unique_lock<std::shared_mutex> lock(shared_data.mutex);
                    // SchedulerUtils::createTrajLNSCopy(agent_id);
                }
                task_costs.reserve(SchedulerUtils::scheduling_target_tasks.size());
                for (int task_id : SchedulerUtils::scheduling_target_tasks)
                {
                    if (LNSScheduler::count_deadends && SchedulerUtils::hasDeadEndInErrand(task_id, env))
                    {
                        // std::cout << "task_id: " << task_id << " countDeadEndsInErrand: " << SchedulerUtils::countDeadEndsInErrand(task_id, env) << std::endl;
                        // for (int i = 0; i < env->task_pool[task_id].locations.size(); i++)
                        // {
                        //     std::cout << env->task_pool[task_id].locations[i] << " ";
                        // }
                        // std::cout << std::endl;
                        continue;
                    }
                    bool is_assigned_agent = env->curr_task_schedule[agent_id] != -1;
                    bool is_assigned_task = env->task_pool[task_id].agent_assigned != -1 && env->task_pool[task_id].agent_assigned != agent_id;
                    if (is_assigned_agent && is_assigned_task)
                    {
                        if (env->curr_timestep > 20)
                        {
                            continue;
                        }
                    }
                    CostType cost = LNSScheduler::calculateTaskCompletionTime(agent_id, task_id, env);
                    if (task_id == current_task_id)
                    {
                        cost += LNSScheduler::same_task_additional_cost;
                    }
                    else if (env->task_pool[task_id].agent_assigned != -1)
                    {
                        cost += LNSScheduler::assinged_task_additional_cost;
                    }
                    if (LNSScheduler::INCLUDE_FLOW_COST)
                    {
                        cost += SchedulerUtils::calculateOppositeFlowCost(agent_id, task_id, env);
                    }
                    // if (LNSScheduler::count_deadends)
                    // {
                    //     cost += SchedulerUtils::countDeadEndsInErrand(task_id, env) * 200;
                    // }
                    task_costs.push_back({task_id, cost});
                }

                std::sort(task_costs.begin(), task_costs.end(),
                          [](const auto &a, const auto &b)
                          { return a.cost < b.cost; });

                {
                    std::unique_lock<std::shared_mutex> lock(shared_data.mutex);
                    shared_data.agent_task_costs[agent_id] = task_costs;
                }
            }

            bool found_task = false;
            for (const auto &task_cost : task_costs)
            {
                if (task_agent_map.find(task_cost.task_id) == task_agent_map.end() &&
                    assigned_tasks.find(task_cost.task_id) == assigned_tasks.end())
                {
                    schedule[agent_id] = task_cost.task_id;
                    assigned_tasks.insert(task_cost.task_id);
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

    // タスクの切り替わり回数を計算する関数の実装
    int calculateTotalSwitches(const std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        int total_switches = 0;
        for (int agent_id : SchedulerUtils::scheduling_target_agents)
        {
            if (env->curr_task_schedule[agent_id] != -1 &&
                proposed_schedule[agent_id] != env->curr_task_schedule[agent_id])
            {
                total_switches++;
            }
        }
        return total_switches;
    }

    OptimizationResult optimizeAssignmentMakespan(std::vector<int> &proposed_schedule,
                                                  OptimizationData &initial_data,
                                                  SharedEnvironment *env,
                                                  std::chrono::steady_clock::time_point end_time,
                                                  SharedData &shared_data)
    {
        std::vector<int> &best_schedule = proposed_schedule;
        auto [best_makespan, critical_agents] = initial_data;

        // std::cout << "initial_makespan: " << best_makespan << std::endl;
        if (std::chrono::steady_clock::now() >= end_time)
        {
            return {best_makespan, 0};
        }

        std::unordered_set<int> nearby_agents;
        for (int agent_id : critical_agents)
        {
            std::vector<int> tmp_nearby_agents = LNSScheduler::getNearbyAgents(agent_id, LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD, critical_agents, env);
            nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
        }
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::scheduling_target_agents)
        {
            if (best_schedule[agent_id] != -1)
            {
                task_agent_map[best_schedule[agent_id]] = agent_id;
            }
        }

        // LNSのパラメータ

        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        std::unordered_map<int, int> tmp_schedule;
        std::unordered_map<int, int> tmp_schedule_completion_times;
        // bestが更新されたフラグ
        bool best_updated = false;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        std::vector<std::pair<int, int>> destroyed_schedule;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < max_failed_attempts)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = LNSScheduler::destroy(current_destroy_ratio, critical_agents, nearby_agents, best_schedule);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                if (best_schedule[agent_id] != -1)
                {
                    destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                    task_agent_map.erase(best_schedule[agent_id]);
                    tmp_schedule[agent_id] = best_schedule[agent_id];
                    // tmp_schedule_completion_times[agent_id] = agent_completion_times[agent_id];
                }
            }

            // 修復
            repair(tmp_schedule, task_agent_map, destroyed_agents, env, end_time, shared_data);
            if (std::chrono::steady_clock::now() >= end_time)
            {
                break;
            }
            // 評価
            auto [new_makespan, new_critical_agents] = LNSScheduler::evaluateSchedule(tmp_schedule, env, best_makespan);

            // より良い解が見つかった場合は更新
            if (new_makespan < best_makespan)
            {
                for (const auto &[agent_id, task_id] : destroyed_schedule)
                {
                    best_schedule[agent_id] = -1;
                }
                for (const auto &[agent_id, task_id] : tmp_schedule)
                {
                    best_schedule[agent_id] = task_id;
                }
                for (int agent_id : destroyed_agents)
                {
                    if (best_schedule[agent_id] != -1)
                    {
                        task_agent_map[best_schedule[agent_id]] = agent_id;
                    }
                    // agent_completion_times[agent_id] = tmp_schedule_completion_times[agent_id];
                }
                std::tie(best_makespan, critical_agents) = LNSScheduler::evaluateSchedule(best_schedule, env);
                nearby_agents.clear();
                for (int agent_id : critical_agents)
                {
                    std::vector<int> tmp_nearby_agents = LNSScheduler::getNearbyAgents(agent_id, LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD, critical_agents, env);
                    nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
                }
                // std::cout << "Iteration " << iteration << ": New best makespan = " << best_makespan << std::endl;
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

        // タスクの切り替わり回数を更新
        int total_switches = calculateTotalSwitches(proposed_schedule, env);

        // std::cout << "best_makespan: " << best_makespan << std::endl;
        // std::cout << "total_switches: " << total_switches << std::endl;
        // std::cout << "iteration: " << iteration << std::endl;

        return {best_makespan, total_switches, iteration};
    }

    OptimizationResult optimizeAssignmentSoC(std::vector<int> &proposed_schedule,
                                             OptimizationData &initial_data,
                                             SharedEnvironment *env,
                                             std::chrono::steady_clock::time_point end_time,
                                             SharedData &shared_data)
    {
        std::vector<int> &best_schedule = proposed_schedule;
        auto [best_soc, critical_agents] = initial_data;

        // std::cout << "initial_makespan: " << best_makespan << std::endl;
        if (std::chrono::steady_clock::now() >= end_time)
        {
            return {best_soc, 0};
        }

        std::unordered_set<int> nearby_agents;
        for (int agent_id : critical_agents)
        {
            std::vector<int> tmp_nearby_agents = LNSScheduler::getNearbyAgents(agent_id, LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD, critical_agents, env);
            nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
        }
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::scheduling_target_agents)
        {
            if (best_schedule[agent_id] != -1)
            {
                task_agent_map[best_schedule[agent_id]] = agent_id;
            }
        }

        // LNSのパラメータ
        // const double initial_destroy_ratio = 0.25;
        // const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        std::unordered_map<int, int> tmp_schedule;
        std::unordered_map<int, int> tmp_schedule_completion_times;
        // bestが更新されたフラグ
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        std::vector<std::pair<int, int>> destroyed_schedule;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < max_failed_attempts)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = LNSScheduler::destroy(current_destroy_ratio, critical_agents, nearby_agents, best_schedule);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                if (best_schedule[agent_id] != -1)
                {
                    destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                    task_agent_map.erase(best_schedule[agent_id]);
                    tmp_schedule[agent_id] = best_schedule[agent_id];
                    // tmp_schedule_completion_times[agent_id] = agent_completion_times[agent_id];
                }
            }
            auto [current_soc, _] = LNSScheduler::evaluateScheduleSoC(tmp_schedule, env);
            int before_size = tmp_schedule.size();
            // 修復
            repair(tmp_schedule, task_agent_map, destroyed_agents, env, end_time, shared_data);
            if (std::chrono::steady_clock::now() >= end_time)
            {
                break;
            }

            // 評価
            auto [new_soc, __] = LNSScheduler::evaluateScheduleSoC(tmp_schedule, env, current_soc);

            // より良い解が見つかった場合は更新
            if (new_soc < current_soc)
            {
                for (const auto &[agent_id, task_id] : destroyed_schedule)
                {
                    best_schedule[agent_id] = -1;
                }
                for (const auto &[agent_id, task_id] : tmp_schedule)
                {
                    best_schedule[agent_id] = task_id;
                }
                for (int agent_id : destroyed_agents)
                {
                    if (best_schedule[agent_id] != -1)
                    {
                        task_agent_map[best_schedule[agent_id]] = agent_id;
                    }
                    // agent_completion_times[agent_id] = tmp_schedule_completion_times[agent_id];
                }
                std::tie(best_soc, critical_agents) = LNSScheduler::evaluateScheduleSoC(best_schedule, env);
                nearby_agents.clear();
                for (int agent_id : critical_agents)
                {
                    std::vector<int> tmp_nearby_agents = LNSScheduler::getNearbyAgents(agent_id, LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD, critical_agents, env);
                    nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
                }
                // std::cout << "Iteration " << iteration << ": New best makespan = " << best_makespan << std::endl;
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

        // タスクの切り替わり回数を更新
        int total_switches = calculateTotalSwitches(proposed_schedule, env);

        // std::cout << "best_makespan: " << best_makespan << std::endl;
        // std::cout << "total_switches: " << total_switches << std::endl;
        // std::cout << "iteration: " << iteration << std::endl;

        return {best_soc, total_switches, iteration};
    }

    void schedule_plan(int time_limit,
                       std::vector<int> &proposed_schedule,
                       SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        LNSScheduler::resetTaskSwitches(env);
        // 初期スケジュールを生成
        // auto initial_result = LNSScheduler::generateInitialSchedule(proposed_schedule, env, end_time);
        LNSScheduler::generateInitialScheduleQuick(proposed_schedule, env, end_time);

        std::cout << "new_free_agents size: " << env->new_freeagents.size() << std::endl;
        std::cout << "new_tasks size: " << env->new_tasks.size() << std::endl;

        if (USE_EARLY_RETURN && env->new_tasks.size() == 0 && env->curr_timestep >= 15)
        {
            std::cout << "new_tasks size is 0 " << std::endl;
            return;
        }

        if (std::chrono::steady_clock::now() >= end_time)
            return;

        std::unordered_map<unsigned short, std::vector<LNSScheduler::TaskCost>> agent_task_costs;
        // 共有データの準備
        // SharedData shared_data(initial_result.agent_task_costs);
        SharedData shared_data(agent_task_costs);
        int initial_objective;
        std::unordered_set<int> initial_critical_agents;
        if (optimization_type == OptimizationType::SumOfCosts)
        {
            std::tie(initial_objective, initial_critical_agents) = LNSScheduler::evaluateScheduleSoC(proposed_schedule, env);
        }
        else
        {
            std::tie(initial_objective, initial_critical_agents) = LNSScheduler::evaluateSchedule(proposed_schedule, env);
        }

        OptimizationData initial_data(initial_objective, initial_critical_agents);

        // 各スレッドのデータを初期化
        thread_data.clear();
        for (int i = 0; i < num_threads; ++i)
        {
            thread_data.emplace_back(proposed_schedule);
        }

        // 各スレッドで最適化を実行
        std::vector<std::thread> threads;
        for (int i = 0; i < num_threads; ++i)
        {
            threads.emplace_back([&, i]()
                                 {
                if (optimization_type == OptimizationType::SumOfCosts) {
                    thread_data[i].result = optimizeAssignmentSoC(
                        thread_data[i].schedule, initial_data, env, end_time, shared_data);
                } else {
                    thread_data[i].result = optimizeAssignmentMakespan(
                        thread_data[i].schedule, initial_data, env, end_time, shared_data);
                } });
        }

        // スレッドの終了を待機
        for (auto &thread : threads)
        {
            thread.join();
        }

        // 最良の結果を選択
        int best_thread = 0;
        int best_objective = std::numeric_limits<int>::max();
        int best_switches = std::numeric_limits<int>::max();
        int best_iteration = 0;

        for (int i = 0; i < num_threads; ++i)
        {
            std::cout << i << " objective: " << thread_data[i].result.objective << " switches: " << thread_data[i].result.task_switches << " iteration: " << thread_data[i].result.iteration << std::endl;
            if (thread_data[i].result.objective < best_objective ||
                (thread_data[i].result.objective == best_objective &&
                 thread_data[i].result.task_switches < best_switches))
            {
                best_objective = thread_data[i].result.objective;
                best_switches = thread_data[i].result.task_switches;
                best_iteration = thread_data[i].result.iteration;
                best_thread = i;
            }
        }
        std::cout << "initial_objective: " << initial_objective << std::endl;
        std::cout << "best_thread: " << best_thread << " objective: " << best_objective << " switches: " << best_switches << " iteration: " << best_iteration << std::endl;

        // 最良の結果を反映
        proposed_schedule = thread_data[best_thread].schedule;
        LNSScheduler::updateTaskSwitches(proposed_schedule, env);
    }
}