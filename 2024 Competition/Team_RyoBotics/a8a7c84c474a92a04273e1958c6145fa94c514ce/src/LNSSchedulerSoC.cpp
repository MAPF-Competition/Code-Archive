#include "LNSSchedulerSoC.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_set>
using namespace SchedulerUtils;
namespace LNSSchedulerSoC
{
    std::mt19937 rng(0); // 乱数生成器

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // 初期化が必要な場合はここで実装
    }
    // タスクの完了時間を計算
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
        {
            std::cerr << "task_id is -1" << std::endl;
            return 0;
        }

        const Task &task = env->task_pool[task_id];
        int total_cost = 0;

        // エージェントの現在位置と向きから最初の位置までのコスト
        int current_loc = env->curr_states[agent_id].location;
        int current_dir = env->curr_states[agent_id].orientation;

        // 最初の位置までのコスト
        auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
            current_loc, current_dir, task.locations[0]);
        total_cost = cost_and_dir.first;
        current_dir = cost_and_dir.second;
        // // 各経由地点間のコスト
        // for (size_t i = 1; i < task.locations.size(); i++)
        // {
        //     auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
        //         task.locations[i - 1], current_dir, task.locations[i]);
        //     total_cost += cost_and_dir.first;
        //     current_dir = cost_and_dir.second;
        // }

        return total_cost;
    }

    // スケジュールの一部を破壊する関数
    std::vector<int> destroy(double destroy_ratio,
                             const std::unordered_set<int> &critical_agents)
    {
        std::unordered_set<unsigned short> &free_agents = SchedulerUtils::global_available_agents;
        int num_destroy = static_cast<int>(free_agents.size() * destroy_ratio);

        // critical_agentsは必ず含める
        std::vector<int> result(critical_agents.begin(), critical_agents.end());

        // critical_agents以外のエージェントをシャッフルして追加
        std::vector<int> remaining_agents;
        for (int agent : free_agents)
        {
            if (critical_agents.find(agent) == critical_agents.end())
            {
                remaining_agents.push_back(agent);
            }
        }

        std::shuffle(remaining_agents.begin(), remaining_agents.end(), rng);

        // 残りの必要な数だけ追加
        int additional_needed = num_destroy - static_cast<int>(critical_agents.size());
        if (additional_needed > 0)
        {
            result.insert(result.end(),
                          remaining_agents.begin(),
                          remaining_agents.begin() + std::min(additional_needed,
                                                              static_cast<int>(remaining_agents.size())));
        }

        return result;
    }

    // 破壊された部分を修復する関数
    void repair(std::unordered_map<int, int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env)
    {
        // 現在割り当て済みのタスクを記録
        std::unordered_set<int> assigned_tasks;
        // std::unordered_set<int> destroyed_agents_set(destroyed_agents.begin(), destroyed_agents.end());

        // 破壊されたエージェントに対して、最も近い未割り当てタスクを割り当てる
        for (int agent_id : destroyed_agents)
        {
            int best_task = -1;
            int min_cost = std::numeric_limits<int>::max();
            for (int task_id : SchedulerUtils::global_available_tasks)
            {
                if (task_agent_map.find(task_id) == task_agent_map.end() && assigned_tasks.find(task_id) == assigned_tasks.end())
                {
                    int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        best_task = task_id;
                    }
                }
            }
            assert(best_task != -1);
            schedule[agent_id] = best_task;
            assigned_tasks.insert(best_task);
        }
    }

    template <typename ScheduleContainer>
    std::pair<int, std::unordered_set<int>> LNSSchedulerSoC::evaluateSchedule(const ScheduleContainer &schedule, SharedEnvironment *env)
    {
        int SoC = 0;
        int makespan = 0;
        std::unordered_set<int> critical_agents;

        // vectorとmapで異なるイテレーション方法に対応
        if constexpr (std::is_same_v<ScheduleContainer, std::vector<int>>)
        {
            for (int agent_id : SchedulerUtils::global_available_agents)
            {
                assert(schedule[agent_id] != -1);
                int completion_time = calculateTaskCompletionTime(agent_id, schedule[agent_id], env);
                SoC += completion_time;
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
            }
        }
        else if constexpr (std::is_same_v<ScheduleContainer, std::unordered_map<int, int>>) // unordered_map用
        {
            for (const auto &[agent_id, task_id] : schedule)
            {
                assert(task_id != -1);
                int completion_time = calculateTaskCompletionTime(agent_id, task_id, env);
                SoC += completion_time;
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
            }
        }
        else
        {
            std::cerr << "evaluateSchedule: invalid schedule type" << std::endl;
        }
        return {SoC, critical_agents};
    }

    void optimizeAssignment(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        std::vector<int> &best_schedule = proposed_schedule;
        auto [best_SoC, critical_agents] = evaluateSchedule(best_schedule, env);
        std::cout << "initial_SoC: " << best_SoC << std::endl;
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            task_agent_map[best_schedule[agent_id]] = agent_id;
        }

        // LNSのパラメータ
        const double initial_destroy_ratio = 0.2;
        const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        const int MAX_FAILED_ATTEMPTS = 1000;
        std::unordered_map<int, int> tmp_schedule;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        std::vector<std::pair<int, int>> destroyed_schedule(SchedulerUtils::global_available_agents.size());
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = destroy(current_destroy_ratio, critical_agents);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                task_agent_map.erase(best_schedule[agent_id]);
                tmp_schedule[agent_id] = best_schedule[agent_id];
            }
            int current_SoC = evaluateSchedule(tmp_schedule, env).first;

            // 修復
            repair(tmp_schedule, task_agent_map, destroyed_agents, env);

            // 評価
            auto [new_SoC, new_critical_agents] = evaluateSchedule(tmp_schedule, env);

            // より良い解が見つかった場合は更新
            if (new_SoC < current_SoC)
            {
                for (const auto &[agent_id, task_id] : tmp_schedule)
                {
                    best_schedule[agent_id] = task_id;
                }
                for (int agent_id : destroyed_agents)
                {
                    task_agent_map[best_schedule[agent_id]] = agent_id;
                }
                std::tie(best_SoC, critical_agents) = evaluateSchedule(best_schedule, env);
                std::cout << "Iteration " << iteration << ": New best SoC = " << best_SoC << std::endl;
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

            iteration++;
        }
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     std::cout << best_schedule[agent_id] << " ";
        // }
        // std::cout << std::endl;
        std::cout << "best_SoC: " << best_SoC << std::endl;
        std::cout << "iteration: " << iteration << std::endl;
    }
    void generateInitialSchedule(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        const std::unordered_set<unsigned short> &global_available_agents = SchedulerUtils::global_available_agents;
        const std::unordered_set<int> &global_available_tasks = SchedulerUtils::global_available_tasks;
        std::unordered_set<int> assigned_tasks;
        for (int agent_id : global_available_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            if (proposed_schedule[agent_id] != -1)
                continue;
            // std::cout << "agent_id: " << agent_id << std::endl;
            int best_task = -1;
            int min_cost = std::numeric_limits<int>::max();
            // global_available_tasksの中で、env->task_poolから未割り当てのタスクを取得
            // 同時に距離を計算
            for (int task_id : global_available_tasks)
            {
                if (env->task_pool[task_id].agent_assigned == -1 && assigned_tasks.find(task_id) == assigned_tasks.end())
                {
                    // int cost = SchedulerUtils::evaluateCost(agent_id, task_id, env);
                    int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        best_task = task_id;
                    }
                }
            }
            // std::cout << "best_task: " << best_task << std::endl;
            // タスクを割り当て
            if (best_task != -1)
            {
                proposed_schedule[static_cast<size_t>(agent_id)] = best_task;
                // global_available_tasks.erase(best_task);
                assigned_tasks.insert(best_task);
                // std::cout << "proposed_schedule[agent_id]: " << proposed_schedule[agent_id] << std::endl;
            }
            else
            {
                std::cerr << "best_task is -1" << std::endl;
            }
        }
        // check if there is -1 in proposed_schedule
        // std::cout << "check if there is -1 in proposed_schedule" << std::endl;
        // for (int i = 0; i < proposed_schedule.size(); i++)
        // {
        //     if (proposed_schedule[i] == -1)
        //     {
        //         std::cout << "agent_id: " << i << std::endl;
        //     }
        // }
    }
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        generateInitialSchedule(proposed_schedule, env, end_time);
        // std::cout << "generate initial schedule done" << std::endl;
        // proposed_scheduleに-1があるかどうかを確認
        // for (int i = 0; i < proposed_schedule.size(); i++)
        // {
        //     if (proposed_schedule[i] == -1)
        //     {
        //         std::cout << "agent_id: " << i << std::endl;
        //         std::cerr << "proposed_schedule[i] is -1" << std::endl;
        //     }
        // }
        // for (int agent_id : SchedulerUtils::global_free_agents)
        // {
        //     std::cout << proposed_schedule[agent_id] << " ";
        // }
        // std::cout << std::endl;
        if (std::chrono::steady_clock::now() < end_time)
        {
            optimizeAssignment(proposed_schedule, env, end_time);
        }
    }
}