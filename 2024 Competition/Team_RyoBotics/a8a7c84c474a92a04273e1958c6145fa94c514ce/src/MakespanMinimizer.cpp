#include "MakespanMinimizer.h"
#include <random>
#include <algorithm>
#include <chrono>
#include <limits>

namespace MakespanMinimizer
{

    std::mt19937 mt(0); // 乱数生成器

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
        total_cost += SchedulerUtils::getMinDirectionalCost(
            current_loc, current_dir, task.locations[0]);

        // 各経由地点間のコスト
        // for (size_t i = 1; i < task.locations.size(); i++)
        // {
        //     total_cost += SchedulerUtils::getMinDirectionalCost(
        //         task.locations[i - 1], 0, task.locations[i]);
        // }

        return total_cost;
    }

    // スケジュール全体のmakespanを計算
    MakespanResult calculateMakespan(const std::vector<int> &schedule, SharedEnvironment *env, bool debug)
    {
        int makespan = 0;
        int max_agent_id = -1;

        for (size_t agent_id = 0; agent_id < schedule.size(); agent_id++)
        {
            if (schedule[agent_id] == -1)
            {
                std::cerr << "schedule[agent_id] is -1" << std::endl;
            }
            int completion_time = calculateTaskCompletionTime(agent_id, schedule[agent_id], env);
            if (debug)
            {
                std::cout << completion_time << " ";
            }
            if (completion_time > makespan)
            {
                makespan = completion_time;
                max_agent_id = agent_id;
            }
        }
        if (debug)
        {
            std::cout << std::endl;
        }
        return {makespan, max_agent_id};
    }

    void assignNewTasks(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        const std::unordered_set<unsigned short> &global_available_agents = SchedulerUtils::global_available_agents;
        const std::unordered_set<int> &global_available_tasks = SchedulerUtils::global_available_tasks;
        std::unordered_set<int> assigned_tasks;
        for (int agent_id : global_available_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            if (env->curr_task_schedule[agent_id] != -1)
                continue;
            // global_available_tasksの中で、env->task_poolから未割り当てのタスクを取得
            // 同時に距離を計算
            std::vector<std::pair<int, int>> candidate_tasks;
            for (int task_id : global_available_tasks)
            {
                if (env->task_pool[task_id].agent_assigned == -1 && assigned_tasks.find(task_id) == assigned_tasks.end())
                {
                    int cost = SchedulerUtils::evaluateCost(agent_id, task_id, env);
                    candidate_tasks.push_back({task_id, cost});
                }
            }
            // 最小コストのタスクを見つける
            std::sort(candidate_tasks.begin(), candidate_tasks.end(),
                      [](const auto &a, const auto &b)
                      {
                          return a.second < b.second;
                      });
            int best_task = candidate_tasks[0].first;

            // タスクを割り当て
            if (best_task != -1)
            {
                proposed_schedule[agent_id] = best_task;
                // global_available_tasks.erase(best_task);
                assigned_tasks.insert(best_task);
            }
        }
    }
    // 2-opt近傍探索による改善（最適化版）
    bool improve2Opt(std::vector<int> &schedule, SharedEnvironment *env)
    {
        int current_makespan = calculateMakespan(schedule, env).makespan;
        bool improved = false;

        for (size_t i = 0; i < schedule.size(); i++)
        {
            for (size_t j = i + 1; j < schedule.size(); j++)
            {
                // タスクを交換
                std::swap(schedule[i], schedule[j]);

                // 交換後の完了時間を計算
                int completion_time_i = calculateTaskCompletionTime(i, schedule[i], env);
                int completion_time_j = calculateTaskCompletionTime(j, schedule[j], env);

                // メイクスパンの見直し
                int potential_makespan = current_makespan;

                // 交換したエージェントの完了時間が現在のメイクスパンに影響を与える場合
                if (completion_time_i > potential_makespan || completion_time_j > potential_makespan)
                {
                    potential_makespan = std::max(completion_time_i, completion_time_j);
                    // 他のエージェントの完了時間も考慮
                    for (size_t k = 0; k < schedule.size(); k++)
                    {
                        if (k != i && k != j)
                        {
                            int completion_time = calculateTaskCompletionTime(k, schedule[k], env);
                            if (completion_time > potential_makespan)
                            {
                                potential_makespan = completion_time;
                            }
                        }
                    }
                }

                // メイクスパンが改善されたかチェック
                if (potential_makespan < current_makespan)
                {
                    current_makespan = potential_makespan;
                    improved = true;
                }
                else
                {
                    // 改善されなければ元に戻す
                    std::swap(schedule[i], schedule[j]);
                }
            }
        }

        return improved;
    }

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // 初期化が必要な場合はここで行う
    }

    void optimizeAssignments(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        auto initial_result = calculateMakespan(proposed_schedule, env, true);
        int best_makespan = initial_result.makespan;
        int critical_agent = initial_result.max_agent_id;
        std::cout << "initial_makespan: " << best_makespan << " (agent " << critical_agent << ")" << std::endl;

        std::vector<int> global_free_agents(SchedulerUtils::global_available_agents.begin(), SchedulerUtils::global_available_agents.end());
        std::vector<int> available_tasks(SchedulerUtils::global_available_tasks.begin(), SchedulerUtils::global_available_tasks.end());
        if (global_free_agents.empty())
        {
            return;
        }

        int max_iterations = 100000000;
        int iteration = 0;
        int failed_attempts = 0;
        const int MAX_FAILED_ATTEMPTS = 10000;

        std::uniform_int_distribution<> dist(0, global_free_agents.size() - 1);
        std::uniform_int_distribution<> swap_type(0, 1); // 0: エージェント間スワップ, 1: 未割り当てタスクとのスワップ

        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : global_free_agents)
        {
            int assigned_task_id = proposed_schedule[agent_id];
            if (assigned_task_id != -1)
            {
                task_agent_map[assigned_task_id] = agent_id;
            }
        }

        while (iteration < max_iterations && std::chrono::steady_clock::now() < end_time && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            bool result = false;

            // critical_agentに割り当てられているタスクをswapする
            int critical_task_id = proposed_schedule[critical_agent];
            int swap_task_id = available_tasks[mt() % available_tasks.size()];
            if (critical_task_id == swap_task_id)
            {
                failed_attempts++;
                continue;
            }
            int new_cost = calculateTaskCompletionTime(critical_agent, swap_task_id, env);
            if (new_cost > best_makespan)
            {
                failed_attempts++;
                continue;
            }
            int current_cost = calculateTaskCompletionTime(critical_agent, critical_task_id, env);
            assert(current_cost == best_makespan);
            int swap_current_cost = 0;
            int swap_new_cost = 0;
            auto it = task_agent_map.find(swap_task_id);
            if (it != task_agent_map.end())
            {
                // swapするagentのid
                int swap_agent_id = it->second;
                swap_current_cost = calculateTaskCompletionTime(swap_agent_id, swap_task_id, env);
                swap_new_cost = calculateTaskCompletionTime(swap_agent_id, critical_task_id, env);
                if (swap_new_cost > best_makespan)
                {
                    failed_attempts++;
                    continue;
                }
            }

            auto makespan_result = calculateMakespan(proposed_schedule, env);
            if (makespan_result.makespan > best_makespan)
            {
                throw std::runtime_error("new_makespan > best_makespan");
            }

            if (makespan_result.makespan < best_makespan)
            {
                best_makespan = makespan_result.makespan;
                critical_agent = makespan_result.max_agent_id;
                result = true;
            }

            if (result)
            {
                proposed_schedule[critical_agent] = swap_task_id;
                task_agent_map[swap_task_id] = critical_agent;
                if (it != task_agent_map.end())
                {
                    task_agent_map[critical_task_id] = it->second;
                    proposed_schedule[it->second] = critical_task_id;
                }
                else
                {
                    task_agent_map.erase(critical_task_id);
                }
                failed_attempts = 0;
            }
            else
            {
                failed_attempts++;
            }

            iteration++;
        }
        std::cout << "best_makespan: " << best_makespan << " (agent " << critical_agent << ")" << std::endl;
    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit); // 半分の時間を使用

        // 現在の割り当てを保持
        // std::vector<int> current_schedule = proposed_schedule;
        assignNewTasks(proposed_schedule, env, end_time);
        // ローカルサーチのメインループ
        optimizeAssignments(proposed_schedule, env, end_time);
        // proposed_scheduleとcurrent_scheduleを比較
        int count = 0;
        for (int i = 0; i < proposed_schedule.size(); i++)
        {
            if (env->curr_task_schedule[i] != -1 && proposed_schedule[i] != env->curr_task_schedule[i])
            {
                count++;
            }
        }
        std::cout << "changed_count: " << count << std::endl;
    }

} // namespace LocalSearchScheduler