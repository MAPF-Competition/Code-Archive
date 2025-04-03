#include "ParallelLNSSchedulerGame.h"
#include <thread>
#include <algorithm>

namespace ParallelLNSSchedulerGame
{
    unsigned int num_threads;
    std::vector<ThreadData> thread_data;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // スレッド数を取得（ハードウェアの同時実行可能なスレッド数）
        num_threads = std::thread::hardware_concurrency();
        // 最低でも2スレッド、最大で10スレッドを使用
        // num_threads = std::max(2u, std::min(10u, num_threads));

        // スレッドごとのデータ構造を初期化
        thread_data.clear();
        thread_data.reserve(num_threads);
    }

    void repair(std::unordered_map<int, int> &schedule,
                const std::unordered_map<int, int> &task_agent_map,
                const std::vector<int> &destroyed_agents,
                SharedEnvironment *env,
                std::chrono::steady_clock::time_point end_time,
                SharedData &shared_data)
    {
        std::unordered_set<int> assigned_tasks;
        int count = 0;
        // 打ち切りのカウント
        // int max_count = SchedulerUtils::map_name == "brc202d.map" || SchedulerUtils::map_name == "random-32-32-20.map" ? schedule.size() : INT_MAX;
        int max_count = SchedulerUtilsGame::map_name == "brc202d.map" ? schedule.size() : INT_MAX;
        schedule.clear();
        for (int agent_id : destroyed_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            if (count >= max_count)
            {
                break;
            }
            count++;

            int current_task_id = env->curr_task_schedule[agent_id];

            std::vector<LNSSchedulerGame::TaskCost> task_costs;
            {
                std::shared_lock<std::shared_mutex> lock(shared_data.mutex);
                if (shared_data.agent_task_costs.find(agent_id) != shared_data.agent_task_costs.end())
                {
                    task_costs = shared_data.agent_task_costs[agent_id];
                }
            }

            if (task_costs.empty())
            {
                task_costs.reserve(SchedulerUtilsGame::global_available_tasks.size());
                for (int task_id : SchedulerUtilsGame::global_available_tasks)
                {
                    int cost = LNSSchedulerGame::calculateTaskCompletionTime(agent_id, task_id, env);
                    if (task_id == current_task_id)
                    {
                        cost += LNSSchedulerGame::same_task_additional_cost;
                    }
                    else if (env->task_pool[task_id].agent_assigned != -1)
                    {
                        cost += LNSSchedulerGame::assinged_task_additional_cost;
                    }
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
        for (int agent_id : SchedulerUtilsGame::global_available_agents)
        {
            if (env->curr_task_schedule[agent_id] != -1 &&
                proposed_schedule[agent_id] != env->curr_task_schedule[agent_id])
            {
                total_switches++;
            }
        }
        return total_switches;
    }

    OptimizationResult optimizeAssignment(std::vector<int> &proposed_schedule,
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
            std::vector<int> tmp_nearby_agents = LNSSchedulerGame::getNearbyAgents(agent_id, LNSSchedulerGame::MANHATTAN_DISTANCE_THRESHOLD, env);
            nearby_agents.insert(tmp_nearby_agents.begin(), tmp_nearby_agents.end());
        }
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtilsGame::global_available_agents)
        {
            if (best_schedule[agent_id] != -1)
            {
                task_agent_map[best_schedule[agent_id]] = agent_id;
            }
        }

        // LNSのパラメータ
        const double initial_destroy_ratio = 0.25;
        const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000;
        int failed_attempts = 0;
        const int MAX_FAILED_ATTEMPTS = 50;
        std::unordered_map<int, int> tmp_schedule;
        std::unordered_map<int, int> tmp_schedule_completion_times;
        // bestが更新されたフラグ
        bool best_updated = false;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        std::vector<std::pair<int, int>> destroyed_schedule;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 破壊（critical_agentsを渡す）
            std::vector<int> destroyed_agents = LNSSchedulerGame::destroy(current_destroy_ratio, critical_agents, nearby_agents);

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                if (best_schedule[agent_id] != -1)
                {
                    destroyed_schedule.push_back({agent_id, best_schedule[agent_id]});
                    task_agent_map.erase(best_schedule[agent_id]);
                    tmp_schedule[agent_id] = -1;
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
            auto [new_makespan, new_critical_agents] = LNSSchedulerGame::evaluateSchedule(tmp_schedule, env);

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
                std::tie(best_makespan, critical_agents) = LNSSchedulerGame::evaluateSchedule(best_schedule, env);
                nearby_agents.clear();
                for (int agent_id : critical_agents)
                {
                    std::vector<int> tmp_nearby_agents = LNSSchedulerGame::getNearbyAgents(agent_id, LNSSchedulerGame::MANHATTAN_DISTANCE_THRESHOLD, env);
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

        return {best_makespan, total_switches};
    }

    void schedule_plan(int time_limit,
                       std::vector<int> &proposed_schedule,
                       SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        LNSSchedulerGame::resetTaskSwitches(env);
        // 初期スケジュールを生成
        auto initial_result = LNSSchedulerGame::generateInitialSchedule(proposed_schedule, env, end_time);

        std::cout << "new_free_agents size: " << env->new_freeagents.size() << std::endl;

        if (env->new_tasks.size() == 0 && env->curr_timestep >= 100)
        {
            std::cout << "new_tasks size is 0 " << std::endl;
            return;
        }

        if (std::chrono::steady_clock::now() >= end_time)
            return;

        // 共有データの準備
        SharedData shared_data(initial_result.agent_task_costs);

        OptimizationData initial_data(initial_result.makespan, initial_result.critical_agents);

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
                                 { thread_data[i].result = optimizeAssignment(thread_data[i].schedule, initial_data, env, end_time, shared_data); });
        }

        // スレッドの終了を待機
        for (auto &thread : threads)
        {
            thread.join();
        }

        // 最良の結果を選択
        int best_thread = 0;
        int best_makespan = std::numeric_limits<int>::max();
        int best_switches = std::numeric_limits<int>::max();

        for (int i = 0; i < num_threads; ++i)
        {
            std::cout << i << " makespan: " << thread_data[i].result.makespan << " switches: " << thread_data[i].result.task_switches << std::endl;
            if (thread_data[i].result.makespan < best_makespan ||
                (thread_data[i].result.makespan == best_makespan &&
                 thread_data[i].result.task_switches < best_switches))
            {
                best_makespan = thread_data[i].result.makespan;
                best_switches = thread_data[i].result.task_switches;
                best_thread = i;
            }
        }
        std::cout << "initial_makespan: " << initial_result.makespan << std::endl;
        std::cout << "best_thread: " << best_thread << " makespan: " << best_makespan << " switches: " << best_switches << std::endl;

        // 最良の結果を反映
        proposed_schedule = thread_data[best_thread].schedule;
        LNSSchedulerGame::updateTaskSwitches(proposed_schedule, env);
    }
}