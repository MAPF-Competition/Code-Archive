#include "ParallelScheduler.h"
#include "NewScheduler.h"
#include "scheduler.h"
#include "distance_table.h"
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace DefaultPlanner
{
    std::mt19937 mt2;
    FastDistanceTable distance_table;

    // グローバル変数としてパラメータを保持
    TabuParameters g_tabu_params;

    void schedule_initialize2(int preprocess_time_limit, SharedEnvironment *env)
    {
        DefaultPlanner::init_heuristics(env);

        // 距離テーブルのファイルパスを構築
        std::string filepath = env->file_storage_path + "/" + env->map_name + "_distance_table.bin";

        // ファイルが存在するか確認
        std::ifstream check_file(filepath);
        if (!check_file.good())
        {
            // ファイルが存在しない場合は計算して保存
            DistanceTable::computeAndSave(env, filepath);
        }
        check_file.close();

        // 距離テーブルを読み込む
        distance_table = DistanceTable::load(filepath);

        // タブーサーチパラメータの読み込み
        try
        {
            std::string param_path = env->file_storage_path + "/tabu_params.json";
            std::ifstream f(param_path);
            if (f.good())
            {
                json params = json::parse(f);
                g_tabu_params.base_tabu_tenure = params.value("base_tabu_tenure", 10);
                g_tabu_params.max_iterations_without_improvement =
                    params.value("max_iterations_without_improvement", 50);
                g_tabu_params.step_for_tabu_increase =
                    params.value("step_for_tabu_increase", 100);
                g_tabu_params.step_for_tabu_reset =
                    params.value("step_for_tabu_reset", 1000);
            }
        }
        catch (const std::exception &e)
        {
            // ファイルが読めない場合はデフォルト値を使用
            g_tabu_params = TabuParameters();
        }

        mt2.seed(0);
    }

    bool canAssignTask(int agent_idx, int task_id, SharedEnvironment *env, const std::vector<int> &current_schedule)
    {
        // 現在のタスクスケジュールをチェック
        int current_task = env->curr_task_schedule[agent_idx];

        // エージェントが進行中のタスクを持っている場合
        if (current_task != -1)
        {
            auto current_task_it = env->task_pool.find(current_task);
            if (current_task_it != env->task_pool.end() &&
                current_task_it->second.idx_next_loc > 0)
            {
                // 進行中のタスクがある場合、同じタスクのみ許可
                return task_id == current_task;
            }
        }

        if (task_id == -1)
            return true;

        auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end())
            return false;

        const Task &tsk = it->second;

        // タスクが進行中の場合のみ、現在割り当てられているエージェントに制限
        if (tsk.idx_next_loc > 0)
        {
            return tsk.agent_assigned == agent_idx;
        }

        // タスクが未開始の場合は、他のエージェントのスケジュールのみをチェック
        for (int a = 0; a < (int)current_schedule.size(); a++)
        {
            if (a == agent_idx)
                continue;
            if (current_schedule[a] == task_id)
                return false;
        }

        return true;
    }

    int computeScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env)
    {
        int total_cost = 0;
        for (int a = 0; a < env->num_of_agents; a++)
        {
            if (schedule[a] == -1)
                continue;

            auto task_it = env->task_pool.find(schedule[a]);
            if (task_it == env->task_pool.end())
                continue;

            int dist_sum = 0;
            int c_loc = env->curr_states[a].location;
            for (auto loc : task_it->second.locations)
            {
                int distance = distance_table.getDistance(c_loc, loc);
                dist_sum += distance;
                c_loc = loc;
            }
            total_cost += dist_sum;
        }
        return total_cost;
    }

    void handleFreeAgents(std::vector<int> &schedule, SharedEnvironment *env)
    {
        for (int free_agent : env->new_freeagents)
        {
            for (int i = 0; i < env->num_of_agents; i++)
            {
                if (i == free_agent)
                    continue;

                int current_task = schedule[i];
                if (current_task == -1)
                    continue;

                auto task_it = env->task_pool.find(current_task);
                if (task_it == env->task_pool.end() || task_it->second.idx_next_loc > 0)
                    continue;

                // 現在の割り当てコストを計算
                int current_cost = 0;
                {
                    int c_loc = env->curr_states[i].location;
                    for (int loc : task_it->second.locations)
                    {
                        current_cost += distance_table.getDistance(c_loc, loc);
                        c_loc = loc;
                    }
                }

                // 新しいエージェントへの割り当てコストを計算
                int new_cost = 0;
                {
                    int c_loc = env->curr_states[free_agent].location;
                    for (int loc : task_it->second.locations)
                    {
                        new_cost += distance_table.getDistance(c_loc, loc);
                        c_loc = loc;
                    }
                }

                if (new_cost < current_cost)
                {
                    schedule[free_agent] = current_task;
                    schedule[i] = -1;
                    task_it->second.agent_assigned = free_agent;
                    break;
                }
            }
        }
    }

    std::vector<int> generateInitialSchedule(SharedEnvironment *env)
    {
        std::vector<int> schedule(env->num_of_agents, -1);

        // まず、現在のタスクスケジュールを維持
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int current_task = env->curr_task_schedule[i];
            if (current_task != -1)
            {
                auto task_it = env->task_pool.find(current_task);
                if (task_it != env->task_pool.end() && task_it->second.idx_next_loc > 0)
                {
                    // 進行中のタスクは必ず維持
                    schedule[i] = current_task;
                    continue;
                }
            }

            // 以下、新規タスクの割り当て処理
            int min_task_i = -1;
            int min_task_makespan = INT_MAX;

            for (const auto &task_pair : env->task_pool)
            {
                const Task &task = task_pair.second;
                if (task.agent_assigned != -1 || task.idx_next_loc > 0)
                    continue;

                int dist = 0;
                int c_loc = env->curr_states.at(i).location;
                for (int loc : task.locations)
                {
                    dist += distance_table.getDistance(c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan)
                {
                    min_task_i = task_pair.first;
                    min_task_makespan = dist;
                }
            }

            if (min_task_i != -1)
            {
                schedule[i] = min_task_i;
                auto it = env->task_pool.find(min_task_i);
                if (it != env->task_pool.end())
                {
                    it->second.agent_assigned = i;
                }
            }
        }
        return schedule;
    }

    bool tryTaskSwap(int agent1, int agent2, std::vector<int> &schedule, SharedEnvironment *env)
    {
        int task1 = schedule[agent1];
        int task2 = schedule[agent2];

        // 両方のタスクが-1の場合はスワップの必要なし
        if (task1 == -1 && task2 == -1)
            return false;

        // 一時的にスワップ
        std::swap(schedule[agent1], schedule[agent2]);

        // スワップ後の割り当てが有効かチェック
        bool valid = canAssignTask(agent1, schedule[agent1], env, schedule) &&
                     canAssignTask(agent2, schedule[agent2], env, schedule);

        if (!valid)
        {
            // 無効な場合は元に戻す
            std::swap(schedule[agent1], schedule[agent2]);
            return false;
        }

        // タスクの割り当て情報を更新
        if (schedule[agent1] != -1)
        {
            auto it1 = env->task_pool.find(schedule[agent1]);
            if (it1 != env->task_pool.end())
            {
                it1->second.agent_assigned = agent1;
            }
        }
        if (schedule[agent2] != -1)
        {
            auto it2 = env->task_pool.find(schedule[agent2]);
            if (it2 != env->task_pool.end())
            {
                it2->second.agent_assigned = agent2;
            }
        }

        return true;
    }

    void performTaskSwaps(std::vector<int> &schedule, SharedEnvironment *env)
    {
        bool improved;
        do
        {
            improved = false;
            for (int i = 0; i < env->num_of_agents; i++)
            {
                for (int j = i + 1; j < env->num_of_agents; j++)
                {
                    int old_cost = computeScheduleCost(schedule, env);

                    if (tryTaskSwap(i, j, schedule, env))
                    {
                        int new_cost = computeScheduleCost(schedule, env);
                        if (new_cost < old_cost)
                        {
                            improved = true;
                        }
                        else
                        {
                            // コスト改善がない場合は元に戻す
                            tryTaskSwap(i, j, schedule, env);
                        }
                    }
                }
            }
        } while (improved);
    }

    SchedulingResult performTabuSearch(std::vector<int> &initial_schedule,
                                       SharedEnvironment *env,
                                       const TabuParameters &params,
                                       const TimePoint &endtime)
    {
        std::vector<int> current_schedule = initial_schedule;
        std::vector<int> best_schedule = current_schedule;
        int current_cost = computeScheduleCost(current_schedule, env);
        int best_cost = current_cost;

        int tabu_tenure = params.base_tabu_tenure;
        std::deque<TabuMove> tabu_list;
        int iterations_without_improvement = 0;

        while (iterations_without_improvement < params.max_iterations_without_improvement &&
               std::chrono::steady_clock::now() < endtime)
        {

            bool found_better_neighbor = false;
            int best_neighbor_cost = INT_MAX;
            int best_i = -1, best_j = -1;

            // 近傍探索
            for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
            {
                for (int j = i + 1; j < env->num_of_agents && std::chrono::steady_clock::now() < endtime; j++)
                {
                    if (current_schedule[i] == -1 && current_schedule[j] == -1)
                        continue;

                    bool is_tabu = false;
                    for (const auto &tabu_move : tabu_list)
                    {
                        if ((tabu_move.agent1 == i && tabu_move.agent2 == j &&
                             tabu_move.task1 == current_schedule[j] && tabu_move.task2 == current_schedule[i]) ||
                            (tabu_move.agent1 == j && tabu_move.agent2 == i &&
                             tabu_move.task1 == current_schedule[i] && tabu_move.task2 == current_schedule[j]))
                        {
                            is_tabu = true;
                            break;
                        }
                    }

                    std::swap(current_schedule[i], current_schedule[j]);

                    if (!canAssignTask(i, current_schedule[i], env, current_schedule) ||
                        !canAssignTask(j, current_schedule[j], env, current_schedule))
                    {
                        std::swap(current_schedule[i], current_schedule[j]);
                        continue;
                    }

                    int neighbor_cost = computeScheduleCost(current_schedule, env);

                    if (!is_tabu || neighbor_cost < best_cost)
                    {
                        if (neighbor_cost < best_neighbor_cost)
                        {
                            best_neighbor_cost = neighbor_cost;
                            best_i = i;
                            best_j = j;
                        }
                    }

                    std::swap(current_schedule[i], current_schedule[j]);
                }
            }

            // 最良の近傍解に移動
            if (best_i != -1)
            {
                std::swap(current_schedule[best_i], current_schedule[best_j]);

                tabu_list.push_back(TabuMove(best_i, best_j,
                                             current_schedule[best_i], current_schedule[best_j]));
                if (tabu_list.size() > (size_t)tabu_tenure)
                {
                    tabu_list.pop_front();
                }

                current_cost = best_neighbor_cost;

                if (current_cost < best_cost)
                {
                    best_cost = current_cost;
                    best_schedule = current_schedule;
                    iterations_without_improvement = 0;
                    tabu_tenure = params.base_tabu_tenure;
                }
                else
                {
                    iterations_without_improvement++;
                    if (iterations_without_improvement % params.step_for_tabu_increase == 0)
                    {
                        tabu_tenure += 2;
                    }
                }
            }
            else
            {
                break;
            }

            // リスタート戦略
            if (iterations_without_improvement > 0 && iterations_without_improvement % 100 == 0)
            {
                current_schedule = generateInitialSchedule(env);
                current_cost = computeScheduleCost(current_schedule, env);
                if (current_cost < best_cost)
                {
                    best_cost = current_cost;
                    best_schedule = current_schedule;
                    iterations_without_improvement = 0;
                    tabu_tenure = params.base_tabu_tenure;
                }
            }
        }

        return {best_schedule, best_cost};
    }

    struct AgentTaskSets
    {
        std::vector<int> available_agents; // フリーエージェントと未open agentの集合
        std::vector<int> assignable_tasks; // 未割り当てタスクと未openタスクの集合
    };

    // 割り当て可能なエージェントとタスクの集合を取得
    AgentTaskSets getAssignableSets(SharedEnvironment *env)
    {
        AgentTaskSets sets;

        // 利用可能なエージェントを収集
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int current_task = env->curr_task_schedule[i];

            // フリーエージェントの場合
            if (current_task == -1)
            {
                sets.available_agents.push_back(i);
                continue;
            }

            // タスクが割り当て済みだが未openの場合
            // auto task_it = env->task_pool.find(current_task);
            // if (task_it != env->task_pool.end() && task_it->second.idx_next_loc == 0)
            // {
            //     sets.available_agents.push_back(i);
            // }
        }

        // 新しく追加されたタスクを収集
        for (int task_id : env->new_tasks)
        {
            sets.assignable_tasks.push_back(task_id);
        }

        // 割り当て済み未openタスクを収集
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int current_task = env->curr_task_schedule[i];
            if (current_task != -1)
            {
                auto task_it = env->task_pool.find(current_task);
                if (task_it != env->task_pool.end() && task_it->second.idx_next_loc == 0)
                {
                    // 既に追加済みでないことを確認
                    if (std::find(sets.assignable_tasks.begin(), sets.assignable_tasks.end(), current_task) == sets.assignable_tasks.end())
                    {
                        sets.assignable_tasks.push_back(current_task);
                        // std::cerr << "Task " << current_task << " is added to assignable_tasks " << i << std::endl;
                    }
                }
            }
        }

        // assignable_tasksの重複チェック
        std::set<int> unique_tasks(sets.assignable_tasks.begin(), sets.assignable_tasks.end());
        if (unique_tasks.size() != sets.assignable_tasks.size())
        {
            std::cerr << "Warning: Duplicate tasks in assignable_tasks list!" << std::endl;
        }

        return sets;
    }

    // タブーサーチ用の新しい初期解生成
    void generateInitialScheduleNew(SharedEnvironment *env, std::vector<int> &proposed_schedule, int time_limit)
    {
        // デバッグ出力を追加
        // std::cerr << "\nAgent positions and task start locations:" << std::endl;
        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     std::cerr << "Agent " << i << " at position " << env->curr_states[i].location;
        //     if (proposed_schedule[i] != -1)
        //     {
        //         auto task_it = env->task_pool.find(proposed_schedule[i]);
        //         std::cerr << "task_it->second.locations: " << task_it->second.locations.empty() << std::endl;

        //         if (task_it != env->task_pool.end() && !task_it->second.locations.empty())
        //         {
        //             std::cerr << " -> Task " << proposed_schedule[i]
        //                       << " starts at " << task_it->second.locations[0]
        //                       << " (distance: " << distance_table.getDistance(env->curr_states[i].location, task_it->second.locations[0])
        //                       << ")";
        //         }
        //     }
        //     std::cerr << std::endl;
        // }
        // std::cerr << std::endl;
        for (int i = 0; i < env->num_of_agents; i++)
        {
            std::cerr << proposed_schedule[i] << " ";
        }
        std::cerr << std::endl;

        // デバッグ出力を追加して、どのような状況で重複が発生するか確認
        // std::cerr << "Initial assignments:" << std::endl;
        // std::map<int, std::vector<int>> task_to_agents; // タスクID -> エージェントIDのリスト

        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     // std::cerr << "Agent " << i << " -> Task " << proposed_schedule[i] << std::endl;
        //     if (proposed_schedule[i] != -1)
        //     {
        //         // std::cerr << "Agent " << i << " -> Task " << proposed_schedule[i] << std::endl;
        //         task_to_agents[proposed_schedule[i]].push_back(i);
        //     }
        // }

        // // 重複チェックと詳細な出力
        // for (const auto &pair : task_to_agents)
        // {
        //     if (pair.second.size() > 1)
        //     {
        //         std::cerr << "Warning: Task " << pair.first << " is assigned to multiple agents: ";
        //         for (int agent_id : pair.second)
        //         {
        //             std::cerr << agent_id << " ";
        //         }
        //         std::cerr << std::endl;
        //     }
        // }
        // 最後に重複チェック（デバッグ用）
        std::set<int> check_tasks1;
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (proposed_schedule[i] != -1)
            {
                if (!check_tasks1.insert(proposed_schedule[i]).second)
                {
                    std::cerr << "Error: Task " << proposed_schedule[i] << " is assigned multiple times!" << std::endl;
                    throw std::runtime_error("Duplicate task assignment detected as start of generateInitialScheduleNew");
                }
            }
        }
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
        std::vector<int> &schedule = proposed_schedule;
        auto sets = getAssignableSets(env);

        // タスクの割り当て状態を追跡するセット
        std::set<int> assigned_tasks;

        // 現在の割り当て状態を記録
        std::map<int, int> current_assignments; // task_id -> agent_id
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int current_task = env->curr_task_schedule[i];
            if (current_task != -1)
            {
                auto task_it = env->task_pool.find(current_task);
                if (task_it != env->task_pool.end() && task_it->second.idx_next_loc == 0)
                {
                    current_assignments[current_task] = i;
                    // schedule[i] = current_task; // 初期状態を設定
                }
            }
        }

        // 利用可能なエージェントのリストをシャッフル
        std::vector<int> shuffled_agents = sets.available_agents;
        std::shuffle(shuffled_agents.begin(), shuffled_agents.end(), mt2);

        // シャッフルされたエージェントに対して貪欲法で初期割り当て
        for (int agent_idx : shuffled_agents)
        {
            if (std::chrono::steady_clock::now() > endtime)
                break;

            int best_task = -1;
            int min_cost = INT_MAX;

            for (int task_id : sets.assignable_tasks)
            {
                // コスト計算
                int c_loc = env->curr_states[agent_idx].location;
                auto task_it = env->task_pool.find(task_id);
                if (task_it == env->task_pool.end())
                    continue;

                int cost = distance_table.getDistance(c_loc, task_it->second.locations[0]);

                // すでに割り当てられているが未openのタスクの場合
                auto current_agent_it = current_assignments.find(task_id);
                if (current_agent_it != current_assignments.end())
                {
                    // 現在割り当てられているエージェントのコストを計算
                    int current_agent_loc = env->curr_states[current_agent_it->second].location;
                    int current_cost = distance_table.getDistance(current_agent_loc, task_it->second.locations[0]);

                    // 現在の割り当ての方がコストが低い場合はスキップ
                    if (current_cost <= cost)
                    {
                        continue;
                    }
                }

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_task = task_id;
                }
            }

            if (best_task != -1)
            {
                // 既存の割り当てがある場合、その割り当てを解除して新しいタスクを割り当てる
                auto it = current_assignments.find(best_task);
                if (it != current_assignments.end())
                {
                    // 古いエージェントに対して、best_task以外の中から最適なタスクを見つける
                    int old_agent = it->second;
                    int alternative_best_task = -1;
                    int alternative_min_cost = INT_MAX;

                    for (int task_id : sets.assignable_tasks)
                    {
                        if (task_id == best_task)
                            continue; // best_task以外を探す

                        int c_loc = env->curr_states[old_agent].location;
                        auto task_it = env->task_pool.find(task_id);
                        if (task_it == env->task_pool.end())
                            continue;

                        int cost = distance_table.getDistance(c_loc, task_it->second.locations[0]);

                        // すでに割り当てられているタスクの場合はスキップ
                        auto current_agent_it = current_assignments.find(task_id);
                        if (current_agent_it != current_assignments.end())
                            continue;

                        if (cost < alternative_min_cost)
                        {
                            alternative_min_cost = cost;
                            alternative_best_task = task_id;
                        }
                    }

                    schedule[old_agent] = alternative_best_task;
                    if (alternative_best_task != -1)
                    {
                        current_assignments[alternative_best_task] = old_agent;
                    }
                    current_assignments.erase(best_task); // 古い割り当てを削除
                    auto alternative_best_task_it = std::find(sets.assignable_tasks.begin(), sets.assignable_tasks.end(), best_task);
                    if (alternative_best_task_it != sets.assignable_tasks.end())
                    {
                        sets.assignable_tasks.erase(alternative_best_task_it);
                    }
                }

                schedule[agent_idx] = best_task;
                current_assignments[best_task] = agent_idx; // 新しい割り当てを記録

                // assignable_tasksからbest_taskを削除
                auto best_task_it = std::find(sets.assignable_tasks.begin(), sets.assignable_tasks.end(), best_task);
                if (best_task_it != sets.assignable_tasks.end())
                {
                    sets.assignable_tasks.erase(best_task_it);
                }
            }
            else
            {
                schedule[agent_idx] = -1;
            }
        }
        for (int i = 0; i < env->num_of_agents; i++)
        {
            std::cerr << schedule[i] << " ";
        }
        std::cerr << std::endl;
        // 最後に重複チェック（デバッグ用）
        std::set<int> check_tasks;
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (schedule[i] != -1)
            {
                if (!check_tasks.insert(schedule[i]).second)
                {
                    std::cerr << "Error: Task " << schedule[i] << " is assigned multiple times!" << std::endl;
                    throw std::runtime_error("Duplicate task assignment detected as end of generateInitialScheduleNew");
                }
            }
        }

        // デバッグ出力を追加
        // std::cerr << "Final assignments:" << std::endl;
        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     std::cerr << "Agent " << i << " -> Task " << schedule[i] << std::endl;

        //     // if (schedule[i] != -1)
        //     // {
        //     //     std::cerr << "Agent " << i << " -> Task " << schedule[i] << std::endl;
        //     // }
        // }

        // return schedule;
        return;
    }

    void schedule_plan2(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        TimePoint endtime = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(time_limit);

        // 初期スケジュール生成
        generateInitialScheduleNew(env, proposed_schedule, time_limit);
        // std::cout << "初期スケジュール生成完了" << std::endl;
        if (std::chrono::steady_clock::now() > endtime)
        {
            return;
        }
        return;
        // マルチプロセス並列化版
        // 例として子プロセスを4つ並列起動して最良解を探す
        int process_count = 10;
        auto result = parallelTabuSearchMP(
            proposed_schedule, // 初期スケジュール
            env,
            g_tabu_params, // 読み込んだタブーサーチパラメータ
            endtime,
            process_count);

        // 最終スケジュール確定
        proposed_schedule = result.schedule;

        // 最終確認
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (!canAssignTask(i, proposed_schedule[i], env, proposed_schedule))
            {
                proposed_schedule[i] = -1;
            }
        }

        return;
    }

    // 近傍操作の種類を定義
    enum class NeighborType
    {
        SWAP,          // 2エージェント間でタスク交換
        REASSIGN,      // タスクの再割り当て
        INSERT,        // 新規タスク挿入
        CHAIN_TRANSFER // 複数エージェント間でのタスク移動連鎖
    };

    struct Neighbor
    {
        NeighborType type;
        int agent1;
        int agent2;
        int task1;
        int task2;
        int cost;
    };

    // コスト評価を改善
    int evaluateAssignment(int agent_idx, int task_id, SharedEnvironment *env,
                           const std::vector<int> &current_schedule)
    {
        if (task_id == -1)
            return 0;

        auto task_it = env->task_pool.find(task_id);
        if (task_it == env->task_pool.end())
            return INT_MAX;

        int c_loc = env->curr_states[agent_idx].location;
        int cost = 0;

        // 基本の移動コスト
        for (int loc : task_it->second.locations)
        {
            cost += distance_table.getDistance(c_loc, loc);
            c_loc = loc;
        }

        // エージェントの現在位置からの距離に応じたペナルティ
        int initial_distance = distance_table.getDistance(env->curr_states[agent_idx].location, task_it->second.locations[0]);
        cost += initial_distance * 2; // 初期移動距離を重視

        return cost;
    }

    // 近傍を生成して評価
    std::vector<Neighbor> generateNeighbors(
        const std::vector<int> &current_schedule,
        const AgentTaskSets &sets,
        SharedEnvironment *env)
    {
        std::vector<Neighbor> neighbors;

        // 1. タスク交換の近傍
        for (int i : sets.available_agents)
        {
            for (int j : sets.available_agents)
            {
                if (i >= j)
                    continue;

                std::vector<int> new_schedule = current_schedule;
                std::swap(new_schedule[i], new_schedule[j]);

                if (canAssignTask(i, new_schedule[i], env, new_schedule) &&
                    canAssignTask(j, new_schedule[j], env, new_schedule))
                {
                    int new_cost = 0;
                    new_cost += evaluateAssignment(i, new_schedule[i], env, new_schedule);
                    new_cost += evaluateAssignment(j, new_schedule[j], env, new_schedule);

                    neighbors.push_back({NeighborType::SWAP,
                                         i, j,
                                         current_schedule[i],
                                         current_schedule[j],
                                         new_cost});
                }
            }
        }

        // 2. タスク再割り当ての近傍
        for (int i : sets.available_agents)
        {
            int original_task = current_schedule[i];
            if (original_task == -1)
                continue;

            for (int j : sets.available_agents)
            {
                if (i == j)
                    continue;

                std::vector<int> new_schedule = current_schedule;
                new_schedule[i] = -1;
                new_schedule[j] = original_task;

                if (canAssignTask(j, original_task, env, new_schedule))
                {
                    int new_cost = evaluateAssignment(j, original_task, env, new_schedule);
                    neighbors.push_back({NeighborType::REASSIGN,
                                         i, j,
                                         original_task,
                                         original_task,
                                         new_cost});
                }
            }
        }

        // 3. 新しい近傍操作：CHAIN_TRANSFER
        // 最大3エージェントまでの連鎖的なタスク移動を試みる
        for (int i : sets.available_agents)
        {
            for (int j : sets.available_agents)
            {
                if (i == j)
                    continue;
                for (int k : sets.available_agents)
                {
                    if (k == i || k == j)
                        continue;

                    std::vector<int> new_schedule = current_schedule;
                    // i -> j -> k の連鎖的なタスク移動
                    int task_i = new_schedule[i];
                    int task_j = new_schedule[j];
                    int task_k = new_schedule[k];

                    new_schedule[k] = task_j;
                    new_schedule[j] = task_i;
                    new_schedule[i] = task_k;

                    bool valid = true;
                    if (task_i != -1)
                        valid &= canAssignTask(j, task_i, env, new_schedule);
                    if (task_j != -1)
                        valid &= canAssignTask(k, task_j, env, new_schedule);
                    if (task_k != -1)
                        valid &= canAssignTask(i, task_k, env, new_schedule);

                    if (valid)
                    {
                        int new_cost = 0;
                        new_cost += evaluateAssignment(i, task_k, env, new_schedule);
                        new_cost += evaluateAssignment(j, task_i, env, new_schedule);
                        new_cost += evaluateAssignment(k, task_j, env, new_schedule);

                        neighbors.push_back({NeighborType::CHAIN_TRANSFER,
                                             i, j,
                                             task_i, task_j,
                                             new_cost});
                    }
                }
            }
        }

        // 4. 未割り当てタスクの挿入
        std::vector<int> uncovered_tasks;
        for (int task_id : sets.assignable_tasks)
        {
            bool is_covered = false;
            for (int t : current_schedule)
            {
                if (t == task_id)
                {
                    is_covered = true;
                    break;
                }
            }
            if (!is_covered)
            {
                uncovered_tasks.push_back(task_id);
            }
        }

        // 未カバーのタスクを優先的に挿入を試みる
        for (int task_id : uncovered_tasks)
        {
            int best_agent = -1;
            int best_cost = INT_MAX;

            for (int i : sets.available_agents)
            {
                if (current_schedule[i] != -1)
                    continue;

                if (canAssignTask(i, task_id, env, current_schedule))
                {
                    int cost = evaluateAssignment(i, task_id, env, current_schedule);
                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_agent = i;
                    }
                }
            }

            if (best_agent != -1)
            {
                neighbors.push_back({NeighborType::INSERT,
                                     best_agent, -1,
                                     task_id, -1,
                                     best_cost});
            }
        }

        return neighbors;
    }
}
