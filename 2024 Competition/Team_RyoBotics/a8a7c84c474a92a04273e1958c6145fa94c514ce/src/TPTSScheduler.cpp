#include "scheduler.h"
#include "TPTSScheduler.h"
#include <fstream>
#include <algorithm>
#include <chrono>
#include <queue>
#include "PathFinder.h"
#include "SchedulerUtils.h"
#include "APSPCalculator.h"
using namespace SchedulerUtils;
namespace DefaultPlanner
{
    // static FastDistanceTable distance_table;
    static std::mt19937 mt3;
    // std::unordered_set<int> global_available_tasks;
    // // std::unordered_set<int> global_ongoing_agents;
    // std::unordered_set<int> global_free_agents;
    // std::unordered_map<int, int> current_task_assignments;
    static std::vector<int> agent_indices;

    void schedule_initialize_TPTS(int preprocess_time_limit, SharedEnvironment *env)
    {
        // if (true || env->map_name == "random-32-32-20.map")
        // {
        //     apsp_table = APSPCalculator::calculateAPSP(env->map, env->rows, env->cols);
        // }
        // 距離テーブルの初期化
        std::cout << "距離テーブルの初期化" << std::endl;
        std::string filepath = env->file_storage_path + "/" + env->map_name + "_distance_table.bin";
        std::ifstream check_file(filepath);
        if (!check_file.good())
        {
            DistanceTable::computeAndSave(env, filepath);
        }
        check_file.close();
        std::cout << "距離テーブルの読み込み" << std::endl;
        distance_table = DistanceTable::load(filepath);
        std::cout << "距離テーブルの読み込み完了" << std::endl;
        mt3.seed(0);

        global_available_agents.clear();
        global_available_agents.reserve(env->num_of_agents);
        // global_ongoing_agents.clear();
        global_available_tasks.clear();
        current_task_assignments.clear();
    }

    // int evaluateDistance(int agent_id, int task_id, SharedEnvironment *env)
    // {
    //     // タスクIDが無効な場合はエラー
    //     if (task_id == -1)
    //     {
    //         throw std::runtime_error("Invalid task ID: -1");
    //     }

    //     auto task_it = env->task_pool.find(task_id);
    //     if (task_it == env->task_pool.end())
    //     {
    //         throw std::runtime_error("Task " + std::to_string(task_id) + " not found in task pool");
    //     }

    //     const Task &task = task_it->second;
    //     if (task.locations.empty())
    //     {
    //         throw std::runtime_error("Task " + std::to_string(task_id) + " has no locations");
    //     }
    //     int agent_location = env->curr_states[agent_id].location;
    //     int task_location = task.locations[0];
    //     int agent_orientation = env->curr_states[agent_id].orientation;

    //     // 基本の距離を計算
    //     int base_distance = distance_table.getDistance(
    //         agent_location,
    //         task_location);

    //     // int path_distance = PathFinder::findPath(agent_location, task_location, env->map, env->rows, env->cols).size() - 1;

    //     // if (base_distance != path_distance)
    //     // {
    //     //     std::cout << "エラー: 基本距離とパス距離が一致しません" << std::endl;
    //     //     std::cout << "agent_location: " << agent_location << std::endl;
    //     //     std::cout << "task_location: " << task_location << std::endl;
    //     //     std::cout << "base_distance: " << base_distance << std::endl;
    //     //     std::cout << "path_distance: " << path_distance << std::endl;
    //     //     // throw std::runtime_error("距離計算エラー");
    //     // }

    //     // 方向による追加コストを計算
    //     int direction_cost = calculateDirectionCost(
    //         agent_location,
    //         task_location,
    //         agent_orientation,
    //         env->cols); // マップの幅を渡して座標計算に使用

    //     return base_distance + direction_cost;
    // }

    // 方向コストを計算する新しい関数
    int calculateDirectionCost(int from_loc, int to_loc, int current_orientation, int map_width)
    {
        // グリッド上の座標に変換
        int from_x = from_loc % map_width;
        int from_y = from_loc / map_width;
        int to_x = to_loc % map_width;
        int to_y = to_loc / map_width;

        // 目標への方向を計算
        int target_orientation;
        if (std::abs(to_x - from_x) > std::abs(to_y - from_y))
        {
            // X方向の差の方が大きい場合
            target_orientation = (to_x > from_x) ? 0 : 2; // 東または西
        }
        else
        {
            // Y方向の差の方が大きい場合
            target_orientation = (to_y > from_y) ? 1 : 3; // 南または北
        }

        // 向きの差を計算（最小回転数）
        int orientation_diff = std::abs(current_orientation - target_orientation);
        if (orientation_diff > 2)
        {
            orientation_diff = 4 - orientation_diff;
        }

        // 各90度回転につき1のコストを追加
        return orientation_diff;
    }

    bool GetTask(int agent_id, std::vector<int> &schedule, SharedEnvironment *env,
                 std::unordered_set<int> &available_tasks, std::unordered_set<int> &processed_agents,
                 std::chrono::steady_clock::time_point endtime, int depth, int cost_threshold)
    {
        const int MAX_RECURSION_DEPTH = 100; // 最大再帰深さを定義

        auto starttime = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() >= endtime)
        {
            return false;
        }
        // エージェントが既に処理済みの場合は終了
        if (processed_agents.find(agent_id) != processed_agents.end())
        {
            return false;
        }
        processed_agents.insert(agent_id);

        // 現在のエージェントの元の割り当てを保存
        int current_agent_original_task = schedule[agent_id];

        // 利用可能なタスクの集合のコピーを作成
        std::unordered_set<int> &T_prime = available_tasks;

        // タスクと距離のペアを配列に格納
        std::vector<std::tuple<int, bool, int>> tasks;        // <distance, is_assigned, task_id>
        std::vector<std::tuple<int, bool, int>> backup_tasks; // バックアップ用のタスクリスト
        tasks.reserve(T_prime.size());
        backup_tasks.reserve(T_prime.size());
        const int DISTANCE_THRESHOLD = 200; // 距離の閾値
        int current_task_distance = -1;
        if (current_agent_original_task != -1)
            current_task_distance = evaluateDistance(agent_id, current_agent_original_task, env);

        if (current_agent_original_task != -1 && cost_threshold != -1)
        {
            cost_threshold = current_task_distance;
        }
        else if (cost_threshold == -1)
        {
            cost_threshold = DISTANCE_THRESHOLD;
        }

        for (int task_id : T_prime)
        {
            if (std::chrono::steady_clock::now() >= endtime)
            {
                return false;
            }
            int distance = evaluateDistance(agent_id, task_id, env);
            if (distance < std::numeric_limits<unsigned short>::max())
            {
                bool is_assigned = current_task_assignments.find(task_id) != current_task_assignments.end();

                // 距離に基づいて仕分け
                if (distance < cost_threshold - 5)
                {
                    tasks.push_back({distance, is_assigned, task_id});
                }
                // else if (tasks.empty())
                // {
                //     backup_tasks.push_back({distance, is_assigned, task_id});
                // }
            }
        }

        // 距離が同じ場合は未割り当てのタスクを優先するようにソート
        auto sort_tasks = [](auto &task_list)
        {
            std::sort(task_list.begin(), task_list.end(),
                      [](const auto &a, const auto &b) noexcept
                      {
                          // 距離を直接比較（タプルの0番目の要素）
                          if (std::get<0>(a) != std::get<0>(b))
                              return std::get<0>(a) < std::get<0>(b);
                          // 割り当て状態を直接比較（タプルの1番目の要素）
                          return std::get<1>(a) < std::get<1>(b); // falseが先（未割り当てを優先）
                      });
        };

        // メインのタスクリストが空の場合、バックアップを使用
        if (tasks.empty())
        {
            // if (depth == 0)
            // {
            //     return false;
            // }
            // const int MAX_SAMPLES = 100; // サンプリングする最大タスク数
            // if (backup_tasks.size() > MAX_SAMPLES)
            // {
            //     // ランダムにMAX_SAMPLES個のタスクを選択
            //     std::vector<std::tuple<int, bool, int>> sampled_tasks;
            //     sampled_tasks.reserve(MAX_SAMPLES);

            //     // Fisher-Yatesシャッフルの変形を使用して効率的にサンプリング
            //     for (size_t i = 0; i < MAX_SAMPLES; ++i)
            //     {
            //         size_t j = i + (mt3() % (backup_tasks.size() - i));
            //         sampled_tasks.push_back(tasks[j]);
            //         std::swap(backup_tasks[j], backup_tasks[i]);
            //     }
            //     tasks = std::move(sampled_tasks);
            // }
            // else
            // {
            //     tasks = std::move(backup_tasks);
            // }
            tasks = std::move(backup_tasks);
        }

        sort_tasks(tasks);

        // インデックスを使って順番にアクセス - O(1)
        for (const auto &[distance, is_assigned, task_id] : tasks)
        {
            if (std::chrono::steady_clock::now() >= endtime)
            {
                break;
            }

            // 深い再帰レベルでは、未割り当てタスクのみを考慮
            if (depth >= MAX_RECURSION_DEPTH && is_assigned)
            {
                continue;
            }

            auto assigned_it = current_task_assignments.find(task_id);
            if (assigned_it == current_task_assignments.end())
            {
                schedule[agent_id] = task_id;
                current_task_assignments[task_id] = agent_id;
                if (current_agent_original_task != -1)
                {
                    current_task_assignments.erase(current_agent_original_task);
                }

                return true;
            }
            else if (depth < MAX_RECURSION_DEPTH && current_task_assignments[task_id] != agent_id) // 深さが制限未満の場合のみ再割り当てを試行
            {
                // タスクが他のエージェントに割り当て済みの場合
                int assigned_agent_id = assigned_it->second;
                int assigned_agent_distance = evaluateDistance(assigned_agent_id, task_id, env);

                if (distance < assigned_agent_distance)
                {
                    // std::cout << "  より良い割り当てが可能なため再割り当てを試行" << std::endl;
                    int assigned_agent_original_task = schedule[assigned_agent_id];
                    if (assigned_agent_id != current_task_assignments[assigned_agent_original_task])
                    {
                        std::cerr << "エラー: 割り当てられたエージェントID (" << assigned_agent_id
                                  << ") と current_task_assignments[" << assigned_agent_original_task
                                  << "] (" << current_task_assignments[assigned_agent_original_task]
                                  << ") が一致しません。" << std::endl;
                        throw std::runtime_error("エージェントIDの不一致");
                    }
                    if (assigned_agent_id != current_task_assignments[task_id])
                    {
                        std::cerr << "エラー: 割り当てられたエージェントID (" << assigned_agent_id
                                  << ") と current_task_assignments[" << assigned_agent_original_task
                                  << "] (" << current_task_assignments[task_id]
                                  << ") が一致しません。" << std::endl;
                        throw std::runtime_error("エージェントIDの不一致");
                    }

                    // 新しい割り当てを更新
                    schedule[agent_id] = task_id;
                    schedule[assigned_agent_id] = -1;
                    current_task_assignments[task_id] = agent_id;
                    if (current_agent_original_task != -1)
                    {
                        current_task_assignments.erase(current_agent_original_task);
                    }
                    int new_cost_threshold = current_task_distance != -1 ? assigned_agent_distance + current_task_distance - distance : -1;

                    if (GetTask(assigned_agent_id, schedule, env, available_tasks,
                                processed_agents, endtime, depth + 1, new_cost_threshold)) // depth を増やして再帰
                    {
                        return true;
                    }

                    // available_tasks.insert(task_id);
                    schedule[agent_id] = current_agent_original_task;
                    if (current_agent_original_task != -1)
                    {
                        current_task_assignments[current_agent_original_task] = agent_id;
                    }
                    schedule[assigned_agent_id] = assigned_agent_original_task;
                    if (assigned_agent_original_task != -1)
                    {
                        current_task_assignments[assigned_agent_original_task] = assigned_agent_id;
                    }
                    else
                    {
                        std::cerr << "エラー: assigned_agent_original_taskが-1です。" << std::endl;
                        throw std::runtime_error("assigned_agent_original_taskのエラー");
                    }
                }
            }
        }

        // std::cout << "\n  === 利用可能なタスクをすべて試行済み ===" << std::endl;

        if (env->curr_task_schedule[agent_id] == -1)
        {
            if (current_task_assignments[current_agent_original_task] == agent_id)
            {
                std::cout << "警告: エージェント " << agent_id
                          << " がタスク " << current_agent_original_task
                          << " を持っているのに、タスクが解放されようとしています" << std::endl;
            }
            schedule[agent_id] = -1;
            if (current_agent_original_task != -1)
            {
                current_task_assignments.erase(current_agent_original_task);
            }
            // processed_agents.erase(agent_id);
            return true;
        }
        else
        {
            // processed_agents.erase(agent_id);
            return false;
        }

        // processed_agents.erase(agent_id);
        return false;
    }

    void schedule_plan_TPTS(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto endtime = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(time_limit);

        // データの更新処理を関数呼び出しに置き換え
        updateRunningTasks(env);
        updateTaskAssignments(env);
        updateAgents(env);
        // updateAgents(env);

        // エージェントのシャッフルと割り当て処理
        // std::shuffle(agent_indices.begin(), agent_indices.end(), mt3);
        std::unordered_set<int> processed_agents;
        std::unordered_set<int> &available_tasks = global_available_tasks;
        std::tuple<double, int> cost_and_count_agent = evaluateScheduleCost(proposed_schedule, env);
        std::cout << "cost as start of TPTS: ";
        std::cout << "cost: " << std::get<0>(cost_and_count_agent) << ", count_agent: " << std::get<1>(cost_and_count_agent) << ", cost / count_agent: " << std::get<0>(cost_and_count_agent) / std::get<1>(cost_and_count_agent) << std::endl;
        std::cout << "global_free_agents.size(): " << global_available_agents.size() << std::endl;

        for (int agent_id : agent_indices)
        {
            if (std::chrono::steady_clock::now() >= endtime)
            {
                break;
            }
            if (available_tasks.empty())
            {
                break;
            }
            processed_agents.clear();
            GetTask(agent_id, proposed_schedule, env, available_tasks, processed_agents, endtime, 0, -1);
        }
        cost_and_count_agent = evaluateScheduleCost(proposed_schedule, env);

        std::cout << "cost as end of TPTS:   ";
        std::cout << "cost: " << std::get<0>(cost_and_count_agent) << ", count_agent: " << std::get<1>(cost_and_count_agent) << ", cost / count_agent: " << std::get<0>(cost_and_count_agent) / std::get<1>(cost_and_count_agent) << std::endl;
        std::cout << "global_free_agents.size(): " << global_available_agents.size() << std::endl;

        // for (int agent_id : global_free_agents)
        // {
        //     int task_id = proposed_schedule[agent_id];
        //     if (task_id != -1)
        //     {
        //         int curr_agent_id = current_task_assignments[task_id];
        //         if (curr_agent_id != agent_id)
        //         {
        //             std::cout << "end of schedule" << std::endl;
        //             std::cout << "エラー: タスク " << task_id << " がエージェント " << agent_id << " に割り当てられているのに、エージェント " << curr_agent_id << " が割り当てられています" << std::endl;
        //             throw std::runtime_error("エラー: タスク " + std::to_string(task_id) + " がエージェント " + std::to_string(agent_id) + " に割り当てられているのに、エージェント " + std::to_string(curr_agent_id) + " が割り当てられています");
        //         }
        //     }
        // }

        // if (!validateSchedule(proposed_schedule, env))
        // {
        //     std::cout << "エラー: スケジュールが無効です" << std::endl;
        //     throw std::runtime_error("エラー: スケジュールが無効です");
        // }
    }
}