#include "SchedulerUtilsGame.h"
#include <random>
#include <algorithm>
namespace SchedulerUtilsGame
{

    DefaultPlanner::FastDistanceTable distance_table;
    std::unordered_set<int> global_available_tasks;
    std::unordered_set<unsigned short> global_available_agents;
    std::unordered_map<int, int> current_task_assignments; // task ID -> agent ID
    DirectionalPathCostMap apsp_table;
    static std::mt19937 mt;
    int task_free = 0;
    int upper1 = 0;
    std::string map_name;

    // エージェントごとの最早開始可能時刻を保持
    std::vector<int> agent_available_time;
    // エージェントごとの開始可能位置を保持
    std::vector<std::pair<int, int>> agent_end_state;

    // タスクの総経路コストテーブル
    std::unordered_map<int, int> task_total_cost_table;

    // タスクは割り当てられているがまだopenしていないエージェントの集合
    std::unordered_set<unsigned short> assigned_but_not_open_agents;

    void schedule_initialize_SchedulerUtils(int preprocess_time_limit, SharedEnvironment *env)
    {
        if (true || env->map_name == "random-32-32-20.map")
        {
            apsp_table = ParallelAPSPCalculatorGame::calculateParallelAPSP(env->map, env->rows, env->cols);
        }
        // 距離テーブルの初期化
        // std::cout << "距離テーブルの初期化" << std::endl;
        // std::string filepath = env->file_storage_path + "/" + env->map_name + "_distance_table.bin";
        // std::ifstream check_file(filepath);
        // if (!check_file.good())
        // {
        //     DistanceTable::computeAndSave(env, filepath);
        // }
        // check_file.close();
        // std::cout << "距離テーブルの読み込み" << std::endl;
        // distance_table = DistanceTable::load(filepath);
        // std::cout << "距離テーブルの読み込み完了" << std::endl;
        std::cout << "cellToIndex.size(): " << cellToIndex.size() << std::endl;

        global_available_agents.clear();
        global_available_agents.reserve(env->num_of_agents);
        // global_ongoing_agents.clear();
        global_available_tasks.clear();
        current_task_assignments.clear();
        task_free = 0;
        upper1 = 0;
        // 新しい変数の初期化
        agent_available_time.resize(env->num_of_agents, 0);
        agent_end_state.resize(env->num_of_agents, std::make_pair(0, 0));
        map_name = env->map_name;

        // タスクの総経路コストテーブルをクリア
        task_total_cost_table.clear();

        // 新しい集合の初期化
        assigned_but_not_open_agents.clear();
        assigned_but_not_open_agents.reserve(env->num_of_agents);
    }
    int evaluateDistance(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
            return 0;

        auto task_it = env->task_pool.find(task_id);
        if (task_it == env->task_pool.end())
            return std::numeric_limits<int>::max();

        const Task &task = task_it->second;
        if (task.locations.empty())
            return std::numeric_limits<int>::max();

        int agent_location = env->curr_states[agent_id].location;
        int task_location = task.locations[0];

        return distance_table.getDistance(agent_location, task_location);
    }

    int evaluateCost(int agent_id, int task_id, SharedEnvironment *env)
    {
        // エージェントの現在の状態を取得
        const State current_state(env->curr_states[agent_id].location, env->curr_states[agent_id].orientation);
        int from_loc = current_state.location;
        int from_dir = current_state.direction;
        int to_loc = env->task_pool[task_id].locations[0];
        int cost = getCostToLocation(from_loc, from_dir, to_loc);

        // 到達不可能な場合は最大コストを返す
        if (cost == std::numeric_limits<int>::max())
        {
            return std::numeric_limits<int>::max();
        }

        // デバッグ: コストが負でないことを確認
        if (cost < 0)
        {
            std::cerr << "Error: Negative cost calculated in evaluateCost for agent " << agent_id << " and task " << task_id << ": " << cost << std::endl;
            // min_cost = 0; // 必要に応じて修正
        }

        return cost;
    }

    int getMinDirectionalCost(int from_loc, int from_dir, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        int cost0 = apsp_table.getCost(from_loc, from_dir, to_loc, 0);
        int cost1 = apsp_table.getCost(from_loc, from_dir, to_loc, 1);
        int cost2 = apsp_table.getCost(from_loc, from_dir, to_loc, 2);
        int cost3 = apsp_table.getCost(from_loc, from_dir, to_loc, 3);

        // 到達不可能な場合は最大コストを返す
        if (cost0 == std::numeric_limits<int>::max())
        {
            return std::numeric_limits<int>::max();
        }

        // 4方向の中で最小のコストを返す
        return std::min({cost0, cost1, cost2, cost3});
    }

    int getCostToLocation(int fromLoc, int fromDir, int toLoc)
    {
        return apsp_table.getCostToLocation(fromLoc, fromDir, toLoc);
    }

    // 最小コストとなる次の方向を計算する関数
    int getNextDirection(int from_loc, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        int cost0 = getMinDirectionalCost(from_loc, 0, to_loc);
        int cost1 = getMinDirectionalCost(from_loc, 1, to_loc);
        int cost2 = getMinDirectionalCost(from_loc, 2, to_loc);
        int cost3 = getMinDirectionalCost(from_loc, 3, to_loc);

        // 到達不可能な場合は-1を返す
        if (cost0 == std::numeric_limits<int>::max())
        {
            throw std::runtime_error("Error: No path found from " + std::to_string(from_loc) + " to " + std::to_string(to_loc));
        }

        // 最小コストとなる方向を見つける
        int min_cost = cost0;
        int best_dir = 0;

        if (cost1 < min_cost)
        {
            min_cost = cost1;
            best_dir = 1;
        }
        if (cost2 < min_cost)
        {
            min_cost = cost2;
            best_dir = 2;
        }
        if (cost3 < min_cost)
        {
            min_cost = cost3;
            best_dir = 3;
        }

        return best_dir;
    }

    std::tuple<double, int> evaluateScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env)
    {
        int total_cost = 0;
        int count_agent = 0;
        for (size_t i = 0; i < schedule.size(); ++i)
        {
            if (schedule[i] != -1)
            {
                total_cost += evaluateDistance(i, schedule[i], env);
                count_agent++;
            }
        }
        return std::make_tuple(static_cast<double>(total_cost), count_agent);
    }

    // validateScheduleの実装をTPTSScheduler.cppから移動
    bool validateSchedule(const std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        for (size_t agent_id = 0; agent_id < env->curr_task_schedule.size(); agent_id++)
        {
            int current_task = env->curr_task_schedule[agent_id];
            int proposed_task = proposed_schedule[agent_id];
            if (current_task != -1 && proposed_task == -1)
            {
                return false;
            }
            if (current_task != -1)
            {
                auto current_task_it = env->task_pool.find(current_task);
                if (current_task_it == env->task_pool.end())
                {
                    return false;
                }
                else if (current_task_it->second.idx_next_loc > 0 && current_task != proposed_task)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // タスクの残りの移動コストを計算する関数を修正
    int calculateRemainingTaskCost(const Task &task, SharedEnvironment *env)
    {
        int total_cost = 0;
        int current_loc = env->curr_states[task.agent_assigned].location;    // 現在の位置
        int current_dir = env->curr_states[task.agent_assigned].orientation; // 現在の方向（初期値は0）

        // idx_next_locから最後の位置までの移動コストを合計
        for (size_t i = task.idx_next_loc; i < task.locations.size(); ++i)
        {
            int to_loc = task.locations[i];

            // 4方向それぞれについて次の位置までのコストを計算
            int cost0 = apsp_table.getCost(current_loc, current_dir, to_loc, 0);
            int cost1 = apsp_table.getCost(current_loc, current_dir, to_loc, 1);
            int cost2 = apsp_table.getCost(current_loc, current_dir, to_loc, 2);
            int cost3 = apsp_table.getCost(current_loc, current_dir, to_loc, 3);

            // 最小コストとその方向を見つける
            int min_cost = cost0;
            current_dir = 0;

            if (cost1 < min_cost)
            {
                min_cost = cost1;
                current_dir = 1;
            }
            if (cost2 < min_cost)
            {
                min_cost = cost2;
                current_dir = 2;
            }
            if (cost3 < min_cost)
            {
                min_cost = cost3;
                current_dir = 3;
            }

            total_cost += min_cost;
            current_loc = to_loc; // 次の計算のために現在位置を更新
        }
        agent_end_state[task.agent_assigned] = std::make_pair(current_loc, current_dir);
        return total_cost;
    }

    // 現在の位置と方向から目標位置までの最小コストとその時の方向を返す関数
    std::pair<int, int> getMinCostAndDirection(int current_loc, int current_dir, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        int cost0 = apsp_table.getCost(current_loc, current_dir, to_loc, 0);
        int cost1 = apsp_table.getCost(current_loc, current_dir, to_loc, 1);
        int cost2 = apsp_table.getCost(current_loc, current_dir, to_loc, 2);
        int cost3 = apsp_table.getCost(current_loc, current_dir, to_loc, 3);

        // 最小コストとその方向を見つける
        int min_cost = cost0;
        int best_dir = 0;

        if (cost1 < min_cost)
        {
            min_cost = cost1;
            best_dir = 1;
        }
        if (cost2 < min_cost)
        {
            min_cost = cost2;
            best_dir = 2;
        }
        if (cost3 < min_cost)
        {
            min_cost = cost3;
            best_dir = 3;
        }

        return std::make_pair(min_cost, best_dir);
    }

    // パスを構築する関数を追加
    std::vector<State> constructPath(int from_loc, int from_dir, int to_loc)
    {
        std::vector<State> path; // pair<location, direction>
        path.push_back(State(from_loc, from_dir));
        int current_loc = from_loc;
        int current_dir = from_dir;
        // int current_idx = apsp_table.getCellToIndex(from_loc) * 4 + from_dir;
        // std::cout << "current_idx: " << current_idx << std::endl;
        while (current_loc != to_loc)
        {
            // 現在の位置のneighborを取得
            const auto &current_neighbors = apsp_table.getNeighbors(current_loc, current_dir);

            // 各neighborについて、そこを経由した場合の最小コストを計算
            int min_cost = std::numeric_limits<int>::max();
            State best_state;
            for (const auto &neighbor : current_neighbors)
            {
                // 各neighborについて4方向の最小コストを計算
                // auto [cost, dir] = getMinCostAndDirection(neighbor.location, neighbor.direction, to_loc);
                int cost = getCostToLocation(neighbor.location, neighbor.direction, to_loc);

                // 現在位置から次の位置への移動コストを加算（1ステップ）
                // cost += 1;

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_state = neighbor;
                }
                else if (cost == min_cost && best_state.location != neighbor.location)
                {
                    // DefaultPlanner::trajLNS.flow[neighbor.location] 各dのsum
                    int flow_neighbor = DefaultPlanner::trajLNS.flow[neighbor.location].d[neighbor.getOppositeDirection()];
                    int flow_best = DefaultPlanner::trajLNS.flow[best_state.location].d[best_state.getOppositeDirection()];
                    if (flow_neighbor < flow_best)
                    {
                        best_state = neighbor;
                    }
                }
            }

            if (best_state.location == -1)
            {
                throw std::runtime_error("No valid path found from " +
                                         std::to_string(from_loc) + " to " + std::to_string(to_loc));
            }

            // パスに追加
            current_loc = best_state.location;
            current_dir = best_state.direction;
            // current_idx = apsp_table.getCellToIndex(current_loc) * 4 + best_state.direction;
            path.push_back(best_state);
        }
        return path;
    }

    bool detectVertexConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts)
    {
        const int max_time = std::max(path1.size(), path2.size());
        bool found_conflict = false;

        for (int t = 0; t < max_time; ++t)
        {
            if (t >= path1.size() || t >= path2.size())
            {
                break;
            }
            // パスの長さを超えた場合は最後の位置にとどまっていると仮定
            int pos1 = t < path1.size() ? path1[t].location : path1.back().location;
            int pos2 = t < path2.size() ? path2[t].location : path2.back().location;

            // 同じ場所にいる場合は衝突
            if (pos1 == pos2)
            {
                Conflict conflict(agent1, agent2, t, pos1);
                conflicts.push_back(conflict);

                found_conflict = true;
            }
        }

        return found_conflict;
    }

    bool detectEdgeConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts)
    {
        bool found_conflict = false;
        const int max_time = std::max(path1.size(), path2.size()) - 1;

        for (int t = 0; t < max_time; ++t)
        {
            if (t >= path1.size() || t >= path2.size())
            {
                break;
            }
            // t時点とt+1時点の位置を取得
            int pos1_t = t < path1.size() ? path1[t].location : path1.back().location;
            int pos1_t1 = (t + 1) < path1.size() ? path1[t + 1].location : path1.back().location;
            int pos2_t = t < path2.size() ? path2[t].location : path2.back().location;
            int pos2_t1 = (t + 1) < path2.size() ? path2[t + 1].location : path2.back().location;

            // エージェントが位置を交換する場合はエッジ衝突
            if (pos1_t == pos2_t1 && pos1_t1 == pos2_t)
            {
                Conflict conflict(agent1, agent2, t, pos1_t, pos1_t1);
                conflicts.push_back(conflict);
                found_conflict = true;
            }
        }

        return found_conflict;
    }

    int countAllAgentConflicts(const std::vector<std::vector<State>> &paths)
    {
        // ConflictInfo result;
        std::vector<Conflict> conflicts;
        const int num_agents = paths.size();

        // すべてのエージェントのペアについて衝突をチェック
        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = i + 1; j < num_agents; ++j)
            {

                // 頂点衝突の検出
                if (detectVertexConflict(paths[i], paths[j], i, j, conflicts))
                {
                }

                // エッジ衝突の検出
                if (detectEdgeConflict(paths[i], paths[j], i, j, conflicts))
                {
                }
            }
        }

        return conflicts.size();
    }
    int countAgentPathConflicts(int location, int time_step,
                                const std::vector<std::vector<State>> &other_paths,
                                int agent_id)
    {
        int conflicts = 0;

        // 他の全てのパスとの衝突をチェック
        for (size_t i = 0; i < other_paths.size(); ++i)
        {
            if (i == static_cast<size_t>(agent_id))
                continue;

            const auto &other_path = other_paths[i];
            if (time_step < other_path.size())
            {
                // 頂点衝突
                if (location == other_path[time_step].location)
                {
                    conflicts++;
                }

                // エッジ衝突
                if (time_step > 0 && time_step < other_path.size())
                {
                    if (location == other_path[time_step - 1].location &&
                        other_path[time_step].location == location)
                    {
                        conflicts++;
                    }
                }
            }
        }

        return conflicts;
    }

    // updateRunningTasksの修正
    void updateRunningTasks(SharedEnvironment *env)
    {
        // curr schedule
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            int task_id = env->curr_task_schedule[agent_id];
            if (task_id != -1)
            {
                current_task_assignments[task_id] = agent_id;
                if (env->task_pool[task_id].idx_next_loc == 0)
                {
                    assigned_but_not_open_agents.insert(agent_id);
                }
            }
        }
        std::vector<int> tasks_to_remove;

        for (int task_id : global_available_tasks)
        {
            auto task_it = env->task_pool.find(task_id);
            const Task &task = task_it->second;

            if (task.idx_next_loc > 0)
            {
                tasks_to_remove.push_back(task_id);
                global_available_agents.erase(task.agent_assigned);
                assigned_but_not_open_agents.erase(task.agent_assigned);
                // タスクの残り時間を計算して設定
                // int remaining_cost = static_cast<int>(pow(calculateRemainingTaskCost(task, env), 1.1));
                // int remaining_cost = calculateRemainingTaskCost(task, env);
                // agent_available_time[task.agent_assigned] = remaining_cost;
            }
        }

        for (int task_id : tasks_to_remove)
        {
            global_available_tasks.erase(task_id);
        }
    }

    void updateOnGoingTasks(SharedEnvironment *env)
    {
        for (size_t agent_id = 0; agent_id < env->curr_task_schedule.size(); agent_id++)
        {
            int task_id = env->curr_task_schedule[agent_id];
            if (task_id != -1 && env->task_pool[task_id].idx_next_loc > 0)
            {
                const Task &task = env->task_pool[task_id];
                int remaining_cost = calculateRemainingTaskCost(task, env);
                agent_available_time[agent_id] = remaining_cost;
            }
        }
    }

    // タスク割り当ての更新
    void updateTaskAssignments(SharedEnvironment *env)
    {
        for (auto it = current_task_assignments.begin(); it != current_task_assignments.end();)
        {
            if (env->task_pool.find(it->first) == env->task_pool.end())
            {
                // agent_available_time[current_task_assignments[it->first]] = 0;
                task_total_cost_table.erase(it->first);
                it = current_task_assignments.erase(it);
                task_free++;
            }
            else
            {
                ++it;
            }
        }

        // 範囲挿入を使用
        global_available_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    }

    // 新しい関数：エージェントの更新
    void updateAgents(SharedEnvironment *env)
    {
        // for (int agent_id : global_free_agents)
        // {
        //     auto task_it = env->task_pool.find(env->curr_task_schedule[agent_id]);
        //     if (task_it != env->task_pool.end() && task_it->second.idx_next_loc > 0)
        //     {
        //         std::cout << "エラー: エージェント " << agent_id << " はタスクを実行していないのに、フリーエージェントとして登録されようとしています" << std::endl;
        //         throw std::runtime_error("エラー: エージェント " + std::to_string(agent_id) + " はタスクを実行していないのに、フリーエージェントとして登録されようとしています");
        //     }
        // }
        for (int agent_id : env->new_freeagents)
        {
            // if (env->map_name == "brc202d.map" && agent_id >= 2500)
            // if (env->rows == 481 && env->cols == 530 && agent_id >= 2500)
            // {
            //     break;
            // }
            // else if (env->map_name == "sortation_large.map" && agent_id > 15000)
            // {
            //     break;
            // }
            // else if (env->map_name == "warehouse_large.map" && agent_id > 7500)
            // {
            //     break;
            // }
            // else if (env->map_name == "random-32-32-20.map" && agent_id > 600)
            // {
            //     // break;
            // }
            global_available_agents.insert(agent_id);
        }
    }

    // タスクの総経路コストを計算して更新する関数
    void updateTaskTotalCosts(SharedEnvironment *env)
    {
        // 新しいタスクに対して総経路コストを計算
        for (int task_id : env->new_tasks)
        {
            const Task &task = env->task_pool.at(task_id);
            int total_cost = 0;

            // タスクの各位置を順番に移動する際のコストを計算
            for (size_t i = 0; i < task.locations.size() - 1; ++i)
            {
                int from_loc = task.locations[i];
                int from_dir = 0; // 初期方向は0と仮定
                int to_loc = task.locations[i + 1];

                // 2点間の最小コストを計算
                int path_cost = getCostToLocation(from_loc, from_dir, to_loc);

                // 経路が見つからない場合は最大値を設定
                if (path_cost == std::numeric_limits<int>::max())
                {
                    throw std::runtime_error("path_cost is std::numeric_limits<int>::max()");
                }

                total_cost += path_cost;
            }

            // 計算した総コストをテーブルに保存
            task_total_cost_table[task_id] = total_cost;
        }
    }

    int getDistance(int from_location, int to_location)
    {
        return distance_table.getDistance(from_location, to_location);
    }

} // namespace SchedulerUtils