#include "SchedulerUtils.h"
#include <random>
#include <algorithm>
#include "TrajLNS.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include "LNSScheduler.h"
#include "ParallelLNSScheduler.h"
#include "LNSSchedulerWeighted.h"
namespace SchedulerUtils
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
    std::unordered_set<unsigned short> deadends;
    std::unordered_set<int> tabu_locs;
    DefaultPlanner::TrajLNS trajLNS_copy;
    std::unordered_map<int, DefaultPlanner::TrajLNS> trajLNS_copies;
    std::unordered_set<unsigned short> must_assign_agents;

    // エージェントごとの最早開始可能時刻を保持
    std::vector<int> agent_available_time;
    // エージェントごとの開始可能位置を保持
    std::vector<std::pair<int, int>> agent_end_state;

    // タスクの総経路コストテーブル
    std::unordered_map<int, WeightedCostType> task_total_cost_table;

    // タスクは割り当てられているがまだopenしていないエージェントの集合
    std::unordered_set<unsigned short> assigned_but_not_open_agents;
    std::unordered_set<unsigned short> unassigned_agents;
    std::unordered_set<int> unassigned_tasks;
    int INITIAL_TIMESTEP_THRESHOLD = 60;

    // スケジューラのパラメータのデフォルト値を設定
    int TIME_LIMIT_DIVISION_FACTOR = 2; // デフォルトで2で割る
    int SCHEDULE_UPDATE_INTERVAL = 1;   // デフォルトで10ステップごとに更新
    bool include_tabu_cost = false;
    int tabu_cost_multiplier = 4;

    // ラッパーを用いて動的に切り替え可能な対象とする
    // template <typename T>
    // struct DynamicSetWrapper
    // {
    //     std::unordered_set<T> *ptr;
    //     // 範囲 for ループ対応の begin()/end() を提供
    //     auto begin() const { return ptr->begin(); }
    //     auto end() const { return ptr->end(); }
    //     // 内部コンテナのサイズを返す関数を追加
    //     auto size() const { return ptr->size(); }
    //     // 普通の参照としても利用できるように変換演算子を定義（必要に応じて）
    //     operator std::unordered_set<T> &() const { return *ptr; }
    // };

    DynamicSetWrapper<unsigned short> scheduling_target_agents = {&global_available_agents};
    DynamicSetWrapper<int> scheduling_target_tasks = {&global_available_tasks};

    void setSchedulingTargetAgents(std::unordered_set<unsigned short> &target_set)
    {
        scheduling_target_agents.ptr = &target_set;
    }

    void setSchedulingTargetTasks(std::unordered_set<int> &target_set)
    {
        scheduling_target_tasks.ptr = &target_set;
    }

    void schedule_initialize_SchedulerUtils(int preprocess_time_limit, SharedEnvironment *env)
    {

        // タブーロケーションの読み込み
        std::string tabu_filepath = env->file_storage_path + "/tabu_locs_" +
                                    env->map_name.substr(0, env->map_name.find(".map")) + ".txt";
        std::ifstream tabu_file(tabu_filepath);
        if (tabu_file.good())
        {
            std::string content;
            std::getline(tabu_file, content);
            // 最初と最後の[]を削除
            content = content.substr(1, content.length() - 2);

            std::stringstream ss(content);
            std::string loc;
            while (std::getline(ss, loc, ','))
            {
                tabu_locs.insert(std::stoi(loc));
            }
            std::cout << "Loaded " << tabu_locs.size() << " tabu locations from " << tabu_filepath << std::endl;
        }
        if (env->map_name == "Paris_1_256.map")
        {
            include_tabu_cost = true;
        }
        else if (env->map_name == "warehouse_large.map" || env->map_name == "sortation_large.map")
        {
            include_tabu_cost = true;
            if (env->map_name == "sortation_large.map")
            {
                tabu_cost_multiplier = 4;
            }
            tabu_cost_multiplier = 4;
        }
        apsp_table = ParallelAPSPCalculator::calculateParallelAPSP(env, std::thread::hardware_concurrency(), include_tabu_cost);

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
        unassigned_agents.clear();
        assigned_but_not_open_agents.reserve(env->num_of_agents);
        unassigned_agents.reserve(env->num_of_agents);
        if (env->map_name == "random-32-32-20.map")
        {
            // std::cout << "deadends.size(): " << deadends.size() << std::endl;
            // for (auto loc : deadends)
            // {
            //     std::cout << loc << " ";
            // }
            // std::cout << std::endl;
            SCHEDULE_UPDATE_INTERVAL = 1;
            INITIAL_TIMESTEP_THRESHOLD = 20;
            if (env->num_of_agents >= 600)
            {
                deadends = findAllDeadEnds(env);
                SCHEDULE_UPDATE_INTERVAL = 1;
            }
        }
        else if (env->map_name == "Paris_1_256.map")
        {
            SCHEDULE_UPDATE_INTERVAL = 1;
            if (env->num_of_agents <= 2000)
            {
                tabu_cost_multiplier = 1;
            }
        }
        else if (env->map_name == "warehouse_large.map" || env->map_name == "sortation_large.map")
        {
            SCHEDULE_UPDATE_INTERVAL = 1;
            tabu_cost_multiplier = 3;
        }
        else
        {
            SCHEDULE_UPDATE_INTERVAL = 1;
        }

        // デフォルトの参照先を設定
        setSchedulingTargetAgents(global_available_agents);
        setSchedulingTargetTasks(global_available_tasks);
    }

    int getVisitedTabuStates(int from_loc, int from_dir, int to_loc, int to_dir)
    {
        return apsp_table.getWeightedCost(from_loc, from_dir, to_loc, to_dir);
    }

    CostType evaluateDistance(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
            return 0;

        auto task_it = env->task_pool.find(task_id);
        if (task_it == env->task_pool.end())
            return std::numeric_limits<CostType>::max();

        const Task &task = task_it->second;
        if (task.locations.empty())
            return std::numeric_limits<CostType>::max();

        int agent_location = env->curr_states[agent_id].location;
        int task_location = task.locations[0];

        return distance_table.getDistance(agent_location, task_location);
    }

    CostType evaluateCost(int agent_id, int task_id, SharedEnvironment *env)
    {
        // エージェントの現在の状態を取得
        const State current_state(env->curr_states[agent_id].location, env->curr_states[agent_id].orientation);
        int from_loc = current_state.location;
        int from_dir = current_state.direction;
        int to_loc = env->task_pool[task_id].locations[0];
        int cost = getCostToLocation(from_loc, from_dir, to_loc);

        // 到達不可能な場合は最大コストを返す
        if (cost == std::numeric_limits<CostType>::max())
        {
            return std::numeric_limits<CostType>::max();
        }

        // デバッグ: コストが負でないことを確認
        if (cost < 0)
        {
            std::cerr << "Error: Negative cost calculated in evaluateCost for agent " << agent_id << " and task " << task_id << ": " << cost << std::endl;
            // min_cost = 0; // 必要に応じて修正
        }

        return cost;
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
    CostType calculateRemainingTaskCost(const Task &task, SharedEnvironment *env)
    {
        CostType total_cost = 0;
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
    std::pair<CostType, int> getMinCostAndDirection(int current_loc, int current_dir, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        CostType cost0 = apsp_table.getCost(current_loc, current_dir, to_loc, 0);
        CostType cost1 = apsp_table.getCost(current_loc, current_dir, to_loc, 1);
        CostType cost2 = apsp_table.getCost(current_loc, current_dir, to_loc, 2);
        CostType cost3 = apsp_table.getCost(current_loc, current_dir, to_loc, 3);

        // 最小コストとその方向を見つける
        CostType min_cost = cost0;
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
    std::vector<State> constructPath(int from_loc, int from_dir, int to_loc, int curr_timestep)
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
            WeightedCostType min_cost = std::numeric_limits<WeightedCostType>::max();
            State best_state;
            for (const auto &neighbor : current_neighbors.neighbors)
            {
                // 各neighborについて4方向の最小コストを計算
                // auto [cost, dir] = getMinCostAndDirection(neighbor.location, neighbor.direction, to_loc);
                WeightedCostType cost = getWeightedCostFromStateToLocation(neighbor.location, neighbor.direction, to_loc);
                // 現在位置から次の位置への移動コストを加算（1ステップ）
                // cost += 1;

                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_state = neighbor;
                }
                // else if (cost == min_cost && best_state.location != neighbor.location)
                // {
                //     if (curr_timestep == 0)
                //     {
                //         continue;
                //     }
                //     // DefaultPlanner::trajLNS.flow[neighbor.location] 各dのsum
                //     int flow_neighbor = trajLNS.flow[neighbor.location].d[neighbor.getOppositeDirection()];
                //     int flow_best = DefaultPlanner::trajLNS.flow[best_state.location].d[best_state.getOppositeDirection()];
                //     if (flow_neighbor < flow_best)
                //     {
                //         best_state = neighbor;
                //     }
                // }
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
        // std::cout << "path.size(): " << path.size() << std::endl;
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
    void createTrajLNSCopy(int agent_id)
    {
        // std::shared_lock<std::shared_mutex> lock(shared_data.mutex);
        if (trajLNS_copies.find(agent_id) == trajLNS_copies.end())
        {
            DefaultPlanner::TrajLNS trajLNS_agent_i = trajLNS_copy;
            // remove_traj(agent_id, trajLNS_agent_i);
            trajLNS_copies.emplace(agent_id, std::move(trajLNS_agent_i));
            // DefaultPlanner::TrajLNS trajLNS_agent_i = trajLNS_copy;
            // for (int available_agent_id : global_available_agents)
            // {
            //     remove_traj(available_agent_id, trajLNS_agent_i);
            // }
            // for (int available_agent_id : global_available_agents)
            // {
            //     trajLNS_copies.emplace(available_agent_id, std::move(trajLNS_agent_i));
            // }
        }
    }
    int calculateOppositeFlowCost(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
        {
            return 0;
        }
        // if (trajLNS_copies.find(agent_id) == trajLNS_copies.end())
        // {
        //     DefaultPlanner::TrajLNS trajLNS_agent_i = trajLNS_copy;
        //     remove_traj(agent_id, trajLNS_agent_i);
        //     trajLNS_copies.emplace(agent_id, std::move(trajLNS_agent_i));
        // }
        // パスを構築
        std::vector<SchedulerUtils::State> path = SchedulerUtils::constructPath(
            env->curr_states[agent_id].location,
            env->curr_states[agent_id].orientation,
            env->task_pool[task_id].locations[0], env->curr_timestep);

        // if (modify_trajectory)
        // {
        //     remove_traj(agent_id);
        // }
        // パスに沿って対向流コストを計算
        int total_cost = 0;
        for (size_t i = 0; i < path.size() - 1; i++)
        {
            const SchedulerUtils::State &curr_state = path[i];
            const SchedulerUtils::State &next_state = path[i + 1];

            // 現在の移動方向
            int curr_dir = curr_state.direction;
            // 対向方向
            int opposite_dir = (curr_dir + 2) % 4;

            // 現在のセルと次のセルでの対向流コストを計算
            // int curr_flow = trajLNS_copies[agent_id].flow[curr_state.location].d[curr_dir] + 1;
            // int next_flow = trajLNS_copies[agent_id].flow[next_state.location].d[opposite_dir];
            int curr_flow = trajLNS_copy.flow[curr_state.location].d[curr_dir] + 1;
            int next_flow = trajLNS_copy.flow[next_state.location].d[opposite_dir];
            int op_flow = curr_flow * next_flow;

            int temp_vertex = 1;
            for (int j = 0; j < 4; j++)
            {
                temp_vertex += trajLNS_copy.flow[next_state.location].d[j];
            }
            int all_vertex_flow = 0;

            all_vertex_flow += (temp_vertex - 1) / 2;

            // 対向流コストを乗算して加算
            total_cost += op_flow + all_vertex_flow;
        }
        // if (modify_trajectory)
        // {
        //     add_traj(agent_id);
        // }
        return total_cost;
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
                unassigned_tasks.erase(task_id);
            }
            else
            {
                unassigned_tasks.insert(task_id);
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
                unassigned_agents.erase(task.agent_assigned);
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
            auto it_pool = env->task_pool.find(it->first);
            if (it_pool == env->task_pool.end())
            {
                // agent_available_time[current_task_assignments[it->first]] = 0;
                task_total_cost_table.erase(it->first);
                it = current_task_assignments.erase(it);
                task_free++;
            }
            else if (it_pool->second.agent_assigned == -1)
            {
                unassigned_tasks.erase(it->first);
                it = current_task_assignments.erase(it);
            }
            else
            {
                ++it;
            }
        }

        global_available_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
        unassigned_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

        // 範囲挿入を使用
        // for (int task_id : env->new_tasks)
        // {
        //     if (tabu_locs.find(env->task_pool[task_id].locations[0]) == tabu_locs.end())
        //     {
        //         global_available_tasks.insert(task_id);
        //         unassigned_tasks.insert(task_id);
        //     }
        // }
        // int max_errand = 0;
        // for (int task_id : global_available_tasks)
        // {
        //     if (env->task_pool[task_id].locations.size() > max_errand)
        //     {
        //         max_errand = env->task_pool[task_id].locations.size();
        //     }
        // }
        // upper1 = std::max(upper1, max_errand) - 1;
        // std::cout << "max_errand: " << max_errand << std::endl;
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
            unassigned_agents.insert(agent_id);
        }
        if (LNSScheduler::disable_agents)
        {
            must_assign_agents.clear();
            for (int agent_id : global_available_agents)
            {
                if (tabu_locs.find(env->curr_states[agent_id].location) != tabu_locs.end())
                {
                    must_assign_agents.insert(agent_id);
                }
            }
        }
    }

    // タスクの総経路コストを計算して更新する関数
    void updateTaskTotalCosts(SharedEnvironment *env)
    {
        // 新しいタスクに対して総経路コストを計算
        for (int task_id : env->new_tasks)
        {
            const Task &task = env->task_pool.at(task_id);
            WeightedCostType total_cost = 0;

            if (LNSSchedulerWeighted::include_waypoints)
            {
                // タスクの各位置を順番に移動する際のコストを計算
                for (size_t i = 0; i < task.locations.size() - 1; ++i)
                {
                    int from_loc = task.locations[i];
                    int from_dir = 0; // 初期方向は0と仮定
                    int to_loc = task.locations[i + 1];

                    // 2点間の最小コストを計算
                    WeightedCostType path_cost = getWeightedCostFromLocationToLocation(from_loc, to_loc);

                    // 経路が見つからない場合は最大値を設定
                    if (path_cost == std::numeric_limits<WeightedCostType>::max())
                    {
                        throw std::runtime_error("path_cost is std::numeric_limits<WeightedCostType>::max()");
                    }
                    total_cost += path_cost;
                    // if (include_tabu_cost)
                    // {
                    //     // total_cost += path_cost;
                    //     total_cost += path_cost + getVisitedTabuStates(from_loc, from_dir, to_loc, 0) * tabu_cost_multiplier;
                    // }
                    // else
                    // {
                    //     total_cost += path_cost;
                    // }
                }

                // 計算した総コストをテーブルに保存
                task_total_cost_table[task_id] = total_cost;
            }
        }
    }
    WeightedCostType getWeightedCostFromLocationToLocation(int from_loc, int to_loc)
    {
        return apsp_table.getWeightedCostFromLocationToLocation(from_loc, to_loc);
    }

    CostType getDistance(int from_location, int to_location)
    {
        return distance_table.getDistance(from_location, to_location);
    }
    WeightedCostType getWeightedCost(int from_location, int to_location)
    {
        return apsp_table.getWeightedCost(from_location, 0, to_location, 0);
    }

    // remove flow for each location's outgoing edge according to the traj
    void remove_traj(int agent, DefaultPlanner::TrajLNS &trajLNS)
    {
        // lns.soc -= lns.trajs[agent].size() - 1;
        if (trajLNS.trajs[agent].size() <= 1)
        {
            return;
        }
        int loc, prev_loc, diff, d, to;

        to = trajLNS.trajs[agent].size();

        for (int j = 1; j < to; j++)
        {
            loc = trajLNS.trajs[agent][j];
            prev_loc = trajLNS.trajs[agent][j - 1];
            diff = loc - prev_loc;
            d = DefaultPlanner::get_d(diff, trajLNS.env);

            trajLNS.flow[prev_loc].d[d] -= 1;
        }
    }

    void add_traj(int agent)
    {

        // lns.soc += lns.trajs[agent].size() - 1;
        if (trajLNS_copy.trajs[agent].size() <= 1)
        {
            return;
        }
        int loc, prev_loc, diff, d;
        for (int j = 1; j < trajLNS_copy.trajs[agent].size(); j++)
        {
            loc = trajLNS_copy.trajs[agent][j];
            prev_loc = trajLNS_copy.trajs[agent][j - 1];
            diff = loc - prev_loc;
            d = DefaultPlanner::get_d(diff, trajLNS_copy.env);

            trajLNS_copy.flow[prev_loc].d[d] += 1;
        }
    }
    void updateTrajLNS()
    {
        // TrajLNSのメンバーをコピー

        // flowのコピー
        trajLNS_copy.flow = DefaultPlanner::trajLNS.flow;

        // trajsのコピー
        trajLNS_copy.trajs = DefaultPlanner::trajLNS.trajs;
        for (int agent_id : global_available_agents)
        {
            remove_traj(agent_id, trajLNS_copy);
        }
        // for (int agent_id : global_available_agents)
        // {
        //     add_traj(agent_id);
    }
    // 指定された位置がdeadendかどうかを確認する関数
    bool isDeadEnd(int loc, SharedEnvironment *env)
    {
        int row = loc / env->cols;
        int col = loc % env->cols;
        int passable_count = 0;

        // 上下左右の4方向をチェック
        const int dx[] = {0, 0, -1, 1};
        const int dy[] = {-1, 1, 0, 0};

        for (int i = 0; i < 4; i++)
        {
            int new_row = row + dy[i];
            int new_col = col + dx[i];

            // マップの範囲内かチェック
            if (new_row >= 0 && new_row < env->rows &&
                new_col >= 0 && new_col < env->cols)
            {
                // 通行可能なセルをカウント
                if (env->map[new_row * env->cols + new_col] == 0)
                {
                    passable_count++;
                }
            }
        }

        // 通行可能なセルが1つ以下ならdeadend
        return passable_count <= 1;
    }

    // マップ全体のdeadendを検出する関数
    std::unordered_set<unsigned short> findAllDeadEnds(SharedEnvironment *env)
    {
        std::unordered_set<unsigned short> deadends;

        // マップの全セルをチェック
        for (int row = 0; row < env->rows; row++)
        {
            for (int col = 0; col < env->cols; col++)
            {
                int loc = row * env->cols + col;

                // 通行可能なセルのみチェック
                if (env->map[loc] == 0 && isDeadEnd(loc, env))
                {
                    deadends.insert(loc);
                }
            }
        }

        return deadends;
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
        return isDeadEndLocation(task.locations.back());
        // タスクのerrandsに含まれるdeadendの数を返す関数
        return countDeadEndsInErrand(task_id, env) > 1;

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

    CostType getMinDirectionalCost(int from_loc, int from_dir, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        CostType cost0 = apsp_table.getCost(from_loc, from_dir, to_loc, 0);
        CostType cost1 = apsp_table.getCost(from_loc, from_dir, to_loc, 1);
        CostType cost2 = apsp_table.getCost(from_loc, from_dir, to_loc, 2);
        CostType cost3 = apsp_table.getCost(from_loc, from_dir, to_loc, 3);

        // 到達不可能な場合は最大コストを返す
        if (cost0 == std::numeric_limits<CostType>::max())
        {
            return std::numeric_limits<CostType>::max();
        }

        // 4方向の中で最小のコストを返す
        return std::min({cost0, cost1, cost2, cost3});
    }

    WeightedCostType getWeightedCostFromStateToLocation(int fromLoc, int fromDir, int toLoc)
    {
        return apsp_table.getWeightedCostFromStateToLocation(fromLoc, fromDir, toLoc);
    }
    CostType getCostToLocation(int fromLoc, int fromDir, int toLoc)
    {
        return apsp_table.getCostToLocation(fromLoc, fromDir, toLoc);
    }

    // 最小コストとなる次の方向を計算する関数
    CostType getNextDirection(int from_loc, int to_loc)
    {
        // 4方向それぞれについて次の位置までのコストを計算
        CostType cost0 = getMinDirectionalCost(from_loc, 0, to_loc);
        CostType cost1 = getMinDirectionalCost(from_loc, 1, to_loc);
        CostType cost2 = getMinDirectionalCost(from_loc, 2, to_loc);
        CostType cost3 = getMinDirectionalCost(from_loc, 3, to_loc);

        // 到達不可能な場合は-1を返す
        if (cost0 == std::numeric_limits<CostType>::max())
        {
            throw std::runtime_error("Error: No path found from " + std::to_string(from_loc) + " to " + std::to_string(to_loc));
        }

        // 最小コストとなる方向を見つける
        CostType min_cost = cost0;
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

    void saveSettingsToJson(const std::string &filename, SharedEnvironment *env)
    {
        nlohmann::json j;

        // メタデータ
        j["metadata"] = {
            {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()},
            {"map_name", env->map_name},
            {"num_agents", env->num_of_agents},
            {"rows", env->rows},
            {"cols", env->cols}};

        // LNSScheduler namespace の変数
        j["LNSScheduler"] = {
            {"assinged_task_additional_cost", LNSScheduler::assinged_task_additional_cost},
            {"same_task_additional_cost", LNSScheduler::same_task_additional_cost},
            {"MANHATTAN_DISTANCE_THRESHOLD", LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD},
            {"INCLUDE_FLOW_COST", LNSScheduler::INCLUDE_FLOW_COST},
            {"include_waypoints", LNSScheduler::include_waypoints},
            {"TASKS_PER_THREAD", LNSScheduler::TASKS_PER_THREAD},
            {"NUM_THREADS", LNSScheduler::NUM_THREADS},
            {"NEARBY_AGENTS_RATIO", LNSScheduler::NEARBY_AGENTS_RATIO},
            {"count_deadends", LNSScheduler::count_deadends},
            {"DISTANCE_THRESHOLD_DIVISOR", LNSScheduler::DISTANCE_THRESHOLD_DIVISOR},
            {"TASK_SWITCH_THRESHOLD", LNSScheduler::TASK_SWITCH_THRESHOLD},
            {"disable_agents", LNSScheduler::disable_agents},
            {"include_additional_cost_in_evaluation", LNSScheduler::include_additional_cost_in_evaluation}};

        // ParallelLNSScheduler namespace の変数
        j["ParallelLNSScheduler"] = {
            {"initial_destroy_ratio", ParallelLNSScheduler::initial_destroy_ratio},
            {"final_destroy_ratio", ParallelLNSScheduler::final_destroy_ratio},
            {"MAX_FAILED_ATTEMPTS", ParallelLNSScheduler::max_failed_attempts},
            {"optimization_type", static_cast<int>(ParallelLNSScheduler::optimization_type)},
            {"USE_EARLY_RETURN", ParallelLNSScheduler::USE_EARLY_RETURN}};

        // SchedulerUtils namespace の変数
        j["SchedulerUtils"] = {
            {"TIME_LIMIT_DIVISION_FACTOR", TIME_LIMIT_DIVISION_FACTOR},
            {"SCHEDULE_UPDATE_INTERVAL", SCHEDULE_UPDATE_INTERVAL},
            {"INITIAL_TIMESTEP_THRESHOLD", INITIAL_TIMESTEP_THRESHOLD},
            {"tabu_cost_multiplier", tabu_cost_multiplier}};

        // 環境変数
        // j["environment"] = {
        //     {"available_tasks", global_available_tasks.size()},
        //     {"available_agents", global_available_agents.size()},
        //     {"task_free", task_free},
        //     {"upper1", upper1}};

        // JSONファイルに書き出し
        std::ofstream o(filename);
        o << std::setw(4) << j << std::endl;
    }

    // void loadSettingsFromJson(const std::string &filename, SharedEnvironment *env)
    // {
    //     try
    //     {
    //         std::ifstream i(filename);
    //         nlohmann::json j;
    //         i >> j;

    //         // メタデータの検証
    //         auto &metadata = j["metadata"];
    //         if (metadata["map_name"] != env->map_name ||
    //             metadata["num_agents"] != env->num_of_agents ||
    //             metadata["rows"] != env->rows ||
    //             metadata["cols"] != env->cols)
    //         {
    //             throw std::runtime_error("設定ファイルの環境情報が現在の環境と一致しません");
    //         }

    //         // LNSScheduler namespace の変数を読み込み
    //         auto &lns = j["LNSScheduler"];
    //         LNSScheduler::assinged_task_additional_cost = lns["assinged_task_additional_cost"];
    //         LNSScheduler::same_task_additional_cost = lns["same_task_additional_cost"];
    //         LNSScheduler::MANHATTAN_DISTANCE_THRESHOLD = lns["MANHATTAN_DISTANCE_THRESHOLD"];
    //         LNSScheduler::INCLUDE_FLOW_COST = lns["INCLUDE_FLOW_COST"];
    //         LNSScheduler::include_waypoints = lns["include_waypoints"];
    //         LNSScheduler::TASKS_PER_THREAD = lns["TASKS_PER_THREAD"];
    //         LNSScheduler::NUM_THREADS = lns["NUM_THREADS"];
    //         LNSScheduler::NEARBY_AGENTS_RATIO = lns["NEARBY_AGENTS_RATIO"];

    //         // ParallelLNSScheduler namespace の変数を読み込み
    //         auto &parallel = j["ParallelLNSScheduler"];
    //         ParallelLNSScheduler::initial_destroy_ratio = parallel["initial_destroy_ratio"];
    //         ParallelLNSScheduler::final_destroy_ratio = parallel["final_destroy_ratio"];
    //         ParallelLNSScheduler::MAX_FAILED_ATTEMPTS = parallel["MAX_FAILED_ATTEMPTS"];
    //         ParallelLNSScheduler::optimization_type =
    //             static_cast<ParallelLNSScheduler::OptimizationType>(parallel["optimization_type"].get<int>());

    //         // SchedulerUtils namespace の変数を読み込み
    //         auto &utils = j["SchedulerUtils"];
    //         TIME_LIMIT_DIVISION_FACTOR = utils["TIME_LIMIT_DIVISION_FACTOR"];
    //         SCHEDULE_UPDATE_INTERVAL = utils["SCHEDULE_UPDATE_INTERVAL"];
    //     }
    //     catch (const std::exception &e)
    //     {
    //         std::cerr << "設定ファイルの読み込み中にエラーが発生しました: " << e.what() << std::endl;
    //         throw;
    //     }
    // }

    void removeUnfeasibleTasks(int remaining_time, SharedEnvironment *env)
    {
        std::vector<int> tasks_to_remove;

        // global_available_tasksの各タスクをチェック
        for (int task_id : global_available_tasks)
        {

            // タスクの総コストを取得
            CostType total_cost = task_total_cost_table[task_id] - 8;

            // タスクの実行に必要な時間が残り時間より長い場合
            if (total_cost > remaining_time)
            {
                tasks_to_remove.push_back(task_id);
            }
        }

        // 実行不可能なタスクを削除
        for (int task_id : tasks_to_remove)
        {
            global_available_tasks.erase(task_id);
        }

        if (!tasks_to_remove.empty())
        {
            std::cout << "Removed " << tasks_to_remove.size()
                      << " unfeasible tasks (remaining time: " << remaining_time << ")" << std::endl;
        }
    }

} // namespace SchedulerUtils