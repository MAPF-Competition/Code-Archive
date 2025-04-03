#include "ECBSTA.h"
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>
namespace ECBSTA_NS
{
    // ハンガリアン法によるタスク割り当てを行う関数
    Assignment constrainedAssignment(const std::unordered_set<AgentTaskPair> &I,
                                     const std::unordered_set<AgentTaskPair> &O,
                                     const std::vector<std::vector<int>> &C)
    {
        const int num_agents = C.size();
        const int num_tasks = C[0].size();

        // コスト行列のコピーを作成
        std::vector<std::vector<int>> cost_matrix = C;

        // 強制割り当て(I)の処理
        const int FORCED_COST = 0;
        for (const auto &forced : I)
        {
            cost_matrix[forced.agent][forced.task] = FORCED_COST;
        }

        // 割り当て禁止(O)の処理
        const int FORBIDDEN_COST = 100000;
        for (const auto &forbidden : O)
        {
            cost_matrix[forbidden.agent][forbidden.task] = FORBIDDEN_COST;
        }

        // ハンガリアン法による割り当ての実行
        return hungarianMethod(cost_matrix);
    }

    // 最初の割り当てを生成する関数
    std::pair<Assignment, std::vector<HighLevelNode>> firstAssignment(const std::vector<std::vector<int>> &C,
                                                                      const std::unordered_set<AgentTaskPair> &I,
                                                                      const std::unordered_set<AgentTaskPair> &O)
    {
        HighLevelNode R;
        R.I = I;
        R.O = O;
        R.root = true;

        // 最初の割り当てを取得
        R.assignment = constrainedAssignment(R.I, R.O, C);
        std::cout << "R.assignment: " << R.assignment.size() << std::endl;

        if (R.assignment.empty())
        {
            return {Assignment(), std::vector<HighLevelNode>()};
        }

        // ASG_OPENの初期化
        std::vector<HighLevelNode> ASG_OPEN;
        ASG_OPEN.push_back(R);

        return {R.assignment, ASG_OPEN};
    }

    // ハンガリアン法の実装（必要な補助関数）
    Assignment hungarianMethod(const std::vector<std::vector<int>> &cost_matrix)
    {
        // bool agent_surplus = cost_matrix.size() > cost_matrix[0].size();
        // SchedulerUtils::Hungarian::Matrix matrix(cost_matrix, agent_surplus);
        // return matrix.solve();
        Assignment assignment;
        return assignment;
    }

    // 割り当ての総コストを計算する補助関数
    int calculateAssignmentCost(const Assignment &assignment,
                                const std::vector<std::vector<int>> &C)
    {
        int total_cost = 0;
        for (const auto &[agent, task] : assignment)
        {
            total_cost += C[agent][task];
        }
        return total_cost;
    }
    int calculateAssignmentCostWithPrint(const Assignment &assignment,
                                         const std::vector<std::vector<int>> &C)
    {
        int total_cost = 0;
        for (const auto &[agent, task] : assignment)
        {
            total_cost += C[agent][task];
        }
        std::cout << "Sorted solutions by agent_id as firstAssignment:" << std::endl;
        for (size_t agent_id = 0; agent_id < assignment.size(); agent_id++)
        {
            int task = assignment.at(agent_id);
            std::cout << C[agent_id][task] << " ";
        }
        std::cout << std::endl;

        return total_cost;
    }

    Path findPath(const Graph &graph,
                  int start,
                  int goal,
                  const ConstraintSet &constraints,
                  int agent_id)
    {

        // ノードの状態からIDへのマップ
        std::unordered_map<LowLevelNodeState, size_t, LowLevelNodeStateHash> state_to_id;
        // IDからノードへのマップ
        std::unordered_map<size_t, LowLevelNode> path_nodes;
        // IDベースの優先度付きキュー
        std::priority_queue<size_t, std::vector<size_t>,
                            std::function<bool(size_t, size_t)>>
            open_list(
                [&](size_t a, size_t b)
                {
                    return path_nodes[a].f_score > path_nodes[b].f_score;
                });

        size_t next_low_level_node_id = 0;

        // 開始ノードの作成
        int h_score = calculateHeuristic(start, goal, graph);
        LowLevelNode start_node(next_low_level_node_id, start, 0, h_score, 0);
        path_nodes[next_low_level_node_id] = start_node;
        state_to_id[LowLevelNodeState{start, 0}] = next_low_level_node_id;
        open_list.push(next_low_level_node_id);
        next_low_level_node_id++;

        while (!open_list.empty())
        {
            size_t current_id = open_list.top();
            open_list.pop();
            LowLevelNode &current = path_nodes[current_id];

            // ノードが既に展開済みの場合はスキップ
            if (!current.is_open)
            {
                continue;
            }
            current.is_open = false;

            // ゴールに到達した場合
            if (current.location == goal)
            {
                return reconstructPath(current_id, path_nodes);
            }

            // 隣接ノードの探索
            for (int next_loc : graph.getNeighbors(current.location))
            {
                int next_time = current.time_step + 1;

                // 制約チェック
                if (isConstrained(agent_id, next_loc, next_time, current.location, constraints))
                {
                    continue;
                }

                int next_g = current.g_score + graph.getCost(current.location, next_loc, goal);
                int next_h = calculateHeuristic(next_loc, goal, graph);
                int next_f = next_g + next_h;

                // 状態が既に存在するかチェック
                LowLevelNodeState next_state{next_loc, next_time};
                auto state_it = state_to_id.find(next_state);

                if (state_it != state_to_id.end())
                {
                    // 既存のノードを取得
                    LowLevelNode &existing_node = path_nodes[state_it->second];
                    if (existing_node.f_score > next_f)
                    {
                        existing_node.f_score = next_f;
                        existing_node.g_score = next_g;
                        existing_node.parent_id = current_id;
                        existing_node.is_open = true;
                        open_list.push(state_it->second);
                    }

                    // より良いパスが見つかった場合は更新
                    // if (next_g < existing_node.g_score)
                    // {
                    //     std::cerr << "より良いパスが見つかった" << std::endl;
                    //     // existing_node.g_score = next_g;
                    //     // existing_node.f_score = next_f;
                    //     // existing_node.parent_id = current_id;
                    //     // existing_node.is_open = true;
                    // }
                    // open_list.push(state_it->second);
                }
                else
                {
                    // 新しいノードを作成
                    LowLevelNode next_node(next_low_level_node_id, next_loc, next_g, next_f,
                                           next_time, current_id);
                    path_nodes[next_low_level_node_id] = next_node;
                    state_to_id[next_state] = next_low_level_node_id;
                    open_list.push(next_low_level_node_id);
                    next_low_level_node_id++;
                }
            }
        }

        // パスが見つからない場合
        return Path();
    }

    // パス再構築関数も修正
    Path reconstructPath(size_t goal_id,
                         const std::unordered_map<size_t, LowLevelNode> &nodes, int f_min)
    {
        Path path;
        size_t current_id = goal_id;

        while (current_id != 0)
        {
            const LowLevelNode &current = nodes.at(current_id);
            path.nodes.insert(path.nodes.begin(), current.location);
            // path.cost += (current.parent_id != 0 ? 1 : 0);
            current_id = current.parent_id;
        }
        // 開始ノードを追加
        if (!nodes.empty())
        {
            path.nodes.insert(path.nodes.begin(), nodes.at(0).location);
        }
        path.cost = nodes.at(goal_id).g_score;
        path.f_min = f_min;
        return path;
    }

    bool isConstrained(int agent_id,
                       int location,
                       int time_step,
                       int from_location,
                       const ConstraintSet &constraints)
    {
        // bool flag = false;
        // if (agent_id == 70 && from_location == 632 && location == 664 && time_step - 1 == 2)
        // {
        //     std::cout << "エッジ制約に引っかかりました" << std::endl;
        //     std::cout << "agent_id: " << agent_id << std::endl;
        //     std::cout << "from: " << from_location << std::endl;
        //     std::cout << "to: " << location << std::endl;
        //     std::cout << "time: " << time_step << std::endl;
        //     Constraint edge_constraint_tmp(agent_id, location, time_step - 1, from_location);
        //     if (constraints.edge_constraints.find(edge_constraint_tmp) !=
        //         constraints.edge_constraints.end())
        //     {

        //         std::cout << "存在" << std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "存在しない" << std::endl;
        //     }
        //     std::cout << "edge_constraint_tmp: " << edge_constraint_tmp.agent << " " << edge_constraint_tmp.fromLocation << " " << edge_constraint_tmp.location << " " << edge_constraint_tmp.timeStep << std::endl;
        //     for (const auto &edge_constraint : constraints.edge_constraints)
        //     {
        //         std::cout << edge_constraint.agent << " " << edge_constraint.fromLocation << " " << edge_constraint.location << " " << edge_constraint.timeStep << std::endl;
        //     }
        //     flag = true;
        // }
        // 頂点制約のチェック
        Constraint vertex_constraint(agent_id, location, time_step);
        if (constraints.vertex_constraints.find(vertex_constraint) !=
            constraints.vertex_constraints.end())
        {
            return true;
        }
        // if (flag)
        // {
        //     std::cout << "vertex_constraintに引っかかりました" << std::endl;
        // }
        assert(time_step > 0);
        // エッジ制約のチェック（前の位置から現在の位置への移動）
        Constraint edge_constraint(agent_id, from_location, location, time_step - 1);
        if (constraints.edge_constraints.find(edge_constraint) !=
            constraints.edge_constraints.end())
        {
            // if (flag)
            // {
            //     std::cout << "true" << std::endl;
            // }

            return true;
        }
        // for (const auto &edge_constraint : constraints.edge_constraints)
        // {
        //     if (edge_constraint.agent == agent_id &&
        //         edge_constraint.timeStep == time_step &&
        //         edge_constraint.location == location &&
        //         edge_constraint.fromLocation == from_location)
        //     {
        //         return true;
        //     }
        // }

        return false;
    }

    int calculateHeuristic(int current, int goal, const Graph &graph)
    {
        // 簡単な実装例：ユークリッド距離やマンハッタン距離を使用
        // グラフの実装に依存するため、適切な距離計算方法を選択する必要があります
        return graph.getEstimatedDistance(current, goal);
    }

    std::vector<Path> findIndividualPaths(const Graph &graph,
                                          const std::vector<int> &starts,
                                          const std::vector<int> &goals,
                                          const ConstraintSet &constraints)
    {
        std::vector<Path> paths;
        paths.reserve(starts.size());

        for (size_t i = 0; i < starts.size(); ++i)
        {
            Path path = findPath(graph, starts[i], goals[i], constraints, i);
            paths.push_back(path);
        }

        return paths;
    }

    ConflictInfo detectConflicts(const std::vector<Path> &paths)
    {
        ConflictInfo result;
        const int num_agents = paths.size();

        // すべてのエージェントのペアについて衝突をチェック
        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = i + 1; j < num_agents; ++j)
            {
                Conflict earliest_vertex_conflict(-1, -1, std::numeric_limits<int>::max(), -1);
                Conflict earliest_edge_conflict(-1, -1, std::numeric_limits<int>::max(), -1);

                // 頂点衝突の検出
                if (detectVertexConflict(paths[i], paths[j], i, j, result.conflicts, earliest_vertex_conflict))
                {
                    result.hasConflict = true;
                    // 最も早い衝突を更新
                    if (earliest_vertex_conflict.timeStep < result.earliest_conflict.timeStep)
                    {
                        result.earliest_conflict = earliest_vertex_conflict;
                    }
                }

                // エッジ衝突の検出
                if (detectEdgeConflict(paths[i], paths[j], i, j, result.conflicts, earliest_edge_conflict))
                {
                    result.hasConflict = true;
                    // 最も早い衝突を更新
                    if (earliest_edge_conflict.timeStep < result.earliest_conflict.timeStep)
                    {
                        result.earliest_conflict = earliest_edge_conflict;
                    }
                }
            }
        }
        // std::cout << "result.earliest_conflict.timeStep: " << result.earliest_conflict.timeStep << std::endl;

        return result;
    }

    bool detectVertexConflict(const Path &path1, const Path &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts,
                              Conflict &earliest_conflict)
    {
        const int max_time = std::max(path1.nodes.size(), path2.nodes.size());
        bool found_conflict = false;

        for (int t = 0; t < max_time; ++t)
        {
            // パスの長さを超えた場合は最後の位置にとどまっていると仮定
            int pos1 = t < path1.nodes.size() ? path1.nodes[t] : path1.nodes.back();
            int pos2 = t < path2.nodes.size() ? path2.nodes[t] : path2.nodes.back();

            // 同じ場所にいる場合は衝突
            if (pos1 == pos2)
            {
                Conflict conflict(agent1, agent2, t, pos1);
                conflicts.push_back(conflict);

                // 最も早い衝突を更新
                if (!found_conflict)
                {
                    earliest_conflict = conflict;
                }
                found_conflict = true;
            }
        }

        return found_conflict;
    }

    bool detectEdgeConflict(const Path &path1, const Path &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts,
                            Conflict &earliest_conflict)
    {
        bool found_conflict = false;
        const int max_time = std::max(path1.nodes.size(), path2.nodes.size()) - 1;

        for (int t = 0; t < max_time; ++t)
        {
            // t時点とt+1時点の位置を取得
            int pos1_t = t < path1.nodes.size() ? path1.nodes[t] : path1.nodes.back();
            int pos1_t1 = (t + 1) < path1.nodes.size() ? path1.nodes[t + 1] : path1.nodes.back();
            int pos2_t = t < path2.nodes.size() ? path2.nodes[t] : path2.nodes.back();
            int pos2_t1 = (t + 1) < path2.nodes.size() ? path2.nodes[t + 1] : path2.nodes.back();

            // エージェントが位置を交換する場合はエッジ衝突
            if (pos1_t == pos2_t1 && pos1_t1 == pos2_t)
            {
                Conflict conflict(agent1, agent2, t, pos1_t, pos1_t1);
                conflicts.push_back(conflict);

                // 最も早い衝突を更新
                if (!found_conflict)
                {
                    earliest_conflict = conflict;
                }
                // std::cout << "earliest_conflict.timeStep: " << earliest_conflict.timeStep << std::endl;
                found_conflict = true;
            }
        }

        return found_conflict;
    }

    int getMaxTimeStep(const std::vector<Path> &paths)
    {
        int max_time = 0;
        for (const auto &path : paths)
        {
            max_time = std::max(max_time, static_cast<int>(path.nodes.size()));
        }
        return max_time;
    }

    ECBSTA::ECBSTA(const Graph &graph, double w)
        : graph(graph), w(w), min_lb(std::numeric_limits<int>::max()), r(0.0)
    {
    }

    bool ECBSTA::shouldGenerateNewRoot(OpenList &OPEN)
    {
        popClosedNodes(OPEN);
        if (OPEN.empty())
            return false;
        HighLevelElement top_element = OPEN.top();
        return nodes.at(top_element.id)->cost > nextRootNodeCost;
    }
    void ECBSTA::popClosedNodes(OpenList &OPEN)
    {
        while (!OPEN.empty() && !nodes.at(OPEN.top().id)->is_open)
        {
            OPEN.pop();
        }
    }

    void ECBSTA::updateFocalListRebuild(OpenList &OPEN, FocalList &FOCAL)
    {
        popClosedNodes(OPEN);
        int old_min_lb = min_lb;
        min_lb = OPEN.top().cost;
        if (min_lb > old_min_lb)
        {
            std::cout << "min_lb updated: " << min_lb << std::endl;
        }

        // FOCALを一旦クリア
        FOCAL.clear();
        auto it_end = OPEN.priority_queues.upper_bound(nextRootNodeCost);
        for (auto it = OPEN.priority_queues.begin(); it != it_end; ++it)
        {
            // あとでelementを消すようのvector
            std::vector<HighLevelElement> elements_to_delete;
            for (auto it_second = it->second.begin(); it_second != it->second.end(); /* no ++it_second here */)
            {
                HighLevelElement element = *it_second;
                if (nodes.at(element.id)->is_open)
                {
                    // FOCALに追加
                    FOCAL.push(element);
                    // イテレータだけ進める
                    ++it_second;
                }
                else
                {
                    // eraseの戻り値は次の要素を指すイテレータ
                    it_second = it->second.erase(it_second);
                }
            }
        }
    }

    void ECBSTA::updateFocalList(OpenList &OPEN, FocalList &FOCAL)
    {
        popClosedNodes(OPEN);
        int old_min_lb = min_lb;
        min_lb = OPEN.top().cost;
        assert(nodes[OPEN.top().id]->is_open);
        if (min_lb > old_min_lb)
        {
            auto it_end = OPEN.priority_queues.upper_bound(static_cast<int>(min_lb * w));
            for (auto it = OPEN.priority_queues.begin(); it != it_end; ++it)
            {
                // あとでelementを消すようのvector
                std::vector<HighLevelElement> elements_to_delete;
                for (auto it_second = it->second.begin(); it_second != it->second.end(); /* no ++it_second here */)
                {
                    HighLevelElement element = *it_second;
                    int val = nodes.at(element.id)->cost;
                    if (!nodes.at(element.id)->is_open)
                    {
                        it_second = it->second.erase(it_second);
                    }
                    else if (val > old_min_lb * w && val <= min_lb * w)
                    {
                        FOCAL.push(element);
                        ++it_second;
                    }
                    else
                    {
                        ++it_second;
                    }
                }
            }
        }
    }
    HighLevelNode ECBSTA::solve(const std::vector<int> &starts,
                                const std::vector<int> &goals,
                                int time_limit_ms,
                                const std::unordered_set<ECBSTA_NS::AgentTaskPair> &I,
                                const std::unordered_set<ECBSTA_NS::AgentTaskPair> &O)
    {
        // 初期化を追加
        initialize();

        auto start_time = std::chrono::steady_clock::now();
        // コスト行列の計算
        calculateCostMatrix(starts, goals, I, O);
        std::cout << "cost_matrix: " << cost_matrix.size() << " " << cost_matrix[0].size() << std::endl;
        for (const auto &row : cost_matrix)
        {
            for (const auto &cost : row)
            {
                std::cout << cost << " ";
            }
            std::cout << std::endl;
        }

        OpenList OPEN(false); // コストベースの優先度
        FocalList FOCAL(true);

        // 初期割り当ての取得
        auto [initial_assignment, ASG_OPEN] = firstAssignment(cost_matrix, I, O);
        if (initial_assignment.empty())
        {
            return HighLevelNode();
        }
        std::cout << "initial_assignment: " << initial_assignment.size() << std::endl;
        for (const auto &pair : initial_assignment)
        {
            std::cout << pair.first << " " << pair.second << std::endl;
        }

        // rootノードの作成
        HighLevelNode *root = createRootNode(initial_assignment, starts, goals, I, O);
        if (root == nullptr)
        {
            return HighLevelNode();
        }
        if (root->num_conflicts == 0)
        {
            return *root;
        }

        // 初期解の検証と下界値の計算
        // if (!validateNode(*root, starts, goals))
        // {
        //     delete root;
        //     return HighLevelNode();
        // }
        // updateMinLBAndR(root->cost);
        nextRootNodeCost = root->cost * w;
        min_lb = root->cost;
        // std::cout << "min_lb: " << min_lb << std::endl;
        std::cout << "nextRootNodeCost: " << nextRootNodeCost << std::endl;

        OPEN.push(HighLevelElement(root->id, root->cost, root->num_conflicts));
        FOCAL.push(HighLevelElement(root->id, root->cost, root->num_conflicts));
        // if (root->cost <= w * nodes[OPEN.top()]->cost)
        // {
        //     FOCAL.push(root->id);
        // }

        // メインループ
        bool should_update_focal = true; // FOCALリスト更新フラグを追加
        HighLevelNode result;
        std::cout << "OPEN.empty(): " << OPEN.empty() << std::endl;
        int iteration_count = 1; // イテレーションカウンターを追加
        while ((!OPEN.empty() || !FOCAL.empty()) && !isTimeOut(start_time, time_limit_ms))
        {
            // if (iteration_count > 100)
            // {
            //     break;
            // }

            // 100イテレーションごとに情報を表示
            if (iteration_count % 1000000 == 0)
            {
                HighLevelElement open_top_element = OPEN.top();
                HighLevelElement focal_top_element = FOCAL.top();
                std::cout << "Iteration " << iteration_count << ":"
                          << " OPEN size: " << OPEN.size()
                          << " FOCAL size: " << FOCAL.size()
                          << " nextRootNodeCost: " << nextRootNodeCost
                          << " open_top->id: " << open_top_element.id
                          << " open_top->cost: " << nodes.at(open_top_element.id)->cost
                          << " open_top->num_conflicts: " << nodes.at(open_top_element.id)->num_conflicts
                          << " focal_top->id: " << focal_top_element.id
                          << " focal_top->cost: " << nodes.at(focal_top_element.id)->cost
                          << " focal_top->num_conflicts: " << nodes.at(focal_top_element.id)->num_conflicts << std::endl;
                // constraintsをforで
                // for (const auto &constraint : nodes[top]->constraints.vertex_constraints)
                // {
                //     std::cout << "vertex_constraint: " << constraint.agent << " " << constraint.location << " " << constraint.timeStep << std::endl;
                // }
                for (const auto &constraint : nodes.at(open_top_element.id)->constraints.edge_constraints)
                {
                    std::cout << "edge_constraint: " << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
                }
                std::cout << "FOCAL nodes num_conflicts: " << nodes.at(focal_top_element.id)->num_conflicts << std::endl;
                std::cout << "FOCAL nodes conflicts: ";
                for (const auto &conflict : nodes.at(focal_top_element.id)->conflicts)
                {
                    std::cout << conflict.agent1 << " " << conflict.agent2 << " " << conflict.fromLocation << " " << conflict.location << " " << conflict.timeStep << std::endl;
                    std::cout << "start loc: " << graph.locationToXY(starts[conflict.agent1]).first << " " << graph.locationToXY(starts[conflict.agent1]).second << ", " << graph.locationToXY(starts[conflict.agent2]).first << " " << graph.locationToXY(starts[conflict.agent2]).second << std::endl;
                    std::cout << "goal loc: " << graph.locationToXY(goals[conflict.agent1]).first << " " << graph.locationToXY(goals[conflict.agent1]).second << ", " << graph.locationToXY(goals[conflict.agent2]).first << " " << graph.locationToXY(goals[conflict.agent2]).second << std::endl;
                }
                // conflicts[0]のpathを表示
                int agent1 = nodes.at(focal_top_element.id)->conflicts[0].agent1;
                int agent2 = nodes.at(focal_top_element.id)->conflicts[0].agent2;
                std::cout << "conflicts[0] agent1 path: ";
                for (const auto &node : nodes.at(focal_top_element.id)->solution[agent1].nodes)
                {
                    std::cout << "(" << graph.locationToXY(node).first << ", " << graph.locationToXY(node).second << ") ";
                }
                std::cout << std::endl;
                std::cout << "conflicts[0] agent2 path: ";
                for (const auto &node : nodes.at(focal_top_element.id)->solution[agent2].nodes)
                {
                    std::cout << "(" << graph.locationToXY(node).first << ", " << graph.locationToXY(node).second << ") ";
                }
                std::cout << std::endl;

                printNodePaths(*nodes.at(focal_top_element.id));
                // FOCALのノードのコストを表示
                // std::vector<HighLevelElement> temp_elements;
                // std::cout << "FOCAL nodes num_conflicts: ";
                // while (!FOCAL.empty())
                // {
                //     HighLevelElement top_element = FOCAL.top();
                //     if (nodes.at(top_element.id)->is_open)
                //     {
                //         std::cout << nodes.at(top_element.id)->num_conflicts << " ";
                //         temp_elements.push_back(top_element);
                //     }
                //     FOCAL.pop();
                // }
                // std::cout << std::endl;

                // // FOCALに戻す
                // for (auto element : temp_elements)
                // {
                //     FOCAL.push(element);
                // }
            }
            iteration_count++; // カウンターをインクリメント

            // 新しいルートが生成された場合またはmin_lbが更新された場合

            // FOCALリストを更新
            std::chrono::steady_clock::time_point start_update_focal = std::chrono::steady_clock::now();
            updateFocalList(OPEN, FOCAL);
            std::chrono::steady_clock::time_point end_update_focal = std::chrono::steady_clock::now();

            HighLevelElement current_element = FOCAL.top();
            FOCAL.pop();
            HighLevelNode *current = nodes.at(current_element.id);
            if (!current->is_open)
            {
                // std::cout << "current->id: " << current->id << " is_open: " << current->is_open << std::endl;
                continue;
            }
            current->is_open = false;

            // std::cout << "current->num_conflicts: " << current->num_conflicts << std::endl;
            // 解が見つかったかチェック
            if (current->num_conflicts == 0)
            {
                std::cout << "current->num_conflicts == 0" << std::endl;
            }
            if (current->num_conflicts == 0 && current->cost <= nextRootNodeCost)
            {
                result = *current;
                std::cout << "solution found" << std::endl;
                break;
            }

            // 子ノード生成と追加
            std::vector<HighLevelNode *> children = createChildNodes(*current, starts, goals);
            // std::cout << "children.size(): " << children.size() << std::endl;
            for (auto &child : children)
            {

                // if (child->is_open && validateNode(*child, starts, goals))
                // std::cout << "child->id: " << child->id << " child->cost: " << child->cost << " child->num_conflicts: " << child->num_conflicts << std::endl;
                OPEN.push(HighLevelElement(child->id, child->cost, child->num_conflicts));
                if (child->cost <= min_lb * w)
                {
                    FOCAL.push(HighLevelElement(child->id, child->cost, child->num_conflicts));
                }
            }
            if (shouldGenerateNewRoot(OPEN))
            {
                std::cout << "shouldGenerateNewRoot" << std::endl;
                HighLevelNode *new_root = generateNextRoot();
                if (new_root != nullptr)
                {
                    if (!validateNode(*new_root, starts, goals))
                    {
                        delete new_root;
                        continue;
                    }
                    // min_lbが更新された場合、FOCALリストの更新フラグを立てる
                    std::cout << "new_root->cost: " << new_root->cost << std::endl;

                    size_t new_root_id = new_root->id;
                    // size_t new_root_id = getOrCreateNodeId(NodeIdentity{new_root->constraints, new_root->assignment});
                    OPEN.push(HighLevelElement(new_root_id, new_root->cost, new_root->num_conflicts));
                    if (new_root->cost <= min_lb * w)
                    {
                        FOCAL.push(HighLevelElement(new_root_id, new_root->cost, new_root->num_conflicts));
                    }
                }
                else
                {
                    std::cerr << "new_rootがnullptrです" << std::endl;
                }
                popClosedNodes(OPEN);
                HighLevelElement top_element = OPEN.top();
                nextRootNodeCost = nodes.at(top_element.id)->cost * w;
            }
            // std::cout << "OPEN.empty() as end of loop: " << OPEN.empty() << std::endl;
        }

        // メモリ解放
        for (auto &[id, node] : nodes)
        {
            delete node;
        }

        return result;
    }

    std::pair<Constraint, Constraint> ECBSTA::createConstraintsFromConflict(const Conflict &conflict)
    {
        Constraint constraint1;
        Constraint constraint2;
        assert(conflict.agent1 != conflict.agent2);

        if (conflict.type == ConflictType::VERTEX)
        {
            // 頂点制約を生成
            constraint1 = Constraint(
                conflict.agent1,
                conflict.location,
                conflict.timeStep);

            constraint2 = Constraint(
                conflict.agent2,
                conflict.location,
                conflict.timeStep);
        }
        else // エッジ衝突の場合
        {
            // エッジ制約を生成
            constraint1 = Constraint(
                conflict.agent1,
                conflict.fromLocation,
                conflict.location,
                conflict.timeStep);

            constraint2 = Constraint(
                conflict.agent2,
                conflict.location,
                conflict.fromLocation,
                conflict.timeStep);
        }

        return {constraint1, constraint2};
    }

    std::vector<HighLevelNode *> ECBSTA::createChildNodes(const HighLevelNode &parent,
                                                          const std::vector<int> &starts,
                                                          const std::vector<int> &goals)
    {
        std::vector<HighLevelNode *> children;

        if (!parent.has_selected_conflict)
        {
            std::cerr << "親ノードに選択された衝突情報がない" << std::endl;
            return children;
        }

        // 選択された衝突から制約を生成
        auto [constraint1, constraint2] = createConstraintsFromConflict(parent.selected_conflict);
        // std::cerr << "constraint1: " << constraint1.agent << " " << constraint1.location << " " << constraint1.timeStep << " " << constraint1.fromLocation << std::endl;
        // std::cerr << "constraint2: " << constraint2.agent << " " << constraint2.location << " " << constraint2.timeStep << " " << constraint2.fromLocation << std::endl;
        bool constraint_flag = false;
        // agent1用の子ノード
        NodeIdentity node_identity1(parent.constraints, parent.assignment);
        if (parent.constraints.vertex_constraints.find(constraint1) != parent.constraints.vertex_constraints.end())
        {
            std::cerr << "child1の制約が既存の制約に含まれている(vertex)" << std::endl;
            constraint_flag = true;
            std::cerr << "parent.selected_conflict: " << parent.selected_conflict.agent1 << " " << parent.selected_conflict.agent2 << " " << parent.selected_conflict.location << " " << parent.selected_conflict.timeStep << " " << parent.selected_conflict.fromLocation << std::endl;
            // std::cerr << "constraint1: " << constraint1.agent << " " << constraint1.location << " " << constraint1.timeStep << " " << constraint1.fromLocation << std::endl;
            // for (const auto &constraint : parent.constraints.vertex_constraints)
            // {
            //     std::cerr << constraint.agent << " " << constraint.location << " " << constraint.timeStep << " " << constraint.fromLocation << std::endl;
            // }
        }
        if (parent.constraints.edge_constraints.find(constraint1) != parent.constraints.edge_constraints.end())
        {
            std::cerr << "child1の制約が既存の制約に含まれている(edge)" << std::endl;
            std::cerr << "start: " << starts[constraint1.agent] << std::endl;
            std::cerr << "goal: " << goals[constraint1.agent] << std::endl;
            std::cerr << "parent.selected_conflict: " << parent.selected_conflict.agent1 << " " << parent.selected_conflict.agent2 << " " << parent.selected_conflict.location << " " << parent.selected_conflict.timeStep << " " << parent.selected_conflict.fromLocation << std::endl;
            std::cerr << "constraint1: " << constraint1.agent << " " << constraint1.fromLocation << " " << constraint1.location << " " << constraint1.timeStep << std::endl;
            for (const auto &constraint : parent.constraints.edge_constraints)
            {
                std::cerr << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
            }
            constraint_flag = true;
        }
        node_identity1.constraints.insert(constraint1);
        HighLevelNode *child1 = getOrCreateNode(node_identity1);
        // constraint1がchild1のconstraintsに含まれているか確認
        bool found_constraint = false;
        if (constraint1.type == ConstraintType::EDGE && child1->constraints.edge_constraints.find(constraint1) ==
                                                            child1->constraints.edge_constraints.end())
        {
            std::cout << "findできない" << std::endl;
            assert(false);
        }
        else if (constraint_flag)
        {
            std::cout << "findできる" << std::endl;
        }

        assert(child1->is_open);
        if (constraint_flag)
        {
            std::cerr << "child1->is_open: " << child1->is_open << std::endl;
            std::cerr << "child1->id: " << child1->id << std::endl;
            std::cerr << "child1->constraints.edge_constraints: " << std::endl;
            for (const auto &constraint : child1->constraints.edge_constraints)
            {
                std::cerr << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
            }
            std::cerr << "child1->constraints.vertex_constraints: " << std::endl;
            for (const auto &constraint : child1->constraints.vertex_constraints)
            {
                std::cerr << constraint.agent << " " << constraint.location << " " << constraint.timeStep << " " << constraint.fromLocation << std::endl;
            }
        }
        if (child1->is_open)
        {
            child1->I = parent.I;
            child1->O = parent.O;
            child1->solution = parent.solution;
            child1->cost = parent.cost;
            child1->lb = parent.lb;
            child1->is_open = true;
            // 特定のエージェントのみのパスを再計算
            if (validateNodeForAgent(*child1, starts, goals, constraint1.agent))
            {
                children.push_back(child1);
                if (constraint_flag)
                {
                    std::cerr << "child1->selected_conflict: " << child1->selected_conflict.agent1 << " " << child1->selected_conflict.agent2 << " " << child1->selected_conflict.location << " " << child1->selected_conflict.timeStep << " " << child1->selected_conflict.fromLocation << std::endl;
                }
            }
        }

        // agent2用の子ノード
        NodeIdentity node_identity2(parent.constraints, parent.assignment);
        node_identity2.constraints.insert(constraint2);
        HighLevelNode *child2 = getOrCreateNode(node_identity2);
        assert(child2->is_open);
        // std::cerr << "child2->is_open: " << child2->is_open << std::endl;
        // std::cerr << "child2->id: " << child2->id << std::endl;
        // std::cerr << "child2->constraints.edge_constraints: " << std::endl;
        // for (const auto &constraint : child2->constraints.edge_constraints)
        // {
        //     std::cerr << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
        // }
        // std::cerr << "vertex_constraints: " << std::endl;
        // for (const auto &constraint : child2->constraints.vertex_constraints)
        // {
        //     std::cerr << constraint.agent << " " << constraint.location << " " << constraint.timeStep << " " << constraint.fromLocation << std::endl;
        // }

        if (child2->is_open)
        {
            child2->I = parent.I;
            child2->O = parent.O;
            child2->solution = parent.solution;
            child2->cost = parent.cost;
            child2->lb = parent.lb;
            child2->is_open = true;
            // 特定のエージェントのみのパスを再計算
            if (validateNodeForAgent(*child2, starts, goals, constraint2.agent))
            {
                children.push_back(child2);
                // std::cerr << "child2->selected_conflict: " << child2->selected_conflict.agent1 << " " << child2->selected_conflict.agent2 << " " << child2->selected_conflict.location << " " << child2->selected_conflict.timeStep << " " << child2->selected_conflict.fromLocation << std::endl;
            }
        }

        return children;
    }

    void ECBSTA::calculateCostMatrix(const std::vector<int> &starts,
                                     const std::vector<int> &goals,
                                     const std::unordered_set<AgentTaskPair> &I,
                                     const std::unordered_set<AgentTaskPair> &O)
    {
        const int num_agents = starts.size();
        const int num_goals = goals.size();

        cost_matrix.resize(num_agents, std::vector<int>(num_goals));

        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = 0; j < num_goals; ++j)
            {
                // Single Agent Pathをコストとして使用
                cost_matrix[i][j] = graph.getEstimatedDistance(starts[i], goals[j]);
            }
        }

        // 強制割り当てのコストを設定
        for (const auto &pair : I)
        {
            cost_matrix[pair.agent][pair.task] = 0;
        }

        // 割り当て禁止のコストを設定
        for (const auto &pair : O)
        {
            cost_matrix[pair.agent][pair.task] = std::numeric_limits<int>::max();
        }
    }

    bool ECBSTA::validateNode(HighLevelNode &node,
                              const std::vector<int> &starts,
                              const std::vector<int> &goals)
    {
        // 割り当ての制約チェック
        for (const auto &[agent, task] : node.assignment)
        {
            // 禁止されている(agent,task)ペアがないかチェック
            if (node.O.count({agent, task}))
            {
                std::cout << "禁止されている(agent,task)ペアがある" << std::endl;
                return false;
            }
        }

        // 強制割り当てがすべて含まれているかチェック
        for (const auto &forced : node.I)
        {
            auto it = node.assignment.find(forced.agent);
            if (it == node.assignment.end() || it->second != forced.task)
            {
                std::cout << "強制割り当てがすべて含まれていない" << std::endl;
                std::cout << "強制割り当て: " << forced.agent << " " << forced.task << std::endl;
                std::cout << "割り当て: " << it->first << " " << it->second << std::endl;
                std::cout << "cost_matrix: " << std::endl;
                std::cout << cost_matrix[forced.agent][forced.task] << std::endl;
                std::cout << cost_matrix[forced.agent][it->second] << std::endl;
                return false;
            }
        }

        // 割り当てに基づいてパスを生成
        node.solution.resize(starts.size());
        for (const auto &[agent, task] : node.assignment)
        {
            int startLoc = starts[agent];
            int goalLoc = goals[task]; // 割り当てられたタスクのゴール位置を使用
            // node.solution[agent] = findPath(graph, startLoc, goalLoc, node.constraints, agent);
            node.solution[agent] = lowLevelFocalSearch(graph, startLoc, goalLoc, node.constraints, node.solution, agent);

            // パスが見つからなかった場合
            if (node.solution[agent].nodes.empty())
            {
                std::cout << "パスが見つからなかった" << std::endl;
                return false;
            }
        }

        // パスが生成された後、衝突を検出して保存
        ConflictInfo conflict_info = detectConflicts(node.solution);
        node.conflicts = conflict_info.conflicts; // 全ての衝突を保存
        node.num_conflicts = conflict_info.conflicts.size();

        // 衝突がある場合、最も早い衝突を選択して保存
        if (!conflict_info.conflicts.empty())
        {
            node.selected_conflict = conflict_info.earliest_conflict;
            node.has_selected_conflict = true;
        }
        else
        {
            node.has_selected_conflict = false;
        }

        // ノードのコストを更新
        node.cost = 0;
        node.lb = 0;
        for (const auto &path : node.solution)
        {
            node.cost += path.cost;
            node.lb += path.f_min;
        }
        // std::cout << "Sorted solutions by agent_id as calculateCostMatrix:" << std::endl;
        // for (size_t agent_id = 0; agent_id < node.solution.size(); agent_id++)
        // {
        //     std::cout << node.solution[agent_id].cost << " ";
        // }
        // std::cout << std::endl;
        return true;
    }

    bool ECBSTA::isTimeOut(const std::chrono::steady_clock::time_point &start_time,
                           int time_limit_ms) const
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           current_time - start_time)
                           .count();
        return elapsed >= time_limit_ms;
    }

    HighLevelNode *ECBSTA::generateNextRoot()
    {
        // 次の割り当て候補を生成
        auto roots = generateNextRoots();
        std::cout << "roots.size(): " << roots.size() << std::endl;
        // コストが最小の割り当てを選択
        HighLevelNode *best_root = nullptr;
        int min_assignment_cost = std::numeric_limits<int>::max();

        for (const auto &root : roots)
        {
            int assignment_cost = calculateAssignmentCost(root.assignment, cost_matrix);
            if (assignment_cost < min_assignment_cost && isValidAssignment(root.assignment))
            {
                if (best_root)
                    delete best_root;
                best_root = new HighLevelNode(root);
                min_assignment_cost = assignment_cost;
            }
        }
        if (best_root == nullptr)
        {
            return nullptr;
        }
        std::cout << "best_root->lb: " << best_root->lb << std::endl;
        size_t best_root_id = getOrCreateNodeId(best_root->getIdentity());
        HighLevelNode *new_root = new HighLevelNode(*best_root);
        new_root->root = true;
        nodes[best_root_id] = new_root;
        std::cout << "best_root_id: " << best_root_id << std::endl;
        new_root->id = best_root_id;
        // new_root->cost = best_root->cost;
        // std::cout << "nodes[best_root_id]->lb: " << nodes[best_root_id]->lb << std::endl;
        return new_root;
    }

    std::vector<HighLevelNode> ECBSTA::generateNextRoots()
    {
        std::vector<HighLevelNode> candidates;

        // 現在の最適割り当てを基に、1つのタスクだけ異なる割り当てを生成
        const int num_agents = cost_matrix.size();
        const int num_tasks = cost_matrix[0].size();

        std::unordered_set<AgentTaskPair> I = nodes[0]->I;
        std::unordered_set<AgentTaskPair> O = nodes[0]->O;

        // 基本となる割り当てを取得
        // auto base_assignment = constrainedAssignment(I, O, cost_matrix);
        auto base_assignment = nodes[0]->assignment;
        std::cout << "base_assignment: " << base_assignment.size() << std::endl;

        // 各エージェントについて
        for (int i = 0; i < num_agents; ++i)
        {
            int current_task = base_assignment[i];
            int forced_task = -1;
            // Iの中からagent iのtaskを探す
            for (const auto &pair : I)
            {
                if (pair.agent == i)
                {
                    forced_task = pair.task;
                    break;
                }
            }

            // 現在のタスク以外の各タスクを試す
            for (int j = 0; j < num_tasks; ++j)
            {
                if (j == current_task)
                    continue;
                AgentTaskPair pair = {i, j};

                if (forced_task != -1 && j != forced_task)
                {
                    continue;
                }

                if (O.count(pair))
                {
                    continue;
                }

                // 新しい割り当ての作成
                HighLevelNode new_root;
                new_root.root = true;
                new_root.assignment = base_assignment;
                new_root.assignment[i] = j; // タスクを変更
                if (hasNodeIdentity(new_root.getIdentity()))
                {
                    continue;
                }
                new_root.I = I;
                new_root.O = O;

                // コストの計算
                // new_root.cost = calculateAssignmentCost(new_root.assignment, cost_matrix);

                candidates.push_back(new_root);
            }
        }

        return candidates;
    }

    bool ECBSTA::isValidAssignment(const Assignment &assignment) const
    {
        std::unordered_set<int> used_tasks;

        // 各割り当てをチェック
        for (const auto &[agent, task] : assignment)
        {
            // タスクの重複チェック
            if (used_tasks.count(task) > 0)
            {
                return false;
            }
            used_tasks.insert(task);
        }

        return true;
    }

    Path ECBSTA::lowLevelFocalSearch(const Graph &graph,
                                     int start,
                                     int goal,
                                     const ConstraintSet &constraints,
                                     const std::vector<Path> &other_paths,
                                     int agent_id)
    {
        // ゴール位置での制約の最大時間を事前に計算
        int max_goal_constraint_time = -1;
        for (const auto &constraint : constraints.vertex_constraints)
        {
            max_goal_constraint_time = std::max(max_goal_constraint_time, constraint.timeStep);
            // if (constraint.location == goal)
            // {
            //     max_goal_constraint_time = std::max(max_goal_constraint_time, constraint.timeStep);
            // }
        }
        // for (const auto &constraint : constraints.edge_constraints)
        // {
        //     if (constraint.agent == agent_id && constraint.location == goal)
        //     {
        //         max_goal_constraint_time = std::max(max_goal_constraint_time, constraint.timeStep);
        //     }
        // }

        // ノードの状態からIDへのマップ
        std::unordered_map<LowLevelNodeState, size_t, LowLevelNodeStateHash> state_to_id;
        // IDからノードへのマップ
        std::unordered_map<size_t, LowLevelNode> path_nodes;

        // OpenListの要素を表す構造体
        struct OpenElement
        {
            size_t id;
            int f_score;
            int g_score;

            OpenElement(size_t id_, int f_, int g_) : id(id_), f_score(f_), g_score(g_) {}
        };

        // FocalListの要素を表す構造体
        struct FocalElement
        {
            size_t id;
            int conflicts;
            int f_score;
            int g_score;

            FocalElement(size_t id_, int conflicts_, int f_score_, int g_score_) : id(id_), conflicts(conflicts_), f_score(f_score_), g_score(g_score_) {}
        };

        // IDベースの優先度付きキュー
        std::priority_queue<OpenElement,
                            std::vector<OpenElement>,
                            std::function<bool(const OpenElement &, const OpenElement &)>>
            open_list(
                [](const OpenElement &a, const OpenElement &b)
                {
                    if (a.f_score != b.f_score)
                    {
                        return a.f_score > b.f_score;
                    }
                    return a.g_score < b.g_score;
                });

        // IDベースの優先度付きキュー
        std::priority_queue<FocalElement,
                            std::vector<FocalElement>,
                            std::function<bool(const FocalElement &, const FocalElement &)>>
            focal_list(
                [](const FocalElement &a, const FocalElement &b)
                {
                    if (a.conflicts != b.conflicts)
                    {
                        return a.conflicts > b.conflicts;
                    }
                    else if (a.f_score != b.f_score)
                    {
                        return a.f_score > b.f_score;
                    }
                    return a.g_score < b.g_score;
                });

        size_t next_low_level_node_id = 0;

        // 開始ノードの作成
        int h_score = calculateHeuristic(start, goal, graph);
        int focal_heuristic = countConflicts(start, 0, other_paths, agent_id);
        LowLevelNode start_node(next_low_level_node_id, start, 0, h_score, 0, focal_heuristic);
        path_nodes[next_low_level_node_id] = start_node;
        state_to_id[LowLevelNodeState{start, 0}] = next_low_level_node_id;
        open_list.push(OpenElement(next_low_level_node_id, h_score, 0));
        focal_list.push(FocalElement(next_low_level_node_id, focal_heuristic, h_score, 0));
        next_low_level_node_id++;
        int min_lb_low_level = h_score;
        // std::cout << "min_lb_low_level: " << min_lb_low_level << std::endl;

        // std::cout << "path_nodes[0].f_score: " << path_nodes[0].f_score << std::endl;
        while (!open_list.empty() || !focal_list.empty())
        {
            int old_min_lb_low_level = min_lb_low_level;
            while (!open_list.empty() && !path_nodes[open_list.top().id].is_open)
            {
                // std::cout << "open_list.top() pop: " << path_nodes[open_list.top()].f_score << std::endl;
                open_list.pop();
            }
            // std::cout << "open_list.top(): " << path_nodes[open_list.top()].f_score << std::endl;
            // std::cout << "open_list.size(): " << open_list.size() << std::endl;
            assert(!open_list.empty());
            min_lb_low_level = path_nodes[open_list.top().id].f_score;
            // int tmp_g = path_nodes[open_list.top().id].g_score;
            // std::cout << "tmp_g: " << tmp_g << std::endl;
            // std::cout << "min_lb_low_level: " << min_lb_low_level << std::endl;
            // std::cout << "old_min_lb_low_level: " << old_min_lb_low_level << std::endl;
            if (min_lb_low_level > old_min_lb_low_level)
            {
                // std::cout << "min_lb_low_level: " << min_lb_low_level << std::endl;

                std::vector<OpenElement> tmp_ids;
                while (!open_list.empty())
                {
                    OpenElement top_element = open_list.top();
                    open_list.pop();
                    size_t top_id = top_element.id;
                    int val = top_element.f_score;
                    // std::cout << "val: " << val << std::endl;
                    if (!path_nodes[top_id].is_open)
                    {
                        continue;
                    }
                    else if (val > old_min_lb_low_level * w && val <= min_lb_low_level * w)
                    {
                        focal_list.push(FocalElement(top_id, path_nodes[top_id].focal_heuristic, path_nodes[top_id].f_score, path_nodes[top_id].g_score));
                        tmp_ids.push_back(top_element);
                    }
                    else if (val > min_lb_low_level * w)
                    {
                        tmp_ids.push_back(top_element);
                        break;
                    }
                }
                for (auto element : tmp_ids)
                {
                    open_list.push(element);
                }
            }
            // std::cout << "open_list.size(): " << open_list.size() << std::endl;
            // std::cout << "focal_list.size(): " << focal_list.size() << std::endl;
            FocalElement current_element = focal_list.top();
            focal_list.pop();
            size_t current_id = current_element.id;
            LowLevelNode &current = path_nodes[current_id];

            // ノードが既に展開済みの場合はスキップ
            if (!current.is_open)
            {
                // std::cout << "current_id: " << current_id << " current.is_open: " << current.is_open << std::endl;
                continue;
            }
            current.is_open = false;

            // ゴールに到達した場合の処理を修正
            if ((current.location == goal && !current.reached_goal) ||                  // 初めてゴールに到達
                (current.reached_goal && current.time_step > max_goal_constraint_time)) // ゴール到達済みで制約時間を超えた
            {
                if (current.location == goal && !current.reached_goal)
                {
                    // 初めてゴールに到達した場合、フラグを立てる
                    current.reached_goal = true;
                }

                if (current.time_step > max_goal_constraint_time)
                {
                    // 制約時間を超えた場合はパスを再構築して終了
                    while (!open_list.empty() && !path_nodes[open_list.top().id].is_open)
                    {
                        open_list.pop();
                    }
                    return reconstructPath(current_id, path_nodes);
                }
            }

            // 隣接ノードの探索
            for (int next_loc : graph.getNeighbors(current.location))
            {
                int next_time = current.time_step + 1;

                // 制約チェック
                if (isConstrained(agent_id, next_loc, next_time, current.location, constraints))
                {
                    continue;
                }

                // コストの計算を修正
                int next_g;
                int next_h;
                if (current.reached_goal)
                {
                    // ゴール到達後は追加コストを0とする
                    next_g = current.g_score;
                    next_h = 0;
                }
                else
                {
                    next_g = current.g_score + graph.getCost(current.location, next_loc, goal);
                    next_h = calculateHeuristic(next_loc, goal, graph);
                }

                int next_f = next_g + next_h;

                // 状態が既に存在するかチェック
                LowLevelNodeState next_state{next_loc, next_time};
                auto state_it = state_to_id.find(next_state);

                if (state_it != state_to_id.end())
                {
                    // 既存のノードを取得
                    LowLevelNode &existing_node = path_nodes[state_it->second];

                    // より良いパスが見つかった場合は更新
                    if (next_g < existing_node.g_score)
                    {
                        int next_focal_heuristic = current.focal_heuristic +
                                                   countConflicts(next_loc, next_time, other_paths, agent_id);
                        existing_node.g_score = next_g;
                        existing_node.f_score = next_f;
                        existing_node.parent_id = current_id;
                        existing_node.is_open = true;
                        existing_node.focal_heuristic = next_focal_heuristic;
                        existing_node.reached_goal = current.reached_goal; // ゴール到達フラグを継承
                        OpenElement open_element(state_it->second, next_f, next_g);
                        open_list.push(open_element);
                        if (existing_node.f_score <= min_lb_low_level * w)
                        {
                            FocalElement focal_element(state_it->second, next_focal_heuristic, next_f, next_g);
                            focal_list.push(focal_element);
                        }
                    }
                }
                else
                {
                    int next_focal_heuristic = current.focal_heuristic +
                                               countConflicts(next_loc, next_time, other_paths, agent_id);
                    // 新しいノードを作成
                    LowLevelNode next_node(next_low_level_node_id, next_loc, next_g, next_f,
                                           next_time, current_id, next_focal_heuristic);
                    next_node.reached_goal = current.reached_goal; // ゴール到達フラグを継承
                    path_nodes[next_low_level_node_id] = next_node;
                    state_to_id[next_state] = next_low_level_node_id;
                    OpenElement open_element(next_low_level_node_id, next_f, next_g);
                    open_list.push(open_element);
                    if (next_node.f_score <= min_lb_low_level * w)
                    {
                        FocalElement focal_element(next_low_level_node_id, next_focal_heuristic, next_f, next_g);
                        focal_list.push(focal_element);
                    }
                    next_low_level_node_id++;

                    // if (next_f < min_lb_low_level)
                    // {
                    //     min_lb_low_level = next_f;
                    //     r_low_level = w * min_lb_low_level;
                    //     should_update_focal_list = true;
                    // }
                }
            }
        }
        if (agent_id == -1)
        {
            std::cout << "state_to_id: " << std::endl;
            for (const auto &[state, id] : state_to_id)
            {
                std::cout << "state: " << state.location << ", " << state.time_step << " -> id: " << id << std::endl;
            }
        }
        // パスが見つからない場合
        return Path();
    }

    int ECBSTA::countConflicts(int location, int time_step,
                               const std::vector<Path> &other_paths,
                               int agent_id)
    {
        int conflicts = 0;

        // 他の全てのパスとの衝突をチェック
        for (size_t i = 0; i < other_paths.size(); ++i)
        {
            if (i == static_cast<size_t>(agent_id))
                continue;

            const auto &other_path = other_paths[i];
            if (time_step < other_path.nodes.size())
            {
                // 頂点衝突
                if (location == other_path.nodes[time_step])
                {
                    conflicts++;
                }

                // エッジ衝突
                if (time_step > 0 && time_step < other_path.nodes.size())
                {
                    if (location == other_path.nodes[time_step - 1] &&
                        other_path.nodes[time_step] == location)
                    {
                        conflicts++;
                    }
                }
            }
        }

        return conflicts;
    }

    size_t ECBSTA::getOrCreateNodeId(const NodeIdentity &identity)
    {
        if (hasNodeIdentity(identity))
        {
            std::cerr << "既存のノードIDを取得" << std::endl;
            return node_id_map[identity];
        }
        size_t new_id = next_node_id++;
        node_id_map[identity] = new_id;
        return new_id;
    }

    HighLevelNode *ECBSTA::getOrCreateNode(const NodeIdentity &identity)
    {
        size_t id = getOrCreateNodeId(identity);
        auto it = nodes.find(id);
        if (it != nodes.end())
        {
            std::cerr << "既存のノードを取得" << std::endl;
            // std::cerr << "id: " << id << std::endl;
            // std::cerr << "new identity: " << std::endl;
            // for (const auto &constraint : identity.constraints.vertex_constraints)
            // {
            //     std::cerr << constraint.agent << " " << constraint.location << " " << constraint.timeStep << std::endl;
            // }
            // for (const auto &constraint : identity.constraints.edge_constraints)
            // {
            //     std::cerr << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
            // }
            // std::cerr << std::endl;

            // std::cerr << "old identity: " << std::endl;
            // for (const auto &constraint : it->second->constraints.vertex_constraints)
            // {
            //     std::cerr << constraint.agent << " " << constraint.location << " " << constraint.timeStep << std::endl;
            // }
            // for (const auto &constraint : it->second->constraints.edge_constraints)
            // {
            //     std::cerr << constraint.agent << " " << constraint.fromLocation << " " << constraint.location << " " << constraint.timeStep << std::endl;
            // }
            // std::cerr << std::endl;

            return it->second;
        }

        // 新しいノードを作成
        HighLevelNode *new_node = new HighLevelNode();
        new_node->id = id;
        new_node->constraints = identity.constraints;
        new_node->assignment = identity.assignment;
        nodes[id] = new_node;
        return new_node;
    }

    bool ECBSTA::hasNodeIdentity(const NodeIdentity &identity) const
    {
        return node_id_map.find(identity) != node_id_map.end();
    }

    void ECBSTA::initialize()
    {
        w = 1.3;
        min_lb = std::numeric_limits<int>::max();
        r = 0.0;
        // メモリ解放
        for (auto &[id, node] : nodes)
        {
            delete node;
        }
        nodes.clear();
        node_id_map.clear();
        next_node_id = 0;
    }

    HighLevelNode *ECBSTA::createRootNode(const Assignment &initial_assignment, std::vector<int> starts, std::vector<int> goals,
                                          const std::unordered_set<AgentTaskPair> &I, const std::unordered_set<AgentTaskPair> &O)
    {
        // 空の制約セットと初期割り当てでNodeIdentityを作成
        NodeIdentity root_identity{
            ConstraintSet(),
            initial_assignment};
        if (hasNodeIdentity(root_identity))
        {
            return nullptr;
        }

        // ノードを取得または作成
        HighLevelNode *root = getOrCreateNode(root_identity);
        root->root = true;
        root->I = I;
        root->O = O;
        // root->cost = calculateAssignmentCostWithPrint(initial_assignment, cost_matrix);
        // root->lb = root->cost;

        validateNode(*root, starts, goals);

        return root;
    }

    // PriorityFIFOQueueのメソッド実装を修正
    void PriorityFIFOQueue::push(const HighLevelElement &element)
    {
        // 優先度を計算
        // OpenリストならコストベースでFocalリストなら衝突数とコストの組み合わせ
        if (is_focal)
        {
            // Focalリストの場合、衝突数を主キー、コストを副キーとする複合キー
            auto priority = std::make_pair(element.num_conflicts, element.cost);
            priority_queues_focal[priority].push_back(element);
        }
        else
        {
            // Openリストの場合、コストのみ
            priority_queues[element.cost].push_back(element);
        }
    }

    HighLevelElement PriorityFIFOQueue::top() const
    {
        if (empty())
        {
            throw std::runtime_error("Queue is empty");
        }
        // 最小の優先度を持つキューの先頭要素を返す
        if (is_focal)
        {
            return priority_queues_focal.begin()->second.front();
        }
        return priority_queues.begin()->second.front();
    }

    void PriorityFIFOQueue::pop()
    {
        if (empty())
        {
            throw std::runtime_error("Queue is empty");
        }
        if (is_focal)
        {
            auto it = priority_queues_focal.begin();
            it->second.pop_front();
            if (it->second.empty())
            {
                priority_queues_focal.erase(it);
            }
        }
        else
        {
            auto it = priority_queues.begin();
            it->second.pop_front();
            if (it->second.empty())
            {
                priority_queues.erase(it);
            }
        }
    }

    bool PriorityFIFOQueue::empty() const
    {
        if (is_focal)
        {
            return priority_queues_focal.empty();
        }
        return priority_queues.empty();
    }

    size_t PriorityFIFOQueue::size() const
    {
        size_t total = 0;
        if (is_focal)
        {
            for (const auto &[_, queue] : priority_queues_focal)
            {
                total += queue.size();
            }
        }
        else
        {
            for (const auto &[_, queue] : priority_queues)
            {
                total += queue.size();
            }
        }
        return total;
    }

    void PriorityFIFOQueue::clear()
    {
        if (is_focal)
        {
            priority_queues_focal.clear();
        }
        else
        {
            priority_queues.clear();
        }
    }

    // 既存のvalidateNodeはそのままに、新しいvalidateNodeForAgentを追加
    bool ECBSTA::validateNodeForAgent(HighLevelNode &node,
                                      const std::vector<int> &starts,
                                      const std::vector<int> &goals,
                                      int agent_id)
    {
        // 割り当ての制約チェック
        auto it = node.assignment.find(agent_id);
        if (it == node.assignment.end())
        {
            std::cout << "エージェント" << agent_id << "の割り当てが見つかりません" << std::endl;
            return false;
        }

        // 禁止されている(agent,task)ペアのチェック
        // if (node.O.count({agent_id, it->second}))
        // {
        //     std::cout << "禁止されている(agent,task)ペアがある" << std::endl;
        //     return false;
        // }

        // 強制割り当てのチェック
        // for (const auto &forced : node.I)
        // {
        //     if (forced.agent == agent_id && forced.task != it->second)
        //     {
        //         std::cout << "強制割り当てが満たされていない" << std::endl;
        //         return false;
        //     }
        // }

        // 古いパスのコストを引く
        node.cost -= node.solution[agent_id].cost;
        node.lb -= node.solution[agent_id].f_min;

        // 特定のエージェントのパスのみを再計算
        int startLoc = starts[agent_id];
        int goalLoc = goals[it->second];
        // node.solution[agent_id] = findPath(graph, startLoc, goalLoc, node.constraints, agent_id);
        node.solution[agent_id] = lowLevelFocalSearch(graph, startLoc, goalLoc, node.constraints, node.solution, agent_id);

        // パスが見つからなかった場合
        if (startLoc != goalLoc && node.solution[agent_id].nodes.empty())
        {
            std::cout << "パスが見つからなかった" << std::endl;
            std::cout << "graph.getMap().at(startLoc): " << graph.getMap().at(startLoc) << std::endl;
            std::cout << "graph.getMap().at(goalLoc): " << graph.getMap().at(goalLoc) << std::endl;
            std::cout << "startLoc: " << startLoc << std::endl;
            std::cout << "goalLoc: " << goalLoc << std::endl;
            std::cout << "agent " << agent_id << "の制約:" << std::endl;
            // for (const auto &constraint : node.constraints.vertex_constraints)
            // {
            //     if (constraint.agent == agent_id)
            //     {
            //         std::cout << "  時刻" << constraint.timeStep << "に位置" << constraint.location << "での"
            //                   << "制約" << std::endl;
            //     }
            // }
            // for (const auto &constraint : node.constraints.edge_constraints)
            // {
            //     if (constraint.agent == agent_id)
            //     {
            //         std::cout << "  時刻" << constraint.timeStep << "に位置" << constraint.fromLocation << "から" << constraint.location << "での"
            //                   << "制約" << std::endl;
            //     }
            // }
            return false;
        }

        // 新しいパスのコストを加算
        node.cost += node.solution[agent_id].cost;
        node.lb += node.solution[agent_id].f_min;

        // 衝突検出を行う
        ConflictInfo conflict_info = detectConflicts(node.solution);
        node.conflicts = conflict_info.conflicts; // 全ての衝突を保存
        node.num_conflicts = conflict_info.conflicts.size();
        if (!conflict_info.conflicts.empty())
        {
            node.selected_conflict = conflict_info.earliest_conflict;
            node.has_selected_conflict = true;
        }
        else
        {
            std::cout << "no conflict" << std::endl;
            node.has_selected_conflict = false;
        }
        return true;
    }

    void ECBSTA::printNodePaths(const HighLevelNode &node) const
    {
        std::cout << "ノードID: " << node.id << " のパス情報:" << std::endl;
        std::cout << "総コスト: " << node.cost << std::endl;
        std::cout << "衝突数: " << node.num_conflicts << std::endl;

        // 各エージェントのパスを表示
        for (size_t agent_id = 0; agent_id < node.solution.size(); agent_id++)
        {
            std::cout << "エージェント " << agent_id << ":" << std::endl;
            std::cout << "  割り当てられたタスク: " << node.assignment.at(agent_id) << std::endl;
            std::cout << "  パスコスト: " << node.solution[agent_id].cost << std::endl;

            // スタート位置とゴール位置を表示
            auto [start_x, start_y] = graph.locationToXY(node.solution[agent_id].nodes.front());
            auto [goal_x, goal_y] = graph.locationToXY(node.solution[agent_id].nodes.back());
            std::cout << "  スタート: (" << start_x << "," << start_y << ")" << std::endl;
            std::cout << "  ゴール: (" << goal_x << "," << goal_y << ")" << std::endl;

            std::cout << "  パス: ";
            // パスの各位置を(x, y)座標で表示
            for (const auto &location : node.solution[agent_id].nodes)
            {
                auto [x, y] = graph.locationToXY(location);
                std::cout << "(" << x << "," << y << ") ";
            }
            std::cout << std::endl;
        }

        // 衝突情報の表示
        if (node.has_selected_conflict)
        {
            std::cout << "選択された衝突:" << std::endl;
            std::cout << "  エージェント " << node.selected_conflict.agent1
                      << " と " << node.selected_conflict.agent2 << std::endl;
            std::cout << "  時刻: " << node.selected_conflict.timeStep << std::endl;
            if (node.selected_conflict.type == ConflictType::VERTEX)
            {
                auto [x, y] = graph.locationToXY(node.selected_conflict.location);
                std::cout << "  頂点衝突 位置: (" << x << "," << y << ")" << std::endl;
            }
            else
            {
                auto [from_x, from_y] = graph.locationToXY(node.selected_conflict.fromLocation);
                auto [to_x, to_y] = graph.locationToXY(node.selected_conflict.location);
                std::cout << "  エッジ衝突 from: (" << from_x << "," << from_y
                          << ") to: (" << to_x << "," << to_y << ")" << std::endl;
            }
        }

        // 制約情報の表示
        std::cout << "頂点制約数: " << node.constraints.vertex_constraints.size() << std::endl;
        std::cout << "エッジ制約数: " << node.constraints.edge_constraints.size() << std::endl;
    }
}