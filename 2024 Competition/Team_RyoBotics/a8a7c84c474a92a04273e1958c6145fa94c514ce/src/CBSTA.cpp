#include "CBSTA.h"
#include <queue>
#include <algorithm>
#include <chrono>
#include <iostream>
namespace CBSTA_NS
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
    std::pair<Assignment, std::vector<Node>> firstAssignment(const std::vector<std::vector<int>> &C)
    {
        Node R;
        R.I.clear();
        R.O.clear();
        R.root = true;

        // 制約なしでの最初の割り当てを取得
        R.assignment = constrainedAssignment(R.I, R.O, C);
        std::cout << "R.assignment: " << R.assignment.size() << std::endl;

        if (R.assignment.empty())
        {
            return {Assignment(), std::vector<Node>()};
        }

        // ASG_OPENの初期化
        std::vector<Node> ASG_OPEN;
        ASG_OPEN.push_back(R);

        return {R.assignment, ASG_OPEN};
    }

    // 次の割り当てを生成する関数
    std::pair<Assignment, std::vector<Node>> nextAssignment(
        const std::vector<std::vector<int>> &C,
        std::vector<Node> &ASG_OPEN)
    {
        if (ASG_OPEN.empty())
        {
            return {Assignment(), ASG_OPEN};
        }

        // コストが最小のノードを取得
        auto min_it = std::min_element(ASG_OPEN.begin(), ASG_OPEN.end(),
                                       [](const Node &a, const Node &b)
                                       { return a.cost < b.cost; });

        Node P = *min_it;
        ASG_OPEN.erase(min_it);

        const int num_tasks = C[0].size();

        // 各タスクについて新しい制約を試す
        for (int task = 0; task < num_tasks; ++task)
        {
            // タスクがすでに強制割り当てリストにある場合はスキップ
            bool task_forced = false;
            for (const auto &forced : P.I)
            {
                if (forced.task == task)
                {
                    task_forced = true;
                    break;
                }
            }
            if (task_forced)
            {
                continue;
            }

            // 新しいノードを作成
            Node Q = P;
            Q.root = false;

            // taskを持つagentを探す
            int agent_with_task = -1;
            for (const auto &[agent, t] : P.assignment)
            {
                if (t == task)
                {
                    agent_with_task = agent;
                    break;
                }
            }
            if (agent_with_task != -1)
            {
                Q.O.insert({agent_with_task, task});
            }

            // 新しい制約での割り当てを試行
            Q.assignment = constrainedAssignment(Q.I, Q.O, C);

            if (!Q.assignment.empty())
            {
                // コストを計算
                Q.cost = calculateAssignmentCost(Q.assignment, C);
                ASG_OPEN.push_back(Q);
            }
        }

        // ASG_OPENから最適な割り当てを見つける
        if (!ASG_OPEN.empty())
        {
            auto best_it = std::min_element(ASG_OPEN.begin(), ASG_OPEN.end(),
                                            [](const Node &a, const Node &b)
                                            { return a.cost < b.cost; });
            return {best_it->assignment, ASG_OPEN};
        }

        return {Assignment(), ASG_OPEN};
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

    Path findPath(const Graph &graph,
                  int start,
                  int goal,
                  const std::vector<Constraint> &constraints,
                  int agent_id)
    {
        std::priority_queue<PathNode *, std::vector<PathNode *>, std::greater<PathNode *>> open_list;
        std::unordered_map<int, std::unordered_map<int, PathNode *>> closed_list; // [time][location]

        // 開始ノードの作成
        int h_score = calculateHeuristic(start, goal, graph);
        PathNode *start_node = new PathNode(start, 0, h_score, 0);
        open_list.push(start_node);

        while (!open_list.empty())
        {
            PathNode *current = open_list.top();
            open_list.pop();

            // ゴールに到達した場合
            if (current->location == goal)
            {
                Path result = reconstructPath(current);

                // メモリの解放
                while (!open_list.empty())
                {
                    delete open_list.top();
                    open_list.pop();
                }
                for (auto &time_map : closed_list)
                {
                    for (auto &loc_node : time_map.second)
                    {
                        delete loc_node.second;
                    }
                }

                return result;
            }

            // 現在のノードを closed_list に追加
            closed_list[current->time_step][current->location] = current;

            // 隣接ノードの探索
            for (int next_loc : graph.getNeighbors(current->location))
            {
                int next_time = current->time_step + 1;

                // 制約チェック
                if (isConstrained(agent_id, next_loc, next_time, constraints))
                {
                    continue;
                }

                // 既に探索済みのノードはスキップ
                if (closed_list[next_time].find(next_loc) != closed_list[next_time].end())
                {
                    continue;
                }

                int next_g = current->g_score + graph.getCost(current->location, next_loc, goal);
                int next_h = calculateHeuristic(next_loc, goal, graph);
                int next_f = next_g + next_h;

                PathNode *next_node = new PathNode(next_loc, next_g, next_f, next_time, current);
                open_list.push(next_node);
            }
        }

        // パスが見つからない場合
        return Path();
    }

    bool isConstrained(int agent_id,
                       int location,
                       int time_step,
                       const std::vector<Constraint> &constraints)
    {
        for (const auto &constraint : constraints)
        {
            if (constraint.agent == agent_id &&
                constraint.location == location &&
                constraint.timeStep == time_step)
            {
                return true;
            }
        }
        return false;
    }

    int calculateHeuristic(int current, int goal, const Graph &graph)
    {
        // 簡単な実装例：ユークリッド距離やマンハッタン距離を使用
        // グラフの実装に依存するため、適切な距離計算方法を選択する必要があります
        return graph.getEstimatedDistance(current, goal);
    }

    Path reconstructPath(PathNode *goal_node)
    {
        Path path;
        PathNode *current = goal_node;

        while (current != nullptr)
        {
            path.nodes.insert(path.nodes.begin(), current->location);
            path.cost += (current->parent ? 1 : 0); // 単純な実装：各ステップのコストを1とする
            current = current->parent;
        }

        return path;
    }

    std::vector<Path> findIndividualPaths(const Graph &graph,
                                          const std::vector<int> &starts,
                                          const std::vector<int> &goals,
                                          const std::vector<Constraint> &constraints)
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
                // 頂点衝突の検出
                if (detectVertexConflict(paths[i], paths[j], i, j, result.conflicts))
                {
                    result.hasConflict = true;
                }

                // エッジ衝突の検出
                if (detectEdgeConflict(paths[i], paths[j], i, j, result.conflicts))
                {
                    result.hasConflict = true;
                }
            }
        }

        return result;
    }

    bool detectVertexConflict(const Path &path1, const Path &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts)
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
                conflicts.emplace_back(agent1, agent2, t, pos1);
                found_conflict = true;
            }
        }

        return found_conflict;
    }

    bool detectEdgeConflict(const Path &path1, const Path &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts)
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

            // エージェントが位置を交換する場合
            if (pos1_t == pos2_t1 && pos1_t1 == pos2_t)
            {
                // エッジ衝突として記録
                // 便宜上、衝突位置は最初のエージェントの位置とする
                conflicts.emplace_back(agent1, agent2, t + 1, pos1_t);
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

    CBSTA::CBSTA(const Graph &g) : graph(g)
    {
    }

    std::vector<Path> CBSTA::solve(const std::vector<int> &starts,
                                   const std::vector<int> &goals,
                                   int time_limit_ms,
                                   const std::unordered_set<int> &I,
                                   const std::unordered_set<int> &O)
    {
        auto start_time = std::chrono::steady_clock::now();
        std::vector<Node *> allocated_nodes; // メモリ管理用

        // コスト行列の計算
        calculateCostMatrix(starts, goals);
        std::cout << "cost_matrix: " << cost_matrix.size() << " " << cost_matrix[0].size() << std::endl;

        // OPENリストを優先度付きキューに変更
        std::priority_queue<Node *, std::vector<Node *>, std::function<bool(Node *, Node *)>> OPEN(
            [](Node *a, Node *b)
            { return a->cost > b->cost; });

        // 初期ノードの追加
        Node *root = new Node();
        root->root = true;

        // 初期割り当ての取得
        auto [initial_assignment, ASG_OPEN] = firstAssignment(cost_matrix);
        if (initial_assignment.empty())
        {
            return std::vector<Path>();
        }

        root->assignment = initial_assignment;

        // 初期解の検証
        if (!validateNode(*root, starts, goals))
        {
            return std::vector<Path>();
        }

        OPEN.push(root);

        // メインループ
        std::vector<Path> result;
        while (!OPEN.empty() && !isTimeOut(start_time, time_limit_ms))
        {
            Node *current = OPEN.top();
            OPEN.pop();

            // 衝突チェック
            ConflictInfo conflict_info = detectConflicts(current->solution);
            if (!conflict_info.hasConflict)
            {
                result = current->solution; // 解を保存
                break;
            }

            // 衝突に基づいて子ノードを生成
            if (current->root)
            {
                // ルートノードの場合は、タスク割り当ての変更を試みる
                auto [next_assignment, updated_ASG_OPEN] = nextAssignment(cost_matrix, ASG_OPEN);
                if (!next_assignment.empty())
                {
                    Node *child = new Node();
                    allocated_nodes.push_back(child);
                    child->root = false;
                    child->assignment = next_assignment;

                    if (validateNode(*child, starts, goals))
                    {
                        OPEN.push(child);
                    }
                }
            }

            // 衝突回避のための子ノードを生成
            auto children = createChildNodes(*current, conflict_info.conflicts[0]);
            for (auto &child : children)
            {
                if (validateNode(child, starts, goals))
                {
                    Node *new_child = new Node(child);
                    allocated_nodes.push_back(new_child);
                    OPEN.push(new_child);
                }
            }
        }

        return result;
    }

    std::vector<Node> CBSTA::createChildNodes(const Node &parent,
                                              const Conflict &conflict)
    {
        std::vector<Node> children;

        // agent1用の子ノード
        Node child1 = parent;
        child1.root = false;
        // 親ノードの制約を引き継ぐ
        child1.I = parent.I;
        child1.O = parent.O;
        child1.assignment = parent.assignment;
        // 新しい衝突回避制約を追加
        child1.constraints.emplace_back(conflict.agent1,
                                        conflict.location,
                                        conflict.timeStep);
        children.push_back(child1);

        // agent2用の子ノード
        Node child2 = parent;
        child2.root = false;
        // 親ノードの制約を引き継ぐ
        child2.I = parent.I;
        child2.O = parent.O;
        child2.assignment = parent.assignment;
        // 新しい衝突回避制約を追加
        child2.constraints.emplace_back(conflict.agent2,
                                        conflict.location,
                                        conflict.timeStep);
        children.push_back(child2);

        return children;
    }

    void CBSTA::calculateCostMatrix(const std::vector<int> &starts,
                                    const std::vector<int> &goals)
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
    }

    bool CBSTA::validateNode(Node &node,
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
                return false;
            }
        }

        // 割り当てに基づいてパスを生成
        node.solution.resize(starts.size());
        for (const auto &[agent, task] : node.assignment)
        {
            int startLoc = starts[agent];
            int goalLoc = goals[task]; // 割り当てられたタスクのゴール位置を使用
            node.solution[agent] = findPath(graph, startLoc, goalLoc, node.constraints, agent);

            // パスが見つからなかった場合
            if (node.solution[agent].nodes.empty())
            {
                std::cout << "パスが見つからなかった" << std::endl;
                return false;
            }
        }

        // ノードのコストを更新
        node.cost = 0;
        for (const auto &path : node.solution)
        {
            node.cost += path.cost;
        }

        return true;
    }

    bool CBSTA::isTimeOut(const std::chrono::steady_clock::time_point &start_time,
                          int time_limit_ms) const
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           current_time - start_time)
                           .count();
        return elapsed >= time_limit_ms;
    }

}