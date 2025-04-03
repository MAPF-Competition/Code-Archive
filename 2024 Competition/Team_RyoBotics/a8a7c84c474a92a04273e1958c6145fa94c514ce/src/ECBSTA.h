#pragma once

#include <vector>
#include <unordered_set>
#include <limits>
#include <memory>
#include <chrono>
#include "Graph.h"
#include <unordered_map>
#include <queue>
#include <functional>
#include <map>

// SchedulerUtils.hの代わりに直接定義
using Assignment = std::unordered_map<int, int>; // agent -> task

namespace ECBSTA_NS
{
    // 制約の種類を表現する列挙型を追加
    enum class ConstraintType
    {
        VERTEX,
        EDGE
    };

    // 制約を表現する構造体を修正
    struct Constraint
    {
        int agent;
        int location;
        int timeStep;
        ConstraintType type;
        int fromLocation; // エッジ制約の場合の始点（頂点制約の場合は未使用）

        // 頂点制約用のコンストラクタ
        Constraint(int a, int l, int t)
            : agent(a), location(l), timeStep(t), type(ConstraintType::VERTEX), fromLocation(-1) {}

        // エッジ制約用のコンストラクタ
        Constraint(int a, int from, int to, int t)
            : agent(a), location(to), timeStep(t), type(ConstraintType::EDGE), fromLocation(from) {}

        // デフォルトコンストラクタ
        Constraint()
            : agent(-1), location(-1), timeStep(-1), type(ConstraintType::VERTEX), fromLocation(-1) {}

        // 等価比較演算子を修正
        bool operator==(const Constraint &other) const
        {
            if (type != other.type)
                return false;

            if (type == ConstraintType::VERTEX)
            {
                return agent == other.agent &&
                       location == other.location &&
                       timeStep == other.timeStep;
            }
            else
            {
                return agent == other.agent &&
                       location == other.location &&
                       timeStep == other.timeStep &&
                       fromLocation == other.fromLocation;
            }
        }
    };

    // 制約のハッシュ関数とequal_to演算子を定義
    struct ConstraintHash
    {
        size_t operator()(const Constraint &c) const
        {
            size_t hash = std::hash<int>()(c.agent) ^
                          (std::hash<int>()(c.location) << 1) ^
                          (std::hash<int>()(c.timeStep) << 2);

            if (c.type == ConstraintType::EDGE)
            {
                hash ^= (std::hash<int>()(c.fromLocation) << 3);
            }
            return hash;
        }
    };

    struct ConstraintEqual
    {
        bool operator()(const Constraint &a, const Constraint &b) const
        {
            return a == b;
        }
    };

    // ConstraintSetの定義を修正
    struct ConstraintSet
    {
        std::unordered_set<Constraint, ConstraintHash, ConstraintEqual> vertex_constraints;
        std::unordered_set<Constraint, ConstraintHash, ConstraintEqual> edge_constraints;

        bool operator==(const ConstraintSet &other) const
        {
            return vertex_constraints == other.vertex_constraints &&
                   edge_constraints == other.edge_constraints;
        }

        void insert(const Constraint &constraint)
        {
            if (constraint.type == ConstraintType::VERTEX)
            {
                vertex_constraints.insert(constraint);
            }
            else
            {
                edge_constraints.insert(constraint);
            }
        }
    };

    // パスを表現する構造体
    struct Path
    {
        std::vector<int> nodes;
        int cost;
        int f_min;
        Path() : cost(0), f_min(0) {}
    };

    // タスク割り当てのコスト計算関数
    int calculateAssignmentCost(const Assignment &assignment,
                                const std::vector<std::vector<int>> &C);
    int calculateAssignmentCostWithPrint(const Assignment &assignment,
                                         const std::vector<std::vector<int>> &C);
    // ハンガリアン法の実装
    Assignment hungarianMethod(const std::vector<std::vector<int>> &cost_matrix);
    Assignment constrainedAssignment(const std::unordered_set<int> &I,
                                     const std::unordered_set<int> &O,
                                     const std::vector<std::vector<int>> &C);

    // AgentTaskPairの定義を追加
    struct AgentTaskPair
    {
        int agent;
        int task;

        bool operator==(const AgentTaskPair &other) const
        {
            return agent == other.agent && task == other.task;
        }
    };
}

// AgentTaskPairのハッシュ関数
namespace std
{
    template <>
    struct hash<ECBSTA_NS::AgentTaskPair>
    {
        size_t operator()(const ECBSTA_NS::AgentTaskPair &p) const
        {
            return hash<int>()(p.agent) ^ (hash<int>()(p.task) << 1);
        }
    };
}

namespace ECBSTA_NS
{
    // ノードの一意性を表現するための構造体
    struct NodeIdentity
    {
        ConstraintSet constraints;
        Assignment assignment;

        NodeIdentity(const ConstraintSet &c, const Assignment &a)
            : constraints(c), assignment(a) {}

        bool operator==(const NodeIdentity &other) const
        {
            return constraints == other.constraints && assignment == other.assignment;
        }
    };

    // NodeIdentityのハッシュ関数を修正
    struct NodeIdentityHash
    {
        size_t operator()(const NodeIdentity &id) const
        {
            size_t hash = 0;
            // constraintsのハッシュ（順序に依存しない）
            for (const auto &constraint : id.constraints.vertex_constraints)
            {
                hash ^= ConstraintHash()(constraint);
            }
            for (const auto &constraint : id.constraints.edge_constraints)
            {
                hash ^= ConstraintHash()(constraint);
            }
            // assignmentのハッシュ
            for (const auto &[agent, task] : id.assignment)
            {
                hash ^= std::hash<int>()(agent) ^
                        (std::hash<int>()(task) << 1);
            }
            return hash;
        }
    };

    // 衝突の種類を表現する列挙型
    enum class ConflictType
    {
        VERTEX,
        EDGE
    };

    // 衝突を表現する構造体を修正
    struct Conflict
    {
        int agent1;
        int agent2;
        int timeStep;
        ConflictType type;
        int location;     // 頂点衝突の場合の位置、またはエッジ衝突の場合の終点
        int fromLocation; // エッジ衝突の場合の始点（頂点衝突の場合は未使用）

        // 頂点衝突用のコンストラクタ
        Conflict(int a1, int a2, int t, int loc)
            : agent1(a1), agent2(a2), timeStep(t), type(ConflictType::VERTEX),
              location(loc), fromLocation(-1) {}

        // エッジ衝突用のコンストラクタ
        Conflict(int a1, int a2, int t, int from, int to)
            : agent1(a1), agent2(a2), timeStep(t), type(ConflictType::EDGE),
              location(to), fromLocation(from) {}

        // デフォルトコンストラクタ（最も早い衝突の初期化用）
        Conflict()
            : agent1(-1), agent2(-1), timeStep(std::numeric_limits<int>::max()),
              type(ConflictType::VERTEX), location(-1), fromLocation(-1) {}
    };

    // CBSTAのノードを表現する構造体
    struct HighLevelNode
    {
        size_t id;    // ノードの一意なID
        bool is_open; // OPENリストに含まれているか
        ConstraintSet constraints;
        Assignment assignment;
        bool root;
        std::vector<Path> solution;
        int cost;
        int lb;                     // 下界値
        int num_conflicts;          // 衝突数
        Conflict selected_conflict; // 選択された衝突情報を保持
        bool has_selected_conflict; // 衝突が選択されているかどうかのフラグ
        std::vector<Conflict> conflicts;

        // I,Oを(agent,task)ペアで管理
        std::unordered_set<AgentTaskPair> I;
        std::unordered_set<AgentTaskPair> O;

        HighLevelNode() : id(0), is_open(true), root(false),
                          cost(std::numeric_limits<int>::max()), lb(0), num_conflicts(0),
                          has_selected_conflict(false), selected_conflict() {}

        // ノードの同一性を判定するための関数
        NodeIdentity getIdentity() const
        {
            return NodeIdentity(constraints, assignment);
        }
    };

    // PathNodeの状態管理のための構造体
    struct LowLevelNodeState
    {
        int location;
        int time_step;

        bool operator==(const LowLevelNodeState &other) const
        {
            return location == other.location && time_step == other.time_step;
        }
    };

    // PathNodeStateのハッシュ関数
    struct LowLevelNodeStateHash
    {
        size_t operator()(const LowLevelNodeState &state) const
        {
            return std::hash<int>()(state.location) ^
                   (std::hash<int>()(state.time_step) << 1);
        }
    };

    struct LowLevelNode
    {
        size_t id;
        int location;
        int g_score;
        int f_score;
        int time_step;
        bool is_open;
        size_t parent_id;
        int focal_heuristic;
        bool reached_goal; // ゴール到達フラグ
        // デフォルトコンストラクタを追加
        LowLevelNode() : id(0), location(0), g_score(0), f_score(0),
                         time_step(0), is_open(true), parent_id(0), focal_heuristic(0), reached_goal(false) {}

        LowLevelNode(size_t id, int loc, int g, int f, int t, size_t p_id = 0, int focal_heuristic = 0)
            : id(id), location(loc), g_score(g), f_score(f),
              time_step(t), is_open(true), parent_id(p_id), focal_heuristic(focal_heuristic), reached_goal(false) {}

        // priority_queueでの比較
        bool operator>(const LowLevelNode &other) const
        {
            return f_score > other.f_score;
        }
    };

    // Low-level searchのための関数宣言
    Path findPath(const Graph &graph,
                  int start,
                  int goal,
                  const ConstraintSet &constraints,
                  int agent_id);

    // 制約チェック関数
    bool isConstrained(int agent_id,
                       int location,
                       int time_step,
                       int from_location,
                       const ConstraintSet &constraints);

    // ヒューリスティック関数（マンハッタン距離など）
    int calculateHeuristic(int current, int goal, const Graph &graph);

    // パスの再構築
    Path reconstructPath(size_t goal_id,
                         const std::unordered_map<size_t, LowLevelNode> &path_nodes, int f_min = 0);

    // 全エージェントのパスを計算
    std::vector<Path> findIndividualPaths(const Graph &graph,
                                          const std::vector<int> &starts,
                                          const std::vector<int> &goals,
                                          const ConstraintSet &constraints);

    // 衝突検出関連の関数
    struct ConflictInfo
    {
        bool hasConflict;
        std::vector<Conflict> conflicts;
        Conflict earliest_conflict; // 最も早い時間での衝突を保持

        ConflictInfo() : hasConflict(false), earliest_conflict(-1, -1, std::numeric_limits<int>::max(), -1) {}
    };

    // パス間の衝突を検出する関数
    ConflictInfo detectConflicts(const std::vector<Path> &paths);

    // 頂点衝突の検出
    bool detectVertexConflict(const Path &path1, const Path &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts,
                              Conflict &earliest_conflict);

    // エッジ衝突の検出
    bool detectEdgeConflict(const Path &path1, const Path &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts,
                            Conflict &earliest_conflict);

    // パスの最大時間ステップを取得
    int getMaxTimeStep(const std::vector<Path> &paths);

    // タスク割り当て関連の関数
    std::pair<Assignment, std::vector<HighLevelNode>> firstAssignment(const std::vector<std::vector<int>> &C,
                                                                      const std::unordered_set<ECBSTA_NS::AgentTaskPair> &I,
                                                                      const std::unordered_set<ECBSTA_NS::AgentTaskPair> &O);

    // Focal Searchのためのノード
    struct FocalNode : public LowLevelNode
    {
        int conflicts; // 他のパスとの衝突数

        FocalNode(int loc, int g, int f, int t, int c, LowLevelNode *p = nullptr)
            : LowLevelNode(loc, g, f, t, c, p ? p->id : 0), conflicts(c) {}

        // FOCALリストでの比較のため
        bool operator>(const FocalNode &other) const
        {
            return conflicts > other.conflicts;
        }
    };

    // クラス内でComparatorの型を定義
    using NodeComparator = std::function<bool(size_t, size_t)>;

    // High-Level searchのノード要素を表す構造体
    struct HighLevelElement
    {
        size_t id;
        int cost;
        int num_conflicts;

        HighLevelElement(size_t id_, int cost_, int conflicts_)
            : id(id_), cost(cost_), num_conflicts(conflicts_) {}
    };

    // PriorityFIFOQueueの定義を修正
    struct PriorityFIFOQueue
    {
        // Openリスト用のキュー（コストベース）
        std::map<int, std::deque<HighLevelElement>> priority_queues;
        // Focalリスト用のキュー（衝突数とコストの複合キー）
        std::map<std::pair<int, int>, std::deque<HighLevelElement>> priority_queues_focal;
        bool is_focal; // OpenリストかFocalリストかを識別

        // コンストラクタ
        PriorityFIFOQueue(bool is_focal_queue = false)
            : is_focal(is_focal_queue) {}

        void push(const HighLevelElement &element);
        HighLevelElement top() const;
        void pop();
        bool empty() const;
        size_t size() const;
        void clear();
    };

    // OPENとFOCALの型を定義
    using OpenList = PriorityFIFOQueue;
    using FocalList = PriorityFIFOQueue;

    // High-level CBS-TA関連の関数
    class ECBSTA
    {
    public:
        ECBSTA(const Graph &graph, double w = 1.0); // サブオプティマル係数を追加

        HighLevelNode solve(const std::vector<int> &starts,
                            const std::vector<int> &goals,
                            int time_limit_ms,
                            const std::unordered_set<ECBSTA_NS::AgentTaskPair> &I,
                            const std::unordered_set<ECBSTA_NS::AgentTaskPair> &O);

        // 初期化関数を追加
        void initialize();

        // パス表示用の関数を追加
        void printNodePaths(const HighLevelNode &node) const;

    private:
        const Graph &graph;
        std::vector<std::vector<int>> cost_matrix;
        double w;                // サブオプティマル係数
        int min_lb;              // 最小下界値
        double r;                // MinRoot閾値
        double nextRootNodeCost; // MinRoot閾値

        // ノード管理用のメンバ変数を追加
        std::unordered_map<NodeIdentity, size_t, NodeIdentityHash> node_id_map; // ノードの内容からIDを取得
        std::unordered_map<size_t, HighLevelNode *> nodes;                      // IDからノードを取得
        size_t next_node_id;                                                    // 次に割り当てるノードID

        // ノード管理用の新しいメンバ関数
        size_t getOrCreateNodeId(const NodeIdentity &identity);
        HighLevelNode *getOrCreateNode(const NodeIdentity &identity);

        // 新しいメンバ関数
        bool updateMinLBAndR(int new_lb);
        void updateNewMinLBandR(std::priority_queue<size_t, std::vector<size_t>, std::function<bool(size_t, size_t)>> &OPEN);
        void setMinLBAndR(int new_lb);
        bool shouldGenerateNewRoot(OpenList &OPEN);
        HighLevelNode *generateNextRoot();
        int calculateLowerBound(const HighLevelNode &node, const std::vector<std::vector<int>> &C) const;
        // double calculateLowerBoundStdout(const Node &node, const std::vector<std::vector<int>> &C) const;
        // 子ノードの生成
        std::vector<HighLevelNode *> createChildNodes(const HighLevelNode &parent,
                                                      const std::vector<int> &starts,
                                                      const std::vector<int> &goals);

        // コスト行列の計算
        void calculateCostMatrix(const std::vector<int> &starts,
                                 const std::vector<int> &goals,
                                 const std::unordered_set<AgentTaskPair> &I,
                                 const std::unordered_set<AgentTaskPair> &O);

        // ノードの評価
        bool validateNode(HighLevelNode &node,
                          const std::vector<int> &starts,
                          const std::vector<int> &goals);

        // 時間制限のチェック
        bool isTimeOut(const std::chrono::steady_clock::time_point &start_time,
                       int time_limit_ms) const;

        // 新しいメンバ関数
        // Path focalSearch(int start, int goal,
        //                  const std::vector<Constraint> &constraints,
        //                  const std::vector<Path> &other_paths,
        //                  int agent_id);
        Path lowLevelFocalSearch(const Graph &graph,
                                 int start, int goal,
                                 const ConstraintSet &constraints,
                                 const std::vector<Path> &other_paths,
                                 int agent_id);
        int countConflicts(int location, int time_step,
                           const std::vector<Path> &other_paths,
                           int agent_id);
        std::vector<HighLevelNode> generateNextRoots();
        bool isValidAssignment(const Assignment &assignment) const;

        // FOCALリストの更新用関数を修正
        void updateFocalList(OpenList &OPEN, FocalList &FOCAL);
        void updateFocalListRebuild(OpenList &OPEN, FocalList &FOCAL);
        // FOCALに入れるべきノードかチェック
        bool shouldBeInFocal(const HighLevelNode *node, int best_cost) const;

        // 新しい関数を追加
        bool hasNodeIdentity(const NodeIdentity &identity) const;

        // rootノード作成用の関数を追加
        HighLevelNode *createRootNode(const Assignment &initial_assignment, std::vector<int> starts, std::vector<int> goals,
                                      const std::unordered_set<ECBSTA_NS::AgentTaskPair> &I, const std::unordered_set<ECBSTA_NS::AgentTaskPair> &O);

        // 衝突から制約を生成する関数を追加
        std::pair<Constraint, Constraint> createConstraintsFromConflict(const Conflict &conflict);

        void popClosedNodes(OpenList &OPEN);

        // 特定のエージェントのみのパスを再計算する関数を追加
        bool validateNodeForAgent(HighLevelNode &node,
                                  const std::vector<int> &starts,
                                  const std::vector<int> &goals,
                                  int agent_id);
    };
}
