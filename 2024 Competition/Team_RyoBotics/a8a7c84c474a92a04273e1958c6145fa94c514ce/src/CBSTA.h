#pragma once

#include <vector>
#include <unordered_set>
#include <limits>
#include <memory>
#include <chrono>
#include "Graph.h"
#include <unordered_map>

// SchedulerUtils.hの代わりに直接定義
using Assignment = std::unordered_map<int, int>; // agent -> task

namespace CBSTA_NS
{
    // 制約を表現する構造体
    struct Constraint
    {
        int agent;
        int location;
        int timeStep;

        Constraint(int a, int l, int t) : agent(a), location(l), timeStep(t) {}
    };

    // パスを表現する構造体
    struct Path
    {
        std::vector<int> nodes;
        int cost;

        Path() : cost(0) {}
    };

    // タスク割り当てのコスト計算関数
    int calculateAssignmentCost(const Assignment &assignment,
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
    struct hash<CBSTA_NS::AgentTaskPair>
    {
        size_t operator()(const CBSTA_NS::AgentTaskPair &p) const
        {
            return hash<int>()(p.agent) ^ (hash<int>()(p.task) << 1);
        }
    };
}

namespace CBSTA_NS
{
    // CBSTAのノードを表現する構造体
    struct Node
    {
        std::vector<Constraint> constraints;
        Assignment assignment;
        bool root;
        std::vector<Path> solution;
        int cost;

        // I,Oを(agent,task)ペアで管理するように変更
        std::unordered_set<AgentTaskPair> I; // 強制割り当て
        std::unordered_set<AgentTaskPair> O; // 割り当て禁止

        Node() : root(false), cost(std::numeric_limits<int>::max()) {}
    };

    // 衝突を表現する構造体
    struct Conflict
    {
        int agent1;
        int agent2;
        int timeStep;
        int location;

        Conflict(int a1, int a2, int t, int l)
            : agent1(a1), agent2(a2), timeStep(t), location(l) {}
    };

    // Low-level searchのための関数宣言
    struct PathNode
    {
        int location;
        int g_score; // 開始点からのコスト
        int f_score; // g_score + ヒューリスティック
        int time_step;
        PathNode *parent;

        PathNode(int loc, int g, int f, int t, PathNode *p = nullptr)
            : location(loc), g_score(g), f_score(f), time_step(t), parent(p) {}

        // priority_queueでの比較のため
        bool operator>(const PathNode &other) const
        {
            return f_score > other.f_score;
        }
    };

    // Low-level search関連の関数
    Path findPath(const Graph &graph,
                  int start,
                  int goal,
                  const std::vector<Constraint> &constraints,
                  int agent_id);

    // 制約チェック関数
    bool isConstrained(int agent_id,
                       int location,
                       int time_step,
                       const std::vector<Constraint> &constraints);

    // ヒューリスティック関数（マンハッタン距離など）
    int calculateHeuristic(int current, int goal, const Graph &graph);

    // パスの再構築
    Path reconstructPath(PathNode *goal_node);

    // 全エージェントのパスを計算
    std::vector<Path> findIndividualPaths(const Graph &graph,
                                          const std::vector<int> &starts,
                                          const std::vector<int> &goals,
                                          const std::vector<Constraint> &constraints);

    // 衝突検出関連の関数
    struct ConflictInfo
    {
        bool hasConflict;
        std::vector<Conflict> conflicts;

        ConflictInfo() : hasConflict(false) {}
    };

    // パス間の衝突を検出する関数
    ConflictInfo detectConflicts(const std::vector<Path> &paths);

    // 頂点衝突の検出
    bool detectVertexConflict(const Path &path1, const Path &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts);

    // エッジ衝突の検出
    bool detectEdgeConflict(const Path &path1, const Path &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts);

    // パスの最大時間ステップを取得
    int getMaxTimeStep(const std::vector<Path> &paths);

    // タスク割り当て関連の関数
    std::pair<Assignment, std::vector<Node>> firstAssignment(const std::vector<std::vector<int>> &C);
    std::pair<Assignment, std::vector<Node>> nextAssignment(const std::vector<std::vector<int>> &C,
                                                            std::vector<Node> &ASG_OPEN);

    // High-level CBS-TA関連の関数
    class CBSTA
    {
    public:
        CBSTA(const Graph &graph);

        // メインのソルバー関数を更新
        std::vector<Path> solve(const std::vector<int> &starts,
                                const std::vector<int> &goals,
                                int time_limit_ms,
                                const std::unordered_set<int> &I,  // 強制割り当て
                                const std::unordered_set<int> &O); // 割り当て禁止

    private:
        const Graph &graph;
        std::vector<std::vector<int>> cost_matrix;

        // 子ノードの生成
        std::vector<Node> createChildNodes(const Node &parent,
                                           const Conflict &conflict);

        // コスト行列の計算
        void calculateCostMatrix(const std::vector<int> &starts,
                                 const std::vector<int> &goals);

        // ノードの評価
        bool validateNode(Node &node,
                          const std::vector<int> &starts,
                          const std::vector<int> &goals);

        // 時間制限のチェック
        bool isTimeOut(const std::chrono::steady_clock::time_point &start_time,
                       int time_limit_ms) const;
    };

}