#ifndef SCHEDULER_UTILS_GAME_H
#define SCHEDULER_UTILS_GAME_H

#include "SharedEnv.h"
#include "distance_table.h"
#include <vector>
#include <unordered_map>
#include <set>
#include <random>
#include "ParallelAPSPCalculatorGame.h"
#include "planner_access.h"
// using namespace DefaultPlanner;
namespace SchedulerUtilsGame
{

    // 共通の変数
    extern DefaultPlanner::FastDistanceTable distance_table;
    extern std::unordered_set<int> global_available_tasks;
    extern std::unordered_set<unsigned short> global_available_agents;
    extern std::unordered_map<int, int> current_task_assignments;
    extern DirectionalPathCostMap apsp_table;
    extern std::vector<int> agent_available_time;
    extern std::vector<std::pair<int, int>> agent_end_state;
    extern int task_free;
    extern int upper1;
    extern std::string map_name;
    // タスクは割り当てられているがまだopenしていないエージェントの集合
    extern std::unordered_set<unsigned short> assigned_but_not_open_agents;
    void schedule_initialize_SchedulerUtils(int preprocess_time_limit, SharedEnvironment *env);
    // 共通の関数
    int evaluateDistance(int agent_id, int task_id, SharedEnvironment *env);
    std::tuple<double, int> evaluateScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env);
    bool validateSchedule(const std::vector<int> &proposed_schedule, SharedEnvironment *env);
    int getMinDirectionalCost(int from_loc, int from_dir, int to_loc);
    int getNextDirection(int from_loc, int to_loc);
    int countAllAgentConflicts(const std::vector<std::vector<State>> &paths);
    int countAgentPathConflicts(int location, int time_step,
                                const std::vector<std::vector<State>> &other_paths,
                                int agent_id);
    bool detectVertexConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts);
    bool detectEdgeConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts);
    int getCostToLocation(int fromLoc, int fromDir, int toLoc);
    // データ更新用の関数
    void updateRunningTasks(SharedEnvironment *env);
    void updateTaskAssignments(SharedEnvironment *env);
    void updateAgents(SharedEnvironment *env);
    void updateOnGoingTasks(SharedEnvironment *env);
    int evaluateCost(int agent_id, int task_id, SharedEnvironment *env);
    using Assignment = std::unordered_map<int, int>; // agent -> task
    // 衝突を表現する構造体を修正
    enum ConflictType
    {
        VERTEX,
        EDGE
    };
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

    // ハンガリアン法関連の関数

    // 新しい関数の宣言を追加
    int getDistance(int from_location, int to_location);

    // 現在の位置と方向から目標位置までの最小コストとその時の方向を返す関数
    std::pair<int, int> getMinCostAndDirection(int current_loc, int current_dir, int to_loc);

    // パス構築関数の宣言を追加
    std::vector<State> constructPath(int from_loc, int from_dir, int to_loc);

    // タスクの総経路コストを保持するテーブル
    extern std::unordered_map<int, int> task_total_cost_table;

    // タスクの総経路コストを計算して更新する関数
    void updateTaskTotalCosts(SharedEnvironment *env);

} // namespace SchedulerUtils

#endif // SCHEDULER_UTILS_GAME_H