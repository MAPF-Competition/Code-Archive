#ifndef SCHEDULER_UTILS_H
#define SCHEDULER_UTILS_H

#include "SharedEnv.h"
#include "distance_table.h"
#include <vector>
#include <unordered_map>
#include <set>
#include <random>
#include "ParallelAPSPCalculator.h"
#include "TrajLNS.h"
#include "planner_access.h"
#include "CommonTypes.h"
// using namespace DefaultPlanner;
namespace SchedulerUtils
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
    extern DefaultPlanner::TrajLNS trajLNS_copy;
    extern std::unordered_map<int, DefaultPlanner::TrajLNS> trajLNS_copies;
    // タスクは割り当てられているがまだopenしていないエージェントの集合
    extern std::unordered_set<unsigned short> assigned_but_not_open_agents;
    extern std::unordered_set<unsigned short> unassigned_agents;
    extern std::unordered_set<int> unassigned_tasks;
    extern bool include_tabu_cost;
    extern int INITIAL_TIMESTEP_THRESHOLD;
    extern int tabu_cost_multiplier;
    extern std::unordered_set<unsigned short> must_assign_agents;
    // 動的に切り替えられるようにするためのラッパークラスを定義
    template <typename T>
    struct DynamicSetWrapper
    {
        std::unordered_set<T> *ptr;
        // 範囲 for ループ対応の begin()/end() を提供
        auto begin() const { return ptr->begin(); }
        auto end() const { return ptr->end(); }
        // 内部コンテナのサイズを返す関数を追加
        auto size() const { return ptr->size(); }
        // 普通の参照としても利用できるように変換演算子を定義（必要に応じて）
        operator std::unordered_set<T> &() const { return *ptr; }
    };

    extern DynamicSetWrapper<unsigned short> scheduling_target_agents;
    extern DynamicSetWrapper<int> scheduling_target_tasks;

    // 参照先を切り替える関数（ラッパー内のポインタを更新）
    void setSchedulingTargetAgents(std::unordered_set<unsigned short> &target_set);
    void setSchedulingTargetTasks(std::unordered_set<int> &target_set);
    void schedule_initialize_SchedulerUtils(int preprocess_time_limit, SharedEnvironment *env);
    // 共通の関数
    CostType evaluateDistance(int agent_id, int task_id, SharedEnvironment *env);
    std::tuple<double, int> evaluateScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env);
    bool validateSchedule(const std::vector<int> &proposed_schedule, SharedEnvironment *env);
    CostType getMinDirectionalCost(int from_loc, int from_dir, int to_loc);
    CostType getNextDirection(int from_loc, int to_loc);
    int countAllAgentConflicts(const std::vector<std::vector<State>> &paths);
    int calculateOppositeFlowCost(int agent_id, int task_id, SharedEnvironment *env);
    void remove_traj(int agent, DefaultPlanner::TrajLNS &trajLNS);
    void add_traj(int agent);
    void updateTrajLNS();
    int getVisitedTabuStates(int from_loc, int from_dir, int to_loc, int to_dir);
    int countAgentPathConflicts(int location, int time_step,
                                const std::vector<std::vector<State>> &other_paths,
                                int agent_id);
    bool detectVertexConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                              int agent1, int agent2,
                              std::vector<Conflict> &conflicts);
    bool detectEdgeConflict(const std::vector<State> &path1, const std::vector<State> &path2,
                            int agent1, int agent2,
                            std::vector<Conflict> &conflicts);
    CostType getCostToLocation(int fromLoc, int fromDir, int toLoc);
    WeightedCostType getWeightedCostFromStateToLocation(int fromLoc, int fromDir, int toLoc);
    WeightedCostType getWeightedCostFromLocationToLocation(int fromLoc, int toLoc);
    // データ更新用の関数
    void updateRunningTasks(SharedEnvironment *env);
    void updateTaskAssignments(SharedEnvironment *env);
    void updateAgents(SharedEnvironment *env);
    void updateOnGoingTasks(SharedEnvironment *env);
    CostType evaluateCost(int agent_id, int task_id, SharedEnvironment *env);

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
    CostType getDistance(int from_location, int to_location);

    // 現在の位置と方向から目標位置までの最小コストとその時の方向を返す関数
    std::pair<CostType, int> getMinCostAndDirection(int current_loc, int current_dir, int to_loc);

    // パス構築関数の宣言を追加
    std::vector<State> constructPath(int from_loc, int from_dir, int to_loc, int curr_timestep = 0);

    // タスクの総経路コストを保持するテーブル
    extern std::unordered_map<int, WeightedCostType> task_total_cost_table;

    // タスクの総経路コストを計算して更新する関数
    void updateTaskTotalCosts(SharedEnvironment *env);

    // 指定された位置がdeadendかどうかを確認する関数
    bool isDeadEnd(int loc, SharedEnvironment *env);

    // マップ全体のdeadendを検出する関数
    std::unordered_set<unsigned short> findAllDeadEnds(SharedEnvironment *env);

    // 指定された位置がdeadendかどうかを確認する関数
    bool isDeadEndLocation(unsigned short location);

    // タスクのerrandsにデッドエンドが含まれているかを確認する関数
    bool hasDeadEndInErrand(int task_id, SharedEnvironment *env);

    // タスクのerrandsに含まれるdeadendの数を返す関数
    int countDeadEndsInErrand(int task_id, SharedEnvironment *env);

    // スケジューラのパラメータ
    extern int TIME_LIMIT_DIVISION_FACTOR; // time_limitを分割する係数
    extern int SCHEDULE_UPDATE_INTERVAL;   // スケジュール更新の間隔

    // JSONファイルに設定を保存する関数を追加
    void saveSettingsToJson(const std::string &filename, SharedEnvironment *env);

    // JSONファイルから設定を読み込む関数を追加
    void loadSettingsFromJson(const std::string &filename, SharedEnvironment *env);

    // trajLNS_copiesの作成を行う関数
    void createTrajLNSCopy(int agent_id);

    // 実行不可能なタスクを除外する関数
    void removeUnfeasibleTasks(int remaining_time, SharedEnvironment *env);

    extern std::unordered_set<int> tabu_locs;

} // namespace SchedulerUtils

#endif // SCHEDULER_UTILS_H