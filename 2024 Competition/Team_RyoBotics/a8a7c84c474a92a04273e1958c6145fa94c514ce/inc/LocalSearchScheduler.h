#pragma once
#include "scheduler.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include "TaskDensityManager.h"
namespace LocalSearchPlanner
{
    // 初期化関数
    void schedule_initialize_LS(int preprocess_time_limit, SharedEnvironment *env);

    // メインのスケジューリング関数
    void schedule_plan_LS(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // 評価関数
    // double evaluateScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env);
    int evaluateAssignmentCost(int agent_id, int task_id, SharedEnvironment *env);

    // スワップ操作の評価
    bool trySwapAssignments(std::vector<int> &schedule,
                            int agent1,
                            int agent2,
                            SharedEnvironment *env,
                            std::unordered_map<int, int> &task_assignments);

    void assignNewTasks(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    void optimizeAssignments(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    // 現在の割り当てが有効かチェック
    bool isValidAssignment(int agent_id, int task_id, const std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // trySwapWithUnassignedTaskの宣言を追加
    bool trySwapWithUnassignedTask(std::vector<int> &schedule,
                                   int agent_id,
                                   SharedEnvironment *env,
                                   std::unordered_map<int, int> &task_assignments);
}