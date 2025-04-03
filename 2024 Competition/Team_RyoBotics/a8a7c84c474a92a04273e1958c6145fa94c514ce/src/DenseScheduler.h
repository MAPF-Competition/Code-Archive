#pragma once

#include "SharedEnv.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace DenseScheduler
{
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env);
    // 初期化関数
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);

    // メインのスケジューリング関数
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // タスクとエージェントの位置が一致しているかチェックする関数
    bool isAgentAtTaskStart(int agent_id, int task_id, SharedEnvironment *env);

    // エージェントが既にタスクを持っているかチェックする関数
    bool hasAssignedTask(int agent_id, const std::vector<int> &proposed_schedule);

    // タスクの全経路のコストを計算する関数
    int calculateTotalTaskCost(const Task &task, int initial_dir);
}