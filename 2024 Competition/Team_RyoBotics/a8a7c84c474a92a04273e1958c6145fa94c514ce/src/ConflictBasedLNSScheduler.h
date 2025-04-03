#pragma once

#include "SharedEnv.h"
#include <vector>
#include <random>
#include "SchedulerUtils.h"
#include "APSPCalculator.h"
namespace CBLNSScheduler
{
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // LNSの破壊・修復に関する関数
    std::vector<int> destroy(const std::vector<int> &current_schedule, double destroy_ratio);
    void repair(std::vector<int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env);
    void generateInitialSchedule(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    void optimizeAssignment(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    // スケジュールの評価用関数
    int evaluateSchedule(const std::vector<int> &schedule, SharedEnvironment *env);
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env);
}