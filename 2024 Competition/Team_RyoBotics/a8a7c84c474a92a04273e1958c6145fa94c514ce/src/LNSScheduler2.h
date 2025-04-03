#pragma once

#include "SharedEnv.h"
#include <vector>
#include <random>
#include "SchedulerUtils.h"

namespace LNSScheduler2
{
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // LNSの破壊・修復に関する関数
    std::vector<int> destroy(double destroy_ratio,
                             const std::unordered_set<int> &critical_agents,
                             const std::unordered_set<int> &nearby_agents);
    void repair(std::unordered_map<int, int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env, std::unordered_map<int, int> &agent_completion_times_local, std::chrono::steady_clock::time_point end_time);
    std::pair<int, std::unordered_set<int>> generateInitialSchedule(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    void optimizeAssignment(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    // スケジュールの評価用関数をテンプレート化
    template <typename ScheduleContainer>
    std::pair<int, std::unordered_set<int>> evaluateSchedule(const ScheduleContainer &schedule, SharedEnvironment *env, std::unordered_map<int, int> &agent_completion_times_local);
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env, bool include_waypoints = true);

    // Manhattan距離を計算する関数を追加
    int calculateManhattanDistance(int from_x, int from_y, int to_x, int to_y);
    int calculateManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int getManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int countAgentsWithLastErrand(SharedEnvironment *env);

    // 指定された距離以内にいるエージェントの集合を返す関数を追加
    std::vector<int> getNearbyAgents(int agent_id, int distance_threshold, SharedEnvironment *env);
    void updateTaskSwitches(SharedEnvironment *env);

    bool isOverTaskSwitches(int agent_id);
    // タスク切り替え回数を追跡するための変数を追加
    extern std::vector<int> agent_task_switches;
}
