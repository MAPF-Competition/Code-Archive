#ifndef TPTSSCHEDULER
#define TPTSSCHEDULER
#pragma once
#include "SharedEnv.h"
#include "distance_table.h"
#include <vector>
#include <unordered_map>
#include <set>
#include <random>
#include "PathFinder.h"

namespace DefaultPlanner
{
    // extern FastDistanceTable distance_table;
    // extern std::mt19937 mt3;

    void schedule_initialize_TPTS(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan_TPTS(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // TPTSの補助関数
    bool GetTask(int agent_id, std::vector<int> &schedule, SharedEnvironment *env,
                 std::unordered_set<int> &available_tasks, std::unordered_set<int> &processed_agents,
                 std::chrono::steady_clock::time_point endtime, int depth = 0, int cost_threshold = -1);

    int calculateDirectionCost(int from_loc, int to_loc, int current_orientation, int map_width);
    // int evaluateDistance(int agent_id, int task_id, SharedEnvironment *env);

    // 新しい関数宣言
    // extern std::unordered_set<int> global_available_tasks;
    // // extern std::unordered_set<int> global_ongoing_agents;
    // extern std::unordered_set<int> global_available_agents;
    // extern FastDistanceTable distance_table;
    // extern std::unordered_map<int, int> current_task_assignments;

    // void updateRunningTasks(SharedEnvironment *env);
    // void updateTaskAssignments(SharedEnvironment *env);
    // void updateAgentsOld(SharedEnvironment *env, std::vector<int> &agent_indices);
    // void updateAgents(SharedEnvironment *env);
}
#endif
