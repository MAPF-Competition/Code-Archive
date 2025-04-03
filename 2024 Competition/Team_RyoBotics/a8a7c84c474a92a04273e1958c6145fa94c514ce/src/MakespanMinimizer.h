#pragma once

#include "SharedEnv.h"
#include <vector>
#include <unordered_map>
#include "SchedulerUtils.h"

namespace MakespanMinimizer
{
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);
    bool improve2Opt(std::vector<int> &schedule, SharedEnvironment *env);
    void assignNewTasks(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env);

    // makespanと最大completion timeを持つagent idを返す構造体
    struct MakespanResult
    {
        int makespan;
        int max_agent_id;
    };

    MakespanResult calculateMakespan(const std::vector<int> &schedule, SharedEnvironment *env, bool debug = false);
}