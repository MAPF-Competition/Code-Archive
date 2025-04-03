#ifndef SCHEDULER
#define SCHEDULER

#include "SharedEnv.h"
#include "Types.h"
#include "heuristics.h"
#include <random>

namespace DefaultPlanner {

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

}// namespace DefaultPlanner

#endif