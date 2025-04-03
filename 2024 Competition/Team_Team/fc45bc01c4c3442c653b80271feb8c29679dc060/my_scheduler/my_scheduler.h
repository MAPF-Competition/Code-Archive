#ifndef MYSCHEDULER
#define MYSCHEDULER

#include "my_types.h"
#include "SharedEnv.h"
#include "my_heuristics.h"
#include <random>

namespace MyPlanner{

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

    std::tuple<std::unordered_set<int>,std::unordered_set<int>,std::unordered_set<int>, std::unordered_set<int>> update(SharedEnvironment* env);

    void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif