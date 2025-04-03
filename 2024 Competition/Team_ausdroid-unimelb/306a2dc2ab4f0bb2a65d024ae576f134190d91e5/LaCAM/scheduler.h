#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>
#include <utility>


namespace LaCAMPlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

std::pair<double, int> calculate_makespan(std::vector<int> & proposed_schedule, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif