#ifndef APLANNER_MAIN_H
#define APLANNER_MAIN_H

#include <random>
#include <cmath>
#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include "SCC.h"

namespace APlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif