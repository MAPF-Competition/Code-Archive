#ifndef SCHEDULERPRE
#define SCHEDULERPRE

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>

namespace schedulerPreReserve{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif