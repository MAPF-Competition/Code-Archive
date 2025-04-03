#ifndef SCHEDULERILP
#define SCHEDULERILP

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "ortools/linear_solver/linear_solver.h"

namespace schedulerILP{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif