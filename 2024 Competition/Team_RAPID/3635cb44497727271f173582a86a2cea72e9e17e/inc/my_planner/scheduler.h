#ifndef SCHEDULER
#define SCHEDULER

// #include "Types.h"
#include <limits.h>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>
// #include "Types.h"

#include "SharedEnv.h"
#include "util/HeuristicTable.h"
// #include "heuristics.h"
#include <random>

#include "util/MyLogger.h"

namespace MyPlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, const std::shared_ptr<HeuristicTable> & HT);
}

#endif