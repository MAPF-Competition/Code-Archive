#ifndef CITY_SCHEDULER
#define CITY_SCHEDULER

#include <limits.h>
#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>
#include <queue>
#include "SharedEnv.h"
#include <random>

namespace CityScheduler{
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
    void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif