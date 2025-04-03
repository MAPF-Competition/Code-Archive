#pragma once

#include "TrajLNS.h"
#include "Types.h"
#include <random>


namespace DefaultPlanner {
    void initialize(int preprocess_time_limit, SharedEnvironment *env);

    void plan(int time_limit, vector<Action> &actions, SharedEnvironment *env);
}// namespace DefaultPlanner
