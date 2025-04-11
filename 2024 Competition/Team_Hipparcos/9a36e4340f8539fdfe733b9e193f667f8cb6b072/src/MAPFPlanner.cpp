#include <chrono>
#include "MAPFPlanner.h"
#include"const.h"

void MAPFPlanner::initialize(int preprocess_time_limit) {
    pibt.initialize();
}

void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
    auto limit {time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now()
         - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE};

    pibt.nextStep(limit, actions);
}
