#include <random>
#include <Entry.h>

//default planner includes
#include "planner.h"
#include "const.h"

// #include "r_planner.hpp"
#include "r_env.hpp"



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    // DefaultPlanner::initialize(limit, env);
    RHCR_Planner::initialize(limit, env);
    return;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    
    // std::cout << "Origin Time Limit is " << time_limit << std::endl; // 1000 ms
    // std::cout << "Planner Limit is " << limit << std::endl; // 990 ms

    Timer t5;
    // DefaultPlanner::plan(limit, actions, env);
    RHCR_Planner::plan(limit, actions, env);
    t5.stop();
    std::cout << "一次MAPFPlanner::plan耗时: " << t5.elapsedSeconds() <<std::endl;
    return;
}
