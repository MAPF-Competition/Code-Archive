#include <random>
#include <Entry.h>

//default planner includes
//#include "wppl_planner.h"
#include <chrono>
#include "planner.h"
#include "const.h"

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 * 
 * This function call the planner initialize function with a time limit fore preprocessing.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit)
{

#ifndef TrafficFlow
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    wppl_planner->initialize(limit);
#else
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit,env);
#endif
    return;
}

/**
 * Plans a path using default planner
 * 
 * This function performs path planning within the timelimit given, and call the plan function in default planner.
 * The planned actions are output to the provided actions vector.
 * 
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{


#ifndef TrafficFlow
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    
    // this code makes result worse, timeout more frequently???
    // // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    // int limit = time_limit - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count();
    
    // // if the scheduler is already timeout, let us padding some time to make full use of this step.
    // while (limit<0) {
    //     limit+=1000;
    // }

    // limit-=MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    // std::cout<<"planner time limit: "<< limit<<std::endl;
    
    wppl_planner->plan(limit, actions);
#else
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::plan(limit, actions, env);
#endif

    return;
}
