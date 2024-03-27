#include <MAPFPlanner.h>
#include <random>



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    planner.initialize(*env, preprocess_time_limit);
}



void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    planner.plan(time_limit, actions);
}

