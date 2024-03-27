#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "ash/astar_planner.hpp"
#include "ash/push_planner.hpp"
#include "ash/hybrid_planner.hpp"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

private:
    //typedef ash::AstarPlanner Planner;
    //typedef ash::PushPlanner Planner;
    typedef ash::HybridPlanner Planner;

    Planner planner;
};
