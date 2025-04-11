#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "PIBT/PIBT.h"

class MAPFPlanner
{
public:
    SharedEnvironment* env;
    PIBT pibt {env};

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

};
