#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "planner.h"
#include <memory>

class MAPFPlanner
{
public:
    SharedEnvironment* env;

#ifndef TrafficFlow
    std::shared_ptr<MyPlanner::WPPLPlanner> wppl_planner;
#endif

	MAPFPlanner(SharedEnvironment* env): env(env){
#ifndef TrafficFlow
        wppl_planner = std::make_shared<MyPlanner::WPPLPlanner>(env);
#endif
    };
    MAPFPlanner(){
        env = new SharedEnvironment();
#ifndef TrafficFlow
        wppl_planner = std::make_shared<MyPlanner::WPPLPlanner>(env);
#endif
    };
	virtual ~MAPFPlanner(){
        // delete env;
    };


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

};
