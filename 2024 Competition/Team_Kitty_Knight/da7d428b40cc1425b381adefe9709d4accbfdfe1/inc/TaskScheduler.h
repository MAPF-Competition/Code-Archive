#pragma once
#include "Tasks.h"
#include "SharedEnv.h"
#include "scheduler.h"
#include "planner.h"

class TaskScheduler
{
    public:
        SharedEnvironment* env;
#ifndef TrafficFlow
        std::shared_ptr<MyPlanner::MyScheduler> my_scheduler;
#endif

#ifndef TrafficFlow
        TaskScheduler(SharedEnvironment* env, std::shared_ptr<MyPlanner::WPPLPlanner> wppl_planner): env(env){
            my_scheduler = std::make_shared<MyPlanner::MyScheduler>(wppl_planner);
        };
        TaskScheduler(std::shared_ptr<MyPlanner::WPPLPlanner> wppl_planner){
            env = new SharedEnvironment();
            my_scheduler = std::make_shared<MyPlanner::MyScheduler>(wppl_planner);
        };

#else
        TaskScheduler(SharedEnvironment* env): env(env){
        
        };
        TaskScheduler(){
            env = new SharedEnvironment();
        };
#endif

        // virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);
};