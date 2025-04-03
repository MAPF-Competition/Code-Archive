#pragma once
#include "Tasks.h"
#include "SharedEnv.h"


class TaskScheduler
{
    public:
        SharedEnvironment* env;
        
        TaskScheduler(SharedEnvironment* env): env(env){};
        TaskScheduler(){env = new SharedEnvironment();};
        virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);
        bool map_is_random = false, map_is_warehouse = false, map_is_game = false, map_is_city = false, map_is_sortation = false;
};