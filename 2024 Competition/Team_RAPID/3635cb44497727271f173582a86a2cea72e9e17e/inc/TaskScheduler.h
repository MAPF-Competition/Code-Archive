#pragma once
#include "Tasks.h"
#include "SharedEnv.h"
#include "util/HeuristicTable.h"
#include "util/MyCommon.h"

class TaskScheduler
{
    public:
        SharedEnvironment* env;
        std::shared_ptr<HeuristicTable> HT; // instance
        std::shared_ptr<std::vector<float> > map_weights; // map weights
        nlohmann::json config;

        TaskScheduler(SharedEnvironment* env): env(env){};
        TaskScheduler(){env = new SharedEnvironment();};
        virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);
        std::string load_map_weights(std::string weights_path);
        void load_configs();
};