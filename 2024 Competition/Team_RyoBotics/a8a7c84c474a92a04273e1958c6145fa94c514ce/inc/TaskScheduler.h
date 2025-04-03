#pragma once
#include "Tasks.h"
#include "SharedEnv.h"
// #include "MultiTPTSScheduler.h"

class TaskScheduler
{
private:
    // MultiTPTSPlanner::ThreadSafeScheduler scheduler;

public:
    SharedEnvironment *env;

    TaskScheduler(SharedEnvironment *env) : env(env) {};
    TaskScheduler() { env = new SharedEnvironment(); };
    virtual ~TaskScheduler() { delete env; };
    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);
};