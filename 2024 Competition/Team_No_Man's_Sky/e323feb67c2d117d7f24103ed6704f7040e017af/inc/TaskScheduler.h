#pragma once

#include <SharedEnv.h>
#include <Tasks.h>

#include <Scheduler/scheduler.hpp>
#include <settings.hpp>

class TaskScheduler {
public:
    SharedEnvironment *env;
    MyScheduler my_scheduler;

    explicit TaskScheduler(SharedEnvironment *env) : env(env), my_scheduler(env) {

    }

    TaskScheduler() {
        env = new SharedEnvironment();
    }

    virtual ~TaskScheduler() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);
};
