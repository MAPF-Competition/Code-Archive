#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "TaskScheduler.h"

#include "CUDAHeuristics.h"
#include "CUDAScheduler.h"
#include "CUDAPlanner.h"

class Entry
{
public:
    SharedEnvironment *env;
    CUDAPlanner *planner;
    CUDAScheduler *scheduler;
    // MAPFPlanner* planner;
    // TaskScheduler* scheduler;

    int cuda_id = 0;
    CUDAHeuristics *cuda_heuristics;

    Entry(SharedEnvironment *env) : env(env)
    {
        cuda_heuristics = new CUDAHeuristics(cuda_id, env);
        scheduler = new CUDAScheduler(env, cuda_heuristics);
        planner = new CUDAPlanner(env, cuda_heuristics);
        // planner = new MAPFPlanner(env);
    };
    Entry()
    {
        env = new SharedEnvironment();
        cuda_heuristics = new CUDAHeuristics(cuda_id, env);
        scheduler = new CUDAScheduler(env, cuda_heuristics);
        planner = new CUDAPlanner(env, cuda_heuristics);
    };
    virtual ~Entry() { delete env; };

    virtual void initialize(int preprocess_time_limit);

    // return next actions and the proposed task schedule for all agents
    virtual void compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule);

    void update_goal_locations(std::vector<int> &proposed_schedule);
};