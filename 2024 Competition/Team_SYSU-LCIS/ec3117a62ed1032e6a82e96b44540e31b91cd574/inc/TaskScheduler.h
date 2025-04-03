#pragma once
#include <thread>
#include <future>
#include <thread>
#include <atomic>

#include "Tasks.h"
#include "SharedEnv.h"

#include "scheduler.h"
#include "CUDAScheduler.h"

class TaskScheduler
{
public:
    SharedEnvironment *env;

    TaskScheduler(SharedEnvironment *env) : env(env), cudaScheduler(device_id) {};
    TaskScheduler() : cudaScheduler(device_id) { env = new SharedEnvironment(); };
    virtual ~TaskScheduler()
    {
        delete env;
        delete &cudaScheduler;
    };
    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

private:
    int device_id = 0;
    CUDAScheduler cudaScheduler;
    std::mt19937 mt;
    std::unordered_set<int> free_agents, free_tasks;

    std::thread cuda_thread;
    std::unordered_set<int> cuda_wo_tasks, cuda_wo_agents;

    int state_num;
    std::vector<int> loc_dict, loc_dict_rever;
    std::map<int, int> task_cost_map;
    std::vector<std::vector<int>> global_heuristictable;
    std::vector<int> global_orientation;

    int re_assign_time, re_assign_num, re_assign_threshold;
    std::map<int, int> task_agent_map, agent_task_map, agent_cost_map;
    std::vector<int> pre_schedule;

    int get_h(int source, int target);
    int cal_task_cost(int task_id);
    void task2cuda(SharedEnvironment *env, int t_id);
    void agent2cuda(SharedEnvironment *env, int a_id);
    void init_heuristic(int map_size, DefaultPlanner::HeuristicTable &ht);
    void init_heuristics(SharedEnvironment *env, int preprocess_time_limit);
    void cal_heuristic(int devNum, SharedEnvironment *env, std::vector<int> &cal_loc);
};
