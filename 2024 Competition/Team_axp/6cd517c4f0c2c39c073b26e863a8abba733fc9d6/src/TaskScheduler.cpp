#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

#define DEBUG true

#if DEBUG
    #define CHECK(expr) \
    auto v = expr; \
    if (!v) { \
        std::cerr << "Fatal: " << #expr << " with value: " << v << std::endl; \
        exit(1); \
    }
#else
    #define CHECK(expr)
#endif


/*
void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);    
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
}
*/

void TaskScheduler::initialize(int preprocess_time_limit) {  
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env); 
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule) {
    proposed_schedule.resize(env->num_of_agents, -1);
    for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++) {
        if (env->curr_task_schedule[agent_id] != -1) {
            proposed_schedule[agent_id] = env->curr_task_schedule[agent_id];
            continue;
        }
        
        const auto& agent_loc = env->curr_states[agent_id].location;

        int min_task_id = -1;
        int min_dis = INT_MAX;
        for (auto& id_and_task : env->task_pool) {
            auto& task = id_and_task.second;
            if (task.agent_assigned != -1) {
                continue;
            }
            CHECK(!task.locations.empty());
            int dis = DefaultPlanner::get_h(env, agent_loc, task.locations[0]);
            if (dis < min_dis) {
                min_dis = dis;
                min_task_id = id_and_task.first;
            }
        }

        if (min_task_id != -1) {
            proposed_schedule[agent_id] = env->task_pool[min_task_id].task_id;
            env->task_pool[min_task_id].agent_assigned = agent_id;
        }
    }
}
