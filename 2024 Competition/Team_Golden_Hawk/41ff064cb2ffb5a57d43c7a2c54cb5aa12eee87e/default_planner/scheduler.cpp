#include "scheduler.h"

namespace DefaultPlanner{

std::mt19937 mt;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
}

// Priorities agent over task
// Only consider the pickup location for distance heuristic
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule

    int i_task, min_agent_i, min_agent_makespan, dist, c_loc, count;
    clock_t start = clock();
    for(i_task=0 ; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime ; i_task++)
    {
        if(env->task_pool[i_task].agent_assigned != -1){
            proposed_schedule[env->task_pool[i_task].agent_assigned] = env->curr_task_schedule[env->task_pool[i_task].agent_assigned];
            continue;
        }
        
        min_agent_i = -1;
        min_agent_makespan = INT_MAX;
        for(int i_agent = 0 ; i_agent < env->num_of_agents && std::chrono::steady_clock::now() < endtime ; i_agent++)
        {
            if(env->curr_task_schedule[i_agent] == -1)
            {
                // count = 0;

                dist = 0;
                c_loc = env->curr_states.at(i_agent).location;
                int loc = env->task_pool[i_task].locations[0];
                dist = DefaultPlanner::get_h(env, c_loc, loc);
                if (dist < min_agent_makespan)
                {
                    min_agent_i = i_agent;
                    min_agent_makespan = dist;
                }
            }
            else
            {
                proposed_schedule[i_agent] = env->curr_task_schedule[i_agent];
            }
        }
        if(min_agent_i != -1)
        {
            proposed_schedule[min_agent_i] = env->task_pool[i_task].task_id;
            env->task_pool[i_task].agent_assigned = min_agent_i;
        }
        else
        {
            proposed_schedule[min_agent_i] = -1;
        }
    }
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}
}