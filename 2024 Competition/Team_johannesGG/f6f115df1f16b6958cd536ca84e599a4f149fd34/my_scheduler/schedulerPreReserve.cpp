#include "schedulerPreReserve.h"

namespace schedulerPreReserve{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::vector<int> reservedTasks;
int avgAssignDist;
int counterAssignDists;
float numTasksReveal;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    reservedTasks.resize(env->num_of_agents, -1);
    avgAssignDist = 30;
    counterAssignDists = 5;
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{   
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    
    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    //Check agents 700
    // if (env->num_of_agents < 700){
    //     int lul = 12/0;
    //     std::cout << "lul: " << lul << "\n";
    // }

    int allowedReserveDist = 10;
    int dist, a_loc, t_id, t_loc, min_agent_dist, min_i, curr_task_id, last_loc, reserved_t_loc, reserved_task_dist, tmp_t_id, min_task_i, min_task_makespan, c_loc, count, task_dist, start_loc, loc;

    if (env->curr_timestep > 0 && numTasksReveal > 1.4){
    
        std::vector<int> newTasks = env->new_tasks;
        std::vector<int>::iterator t_it = newTasks.begin();

        while (t_it != newTasks.end()){
            if (std::chrono::steady_clock::now() > endtime){break;}

            t_id = *t_it;
            t_loc = env->task_pool[t_id].locations[0];

            min_agent_dist = INT_MAX;
            min_i = -1;
            for (int i = 0; i < env->num_of_agents; i++){

                if (proposed_schedule[i] == -1){
                    a_loc = env->curr_states.at(i).location;
                    dist = DefaultPlanner::get_h(env, a_loc, t_loc);

                    if (dist < min_agent_dist){
                        min_agent_dist = dist;
                        min_i = i;
                    }
                }
                //potentially add case to look in all already assigned agents who didnt start task yet to change task to new task if dist is shorter
                else {
                    curr_task_id = env->curr_task_schedule[i];
                    last_loc = env->task_pool[curr_task_id].locations.back();
                    dist = DefaultPlanner::get_h(env, last_loc, t_loc);
                    
                    if (dist < min_agent_dist){

                        if (reservedTasks[i] != -1){
                            reserved_t_loc = env->task_pool[reservedTasks[i]].locations[0];
                            reserved_task_dist = DefaultPlanner::get_h(env, last_loc, reserved_t_loc);

                            if (reserved_task_dist <= dist){
                                continue;
                            }
                        }
                        min_agent_dist = dist;
                        min_i = i;
                    }
                }
            }
            if (min_i != -1 && min_agent_dist < avgAssignDist/3){
                
                //compare closest new task with all other free tasks
                min_task_i = -1;
                min_task_makespan = INT_MAX;
                if (proposed_schedule[min_i] == -1)     start_loc = env->curr_states.at(min_i).location;
                else                                    start_loc = env->task_pool[proposed_schedule[min_i]].locations.back();
                for (int all_t_id : free_tasks){

                    task_dist = 0;
                    loc = env->task_pool[all_t_id].locations[0];
                    task_dist = DefaultPlanner::get_h(env, start_loc, loc);

                    if (task_dist < min_task_makespan){
                        min_task_i = all_t_id;
                        min_task_makespan = task_dist;
                    }        
                }
                
                if (min_agent_dist <= min_task_makespan){

                    if (proposed_schedule[min_i] == -1){
                        // proposed_schedule[min_i] = t_id;
                        // free_agents.erase(min_i);
                        // t_it = newTasks.erase(t_it);
                        t_it++;
                    }
                    else if (reservedTasks[min_i] == -1){
                        reservedTasks[min_i] = t_id;
                        t_it = newTasks.erase(t_it);
                    }
                    else{
                        tmp_t_id = reservedTasks[min_i];
                        reservedTasks[min_i] = t_id;
                        *t_it = tmp_t_id;
                        t_it++;
                    }
                }
                else{
                    t_it++;
                    if (reservedTasks[min_i] == -1){
                        reservedTasks[min_i] = min_task_i;
                        free_tasks.erase(min_task_i);
                    }
                    else{
                        free_tasks.insert(reservedTasks[min_i]);
                        reservedTasks[min_i] = min_task_i;
                        free_tasks.erase(min_task_i);
                    }
                }
            }
            else{
                t_it++;
            }
        }
        free_tasks.insert(newTasks.begin(), newTasks.end());
    }
    else{
        free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
        numTasksReveal = static_cast<float>(env->task_pool.size())/env->num_of_agents;
    }






   

    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        if (reservedTasks[i] != -1){
            proposed_schedule[i] = reservedTasks[i];
            reservedTasks[i] = -1;
            it = free_agents.erase(it);
        }
        else{
            
            min_task_i = -1;
            min_task_makespan = INT_MAX;
            count = 0;

            // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
            for (int t_id : free_tasks)
            {
                //check for timeout every 10 task evaluations
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                dist = 0;
                c_loc = env->curr_states.at(i).location;

                // iterate over the locations (errands) of the task to compute the makespan to finish the task
                // makespan: the time for the agent to complete all the errands of the task t_id in order
                for (int loc : env->task_pool[t_id].locations){
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }

                // update the new minimum makespan
                if (dist < min_task_makespan){
                    min_task_i = t_id;
                    min_task_makespan = dist;
                }
                count++;            
            }

            // assign the best free task to the agent i (assuming one exists)
            if (min_task_i != -1){
                proposed_schedule[i] = min_task_i;
                it = free_agents.erase(it);
                free_tasks.erase(min_task_i);
                if (env->curr_timestep < 20){
                    avgAssignDist = (counterAssignDists * avgAssignDist + min_task_makespan)/(counterAssignDists + 1);
                    counterAssignDists++;
                }
            }
            // nothing to assign
            else{
                proposed_schedule[i] = -1;
                it++;
            }
        }
    }
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}
