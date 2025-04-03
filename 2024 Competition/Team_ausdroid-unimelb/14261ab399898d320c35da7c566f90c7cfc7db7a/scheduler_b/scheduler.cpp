#include "scheduler.h"


// SchedulerB Enable task swapping when task not opened yet
namespace SchedulerB{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_set<int> assigned_tasks;
// format: task_id : {agent_id, min_makespan}
std::unordered_map<int, pair<int,int>> min_makespan;   

// int preprocess_time;



void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    SchedulerB::init_heuristics(env);

    
    auto start = std::chrono::steady_clock::now();
    TimePoint limit = std::chrono::steady_clock::now() + std::chrono::milliseconds(1800000);
     
    
    for (int source = 0; source < env->map.size(); source++) {
        std::cout << "Visiting index: " << source << "\n";
        
        if (env->map[source] == 0) {
            for (int target = env->map.size() - 1; target > source; target--) {
                if (env->map[target] == 0) {
                    get_h(env, source, target);
                    get_h(env, target, source);
                }
            }
        }
        
        if (std::chrono::steady_clock::now() > limit)
        {
            break;
        }
    }

    auto end = std::chrono::steady_clock::now();


    // std::chrono::duration<double> duration = end - start;

    // preprocess_time = duration.count();

    mt.seed(0);

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
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());



    for (auto it = min_makespan.begin(); it != min_makespan.end(); ) {
        
        // Check if the task is already opened
        if (env->task_pool[it->first].idx_next_loc != 0) {
            free_tasks.erase(it->first);
            free_agents.erase(it->second.first);
            it = min_makespan.erase(it); 
        } else {
            it->second.second -= 1;
            ++it; // Move to the next element
        }
    }
 

    vector<int> unassigned_agents;
    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    int assigned = 0;

    while (!free_agents.empty())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *free_agents.begin();

        assert(env->curr_task_schedule[i] == -1);
            
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
                dist += SchedulerB::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                if (min_makespan.find(t_id) != min_makespan.end() && dist >= min_makespan[t_id].second) {
                    continue;
                }
                
                min_task_i = t_id;
                min_task_makespan = dist;
            }

            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            
            
            // auto repeated_task = std::find(proposed_schedule.begin(), proposed_schedule.end(), min_task_i);
            if (min_makespan.find(min_task_i) != min_makespan.end()) {

                int assigned_agent = min_makespan[min_task_i].first;

                // cout << "Replicated assignmnent" << " Assigned Agent: " << assigned_agent << " Current Agent " << i << "\n";

                proposed_schedule[assigned_agent] = -1;
                
                //
                free_agents.insert(assigned_agent);
                
                // free_agents.insert(assigned_agent);
            }
            min_makespan[min_task_i] =  {i, min_task_makespan};
            proposed_schedule[i] = min_task_i;

            free_agents.erase(i);

            assigned ++;

        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            

            free_agents.erase(i);
            unassigned_agents.push_back(i);
        }

        
    }


    for (int unassigned_agent : unassigned_agents) {
        free_agents.insert(unassigned_agent);
    }


    cout << "Number of free agents: " << free_agents.size() << "\n";
    cout << "B: Assigned Task: " << assigned << "\n";
    // std::cout << "Time taken for the preprocessing: " << preprocess_time << " seconds\n";
    
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}