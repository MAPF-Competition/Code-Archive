#include "scheduler.h"


// SchedulerA Enable task swapping within the current time step only
namespace SchedulerA{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
int preprocess_time; 


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    SchedulerA::init_heuristics(env);

    
    auto start = std::chrono::steady_clock::now();
    // TimePoint limit = std::chrono::steady_clock::now() + std::chrono::milliseconds(1800000);
     
    
    for (int source = 0; source < env->map.size(); source++) {
        if (env->map[source] == 0) {
            for (int target = source + 1; target < env->map.size(); target++) {
                if (env->map[target] == 0) {
                    get_h(env, source, target);
                    get_h(env, target, source);
                }
            
            }
        }
        // if (std::chrono::steady_clock::now() > limit)
        // {
        //     break;
        // }
    }

    auto end = std::chrono::steady_clock::now();


    std::chrono::duration<double> duration = end - start;


    
    preprocess_time = duration.count();

    mt.seed(0);

    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    
    
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    std::unordered_map<int, int> min_makespan;
    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    vector<int> unassigned_agents;
    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    int assigned = 0;
    std::unordered_set<int> assigned_tasks;
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
                dist += SchedulerA::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            // if (dist < min_task_makespan){
            //     min_task_i = t_id;
            //     min_task_makespan = dist;
            // }
            if (dist < min_task_makespan){
                if (min_makespan.find(t_id) != min_makespan.end() && dist >= min_makespan[t_id]) {
                    continue;
                }
                
                min_task_i = t_id;
                min_task_makespan = dist;
            }

            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            
            
            auto repeated_task = std::find(proposed_schedule.begin(), proposed_schedule.end(), min_task_i);
            if (repeated_task != proposed_schedule.end()) {

                int assigned_agent = std::distance(proposed_schedule.begin(), repeated_task);

                // cout << "Replicated assignmnent" << " Assigned Agent: " << assigned_agent << " Current Agent " << i << "\n";

                proposed_schedule[assigned_agent] = -1;
                
                //
                free_agents.insert(assigned_agent);
                
                // free_agents.insert(assigned_agent);
            }
            min_makespan[min_task_i] =  min_task_makespan;
            proposed_schedule[i] = min_task_i;

            free_agents.erase(i);
            // it++;
            // free_tasks.erase(min_task_i);
            assigned ++;
            assigned_tasks.insert(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            
            // it++;
            free_agents.erase(i);
            unassigned_agents.push_back(i);
        }

        
    }
    for (int assigned_id : assigned_tasks) {
        free_tasks.erase(assigned_id);
    }

    for (int unassigned_agent : unassigned_agents) {
        free_agents.insert(unassigned_agent);
    }

    // for (int i = 0; i < proposed_schedule.size(); i++) {
    //     // cout << "Agent: " << i << " is assigned to Task " << proposed_schedule[i] << "\n";
        
    //     // if (proposed_schedule[i] != -1) free_agents.erase(i);
    // }

    cout << "Assigned Task: " << assigned << "\n";
    std::cout << "Time taken for the preprocessing: " << preprocess_time << " seconds\n";
    
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}