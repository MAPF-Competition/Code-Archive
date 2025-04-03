#include "scheduler.h"



namespace SchedulerC{




std::mt19937 mt;

std::unordered_set<int> free_agents;
std::vector<int> agents_expected_finish;
std::vector<int> agents_finish_loc;

std::unordered_set<int> free_tasks;
std::unordered_set<int> assigned_tasks;

// format: task_id : {agent_id, min_makespan}
std::unordered_map<int, pair<int,int>> min_makespan;   

std::vector<int> tasks_queue;
std::unordered_map<int, int> cost_table;


// For logging the data
std::string file_path;


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    SchedulerC::init_heuristics(env);
    mt.seed(0);
    tasks_queue = vector<int>(env->num_of_agents, -1);
    agents_expected_finish = vector<int>(env->num_of_agents, -1);
    agents_finish_loc = vector<int>(env->num_of_agents, -1);


    // Data log
    file_path = "log.txt";

    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    
    TimePoint starttime = std::chrono::steady_clock::now();
    TimePoint endtime = starttime + std::chrono::milliseconds(time_limit);
    


    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    for (int agent : env->new_freeagents) {
        agents_expected_finish[agent] = - 1;
        agents_finish_loc[agent] = env->curr_states.at(agent).location;
    }



    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    std::unordered_set<int>::iterator it_task = free_tasks.begin();
    // std::unordered_set<Agent>::iterator it_agent = free_agents.begin();
    int task_added = 0;

    while (it_task != free_tasks.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime - std::chrono::milliseconds(350))
        {
            break;
        }

        int task = *it_task;
        int dist_task = 0;
        int c_loc_task = env->task_pool[task].locations[0];
        for (int i = 1; i < env->task_pool[task].locations.size(); i++){
            int next_loc_task = env->task_pool[task].locations[i];
            dist_task += get_h(env, c_loc_task, next_loc_task);
            c_loc_task = next_loc_task;
        }

        cost_table[task] = dist_task;
        it_task++;
        task_added++;
        free_tasks.erase(task);
    }

    for (auto it = min_makespan.begin(); it != min_makespan.end(); ) {
        
        //keep assigning until timeout

        // Check if the task is already opened, remove task from list
        int mm_task_id = it->first;
        int mm_agent_id = it->second.first;


        if (env->task_pool[mm_task_id].idx_next_loc != 0) {
            vector<int> tasks_list = env->task_pool[mm_task_id].locations;

            // Update agents info
            // free_agents.insert(it->second.first);
            agents_expected_finish[mm_agent_id] = cost_table[mm_task_id] + env->curr_timestep;
            agents_finish_loc[mm_agent_id] = tasks_list[tasks_list.size() - 1];
            
            // Remove the current list from free task lisk
            cost_table.erase(mm_task_id);
            it = min_makespan.erase(it); 
        } else {
            // update min makespan
            it->second.second = get_h(env, env->curr_states.at(mm_agent_id).location, env->task_pool[mm_task_id].locations[0]) + cost_table[mm_task_id];
        
            ++it; // Move to the next element
        }
    }
 

    vector<int> unassigned_agents;
    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    // std::unordered_set<int>::iterator it = free_agents.begin();
    int assigned = 0;

    while (!free_agents.empty())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // auto curr_agent = *free_agents.begin();
        int i = *free_agents.begin();
        // auto [agent_loc, expected_finish_time] = curr_agent.second;
        
        bool isFree = agents_expected_finish[i] == -1;

        // assign the task in the queue to the agent if there exist a task
        if (tasks_queue[i] != -1 && proposed_schedule[i] == -1) {
            proposed_schedule[i] = tasks_queue[i];
            tasks_queue[i] = -1;
            free_agents.erase(i);
            continue;
        }

        // assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;


        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i

        for (auto curr : cost_table)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }


            int curr_task_id = curr.first;
            int curr_task_cost = curr.second;
            


            int first_task_loc = env->task_pool[curr_task_id].locations[0];

            dist = get_h(env, agents_finish_loc[i], first_task_loc) + curr_task_cost;
            
            // if the agent is currently on other task, the dist should include the remaining time needed for them to finish their current task
            if (!isFree) dist += max(0, agents_expected_finish[i] - env->curr_timestep);


            // update the new minimum makespan
            if (dist < min_task_makespan){
                if (min_makespan.find(curr_task_id) != min_makespan.end() && dist >= min_makespan[curr_task_id].second) {
                    continue;
                }
                
                min_task_i = curr_task_id;
                min_task_makespan = dist;
            }

            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            
            
            // auto repeated_task = std::find(proposed_schedule.begin(), proposed_schedule.end(), min_task_i);

            // Swap task
            if (min_makespan.find(min_task_i) != min_makespan.end()) {

                int assigned_agent = min_makespan[min_task_i].first;

                // cout << "Replicated assignmnent" << " Assigned Agent: " << assigned_agent << " Current Agent " << i << "\n";
                
                // if the agent is currently on another task
                if (tasks_queue[assigned_agent] != -1) {
                    // cout << "Took task from queue\n" ; 
                    
                    tasks_queue[assigned_agent] = -1;
                    
                } else {
                    proposed_schedule[assigned_agent] = -1;
                }

                free_agents.insert(assigned_agent);
 
            }
            min_makespan[min_task_i] =  {i, min_task_makespan};
            
            
            if (isFree) {
                proposed_schedule[i] = min_task_i;
            } else {
                // cout << "Task added to queue" << "\n";
                tasks_queue[i] = min_task_i;
            }


            free_agents.erase(i);

            assigned ++;

        }
        // nothing to assign
        else{
            
            if (isFree) proposed_schedule[i] = -1;
            free_agents.erase(i);
            unassigned_agents.push_back(i);
        }

        
    }


    for (int unassigned_agent : unassigned_agents) {
        free_agents.insert(unassigned_agent);
    }

    cout << "Number of free agents: " << free_agents.size() << "\n";
    cout << "C: Task added: " << task_added << "\n";
    cout << "C: Assigned Task: " << assigned << "\n";
    cout << "C: Time taken for scheduling: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - starttime).count() << "\n";
    // std::cout << "Time taken for the preprocessing: " << preprocess_time << " seconds\n";
    
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}