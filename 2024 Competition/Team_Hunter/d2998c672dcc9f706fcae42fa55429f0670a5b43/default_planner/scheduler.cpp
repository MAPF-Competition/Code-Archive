#include "scheduler.h"
#include <queue>
#include <vector>
#include <functional>
#include <utility>

namespace DefaultPlanner{

std::mt19937 mt;
typedef pair<int, int> pi;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule

    clock_t start = clock();
    int i_task, min_task_i, min_task_makespan, dist, c_loc, count, max_task_id;
    max_task_id = 0;
    for (int i = 0; i < env->task_pool.size(); ++i)
    {
        if(env->task_pool[i].task_id > max_task_id)
            max_task_id = env->task_pool[i].task_id;
    }
    // cout << "num of agents " << env->num_of_agents << " num of tasks " << env->task_pool.size() << " max task id " << max_task_id << endl;
    std::vector<priority_queue<pi, std::vector<pi>, std::greater<pi> >> pqVector(env->num_of_agents);
    std::vector<pi> possible_assignments(max_task_id+1, make_pair(-1,-1)); // Distance, Robot
    for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
    {
        if (env->curr_task_schedule[i] == -1)
        {
            for (i_task=0 ; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime ;i_task++)
            {
                if (env->task_pool[i_task].agent_assigned != -1){
                    // dist_matrix[i][i_task] = -1;
                    continue;
                }
                dist = 0;
                c_loc = env->curr_states.at(i).location;
                int loc = env->task_pool[i_task].locations.at(0);
                dist = DefaultPlanner::get_h(env, c_loc, loc);
                pqVector[i].push(make_pair(dist, env->task_pool[i_task].task_id));
            }
        }
        else
        {
            // proposed_schedule[i] = env->curr_task_schedule[i]; 
            // cout << "Already assigned previously " << i << " " << env->curr_task_schedule[i] << endl;
            possible_assignments[env->curr_task_schedule[i]] = make_pair(0, i);
        }
    }
    for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++) { 
        if (env->curr_task_schedule[i] == -1)
        {
            pair<int, int> top = pqVector[i].top();
            // cout << "Top of pqVector " << i << " : " << top.first << " " << top.second << endl; // Robot, Distance, Task
            bool currently_assigned = 0;
            while(currently_assigned == 0 && !pqVector[i].empty() && std::chrono::steady_clock::now() < endtime)
            {
                if(possible_assignments[top.second] == make_pair(-1,-1))
                {
                    possible_assignments[top.second] = make_pair(top.first,i);
                    // cout << "Assigned robot " << i << " to task " << top.second << " with distance " << top.first << endl;
                    currently_assigned = 1;
                }
                else if(top.first < possible_assignments[top.second].first)
                {
                    int unassigned_robot = possible_assignments[top.second].second;
                    // cout << "Removed robot " << unassigned_robot << " from task " << top.second << " with distance " << possible_assignments[top.second].first << endl;
                    possible_assignments[top.second] = make_pair(top.first,i);
                    // cout << "Assigned robot " << i << " to task " << top.second << " with distance " << top.first << endl;
                    currently_assigned = 1;
                    while(unassigned_robot > 0 && !pqVector[unassigned_robot].empty() && std::chrono::steady_clock::now() < endtime)
                    {
                        pqVector[unassigned_robot].pop();
                        pair<int, int> unassigned_top = pqVector[unassigned_robot].top();
                        // cout << "Top of Unassigned pqVector " << unassigned_robot << " : " << unassigned_top.first << " " << unassigned_top.second << endl; // Robot, Distance, Task
                        if(possible_assignments[unassigned_top.second] == make_pair(-1,-1))
                        {
                            possible_assignments[unassigned_top.second] = make_pair(unassigned_top.first,unassigned_robot);
                            // cout << "Assigned robot " << unassigned_robot << " to task " << unassigned_top.second << " with distance " << unassigned_top.first << endl;
                            unassigned_robot = -1;
                        }
                        else if(unassigned_top.first < possible_assignments[unassigned_top.second].first)
                        {
                            int new_unassigned_robot = possible_assignments[unassigned_top.second].second;
                            // cout << "Removed robot " << new_unassigned_robot << " from task " << unassigned_top.second << " with distance " << possible_assignments[unassigned_top.second].first << endl;
                            possible_assignments[unassigned_top.second] = make_pair(unassigned_top.first,unassigned_robot);
                            // cout << "Assigned robot " << unassigned_robot << " to task " << unassigned_top.second << " with distance " << unassigned_top.first << endl;
                            unassigned_robot = new_unassigned_robot;
                        }
                    }
                }
                else
                {
                    pqVector[i].pop();
                    top = pqVector[i].top();
                    // cout << "New Top of pqVector " << i << " : " << top.first << " " << top.second << endl; // Robot, Distance, Task
                    currently_assigned = 0;
                }
            }
        }
    }

    // cout << "possible_assignments" << endl;
    for (size_t i = 0; i < possible_assignments.size(); ++i) {
        // cout << "Element " << i << " : " << possible_assignments[i].first << " " << possible_assignments[i].second << endl;
        if(possible_assignments[i].second >= 0)
        {
            proposed_schedule[possible_assignments[i].second] = i;
        }
    }

    
    // for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
    // {
        
    //     if (env->curr_task_schedule[i] == -1)
    //     {
            
    //         min_task_i = -1;
    //         min_task_makespan = INT_MAX;
    //         count = 0;
    //         for (i_task=0 ; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime ;i_task++)
    //         {                

    //             if (env->task_pool[i_task].agent_assigned != -1)
    //                 continue;
    //             dist = 0;
    //             c_loc = env->curr_states.at(i).location;
    //             // int loc = env->task_pool[i_task].locations.at(0);
    //             // dist = DefaultPlanner::get_h(env, c_loc, loc);
    //             for (int loc : env->task_pool[i_task].locations){
    //                 dist += DefaultPlanner::get_h(env, c_loc, loc);
    //                 c_loc = loc;
    //             }
    //             if (dist < min_task_makespan){
    //                 min_task_i = i_task;
    //                 min_task_makespan = dist;
    //             }
    //             count++;            
    //         }


    //         if (min_task_i != -1){
    //             proposed_schedule[i] = env->task_pool[min_task_i].task_id;
    //             env->task_pool[min_task_i].agent_assigned = i;
    //         }
    //         else{
    //             proposed_schedule[i] = -1;
    //         }
            

    //     }
    //     else
    //     {
    //         proposed_schedule[i] = env->curr_task_schedule[i];
    //     }
    // }
    // cout << "proposed_schedule" << endl;
    // for (size_t i = 0; i < proposed_schedule.size(); ++i) {
    //     cout << "Element " << i << " : " << proposed_schedule[i] << endl;
    // }
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}
}
