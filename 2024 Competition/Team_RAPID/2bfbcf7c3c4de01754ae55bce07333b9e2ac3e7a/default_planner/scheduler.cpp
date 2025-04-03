#include "scheduler.h"

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
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

    // 输出检查这两个集合
    // std::cout << "Free Agents Size: " << free_agents.size() << std::endl;
    // std::cout << "Free Tasks Size: " << free_tasks.size() << std::endl;

    // if (free_agents.size() < 2) {
    //     return;
    // }

    int min_task_i, min_task_makespan, dist, c_loc, count;
    int t_id, best_agent, best_makespan, existenceTime, dynamic_threshold;
    int c_dir;
    clock_t start = clock();

    // 反过来任务选智能体
    // 为每个任务设置一个随时间动态增长的最大makespan阈值，只有最优的智能体可以满足该阈值才分配执行

    // // iterate over the free tasks to decide which agent to assign each task
    // std::unordered_set<int>::iterator task_it = free_tasks.begin();
    // while (task_it != free_tasks.end())
    // {
    //     // Check for timeout before processing each task
    //     if (std::chrono::steady_clock::now() > endtime)
    //     {
    //         break;
    //     }
        
    //     t_id = *task_it;
        
    //     best_agent = -1;
    //     best_makespan = INT_MAX;
    //     count = 0;
        
    //     existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 0);
    //     dynamic_threshold = std::min(existenceTime + 5, 25);

    //     // Iterate over all free agents to find the one with minimum makespan for task t_id
    //     for (int i : free_agents)
    //     {
    //         //check for timeout every 10 task evaluations
    //         if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
    //         {
    //             break;
    //         }
    //         // For each free agent, calculate the makespan for completing task t_id
    //         int makespan = 0;
    //         int current_loc = env->curr_states.at(i).location;
            
    //         // Iterate over the locations (errands) of task t_id to compute the makespan
    //         for (int loc : env->task_pool[t_id].locations) {
    //             makespan += DefaultPlanner::get_h(env, current_loc, loc);
    //             current_loc = loc;
    //             break;  // 考虑第一个点位即可
    //         }
            
    //         if (makespan < best_makespan) {
    //             best_makespan = makespan;
    //             best_agent = i;
    //         }
    //         count++;
    //     }
        
    //     // If a suitable agent is found, assign task t_id to that agent
    //     if (best_agent != -1) {
    //         // proposed_schedule[best_agent] = t_id;
    //         // // Remove the selected agent from free_agents so it is not assigned another task
    //         // free_agents.erase(best_agent);
    //         // // Erase the current task from free_tasks
    //         // task_it = free_tasks.erase(task_it);
            
    //         // // Output best_makespan, best_agent, and t_id for checking
    //         // std::cout << "Best Makespan: " << best_makespan << ", Best Agent: " << best_agent << ", Task ID: " << t_id << std::endl;
    //         if (best_makespan <= dynamic_threshold)
    //         {
    //             proposed_schedule[best_agent] = t_id;
    //             // 从空闲集合移除该智能体，以防再次分配
    //             free_agents.erase(best_agent);
    //             // 从任务集合移除该任务
    //             task_it = free_tasks.erase(task_it);

    //             // 输出用于调试或观察
    //             // std::cout << "[Assign] Task " << t_id 
    //             //           << " bestAgent=" << best_agent 
    //             //           << ", bestMakespan=" << best_makespan
    //             //           << ", threshold=" << dynamic_threshold
    //             //           << std::endl;
    //         }
    //     }
    //     else {
    //         // 如果没有找到合适的智能体，则保留任务（或将其分配为 -1 表示未分配），然后继续下一个任务
    //         task_it++;
    //     }
    // }

    // 智能体选任务 动态阈值决定最大可接受的makespan
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
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;


        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 5);
            // dynamic_threshold = std::min(existenceTime, 10);
            // dynamic_threshold = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 20);
            //根据地图规模和智能体数量设置区间
            int x = env->rows * env->cols / env->num_of_agents;
            // x = x + 2*sqrt(x);
            // std::cout << "x: " << x << std::endl;
            // random_32_32_20_100.json x=10
            // warehouse_large_5000.json x=14
            // sortation_large_2000.json x=35
            // existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, x/2);
            // dynamic_threshold = std::min(existenceTime, x);
            dynamic_threshold = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, x);

            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                // std::cout << "Timeout" << std::endl;
                break;
            }   //每 10 次评估后检查当前时间是否超过截止时间 endtime，如果超时则退出当前任务遍历。
            dist = 0;
            c_loc = env->curr_states.at(i).location;
            c_dir = env->curr_states.at(i).orientation;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                // int dist1 = DefaultPlanner::get_h(env, c_loc, loc);
                int dist2 = DefaultPlanner::get_h_rot(env, c_loc, c_dir, loc, c_dir);
                // std::cout << "dist1: " << dist1 << ", dist2: " << dist2 << std::endl;
                dist += dist2;

                c_loc = loc;
                break;  // 考虑第一个点位即可
            }

            // update the new minimum makespan
            if (dist < dynamic_threshold and dist < min_task_makespan){
            // if (dist < min_task_makespan){
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
            // 输出用于调试或观察
            // std::cout << "[Assign] Agent " << i 
            //       << " assigned to Task " << min_task_i 
            //       << " with Makespan " << min_task_makespan 
            //       << std::endl;
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
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
