#include "scheduler.h"

#include <Objects/Basic/assert.hpp>
#include <settings.hpp>

namespace DefaultPlanner {

    std::mt19937 mt;
    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env) {
        DefaultPlanner::init_heuristics(env);
        mt.seed(0);
    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env) {
        //use at most half of time_limit to compute schedule, -10 for timing error tolerance
        //so that the remainning time are left for path planner
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
        // cout<<"schedule plan limit" << time_limit <<endl;

        // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
        free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
        free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

        for (uint32_t r: free_agents) {
            //[timestep=50] Scheduler Error: schedule agent 79 to task 496 wrong because the task is already assigned to agent 9
            //proposed_schedule[r] = -1; // fix problems
        }

        PRINT(Printer() << "free_agents: " << free_agents.size() << '\n';);
        PRINT(Printer() << "free_tasks: " << free_tasks.size() << '\n';);

        int min_task_i, min_task_makespan, dist, c_loc, count;
        clock_t start = clock();

        /*for (int t_id: free_tasks) {
            ASSERT(t_id != -1 && env->task_pool.count(t_id), "invalid task_id");
            auto &task = env->task_pool.at(t_id);

            //Printer() << "task: " << t_id << ", idx_next_loc: " << task.idx_next_loc << ", target: " << task.locations[0] + 1 << ", agent_assigned: " << task.agent_assigned << '\n';

            ASSERT(env->task_pool.count(t_id), "task is no contains");
            ASSERT(env->task_pool.at(t_id).idx_next_loc == 0, "task is close: " + std::to_string(t_id) + " " + std::to_string(env->task_pool.at(t_id).idx_next_loc));
        }*/

        // iterate over the free agents to decide which task to assign to each of them
        std::unordered_set<int>::iterator it = free_agents.begin();
        while (it != free_agents.end()) {
            //keep assigning until timeout
            if (std::chrono::steady_clock::now() > endtime) {
                break;
            }
            int i = *it;

            ASSERT(0 <= i && i < env->num_of_agents, "invalid robot");

            assert(env->curr_task_schedule[i] == -1);

            min_task_i = -1;
            min_task_makespan = INT_MAX;
            count = 0;

            // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
            for (int t_id: free_tasks) {
                //ASSERT(t_id != -1 && env->task_pool.count(t_id), "invalid task_id");

                //ASSERT(env->task_pool.count(t_id), "task is no contains");
                //ASSERT(env->task_pool.at(t_id).idx_next_loc == 0, "task is close: " + std::to_string(t_id) + " " + std::to_string(env->task_pool.at(t_id).idx_next_loc));

                //check for timeout every 10 task evaluations
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) {
                    break;
                }
                dist = 0;
                c_loc = env->curr_states.at(i).location;

                // iterate over the locations (errands) of the task to compute the makespan to finish the task
                // makespan: the time for the agent to complete all the errands of the task t_id in order
                for (int loc: env->task_pool[t_id].locations) {
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }

                // update the new minimum makespan
                if (dist < min_task_makespan) {
                    min_task_i = t_id;
                    min_task_makespan = dist;
                }
                count++;
            }

            // assign the best free task to the agent i (assuming one exists)
            if (min_task_i != -1) {
                proposed_schedule[i] = min_task_i;
                it = free_agents.erase(it);
                free_tasks.erase(min_task_i);
            }
            // nothing to assign
            else {
                proposed_schedule[i] = -1;
                it++;
            }
        }
#ifndef NDEBUG
        //cout << "Time Usage: " << ((float) (clock() - start)) / CLOCKS_PER_SEC << endl;
        //cout << "new free agents: " << env->new_freeagents.size() << " new tasks: " << env->new_tasks.size() << endl;
        //cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
    }
}// namespace DefaultPlanner
