#include "my_scheduler.h"
#include "hungarian.h"
#include "lap.h"
#include <iostream>
#include <vector>
#include <unordered_set>
#include <cassert>
#include <algorithm>

namespace MyPlanner {

    // Random number generator
    std::random_device rd;  // Non-deterministic random seed
    std::mt19937 g(rd());   // Mersenne Twister PRNG

    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;

    std::unordered_set<int> available_agents;
    std::unordered_set<int> available_tasks;

    int MAX_CHUNK_SIZE = 300;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env) {
        // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(int(preprocess_time_limit*0.9));

        MyPlanner::init_heuristics(env);
        g.seed(0);

        // use left time to initialise heuristic table
        for (int c_loc=0; c_loc < env->map.size(); ++c_loc){
            if (env->map[c_loc] == 1){
                continue;
            }

            for (int loc=0; loc < env->map.size(); ++loc){

                if (env->map[loc] == 1){
                    continue;
                }

                if ( ( std::chrono::steady_clock::now() < end_time) ){
                    int dist = MyPlanner::get_h(env, loc, c_loc);
                }
                else{
                    return;
                }
            }
        }
        // MyPlanner::export_heuristic_table(env->map_name);
        return;
    }

    std::tuple<std::unordered_set<int>, std::unordered_set<int>, std::unordered_set<int>, std::unordered_set<int>>
    update(SharedEnvironment *env) {
        std::unordered_set<int> t_agents;  // traverse
        std::unordered_set<int> t_tasks;
        std::unordered_set<int> o_agents;  // occupied
        std::unordered_set<int> o_tasks;

        for (size_t agent_id = 0; agent_id < env->curr_task_schedule.size(); ++agent_id) {  // C++17 structured bindings
            // Use id and task
            auto task_id = env->curr_task_schedule[agent_id];
            if (task_id != -1)  // this agent is already assigned with a task
            {
                if (env->task_pool[task_id].idx_next_loc == 0)  // this agent hasn't reached the first location
                {
                    t_agents.insert(agent_id);
                    t_tasks.insert(task_id);
                } else {
                    o_agents.insert(agent_id);
                    o_tasks.insert(task_id);
                }
            }
        }
        return {t_agents, t_tasks, o_agents, o_tasks};

    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env) {
        //use at most half of time_limit to compute schedule, -10 for timing error tolerance
        //so that the remainning time are left for path planner
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
        // cout<<"schedule plan limit" << time_limit <<endl;

        // Insert newly freed agents/tasks
        free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
        free_tasks.insert(env->new_tasks.begin(),    env->new_tasks.end());

        // Gather info on traversing/occupied
        auto [traversing_agents, traversing_tasks, occupied_agents, occupied_tasks] = update(env);

        // available_agents = free_agents + traversing_agents
        for (int ag : free_agents)        available_agents.insert(ag);
        for (int ag : traversing_agents)  available_agents.insert(ag);
        for (int ag : occupied_agents)    available_agents.erase(ag);
        // available_tasks  = free_tasks + traversing_tasks
        for (int tk : free_tasks)         available_tasks.insert(tk);
        for (int tk : traversing_tasks)   available_tasks.insert(tk);
        for (int tk : occupied_tasks)     available_tasks.erase(tk);

        // Prepare data for the Hungarian algorithm.
        clock_t start = clock();

        std::vector<int> vec_agents;
        std::vector<int> vec_tasks;

        // Convert to vectors
        if (available_agents.size() > MAX_CHUNK_SIZE) {
            if (free_agents.size() > MAX_CHUNK_SIZE) {
                std::vector<int> free_agents_vec(free_agents.begin(), free_agents.end());
                vec_agents.assign(free_agents_vec.begin(), free_agents_vec.begin() + MAX_CHUNK_SIZE);
                vec_tasks.assign(free_tasks.begin(), free_tasks.end());
            } else {
                std::vector<int> available_agents_vec(available_agents.begin(), available_agents.end());
                std::shuffle(available_agents_vec.begin() + available_agents.size(), available_agents_vec.end(), g);

                vec_agents.assign(available_agents_vec.begin(), available_agents_vec.begin() + MAX_CHUNK_SIZE);
                vec_tasks.assign(free_tasks.begin(), free_tasks.end());

                for (int i = static_cast<int>(free_agents.size()); i < MAX_CHUNK_SIZE; ++i) {
                    auto task_id = env->curr_task_schedule[vec_agents[i]];
                    if (task_id != -1){
                        vec_tasks.push_back(task_id);
                    }
                }
            }
        } else {
            vec_agents.assign(available_agents.begin(), available_agents.end());
            vec_tasks.assign(available_tasks.begin(), available_tasks.end());
        }


        const int N_agents = static_cast<int>(vec_agents.size());
        const int N_tasks  = static_cast<int>(vec_tasks.size());

        if (N_agents == 0 ) {
            return;  // No scheduling needed
        }

        // Build a full cost table for (agent, task)
        std::vector<std::vector<int>> cost_all(N_agents, std::vector<int>(N_tasks, 0));
        for (int i = 0; i < N_agents; i++) {
            int agent_id = vec_agents[i];
            for (int j = 0; j < N_tasks; j++) {
                int task_id = vec_tasks[j];
                int dist = 0;
                int locA = env->curr_states.at(agent_id).location;
                for (int locB : env->task_pool[task_id].locations) {
                    dist += MyPlanner::get_h(env, locA, locB);
                    locA  = locB;
                }
                cost_all[i][j] = dist;
            }
        }

#ifndef NDEBUG
        // // Create a random number engine and distribution
        // std::random_device rd;   // to seed the generator
        // std::mt19937 gen(rd());  // Mersenne Twister engine
        // // Define a uniform distribution from 0 to 100
        // std::uniform_int_distribution<int> dist(0, 100);
//
        // // Fill the 2D vector with random numbers
        // for (auto &row : cost_all) {
        //     for (auto &val : row) {
        //         val = dist(gen);
        //     }
        // }
#endif

        // Find each task's min cost across all agents
        std::vector<std::pair<int,int>> tasks_with_mincost;
        tasks_with_mincost.reserve(N_tasks);
        for (int j = 0; j < N_tasks; j++) {
            int min_cost = INT_MAX;
            for (int i = 0; i < N_agents; i++) {
                if (cost_all[i][j] < min_cost) {
                    min_cost = cost_all[i][j];
                }
            }
            tasks_with_mincost.emplace_back(min_cost, vec_tasks[j]);
        }
        std::sort(tasks_with_mincost.begin(), tasks_with_mincost.end(),
                  [](auto &a, auto &b){ return a.first < b.first; });

        // Pick top N=min(N_agents, N_tasks) tasks
        int final_task_count = std::min(N_agents, N_tasks);
        std::unordered_set<int> chosen_tasks_set;
        chosen_tasks_set.reserve(final_task_count);
        for (int k = 0; k < final_task_count; k++) {
            chosen_tasks_set.insert(tasks_with_mincost[k].second);
        }

        // Create a quick lookup for (task_id -> original cost_all column)
        std::unordered_map<int,int> task_index_map;
        task_index_map.reserve(N_tasks);
        for (int j = 0; j < N_tasks; j++) {
            task_index_map[vec_tasks[j]] = j;
        }

        // Build an NxN cost matrix
        std::vector<int> chosen_tasks;
        chosen_tasks.reserve(final_task_count);
        for (int tk : vec_tasks) {
            if (chosen_tasks_set.find(tk) != chosen_tasks_set.end()) {
                chosen_tasks.push_back(tk);
                if ((int)chosen_tasks.size() == final_task_count) break;
            }
        }
        std::vector<std::vector<int>> cost_matrix(N_agents, std::vector<int>(final_task_count, 0));
        for (int i = 0; i < N_agents; i++) {
            for (int c = 0; c < final_task_count; c++) {
                cost_matrix[i][c] = cost_all[i][task_index_map[chosen_tasks[c]]];
            }
        }

        // Run Hungarian on the NxN matrix
        // std::vector<int> assignment = hungarian_solve(cost_matrix);
        std::vector<int> assignment = lap(cost_matrix);

        // Apply the assignment
        for (int i = 0; i < N_agents; i++) {
            int ag_id = vec_agents[i];
            int col = assignment[i];
            if (col < 0 || col >= final_task_count) {
                if (available_agents.find(ag_id) != available_agents.end()) {
                    proposed_schedule[ag_id] = -1;
                }
                continue;
            }
            int matched_task = chosen_tasks[col];
            if (proposed_schedule[ag_id] != -1 && proposed_schedule[ag_id] != matched_task) {
                free_tasks.insert(proposed_schedule[ag_id]);
            }
            proposed_schedule[ag_id] = matched_task;
            free_agents.erase(ag_id);
        }

        // Remove newly assigned tasks from free_tasks
        for (int ag : vec_agents) {
            int tk = proposed_schedule[ag];
            if (tk != -1) free_tasks.erase(tk);
        }

#ifndef NDEBUG
            cout << "Time Usage: " << ((float) (clock() - start)) / CLOCKS_PER_SEC << endl;
            cout << "new free agents: " << env->new_freeagents.size() << " new tasks: " << env->new_tasks.size()
                 << endl;
            cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
            return;

    }
}