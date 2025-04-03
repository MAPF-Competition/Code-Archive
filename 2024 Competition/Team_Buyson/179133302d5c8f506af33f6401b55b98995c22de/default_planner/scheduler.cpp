#include "scheduler.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <queue>
#include <iostream>

namespace DefaultPlanner{

// Global RNG and sets
std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

// ------------------------------------------------------
// 1. Global 2D grid storing tasks by first location
// ------------------------------------------------------
static std::vector<std::vector<std::vector<int>>> taskGrid; 
// taskGrid[r][c] is a std::vector<int> of all tasks whose "representative location" is (r,c).

static int ROWS = 0;
static int COLS = 0;

// Helper: get row,col from a location index
inline void toRC(int loc, int& r, int& c){
    r = loc / COLS;
    c = loc % COLS;
}

// ------------------------------------------------------
// Insert a single task into the grid based on its first location
// ------------------------------------------------------
static void insert_task_into_grid(int task_id, SharedEnvironment* env)
{
    if(env->task_pool[task_id].locations.empty()){
        // if no errands, skip or store in some special bucket
        return;
    }
    int first_loc = env->task_pool[task_id].locations.front();
    int r, c;
    toRC(first_loc, r, c);
    if(r < 0 || r >= ROWS || c < 0 || c >= COLS){
        // out of bounds? skip or clamp
        return;
    }
    taskGrid[r][c].push_back(task_id);
}

// ------------------------------------------------------
// Remove a task from the grid. This is optional if tasks
// vanish after assignment, but can keep data consistent.
// ------------------------------------------------------
static void remove_task_from_grid(int task_id, SharedEnvironment* env)
{
    if(env->task_pool[task_id].locations.empty()) return;
    int loc = env->task_pool[task_id].locations.front();
    int r, c; 
    toRC(loc, r, c);
    if(r < 0 || r >= ROWS || c < 0 || c >= COLS) return;

    // Erase from the vector. 
    // Possibly O(#tasksInCell), but typically small if tasks are well distributed.
    auto& cell = taskGrid[r][c];
    auto it = std::find(cell.begin(), cell.end(), task_id);
    if(it != cell.end()){
        cell.erase(it);
    }
}

// ------------------------------------------------------
// For an agent at (ar, ac), gather tasks from a local region 
// with "searchRadius". The bigger the radius, the more tasks
// we check (slower, but more accurate).
// 
// If we get too few tasks in that region, we might expand radius
// or fallback to a bigger search.
// ------------------------------------------------------
static std::vector<int> get_local_tasks(int ar, int ac, int searchRadius)
{
    std::vector<int> results;

    int r1 = std::max(0, ar - searchRadius);
    int r2 = std::min(ROWS - 1, ar + searchRadius);
    int c1 = std::max(0, ac - searchRadius);
    int c2 = std::min(COLS - 1, ac + searchRadius);

    for(int rr = r1; rr <= r2; rr++){
        for(int cc = c1; cc <= c2; cc++){
            // gather tasks from that cell
            auto& cellTasks = taskGrid[rr][cc];
            // just insert all
            results.insert(results.end(), cellTasks.begin(), cellTasks.end());
        }
    }

    return results;
}

// ------------------------------------------------------
// schedule_initialize: Build heuristics, init the grid, etc.
// ------------------------------------------------------
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // 1. Init heuristics
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);

    // 2. Build the grid structure to hold tasks.
    ROWS = env->rows;
    COLS = env->cols;
    taskGrid.clear();
    taskGrid.resize(ROWS);
    for(int r=0; r<ROWS; r++){
        taskGrid[r].resize(COLS);
    }
    return;
}

// ------------------------------------------------------
// schedule_plan: 
// - Insert newly arrived tasks to the grid
// - For each free agent, gather tasks from a local region
// - Pick the best by partial-sum bounding
// ------------------------------------------------------
void schedule_new_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    // (1) Update free agents/tasks sets
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // (2) Insert new tasks into the grid
    for(int t_id : env->new_tasks){
        insert_task_into_grid(t_id, env);
    }

    // (Optional) If tasks that are no longer free should be removed from the grid, 
    // we do that after assignment. See below.

    clock_t start = clock();

    // (3) For each free agent, find a local set of tasks to check
    auto it = free_agents.begin();
    while(it != free_agents.end())
    {
        if(std::chrono::steady_clock::now() > endtime) 
            break; // out of time

        int agent_id = *it;
        assert(env->curr_task_schedule[agent_id] == -1);

        // Agent's location -> row,col
        int agent_loc = env->curr_states.at(agent_id).location;
        int ar, ac;
        toRC(agent_loc, ar, ac);

        // We'll do a small radius search, e.g. +/- 5 cells around the agent
        const int INITIAL_RADIUS = 5;
        std::vector<int> candidate_tasks = get_local_tasks(ar, ac, INITIAL_RADIUS);

        // If too few tasks found, we can enlarge the radius or fallback 
        // to a bigger region. Example:
        if(candidate_tasks.size() < 5 && free_tasks.size() > 5){
            // let's expand radius 
            candidate_tasks = get_local_tasks(ar, ac, 2*INITIAL_RADIUS);
        }

        // If STILL too few, fallback to scanning all free_tasks? 
        // That depends on your preference:
        if(candidate_tasks.size() < 3 && free_tasks.size() > 3){
            // fallback to all free tasks
            candidate_tasks.assign(free_tasks.begin(), free_tasks.end());
        }

        // (4) Among candidate tasks, pick the best by partial-sum bounding
        int best_task     = -1;
        int best_distance = INT_MAX;
        int count = 0;

        for(int task_id : candidate_tasks)
        {
            // Skip tasks that might already be assigned by another agent in this same iteration
            // if we remove them from free_tasks immediately. Let's only check tasks still in free_tasks.
            if(free_tasks.find(task_id) == free_tasks.end()) 
                continue;

            if(count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;
            count++;

            int dist_sum = 0;
            int curr_loc = agent_loc;
            for(int loc : env->task_pool[task_id].locations){
                dist_sum += DefaultPlanner::get_h(env, curr_loc, loc);
                // bounding
                if(dist_sum >= best_distance){
                    break;
                }
                curr_loc = loc;
            }

            if(dist_sum < best_distance){
                best_distance = dist_sum;
                best_task     = task_id;
            }
        }

        // (5) Assign if we found something
        if(best_task != -1){
            proposed_schedule[agent_id] = best_task;
            free_tasks.erase(best_task);
            remove_task_from_grid(best_task, env);  // keep the grid consistent
            it = free_agents.erase(it);
        } else {
            proposed_schedule[agent_id] = -1;
            ++it;
        }
    }

    // (6) Any leftover free agents get -1
    while(it != free_agents.end()){
        proposed_schedule[*it] = -1;
        ++it;
    }

#ifndef NDEBUG
    double cpu_time = double(clock() - start) / CLOCKS_PER_SEC;
    std::cout << " s, free_agents: " << free_agents.size()
              << ", free_tasks: " << free_tasks.size()
              << std::endl;

              //<< "[Grid-based schedule_plan] CPU Time: " << cpu_time 
#endif
}

void schedule_old_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    //clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    //int counter = 0;
    while (it != free_agents.end())
    {
        // print the agent id
        //std::cout << "Agent: " << *it << std::endl;
        //counter++;
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            //std::cout << "Exiting due to timeout at step" << counter << std::endl;
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
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
   #ifndef NDEBUG
    //cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}

// plan enum
enum PlanType
{
    NEW_PLAN,
    OLD_PLAN
};


void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    PlanType plan_type = NEW_PLAN;
    //PlanType plan_type = OLD_PLAN;
    if (plan_type == NEW_PLAN)
    {
        schedule_new_plan(time_limit, proposed_schedule, env);
    }
    else
    {
        schedule_old_plan(time_limit, proposed_schedule, env);
    }    
}




} // end namespace
