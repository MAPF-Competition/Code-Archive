#include <Entry.h>
#include <random>

//default planner includes
#include "const.h"
#include "planner.h"

#include <Objects/Environment/environment.hpp>
#include <settings.hpp>

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 *
 * This function call the planner initialize function with a time limit fore preprocessing.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit) {
#ifdef ENABLE_DEFAULT_PLANNER
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit, env);
#elif defined(ENABLE_SMART_PLANNER)
    smart_planner.initialize(preprocess_time_limit);
#else
    init_environment(*env);
#endif
}

namespace DefaultPlanner {
    extern std::unordered_set<int> free_agents;
    extern std::unordered_set<int> free_tasks;
};// namespace DefaultPlanner

/**
 * Plans a path using default planner
 *
 * This function performs path planning within the timelimit given, and call the plan function in default planner.
 * The planned actions are output to the provided actions vector.
 *
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit, vector<Action> &actions) {
#ifdef ENABLE_DEFAULT_PLANNER
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::plan(limit, actions, env);
#elif defined(ENABLE_SMART_PLANNER)
    actions.clear();
    smart_planner.plan(time_limit, actions);
#else
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 30);

    update_environment(*env);
#ifdef ENABLE_GUIDANCE_PATH_PLANNER
    get_gpp().update(env->curr_timestep, end_time);
#endif

#ifdef ENABLE_DEFAULT_SCHEDULER_TRICK
    {
        for (uint32_t r = 0; r < get_robots_handler().size(); r++) {
            uint32_t t = env->curr_task_schedule[r];
            if (t != -1) {
                env->task_pool.at(t).agent_assigned = r;
            }
        }

        update_environment(*env);
        static MyScheduler my_scheduler(env);
        std::vector<int> plan;
        my_scheduler.plan(end_time, plan);

        DefaultPlanner::free_agents.clear();
        DefaultPlanner::free_tasks.clear();

        DefaultPlanner::free_agents.insert(my_scheduler.solver.free_robots.begin(), my_scheduler.solver.free_robots.end());
        for (uint32_t t: my_scheduler.solver.free_tasks) {
            ASSERT(env->task_pool.count(t), "task no contains");
            ASSERT(env->task_pool.at(t).task_id == t, "invalid t");
            ASSERT(env->task_pool.at(t).agent_assigned == -1, "already assigned");
            ASSERT(env->task_pool.at(t).idx_next_loc == 0, "idx_next_loc != 0");
            DefaultPlanner::free_tasks.insert(t);
        }
        //free_tasks.insert(my_scheduler.solver.free_tasks.begin(), my_scheduler.solver.free_tasks.end());
    }
#endif
    auto [plan, desired_plan] = eplanner.plan(end_time);
    actions = std::move(plan);
#endif
}
