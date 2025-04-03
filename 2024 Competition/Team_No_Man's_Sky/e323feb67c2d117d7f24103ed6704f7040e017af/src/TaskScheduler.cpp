#include "TaskScheduler.h"

#include "const.h"
#include "scheduler.h"

#include <settings.hpp>
#include <Objects/Environment/environment.hpp>

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 * 
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit 
 * and adjust for a specified tolerance to account for potential timing errors. 
 * It ensures that initialization does not exceed the allocated time.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit) {
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
#else
    init_environment(*env);
#endif
}

/**
 * Plans a task schedule within a specified time limit.
 * 
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 * 
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    ETimer timer;
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    PRINT(Printer() << "Default Scheduler: " << timer << '\n';);
#else
    TimePoint end_time = std::min(env->plan_start_time + Milliseconds(time_limit - 10), get_now() + Milliseconds(SCHEDULER_REBUILD_DP_TIME + SCHEDULER_TRIV_SOLVE_TIME + SCHEDULER_LNS_TIME + SCHEDULER_TRICK_TIME));
    update_environment(*env);
    my_scheduler.plan(end_time, proposed_schedule);
#endif
}
