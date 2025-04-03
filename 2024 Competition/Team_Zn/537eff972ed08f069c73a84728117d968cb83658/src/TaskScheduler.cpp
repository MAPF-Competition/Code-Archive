#include "TaskScheduler.h"

// #include "z_scheduler.h"
#include "game_scheduler.h"
#include "city_scheduler.h"
#include "sortation_scheduler.h"
#include "random_scheduler.h"
#include "warehouse_scheduler.h"
#include "scheduler.h"
#include "const.h"

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
void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = 0;//preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    map_is_random = false;
    map_is_warehouse = false;
    map_is_game = false;
    map_is_city = false;
    map_is_sortation = false;
    if (env->map_name[0] == 'r')
    {
        map_is_random = true;
        RandomScheduler::schedule_initialize(limit, env);
    }
    else if (env->map_name[0] == 'w')
    {
        map_is_warehouse = true;
        WarehouseScheduler::schedule_initialize(limit, env);
    }
    else if(env->map_name[0] == 's'){
        map_is_sortation = true;
        SortationScheduler::schedule_initialize(limit, env);
    }
    else if(env->map_name[0] == 'b'){
        map_is_game = true;
        GameScheduler::schedule_initialize(limit, env);
    }
    else if(env->map_name[0] == 'P'){
        map_is_city = true;
        CityScheduler::schedule_initialize(limit, env);
    }
    // ZainPlanner::schedule_initialize(limit, env);
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

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    int limit = 0;
    if(map_is_game){
        GameScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    if(map_is_warehouse){
        WarehouseScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    if(map_is_sortation){
        SortationScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    if(map_is_random){
        RandomScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    if(map_is_city){
        CityScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    return;
}
