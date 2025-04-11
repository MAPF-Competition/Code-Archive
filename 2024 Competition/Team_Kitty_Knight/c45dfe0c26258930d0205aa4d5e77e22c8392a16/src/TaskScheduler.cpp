#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - MyPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    MyPlanner::schedule_initialize(limit, env);    
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - MyPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    MyPlanner::schedule_plan(limit, proposed_schedule, env);
}
