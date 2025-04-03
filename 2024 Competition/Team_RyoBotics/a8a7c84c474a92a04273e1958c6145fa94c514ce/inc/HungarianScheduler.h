#ifndef HUNGARIAN_SCHEDULER_H
#define HUNGARIAN_SCHEDULER_H

#include "SharedEnv.h"
#include <vector>
#include "SchedulerUtils.h"
using Assignment = SchedulerUtils::Assignment; // Assignmentのみを使用

void schedule_plan_hungarian(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

#endif // HUNGARIAN_SCHEDULER_H