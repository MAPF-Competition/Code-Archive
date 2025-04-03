#ifndef EXTENDED_HUNGARIAN_SCHEDULER_H
#define EXTENDED_HUNGARIAN_SCHEDULER_H

#include "SharedEnv.h"
#include "SchedulerUtils.h"
#include <vector>
#include <unordered_map>
#include "hungarian_matrix.h"
#include "planner.h"
void schedule_plan_extended_hungarian(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

#endif // EXTENDED_HUNGARIAN_SCHEDULER_H