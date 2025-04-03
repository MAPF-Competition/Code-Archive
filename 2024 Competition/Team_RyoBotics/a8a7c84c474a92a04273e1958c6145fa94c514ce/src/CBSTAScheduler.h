#pragma once

#include "SharedEnv.h"
#include "ECBSTA.h"
#include "SchedulerUtils.h"

void schedule_plan_CBSTA(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);