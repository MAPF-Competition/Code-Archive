#ifndef PARALLEL_SCHEDULER_H
#define PARALLEL_SCHEDULER_H

#include "NewScheduler.h"
#include "SharedEnv.h"
#include <vector>

namespace DefaultPlanner
{
    // マルチプロセスを用いてタブーサーチでスケジュールを探索する
    // process_count: 生成する子プロセスの数
    SchedulingResult parallelTabuSearchMP(
        const std::vector<int> &initial_schedule,
        SharedEnvironment *env,
        const TabuParameters &params,
        const TimePoint &endtime,
        int process_count);
}

#endif // PARALLEL_SCHEDULER_H