#include <chrono>
#include "MAPFPlanner.h"
#include"const.h"

void MAPFPlanner::initialize(int preprocess_time_limit) {
    auto start {std::chrono::steady_clock::now()};

    pibt.initialize();

    auto end {std::chrono::steady_clock::now()};
    auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    std::cout << "******** Initialization took " << duration.count() << " ms ********\n";
}

void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
    static int cnt {0};
    auto start {std::chrono::steady_clock::now()};

    auto limit {time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now()
         - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE};

    pibt.nextStep(limit, actions);

    auto end {std::chrono::steady_clock::now()};
    auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    std::cout << "******** Planning at timestep " << env->curr_timestep << " took " << duration.count() << " ms ********\n";
    cnt += duration.count();

    if (env->curr_timestep == 4999) {
        std::cout << "******** Average planning time " << cnt / 5000.f << " ms ********\n";
    }
}
