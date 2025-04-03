#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>
#include <deque>
#include <chrono>

namespace DefaultPlanner
{
    using TimePoint = std::chrono::steady_clock::time_point;

    // タブーサーチのパラメータを管理する構造体
    struct TabuParameters
    {
        int base_tabu_tenure = 10;
        int max_iterations_without_improvement = 10;
        int step_for_tabu_increase = 100;
        int step_for_tabu_reset = 1000;
    };

    // タブーサーチ用の移動を表す構造体
    struct TabuMove
    {
        int agent1;
        int agent2;
        int task1;
        int task2;

        TabuMove(int a1, int a2, int t1, int t2)
            : agent1(a1), agent2(a2), task1(t1), task2(t2) {}
    };

    // スケジューリングの結果を表す構造体
    struct SchedulingResult
    {
        std::vector<int> schedule;
        int cost;
    };

    void schedule_initialize2(int preprocess_time_limit, SharedEnvironment *env);

    void schedule_plan2(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // 新しく追加する関数
    bool canAssignTask(int agent_idx, int task_id, SharedEnvironment *env, const std::vector<int> &current_schedule);
    int computeScheduleCost(const std::vector<int> &schedule, SharedEnvironment *env);
    void handleFreeAgents(std::vector<int> &schedule, SharedEnvironment *env);
    SchedulingResult performTabuSearch(std::vector<int> &initial_schedule,
                                       SharedEnvironment *env,
                                       const TabuParameters &params,
                                       const TimePoint &endtime);
    std::vector<int> generateInitialSchedule(SharedEnvironment *env);
    void generateInitialScheduleNew(SharedEnvironment *env, std::vector<int> &proposed_schedule, int time_limit);
}

#endif