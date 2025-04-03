#pragma once

#include "SharedEnv.h"
#include "LNSSchedulerGame.h"
#include <vector>
#include <chrono>
#include <future>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include "SchedulerUtilsGame.h"

namespace ParallelLNSSchedulerGame
{
    struct ScheduleResult
    {
        std::vector<int> schedule;
        int makespan;
        int task_switches;
    };

    struct SharedData
    {
        std::shared_mutex mutex;
        std::unordered_map<unsigned short, std::vector<LNSSchedulerGame::TaskCost>> &agent_task_costs;

        explicit SharedData(std::unordered_map<unsigned short, std::vector<LNSSchedulerGame::TaskCost>> &costs)
            : agent_task_costs(costs) {}
    };

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);

    void repair(std::unordered_map<int, int> &schedule,
                const std::unordered_map<int, int> &task_agent_map,
                const std::vector<int> &destroyed_agents,
                SharedEnvironment *env,
                std::chrono::steady_clock::time_point end_time,
                SharedData &shared_data);

    // 最適化の結果を格納する構造体
    struct OptimizationResult
    {
        int makespan;
        int task_switches;
    };

    // 初期スケジュールの結果を格納する構造体
    struct OptimizationData
    {
        int makespan;
        std::unordered_set<int> critical_agents;
        OptimizationData(int ms, const std::unordered_set<int> &ca)
            : makespan(ms), critical_agents(ca) {}
    };
    OptimizationResult optimizeAssignment(std::vector<int> &proposed_schedule,
                                          OptimizationData &initial_data,
                                          SharedEnvironment *env,
                                          std::chrono::steady_clock::time_point end_time,
                                          SharedData &shared_data);

    void schedule_plan(int time_limit,
                       std::vector<int> &proposed_schedule,
                       SharedEnvironment *env);

    struct ThreadData
    {
        std::vector<int> schedule;
        OptimizationResult result;
        std::unordered_map<int, int> tmp_schedule;
        std::vector<std::pair<int, int>> destroyed_schedule;

        explicit ThreadData(const std::vector<int> &initial_schedule)
            : schedule(initial_schedule), destroyed_schedule(SchedulerUtilsGame::global_available_agents.size())
        {
        }
    };

    extern std::vector<ThreadData> thread_data;
    extern unsigned int num_threads;

    // タスクの切り替わり回数を計算する関数を追加
    int calculateTotalSwitches(const std::vector<int> &proposed_schedule, SharedEnvironment *env);
}