#pragma once

#include "SharedEnv.h"
#include <vector>
#include <random>
#include "SchedulerUtils.h"
#include "APSPCalculator.h"
#include "CommonTypes.h"

namespace LNSScheduler
{
    extern int assinged_task_additional_cost;
    extern int same_task_additional_cost;
    extern int MANHATTAN_DISTANCE_THRESHOLD;
    extern bool INCLUDE_FLOW_COST;
    extern bool include_waypoints;
    extern const int TASKS_PER_THREAD;
    extern const int NUM_THREADS;
    extern const double NEARBY_AGENTS_RATIO;
    extern bool count_deadends;
    extern const int DISTANCE_THRESHOLD_DIVISOR;
    extern const int TASK_SWITCH_THRESHOLD;
    extern int estimated_sim_time;
    extern std::mt19937 rng;
    extern bool disable_agents;
    extern bool include_additional_cost_in_evaluation;
    // タスクのコストを格納する構造体
    struct TaskCost
    {
        int task_id;
        CostType cost;
    };

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // スケジュール結果を格納する構造体
    struct ScheduleResult
    {
        std::vector<int> schedule;
        CostType objective;
        int task_switches;
        int iteration;
    };

    // LNSの破壊・修復に関する関数
    std::vector<int> destroy(double destroy_ratio,
                             const std::unordered_set<int> &critical_agents,
                             const std::unordered_set<int> &nearby_agents,
                             const std::vector<int> &schedule);
    void repair(std::unordered_map<int, int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time, std::unordered_map<unsigned short, std::vector<TaskCost>> &agent_task_costs);

    // 初期スケジュールの結果を格納する構造体
    struct InitialScheduleResult
    {
        CostType objective;
        std::unordered_set<int> critical_agents;
        std::unordered_map<unsigned short, std::vector<TaskCost>> agent_task_costs;
    };

    // 関数シグネチャを変更
    InitialScheduleResult generateInitialSchedule(std::vector<int> &proposed_schedule,
                                                  SharedEnvironment *env,
                                                  std::chrono::steady_clock::time_point end_time);

    void optimizeAssignment(std::vector<int> &proposed_schedule, InitialScheduleResult &initial_result, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);

    CostType calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env);

    // Manhattan距離を計算する関数を追加
    int calculateManhattanDistance(int from_x, int from_y, int to_x, int to_y);
    int calculateManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int getManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int countAgentsWithLastErrand(SharedEnvironment *env);

    // 指定された距離以内にいるエージェントの集合を返す関数を追加
    std::vector<int> getNearbyAgents(int agent_id, int distance_threshold, const std::unordered_set<int> &critical_agents, SharedEnvironment *env);
    void updateTaskSwitches(std::vector<int> &proposed_schedule, SharedEnvironment *env);
    void resetTaskSwitches(SharedEnvironment *env);
    void decreaseTaskSwitches();
    bool isOverTaskSwitches(int agent_id);
    CostType calculateAdditionalCost(int agent_id, int task_id, SharedEnvironment *env);
    // タスク切り替え回数を追跡するための変数を追加
    extern std::vector<int> agent_task_switches;

    // 新しく追加する関数宣言：agent_task_switches[agent_id] + 9) / 10 を計算する関数
    int calculateTaskSwitchLevel(int agent_id);
    std::vector<TaskCost> computeTaskCostsForAgent(int agent_id, SharedEnvironment *env, const std::unordered_set<int> &assigned_tasks, bool ignore_assigned);

    // スケジュールの評価用関数をテンプレート化
    template <typename ScheduleContainer>
    std::pair<CostType, std::unordered_set<int>> evaluateSchedule(const ScheduleContainer &schedule, SharedEnvironment *env, CostType lower_bound = std::numeric_limits<CostType>::max())
    {
        CostType makespan = 0;
        std::unordered_set<int> critical_agents;

        // vectorとmapで異なるイテレーション方法に対応
        if constexpr (std::is_same_v<ScheduleContainer, std::vector<int>>)
        {
            for (int agent_id : SchedulerUtils::global_available_agents)
            {
                assert(schedule[agent_id] != -1);
                if (schedule[agent_id] == -1)
                {
                    continue;
                }
                CostType completion_time = calculateTaskCompletionTime(agent_id, schedule[agent_id], env);
                if (include_additional_cost_in_evaluation)
                {
                    completion_time += calculateAdditionalCost(agent_id, schedule[agent_id], env);
                }
                if (INCLUDE_FLOW_COST)
                {
                    completion_time += SchedulerUtils::calculateOppositeFlowCost(agent_id, schedule[agent_id], env);
                }
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
            }
        }
        else
        {
            for (const auto &[agent_id, task_id] : schedule)
            {
                assert(task_id != -1);
                if (task_id == -1)
                {
                    continue;
                }
                CostType completion_time = calculateTaskCompletionTime(agent_id, task_id, env);
                if (include_additional_cost_in_evaluation)
                {
                    completion_time += calculateAdditionalCost(agent_id, task_id, env);
                }
                if (INCLUDE_FLOW_COST)
                {
                    completion_time += SchedulerUtils::calculateOppositeFlowCost(agent_id, task_id, env);
                }
                if (completion_time > lower_bound)
                {
                    return {INT_MAX, {}};
                }
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
            }
        }

        return {makespan, critical_agents};
    }

    // スケジュールの評価用関数をテンプレート化
    template <typename ScheduleContainer>
    std::pair<int, std::unordered_set<int>> evaluateScheduleSoC(const ScheduleContainer &schedule, SharedEnvironment *env, int lower_bound = INT_MAX)
    {
        CostType makespan = 0;
        std::unordered_set<int> critical_agents;
        CostType soc = 0;

        // vectorとmapで異なるイテレーション方法に対応
        if constexpr (std::is_same_v<ScheduleContainer, std::vector<int>>)
        {
            for (int agent_id : SchedulerUtils::global_available_agents)
            {
                assert(schedule[agent_id] != -1);
                if (schedule[agent_id] == -1)
                {
                    continue;
                }
                CostType completion_time = calculateTaskCompletionTime(agent_id, schedule[agent_id], env);
                if (include_additional_cost_in_evaluation)
                {
                    completion_time += calculateAdditionalCost(agent_id, schedule[agent_id], env);
                }
                if (INCLUDE_FLOW_COST)
                {
                    completion_time += SchedulerUtils::calculateOppositeFlowCost(agent_id, schedule[agent_id], env);
                }
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
                soc += completion_time;
            }
        }
        else
        {
            for (const auto &[agent_id, task_id] : schedule)
            {
                assert(task_id != -1);
                if (task_id == -1)
                {
                    continue;
                }
                CostType completion_time = calculateTaskCompletionTime(agent_id, task_id, env);
                if (include_additional_cost_in_evaluation)
                {
                    completion_time += calculateAdditionalCost(agent_id, task_id, env);
                }
                if (INCLUDE_FLOW_COST)
                {
                    completion_time += SchedulerUtils::calculateOppositeFlowCost(agent_id, task_id, env);
                }
                if (completion_time > makespan)
                {
                    makespan = completion_time;
                    critical_agents.clear();
                    critical_agents.insert(agent_id);
                }
                else if (completion_time == makespan)
                {
                    critical_agents.insert(agent_id);
                }
                soc += completion_time;
                if (soc > lower_bound)
                {
                    return {INT_MAX, {}};
                }
            }
        }

        return {soc, critical_agents};
    }

    // ↓ 新たに追加する関数の宣言
    std::vector<TaskCost> computeSortedTaskCostsForAgent(int agent_id, SharedEnvironment *env);
    void generateInitialScheduleQuick(std::vector<int> &proposed_schedule,
                                      SharedEnvironment *env,
                                      std::chrono::steady_clock::time_point end_time);

    // 最小コストのタスクを見つける関数を追加
    struct MinTaskCost
    {
        int task_id;
        CostType cost;
        bool found;
    };

    MinTaskCost findMinCostTask(int agent_id,
                                const std::unordered_set<int> &available_tasks,
                                const std::unordered_set<int> &assigned_tasks,
                                SharedEnvironment *env,
                                std::chrono::steady_clock::time_point end_time,
                                bool ignore_assigned = true);
}
