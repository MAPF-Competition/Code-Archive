#pragma once

#include "SharedEnv.h"
#include <vector>
#include <random>
#include "SchedulerUtilsGame.h"

namespace LNSSchedulerGame
{
    extern int assinged_task_additional_cost;
    extern int same_task_additional_cost;
    extern int MANHATTAN_DISTANCE_THRESHOLD;
    // タスクのコストを格納する構造体
    struct TaskCost
    {
        int task_id;
        int cost;
    };

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env);
    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

    // スケジュール結果を格納する構造体
    struct ScheduleResult
    {
        std::vector<int> schedule;
        int makespan;
        int task_switches;
    };

    // LNSの破壊・修復に関する関数
    std::vector<int> destroy(double destroy_ratio,
                             const std::unordered_set<int> &critical_agents,
                             const std::unordered_set<int> &nearby_agents);
    void repair(std::unordered_map<int, int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time, std::unordered_map<unsigned short, std::vector<TaskCost>> &agent_task_costs);

    // 初期スケジュールの結果を格納する構造体
    struct InitialScheduleResult
    {
        int makespan;
        std::unordered_set<int> critical_agents;
        std::unordered_map<unsigned short, std::vector<TaskCost>> agent_task_costs;
    };

    // 関数シグネチャを変更
    InitialScheduleResult generateInitialSchedule(std::vector<int> &proposed_schedule,
                                                  SharedEnvironment *env,
                                                  std::chrono::steady_clock::time_point end_time);

    void optimizeAssignment(std::vector<int> &proposed_schedule, InitialScheduleResult &initial_result, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time);

    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env);

    // Manhattan距離を計算する関数を追加
    int calculateManhattanDistance(int from_x, int from_y, int to_x, int to_y);
    int calculateManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int getManhattanDistance(int from_location, int to_location, SharedEnvironment *env);
    int countAgentsWithLastErrand(SharedEnvironment *env);

    // 指定された距離以内にいるエージェントの集合を返す関数を追加
    std::vector<int> getNearbyAgents(int agent_id, int distance_threshold, SharedEnvironment *env);
    void updateTaskSwitches(std::vector<int> &proposed_schedule, SharedEnvironment *env);
    void resetTaskSwitches(SharedEnvironment *env);
    void decreaseTaskSwitches();
    bool isOverTaskSwitches(int agent_id);
    // タスク切り替え回数を追跡するための変数を追加
    extern std::vector<int> agent_task_switches;

    // スケジュールの評価用関数をテンプレート化
    template <typename ScheduleContainer>
    std::pair<int, std::unordered_set<int>> evaluateSchedule(const ScheduleContainer &schedule, SharedEnvironment *env)
    {
        int makespan = 0;
        std::unordered_set<int> critical_agents;

        // vectorとmapで異なるイテレーション方法に対応
        if constexpr (std::is_same_v<ScheduleContainer, std::vector<int>>)
        {
            for (int agent_id : SchedulerUtilsGame::global_available_agents)
            {
                assert(schedule[agent_id] != -1);
                int completion_time = calculateTaskCompletionTime(agent_id, schedule[agent_id], env);
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
                int completion_time = calculateTaskCompletionTime(agent_id, task_id, env);
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
}
