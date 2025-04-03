#include "LocalSearchScheduler.h"
#include "PathFinder.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include "TPTSScheduler.h"
#include "SchedulerUtils.h"
#include "TaskDensityManager.h"
using namespace SchedulerUtils;
namespace LocalSearchPlanner
{
    std::mt19937 rng;
    std::unordered_map<int, TaskDifficulty> task_difficulties;
    // FastDistanceTable distance_table;

    void schedule_initialize_LS(int preprocess_time_limit, SharedEnvironment *env)
    {
        // // 距離テーブルの初期化
        // std::string filepath = env->file_storage_path + "/" + env->map_name + "_distance_table.bin";
        // std::ifstream check_file(filepath);
        // if (!check_file.good())
        // {
        //     DistanceTable::computeAndSave(env, filepath);
        // }
        // check_file.close();
        // distance_table = DistanceTable::load(filepath);

        // int distance = apsp_table.getCost(0, 0, 32 * 5 + 2, 0);
        // std::cout << "Distance from loc 0 to loc " << 32 * 5 + 2 << ": " << distance << std::endl;
        // // 乱数生成器の初期化
        rng.seed(std::random_device()());
    }

    bool isValidAssignment(int agent_id, int task_id, const std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        // エージェントが現在タスクを実行中の場合
        if (env->curr_task_schedule[agent_id] != -1)
        {
            // 現在のタスクが進行中かチェック
            auto task_it = env->task_pool.find(env->curr_task_schedule[agent_id]);
            if (task_it != env->task_pool.end() && task_it->second.idx_next_loc > 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else if (proposed_schedule[agent_id] != -1)
        {
            return true;
        }

        return false;
    }

    int evaluateAssignmentCost(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
            return 0;

        auto task_it = env->task_pool.find(task_id);
        if (task_it == env->task_pool.end())
            return std::numeric_limits<int>::max();

        const Task &task = task_it->second;
        if (task.locations.empty())
            return std::numeric_limits<int>::max();

        int agent_location = env->curr_states[agent_id].location;
        int task_location = task.locations[0];

        // 距離テーブルから距離を取得
        return distance_table.getDistance(agent_location, task_location);
    }

    bool trySwapAssignments(std::vector<int> &schedule,
                            int agent1,
                            int agent2,
                            SharedEnvironment *env,
                            std::unordered_map<int, int> &task_assignments)
    {

        int task1 = schedule[agent1];
        int task2 = schedule[agent2];

        // 一方のタスクが-1の場合はスワップの意味がない
        if (task1 == -1 || task2 == -1)
            return false;
        if (!(isValidAssignment(agent1, task2, schedule, env) && isValidAssignment(agent2, task1, schedule, env)))
            return false;

        // 現在の割り当てのコストを計算
        // int cost_agent1 = SchedulerUtils::evaluateDistance(agent1, task1, env);
        // int cost_agent2 = SchedulerUtils::evaluateDistance(agent2, task2, env);
        int cost_agent1 = SchedulerUtils::evaluateCost(agent1, task1, env);
        int cost_agent2 = SchedulerUtils::evaluateCost(agent2, task2, env);

        if (cost_agent1 == 0 || cost_agent2 == 0)
        {
            return false;
        }
        int current_cost = cost_agent1 + cost_agent2;
        // int cost_agent1_old = SchedulerUtils::evaluateDistance(agent1, task1, env);
        // std::cout << "cost_agent " << agent1 << ": " << cost_agent1 << ", cost_agent1_old: " << cost_agent1_old << std::endl;

        // int cost_agent1_new = SchedulerUtils::evaluateCost(agent1, task2, env);
        // std::cout << "cost_agent1: " << cost_agent1 << ", cost_agent1_new: " << cost_agent1_new << std::endl;

        // スワップ後のコストを計算
        int new_cost = SchedulerUtils::evaluateCost(agent1, task2, env) +
                       SchedulerUtils::evaluateCost(agent2, task1, env);
        const int threshold = 0;
        // スワップが有効で、かつコストが改善する場合
        if (new_cost < current_cost - threshold)
        {
            // std::cout << "trySwapAssignments: " << std::endl;
            // std::cout << "agent1: " << agent1 << ", task1: " << task1 << ", task2: " << task2 << std::endl;
            // std::cout << "agent2: " << agent2 << ", task1: " << task1 << ", task2: " << task2 << std::endl;
            // std::cout << "current_cost: " << current_cost << ", new_cost: " << new_cost << std::endl;

            // タスク割り当ての更新
            schedule[agent1] = task2;
            schedule[agent2] = task1;

            // task_assignmentsの更新
            if (task1 != -1)
                task_assignments[task1] = agent2;
            if (task2 != -1)
                task_assignments[task2] = agent1;

            return true;
        }

        return false;
    }
    void assignNewTasks(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        std::unordered_set<int> assigned_new_tasks;
        for (int agent_id : global_available_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            if (env->curr_task_schedule[agent_id] != -1)
                continue;
            // 利用可能なタスクが10個以下の場合は全て使用、それ以外は10個をランダムに選択
            std::vector<int> candidate_tasks;
            if (global_available_tasks.size() <= 5000)
            {
                candidate_tasks.assign(global_available_tasks.begin(), global_available_tasks.end());
            }
            else
            {
                std::vector<int> all_tasks(global_available_tasks.begin(), global_available_tasks.end());
                std::shuffle(all_tasks.begin(), all_tasks.end(), rng);
                candidate_tasks.assign(all_tasks.begin(), all_tasks.begin() + 5000);
            }

            // 最小距離のタスクを見つける
            int best_task = -1;
            int min_distance = std::numeric_limits<int>::max();

            for (int task_id : candidate_tasks)
            {
                if (assigned_new_tasks.find(task_id) != assigned_new_tasks.end() ||
                    env->task_pool[task_id].agent_assigned != -1)
                    continue;
                int distance = SchedulerUtils::evaluateCost(agent_id, task_id, env) + TaskDensityManager::calculateTaskDensity(task_id);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    best_task = task_id;
                }
            }

            // タスクを割り当て
            if (best_task != -1)
            {
                proposed_schedule[agent_id] = best_task;
                // global_available_tasks.erase(best_task);
                current_task_assignments[best_task] = agent_id;
                assigned_new_tasks.insert(best_task);
                TaskDensityManager::updateTaskDifficultyOnAssignment(best_task, true, env);
            }
        }
    }

    void optimizeAssignments(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        if (global_available_agents.empty())
        {
            return;
        }

        int max_iterations = 100000000;
        int iteration = 0;
        int failed_attempts = 0;
        const int MAX_FAILED_ATTEMPTS = 10000;

        std::vector<int> agents(global_available_agents.begin(), global_available_agents.end());
        std::uniform_int_distribution<> dist(0, agents.size() - 1);
        std::uniform_int_distribution<> swap_type(0, 1); // 0: エージェント間スワップ, 1: 未割り当てタスクとのスワップ

        while (iteration < max_iterations && std::chrono::steady_clock::now() < end_time)
        {
            bool result = false;

            // スワップのタイプをランダムに選択
            if (swap_type(rng) == 0 && agents.size() >= 2)
            {
                // エージェント間のスワップ
                int idx1 = dist(rng);
                int idx2 = dist(rng);

                if (idx1 != idx2)
                {
                    int agent1 = agents[idx1];
                    int agent2 = agents[idx2];

                    if (proposed_schedule[agent1] != -1 && proposed_schedule[agent2] != -1)
                    {
                        result = trySwapAssignments(proposed_schedule, agent1, agent2, env, current_task_assignments);
                    }
                }
            }
            else
            {
                // 未割り当てタスクとのスワップ
                int idx = dist(rng);
                int agent_id = agents[idx];

                if (proposed_schedule[agent_id] != -1)
                {
                    result = trySwapWithUnassignedTask(proposed_schedule, agent_id, env, current_task_assignments);
                }
            }

            if (!result)
            {
                failed_attempts++;
                if (failed_attempts >= MAX_FAILED_ATTEMPTS)
                {
                    break;
                }
            }
            else
            {
                failed_attempts = 0;
            }

            iteration++;
        }
    }

    bool trySwapWithUnassignedTask(std::vector<int> &schedule,
                                   int agent_id,
                                   SharedEnvironment *env,
                                   std::unordered_map<int, int> &task_assignments)
    {
        int current_task = schedule[agent_id];
        if (current_task == -1)
            return false;

        // 現在の割り当てのコスト
        int current_cost = SchedulerUtils::evaluateCost(agent_id, current_task, env);
        if (current_cost == 0)
            return false;
        current_cost += TaskDensityManager::calculateTaskDensity(current_task);

        // 利用可能なタスクからランダムに候補を選択
        std::vector<int> candidate_tasks;
        for (int task_id : global_available_tasks)
        {
            if (task_assignments.find(task_id) == task_assignments.end())
            {
                candidate_tasks.push_back(task_id);
            }
        }

        if (candidate_tasks.empty())
            return false;

        // ランダムに最大10個のタスクを試す
        std::shuffle(candidate_tasks.begin(), candidate_tasks.end(), rng);
        int num_tries = std::min(1000, static_cast<int>(candidate_tasks.size()));

        for (int i = 0; i < num_tries; i++)
        {
            int new_task = candidate_tasks[i];
            int new_cost = SchedulerUtils::evaluateCost(agent_id, new_task, env) + TaskDensityManager::calculateTaskDensity(new_task);

            const int threshold = 0;
            if (new_cost < current_cost - threshold)
            {
                // タスク割り当ての更新
                schedule[agent_id] = new_task;
                task_assignments.erase(current_task);
                task_assignments[new_task] = agent_id;

                // 困難度の更新
                TaskDensityManager::updateTaskDifficultyOnAssignment(current_task, false, env);
                TaskDensityManager::updateTaskDifficultyOnAssignment(new_task, true, env);

                return true;
            }
        }

        return false;
    }

    void schedule_plan_LS(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        // std::cout << "proposed_schedule as start of LS: ";
        // for (const auto &task : proposed_schedule)
        // {
        //     std::cout << task << " ";
        // }
        std::cout << std::endl;

        // データの更新処理を関数呼び出しに置き換え
        updateRunningTasks(env);
        updateTaskAssignments(env);
        updateAgents(env);
        TaskDensityManager::updateTaskDifficulties(env);
        assignNewTasks(proposed_schedule, env, end_time);
        optimizeAssignments(proposed_schedule, env, end_time);
        // std::cout << "proposed_schedule as end of LS  : ";
        // for (const auto &task : proposed_schedule)
        // {
        //     std::cout << task << " ";
        // }
        std::cout << std::endl;
    }
}