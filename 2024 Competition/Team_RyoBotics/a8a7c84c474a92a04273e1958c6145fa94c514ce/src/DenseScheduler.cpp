#include "DenseScheduler.h"
#include "SchedulerUtils.h"

namespace DenseScheduler
{
    std::unordered_set<int> tabu_locations;
    // タスクの完了時間を計算
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
        {
            // std::cerr << "task_id is -1" << std::endl;
            return 0;
        }

        const Task &task = env->task_pool[task_id];
        int total_cost = 0;

        // エージェントの現在位置と向きから最初の位置までのコスト
        int current_loc = env->curr_states[agent_id].location;
        int current_dir = env->curr_states[agent_id].orientation;

        // 最初の位置までのコスト
        total_cost = SchedulerUtils::getCostToLocation(current_loc, current_dir, task.locations[0]);
        // auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
        //     current_loc, current_dir, task.locations[0]);
        // 各経由地点間のコスト
        for (size_t i = 1; i < task.locations.size(); i++)
        {
            total_cost += SchedulerUtils::getCostToLocation(
                task.locations[i - 1], current_dir, task.locations[i]);
        }

        return total_cost;
    }

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        // 必要な初期化があればここで行う
        tabu_locations = {20 * env->cols + 26, 31 * env->cols + 17, 31 * env->cols + 31, 7 * env->cols + 27, 8 * env->cols + 27};
    }

    bool isAgentAtTaskStart(int agent_id, int task_id, SharedEnvironment *env)
    {
        return env->curr_states[agent_id].location == env->task_pool[task_id].locations[0];
    }

    bool hasAssignedTask(int agent_id, const std::vector<int> &proposed_schedule)
    {
        return proposed_schedule[agent_id] != -1;
    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        // 利用可能なタスクを取得
        std::vector<std::pair<int, int>> sorted_tasks; // <task_id, total_cost>

        // 各タスクのtotal costを計算し、ペアとして保存
        for (int task_id : SchedulerUtils::global_available_tasks)
        {
            const Task &task = env->task_pool[task_id];
            int initial_dir = 0; // 初期方向は0（北）と仮定
            int total_cost = SchedulerUtils::task_total_cost_table[task_id];
            sorted_tasks.push_back({task_id, total_cost});
        }

        // total costの昇順でソート
        std::sort(sorted_tasks.begin(), sorted_tasks.end(),
                  [](const auto &a, const auto &b)
                  { return a.second < b.second; });

        const auto &free_agents = SchedulerUtils::global_available_agents;
        // (20, 26),  (31, 17),  (31, 31),  (7,27),  (8,27)
        int num_ongoing_tasks = SchedulerUtils::current_task_assignments.size();
        // if (num_ongoing_tasks > 800)
        // {
        //     return;
        // }

        // 各タスクについて処理（コストが小さい順）
        for (const auto &task_pair : sorted_tasks)
        {
            int task_id = task_pair.first;

            // if (num_ongoing_tasks > 800)
            // {
            //     return;
            // }
            if (tabu_locations.count(env->task_pool[task_id].locations[1]))
            {
                continue;
            }
            // タスクが既に割り当て済みの場合はスキップ
            if (env->task_pool[task_id].agent_assigned != -1 && env->task_pool[task_id].idx_next_loc > 0)
            {
                continue;
            }
            int task_location = env->task_pool[task_id].locations[0];

            // タスクの開始位置にいるエージェントを探す
            for (int agent_id : free_agents)
            {

                // エージェントが既にタスクを持っている場合はスキップ
                if (hasAssignedTask(agent_id, proposed_schedule))
                {
                    if (env->task_pool[proposed_schedule[agent_id]].idx_next_loc > 0)
                    {
                        continue;
                    }
                    int assigned_task_id = proposed_schedule[agent_id];
                    if (env->curr_task_schedule[agent_id] == -1 && isAgentAtTaskStart(agent_id, task_id, env))
                    {
                        int task_makespan = calculateTaskCompletionTime(agent_id, task_id, env);
                        int assigned_task_makespan = calculateTaskCompletionTime(agent_id, assigned_task_id, env);
                        if (task_makespan < assigned_task_makespan)
                        {
                            // if (SchedulerUtils::getNextDirection(env->curr_states[agent_id].location, task_location) == env->curr_states[agent_id].orientation)
                            // {
                            //     proposed_schedule[agent_id] = task_id;
                            //     std::cout << "reassign task " << task_id << " to agent " << agent_id << std::endl;
                            // }

                            proposed_schedule[agent_id] = task_id;
                            // std::cout << "reassign task " << task_id << " to agent " << agent_id << std::endl;
                            break;
                        }
                    }
                    continue;
                }

                // エージェントがタスクの開始位置にいる場合、タスクを割り当てる
                if (isAgentAtTaskStart(agent_id, task_id, env))
                {
                    if (SchedulerUtils::getNextDirection(env->curr_states[agent_id].location, task_location) == env->curr_states[agent_id].orientation)
                    {
                        proposed_schedule[agent_id] = task_id;
                        num_ongoing_tasks++;
                    }
                    break;
                }
            }
        }

        // 残りの未割り当てエージェントに対して、最も近いタスクを割り当てる
        // for (int agent_id : free_agents)
        // {
        //     if (hasAssignedTask(agent_id, proposed_schedule))
        //     {
        //         continue;
        //     }

        //     int best_task = -1;
        //     int min_distance = std::numeric_limits<int>::max();

        //     for (int task_id : available_tasks)
        //     {
        //         if (env->task_pool[task_id].agent_assigned != -1)
        //         {
        //             continue;
        //         }

        //         // タスクまでの距離を計算
        //         int distance = SchedulerUtils::getCostToLocation(
        //             env->curr_states[agent_id].location,
        //             env->curr_states[agent_id].orientation,
        //             env->task_pool[task_id].locations[0]);

        //         if (distance < min_distance)
        //         {
        //             min_distance = distance;
        //             best_task = task_id;
        //         }
        //     }

        //     if (best_task != -1)
        //     {
        //         proposed_schedule[agent_id] = best_task;
        //     }
        // }
    }

    // タスクの全経路のコストを計算する関数
    int calculateTotalTaskCost(const Task &task, int initial_dir)
    {
        int total_cost = 0;
        int current_dir = initial_dir;

        // 各経由地点間のコストを計算
        for (size_t i = 1; i < task.locations.size(); i++)
        {
            int cost = SchedulerUtils::getCostToLocation(
                task.locations[i - 1], current_dir, task.locations[i]);
            total_cost += cost;
            // 次の地点への方向を更新
            // current_dir = SchedulerUtils::getNextDirection(task.locations[i - 1], task.locations[i]);
        }

        return total_cost;
    }
}