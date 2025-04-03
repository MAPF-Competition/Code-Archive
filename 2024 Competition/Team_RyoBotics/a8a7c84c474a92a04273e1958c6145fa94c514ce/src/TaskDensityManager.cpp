#include "TaskDensityManager.h"
#include "SchedulerUtils.h"
namespace LocalSearchPlanner
{

    std::unordered_map<int, TaskDifficulty> TaskDensityManager::task_difficulties;

    void TaskDensityManager::updateTaskDifficulties(SharedEnvironment *env)
    {
        task_difficulties.clear();

        // アサイン済みタスクの位置を収集
        std::vector<std::pair<int, int>> assigned_task_positions;
        for (const auto &[task_id, agent_id] : current_task_assignments)
        {
            const Task &task = env->task_pool[task_id];
            if (task.agent_assigned != -1)
            {
                assigned_task_positions.emplace_back(task_id, task.locations[task.idx_next_loc]);
            }
        }

        // 各タスクの密集度を計算
        for (const int task_id : global_available_tasks)
        {
            const Task &task = env->task_pool[task_id];

            TaskDifficulty difficulty(task_id);
            int task_location = task.locations[0];

            // アサイン済みの近隣タスクをカウント
            for (const auto &[other_task_id, other_location] : assigned_task_positions)
            {
                if (other_task_id == task_id)
                    continue;

                int distance = distance_table.getDistance(task_location, other_location);
                if (distance <= DENSITY_RADIUS)
                {
                    difficulty.nearby_tasks++;
                    difficulty.nearby_task_ids.insert(other_task_id);
                    difficulty.density_score += 1.0 / (distance * 0.25 + 1.0);
                }
            }

            task_difficulties[task_id] = difficulty;
        }
    }

    double TaskDensityManager::calculateTaskDensity(int task_id)
    {
        auto it = task_difficulties.find(task_id);
        if (it != task_difficulties.end())
        {
            return 1 * it->second.density_score;
        }
        return 0.0;
    }

    void TaskDensityManager::updateTaskDifficultyOnAssignment(int task_id, bool is_assigned, SharedEnvironment *env)
    {
        if (task_id == -1)
            return;

        const Task &assigned_task = env->task_pool[task_id];
        if (assigned_task.locations.empty())
            return;
        int assigned_location = assigned_task.locations[0];

        // 影響を受ける可能性のあるタスクを更新
        for (const int available_task_id : global_available_tasks)
        {
            if (available_task_id == task_id)
                continue;

            const Task &available_task = env->task_pool[available_task_id];
            int available_location = available_task.locations[0];
            int distance = distance_table.getDistance(assigned_location, available_location);

            if (distance <= DENSITY_RADIUS)
            {
                auto &difficulty = task_difficulties[available_task_id];

                if (is_assigned)
                {
                    if (difficulty.nearby_task_ids.find(task_id) == difficulty.nearby_task_ids.end())
                    {
                        difficulty.nearby_tasks++;
                        difficulty.nearby_task_ids.insert(task_id);
                        difficulty.density_score += 1.0 / (distance + 1.0);
                    }
                }
                else
                {
                    if (difficulty.nearby_task_ids.find(task_id) != difficulty.nearby_task_ids.end())
                    {
                        difficulty.nearby_tasks--;
                        difficulty.nearby_task_ids.erase(task_id);
                        difficulty.density_score -= 1.0 / (distance + 1.0);
                    }
                }
            }
        }
    }

} // namespace LocalSearchPlanner