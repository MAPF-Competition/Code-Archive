#pragma once
#include <unordered_map>
#include <unordered_set>
#include "SharedEnv.h"
#include "SchedulerUtils.h"

namespace LocalSearchPlanner
{
    // SchedulerUtilsの必要な要素を使用
    using SchedulerUtils::current_task_assignments;
    using SchedulerUtils::distance_table;
    using SchedulerUtils::global_available_tasks;

    struct TaskDifficulty
    {
        int task_id;
        double density_score;
        int nearby_tasks;
        std::unordered_set<int> nearby_task_ids;

        TaskDifficulty(int id = -1) : task_id(id), density_score(0.0), nearby_tasks(0) {}
    };

    class TaskDensityManager
    {
    public:
        static const int DENSITY_RADIUS = 4;

        static void updateTaskDifficulties(SharedEnvironment *env);
        static double calculateTaskDensity(int task_id);
        static void updateTaskDifficultyOnAssignment(int task_id, bool is_assigned, SharedEnvironment *env);

    private:
        static std::unordered_map<int, TaskDifficulty> task_difficulties;
    };

} // namespace LocalSearchPlanner