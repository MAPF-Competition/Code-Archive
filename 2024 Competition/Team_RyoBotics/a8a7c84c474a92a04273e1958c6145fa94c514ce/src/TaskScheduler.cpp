// #include "scheduler.h"
// #include "TPTSScheduler.h"
#include "TaskScheduler.h"
// #include "NewScheduler.h"
#include "const.h"
#include "distance_table.h"
#include <thread>
#include "SchedulerUtils.h"
#include "SchedulerUtilsGame.h"
#include "LNSScheduler.h"
#include "DenseScheduler.h"
#include "ParallelLNSScheduler.h"
#include "LNSSchedulerRND.h"
#include "ParallelLNSSchedulerWeighted.h"
#include "LNSSchedulerWeighted.h"
#include "LNSSchedulerGame.h"
#include "ParallelLNSSchedulerGame.h"
void TaskScheduler::initialize(int preprocess_time_limit)
{
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (env->map_name == "brc202d.map")
    {
        SchedulerUtilsGame::schedule_initialize_SchedulerUtils(limit, env);
    }
    else
    {
        SchedulerUtils::schedule_initialize_SchedulerUtils(limit, env);
    }
    std::cout << "SchedulerUtils::cellToIndex.size(): " << SchedulerUtils::cellToIndex.size() << std::endl;

    // マップに応じて初期化するスケジューラを選択
    // if (false)
    // {
    //     LNSScheduler2::schedule_initialize(limit, env);
    // }
    if (env->map_name == "brc202d.map")
    {
        LNSSchedulerGame::schedule_initialize(limit, env);
        ParallelLNSSchedulerGame::schedule_initialize(limit, env);
    }
    else if (env->map_name == "random-32-32-20.map" && env->num_of_agents == 801)
    {
        std::cout << "random-32-32-20.map" << std::endl;
        DenseScheduler::schedule_initialize(limit, env);
    }
    else if (false && env->map_name == "random-32-32-20.map")
    {
        LNSSchedulerRND::schedule_initialize(limit, env);
    }
    else
    {
        // LNSScheduler::schedule_initialize(limit, env);
        // ParallelLNSScheduler::schedule_initialize(limit, env);
        LNSSchedulerWeighted::schedule_initialize(limit, env);
        ParallelLNSSchedulerWeighted::schedule_initialize(limit, env);
    }
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    auto tm = std::localtime(&time);
    tm->tm_hour += 9; // 日本時間に変換 (UTC+9)
    ss << std::put_time(tm, "%Y-%m-%d_%H-%M-%S");
    std::string filename = "settings/settings_" + env->map_name.substr(0, env->map_name.find(".map")) + "_a" + std::to_string(env->num_of_agents) + "_" + ss.str() + ".json";
    SchedulerUtils::saveSettingsToJson(filename, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // std::cout << "開始時刻: " << std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count() << "us" << std::endl;
    int limit = time_limit / SchedulerUtils::TIME_LIMIT_DIVISION_FACTOR - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (env->map_name == "brc202d.map")
    {
        SchedulerUtilsGame::updateRunningTasks(env);
        SchedulerUtilsGame::updateTaskAssignments(env);
        SchedulerUtilsGame::updateAgents(env);
        SchedulerUtilsGame::updateTaskTotalCosts(env);
        std::cout << "SchedulerUtilsGame::task_free: " << SchedulerUtilsGame::task_free << std::endl;
    }
    else
    {
        SchedulerUtils::updateRunningTasks(env);
        SchedulerUtils::updateTaskAssignments(env);
        SchedulerUtils::updateAgents(env);
        SchedulerUtils::updateTaskTotalCosts(env);
        if (LNSSchedulerWeighted::INCLUDE_FLOW_COST)
        {
            SchedulerUtils::updateTrajLNS();
        }
    }

    // 残り時間に基づいて実行不可能なタスクを除外
    // int remaining_time = LNSScheduler::estimated_sim_time - env->curr_timestep;
    // if (remaining_time <= LNSScheduler::estimated_sim_time / 10)
    // {
    //     SchedulerUtils::removeUnfeasibleTasks(remaining_time, env);
    // }

    // SchedulerUtils::updateOnGoingTasks(env);

    std::cout << "global_available_agents.size(): " << SchedulerUtils::global_available_agents.size() << std::endl;
    std::cout << "global_available_tasks.size(): " << SchedulerUtils::global_available_tasks.size() << std::endl;
    std::cout << "current_task_assignments.size(): " << SchedulerUtils::current_task_assignments.size() << std::endl;
    // int limit = time_limit - 900;
    // std::cout << "time_limit: " << time_limit << std::endl;
    // DefaultPlanner::schedule_plan2(limit, proposed_schedule, env);
    // DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    // LocalSearchPlanner::schedule_plan_LS(limit, proposed_schedule, env);
    // schedule_plan_CBSTA(limit, proposed_schedule, env);
    std::cout << "経過時間(更新): " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << "ms" << std::endl;
    std::chrono::steady_clock::time_point start_time_2 = std::chrono::steady_clock::now();

    // マップに応じてスケジューラを選択
    if (env->map_name == "brc202d.map")
    {
        ParallelLNSSchedulerGame::schedule_plan(limit, proposed_schedule, env);
        std::cout << "ParallelLNSSchedulerGame::schedule_plan" << std::endl;
    }
    else if (env->map_name == "random-32-32-20.map" && env->num_of_agents == 801)
    {
        DenseScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    else if (false && env->map_name == "random-32-32-20.map")
    {
        LNSSchedulerRND::schedule_plan(limit, proposed_schedule, env);
    }
    else
    {
        if (env->curr_timestep % SchedulerUtils::SCHEDULE_UPDATE_INTERVAL == 0 || env->curr_timestep <= SchedulerUtils::INITIAL_TIMESTEP_THRESHOLD)
        {
            SchedulerUtils::setSchedulingTargetAgents(SchedulerUtils::global_available_agents);
            SchedulerUtils::setSchedulingTargetTasks(SchedulerUtils::global_available_tasks);
            // ParallelLNSScheduler::schedule_plan(limit, proposed_schedule, env);
            ParallelLNSSchedulerWeighted::schedule_plan(limit, proposed_schedule, env);
        }
        else
        {
            SchedulerUtils::setSchedulingTargetAgents(SchedulerUtils::unassigned_agents);
            SchedulerUtils::setSchedulingTargetTasks(SchedulerUtils::unassigned_tasks);
        }
        // std::cout << "scheduling_target_tasks.size(): " << SchedulerUtils::scheduling_target_tasks.size() << std::endl;
        // std::cout << "scheduling_target_agents.size(): " << SchedulerUtils::scheduling_target_agents.size() << std::endl;
        // ParallelLNSScheduler::schedule_plan(limit, proposed_schedule, env);
    }
    std::cout << "current timestep: " << env->curr_timestep << std::endl;
    std::cout << "SchedulerUtils::task_free: " << SchedulerUtils::task_free << std::endl;
    double throughput = static_cast<double>(SchedulerUtils::task_free) / (static_cast<double>(env->curr_timestep) + 1);
    std::cout << "throughput: " << std::fixed << std::setprecision(6) << throughput << std::endl;

    // DefaultPlanner::schedule_plan_TPTS(limit, proposed_schedule, env);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    // std::cout << "終了時刻: " << std::chrono::duration_cast<std::chrono::microseconds>(end_time.time_since_epoch()).count() << "us" << std::endl;
    // std::cout << "経過時間(最適化): " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_2).count() << "ms" << std::endl;
    std::cout << "経過時間(全体): " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms" << std::endl;
    // if (env->curr_timestep % 2 == 0)
    // {
    //     DefaultPlanner::schedule_plan_TPTS(limit, proposed_schedule, env);
    // }
    // else
    // {
    //     LocalSearchPlanner::schedule_plan_LS(limit, proposed_schedule, env);
    // }
    // staticを削除し、メンバー変数を使用
    // scheduler.schedule_plan_MultiTPTS(limit, proposed_schedule, env);
}
