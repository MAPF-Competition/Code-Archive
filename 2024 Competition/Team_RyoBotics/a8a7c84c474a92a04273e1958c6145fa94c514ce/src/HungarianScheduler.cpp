#include "HungarianScheduler.h"
void schedule_plan_hungarian(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    // 現在の状態を更新
    SchedulerUtils::updateRunningTasks(env);
    SchedulerUtils::updateTaskAssignments(env);
    SchedulerUtils::updateAgents(env);

    const auto &free_agents = SchedulerUtils::global_available_agents;
    const auto &available_tasks = SchedulerUtils::global_available_tasks;

    if (free_agents.empty() || available_tasks.empty())
    {
        return;
    }
    bool agent_surplus = free_agents.size() >= available_tasks.size();

    // エージェントとタスクを分類
    std::vector<int> assigned_agents;
    std::vector<int> unassigned_agents;
    std::vector<int> assigned_tasks;
    std::vector<int> unassigned_tasks;

    // エージェントの分類
    for (int agent_id : free_agents)
    {
        if (env->curr_task_schedule[agent_id] != -1)
        {
            assigned_agents.push_back(agent_id);
        }
        else
        {
            unassigned_agents.push_back(agent_id);
        }
    }

    // タスクの分類
    for (int task_id : available_tasks)
    {
        const auto &task = env->task_pool.find(task_id)->second;
        if (task.agent_assigned != -1 && task.idx_next_loc == 0)
        {
            assigned_tasks.push_back(task_id);
        }
        else if (task.agent_assigned == -1)
        {
            unassigned_tasks.push_back(task_id);
        }
    }

    // 結果を格納するための割り当てマップ
    std::unordered_map<int, int> final_assignment;

    // if (agent_surplus)
    // {
    //     // 1. 割り当て済みエージェントと割り当て済みタスクのマッチング
    //     if (!assigned_agents.empty() && !assigned_tasks.empty())
    //     {
    //         std::vector<std::vector<int>> cost_matrix1;
    //         cost_matrix1.resize(assigned_agents.size(), std::vector<int>(assigned_tasks.size()));

    //         for (size_t i = 0; i < assigned_agents.size(); ++i)
    //         {
    //             for (size_t j = 0; j < assigned_tasks.size(); ++j)
    //             {
    //                 cost_matrix1[i][j] = SchedulerUtils::evaluateCost(assigned_agents[i], assigned_tasks[j], env);
    //             }
    //         }

    //         SchedulerUtils::Hungarian::Matrix solver1(cost_matrix1, assigned_agents.size() >= assigned_tasks.size());
    //         auto assignment1 = solver1.solve();

    //         for (const auto &[agent_idx, task_idx] : assignment1)
    //         {
    //             if (agent_idx < assigned_agents.size() && task_idx < assigned_tasks.size())
    //             {
    //                 final_assignment[assigned_agents[agent_idx]] = assigned_tasks[task_idx];
    //             }
    //         }
    //     }

    //     // 2. 未割り当てエージェントと未割り当てタスクのマッチング
    //     if (!unassigned_agents.empty() && !unassigned_tasks.empty())
    //     {
    //         std::vector<std::vector<int>> cost_matrix2;
    //         cost_matrix2.resize(unassigned_agents.size(), std::vector<int>(unassigned_tasks.size()));

    //         for (size_t i = 0; i < unassigned_agents.size(); ++i)
    //         {
    //             for (size_t j = 0; j < unassigned_tasks.size(); ++j)
    //             {
    //                 cost_matrix2[i][j] = SchedulerUtils::evaluateCost(unassigned_agents[i], unassigned_tasks[j], env);
    //             }
    //         }

    //         SchedulerUtils::Hungarian::Matrix solver2(cost_matrix2, unassigned_agents.size() >= unassigned_tasks.size());
    //         auto assignment2 = solver2.solve();

    //         for (const auto &[agent_idx, task_idx] : assignment2)
    //         {
    //             if (agent_idx < unassigned_agents.size() && task_idx < unassigned_tasks.size())
    //             {
    //                 final_assignment[unassigned_agents[agent_idx]] = unassigned_tasks[task_idx];
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     // エージェント数が少ない場合は通常のハンガリアン法を適用
    //     std::vector<std::vector<int>> cost_matrix;
    //     std::vector<int> agent_ids(free_agents.begin(), free_agents.end());
    //     std::vector<int> task_ids(available_tasks.begin(), available_tasks.end());

    //     cost_matrix.resize(agent_ids.size(), std::vector<int>(task_ids.size()));
    //     for (size_t i = 0; i < agent_ids.size(); ++i)
    //     {
    //         for (size_t j = 0; j < task_ids.size(); ++j)
    //         {
    //             cost_matrix[i][j] = SchedulerUtils::evaluateCost(agent_ids[i], task_ids[j], env);
    //             // cost_matrix[i][j] = 0;
    //         }
    //     }

    //     SchedulerUtils::Hungarian::Matrix solver(cost_matrix, false);
    //     Assignment assignment = solver.solve();
    //     std::cout << "assignment: " << assignment.size() << std::endl;

    //     for (const auto &[agent_idx, task_idx] : assignment)
    //     {
    //         if (agent_idx < agent_ids.size() && task_idx < task_ids.size())
    //         {
    //             final_assignment[agent_ids[agent_idx]] = task_ids[task_idx];
    //         }
    //     }
    // }

    // 最終的な割り当てをproposed_scheduleに反映
    // proposed_schedule.resize(env->num_agents, -1);
    for (const auto &[agent_id, task_id] : final_assignment)
    {
        proposed_schedule[agent_id] = task_id;
        SchedulerUtils::current_task_assignments[task_id] = agent_id;
    }
    // std::cout << "proposed_schedule: " << std::endl;
    // for (int i = 0; i < proposed_schedule.size(); ++i)
    // {
    //     std::cout << proposed_schedule[i] << " ";
    // }
    // std::cout << std::endl;
}
