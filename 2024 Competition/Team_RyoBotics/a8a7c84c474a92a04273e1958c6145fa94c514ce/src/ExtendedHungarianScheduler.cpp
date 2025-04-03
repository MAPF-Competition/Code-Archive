#include "ExtendedHungarianScheduler.h"
#include <thread>
void schedule_plan_extended_hungarian(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    // 現在の状態を更新
    // SchedulerUtils::updateRunningTasks(env);
    // SchedulerUtils::updateTaskAssignments(env);
    // SchedulerUtils::updateAgents(env);
    // const auto &available_tasks = SchedulerUtils::global_available_tasks;
    std::vector<int> available_tasks;
    for (const auto &task_id : SchedulerUtils::global_available_tasks)
    {
        if (env->task_pool[task_id].agent_assigned == -1)
        {
            available_tasks.push_back(task_id);
        }
    }
    if (available_tasks.empty())
    {
        return;
    }

    // エージェントを3つのカテゴリに分類
    std::vector<int> free_agents;      // タスクを持っていないエージェント
    std::vector<int> starting_agents;  // タスクを持っているが、まだ開始していないエージェント
    std::vector<int> executing_agents; // タスクを実行中のエージェント
    // エージェントの分類
    for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
    {
        int current_task = env->curr_task_schedule[agent_id];

        if (current_task == -1)
        {
            free_agents.push_back(agent_id);
        }
        else
        {
            auto task_it = env->task_pool.find(current_task);
            if (task_it != env->task_pool.end())
            {
                if (task_it->second.idx_next_loc == 0)
                {
                    starting_agents.push_back(agent_id);
                }
                else
                {
                    executing_agents.push_back(agent_id);
                }
            }
        }
    }
    // エージェント数がタスク数以下かどうかのフラグを作成
    bool is_agent_surplus = env->num_of_agents >= available_tasks.size();

    // 利用可能なタスクのリストを作成
    std::vector<int> agents_ids(free_agents.begin(), free_agents.end());
    agents_ids.insert(agents_ids.end(), starting_agents.begin(), starting_agents.end());
    // std::vector<int> task_ids(available_tasks.begin(), available_tasks.end());
    agents_ids.insert(agents_ids.end(), executing_agents.begin(), executing_agents.end());
    std::vector<int> task_ids;
    if (true)
    {
        task_ids = std::vector<int>(available_tasks.begin(), available_tasks.end());
    }
    else
    {
        // エージェント数分だけタスクを追加
        auto it = available_tasks.begin();
        // for (int i = 0; i < env->num_of_agents && it != available_tasks.end(); ++i, ++it)
        for (it; it != available_tasks.end(); ++it)
        {
            if (env->task_pool[*it].agent_assigned == -1)
            {
                task_ids.push_back(*it);
            }
        }
    }

    // cost_matrixの作成前に追加
    // std::cout << "Global free agents: ";
    // for (const auto &agent : SchedulerUtils::global_free_agents)
    // {
    //     std::cout << agent << " ";
    // }
    // std::cout << std::endl;

    // コストマトリックスの作成
    int total_agents;
    std::vector<std::vector<int>> cost_matrix;
    total_agents = free_agents.size() + starting_agents.size() + executing_agents.size();
    // total_agents = free_agents.size() + executing_agents.size();
    cost_matrix.resize(total_agents, std::vector<int>(task_ids.size(), 0));

    // コストマトリックスの計算
    int row = 0;
    const int UNASSIGNED_TASK_COST_MULTIPLIER = 1.0;
    const int ASSIGNED_TASK_COST_MULTIPLIER = 1.0;
    // costに加算する値
    const int UNASSIGNED_TASK_ADDITIONAL_COST = 0;
    const int ASSIGNED_TASK_ADDITIONAL_COST = 5;
    // フリーエージェントのコスト計算
    for (int agent_id : free_agents)
    {
        for (size_t j = 0; j < task_ids.size(); j++)
        {
            int cost = SchedulerUtils::evaluateCost(agent_id, task_ids[j], env);
            if (env->task_pool[task_ids[j]].agent_assigned != -1)
            {
                cost = static_cast<int>(pow(cost, ASSIGNED_TASK_COST_MULTIPLIER)) + ASSIGNED_TASK_ADDITIONAL_COST;
            }
            else
            {
                cost = static_cast<int>(pow(cost, UNASSIGNED_TASK_COST_MULTIPLIER)) + UNASSIGNED_TASK_ADDITIONAL_COST;
            }
            cost_matrix[row][j] = cost;
        }
        row++;
    }

    // 開始前エージェントのコスト計算
    for (int agent_id : starting_agents)
    {
        for (size_t j = 0; j < task_ids.size(); j++)
        {
            int agent_loc = env->curr_states[agent_id].location;
            int assigned_task_loc = env->task_pool[env->curr_task_schedule[agent_id]].locations[0];
            int cost = SchedulerUtils::evaluateCost(agent_id, task_ids[j], env);
            if (env->curr_task_schedule[agent_id] == task_ids[j])
            {
                cost = static_cast<int>(pow(cost, 1.0));
            }
            // else if (SchedulerUtils::getNextDirection(agent_loc, assigned_task_loc) != SchedulerUtils::getNextDirection(agent_loc, env->task_pool[task_ids[j]].locations[0]))
            // {
            //     cost = 100000;
            // }
            else if (env->task_pool[task_ids[j]].agent_assigned != -1)
            {
                cost = static_cast<int>(pow(cost, ASSIGNED_TASK_COST_MULTIPLIER)) + ASSIGNED_TASK_ADDITIONAL_COST;
            }
            else
            {
                cost = static_cast<int>(pow(cost, UNASSIGNED_TASK_COST_MULTIPLIER)) + UNASSIGNED_TASK_ADDITIONAL_COST;
            }
            cost_matrix[row][j] = cost;
        }
        // if (env->curr_timestep < 20)
        // {
        //     std::cout << "starting agent row: " << row << std::endl;
        //     for (int j = 0; j < task_ids.size(); j++)
        //     {
        //         std::cout << cost_matrix[row][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        row++;
    }
    std::unordered_set<int> high_cost_agents;

    if (true || env->num_of_agents <= available_tasks.size())
    {
        // 実行中エージェントのコスト計算
        for (int agent_id : executing_agents)
        {
            int current_task = env->curr_task_schedule[agent_id];
            auto current_task_it = env->task_pool.find(current_task);
            int end_location = SchedulerUtils::agent_end_state[agent_id].first;
            int end_dir = SchedulerUtils::agent_end_state[agent_id].second;

            int available_time = SchedulerUtils::agent_available_time[agent_id];

            for (size_t j = 0; j < task_ids.size(); j++)
            {
                // 現在のタスク完了時間 + 完了位置から次のタスクまでの距離
                int base_cost = available_time;
                int to_loc = env->task_pool[task_ids[j]].locations[0];
                int travel_cost = SchedulerUtils::getMinDirectionalCost(end_location, end_dir, to_loc);
                if (env->task_pool[task_ids[j]].agent_assigned != -1)
                {
                    travel_cost = static_cast<int>(pow(travel_cost, ASSIGNED_TASK_COST_MULTIPLIER)) + ASSIGNED_TASK_ADDITIONAL_COST;
                }
                else
                {
                    travel_cost = static_cast<int>(pow(travel_cost, UNASSIGNED_TASK_COST_MULTIPLIER)) + UNASSIGNED_TASK_ADDITIONAL_COST;
                }
                cost_matrix[row][j] = static_cast<int>(pow(base_cost, 1.0)) + travel_cost + 10;
            }
            row++;
        }
        size_t start_idx = free_agents.size();
        size_t end_idx = start_idx + starting_agents.size();
        for (size_t i = start_idx; i < end_idx; i++)
        {
            high_cost_agents.insert(i);
        }
        // for (int i : high_cost_agents)
        // {
        //     std::cout << i << " ";
        // }
        // std::cout << std::endl;
    }
    // if (env->curr_timestep < 20)
    // {
    //     std::cout << "cost_matrix: " << cost_matrix.size() << " " << cost_matrix[0].size() << std::endl;
    //     for (int i = 0; i < cost_matrix.size(); i++)
    //     {
    //         for (int j = 0; j < cost_matrix[i].size(); j++)
    //         {
    //             std::cout << cost_matrix[i][j] << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    // }
    // int tmp_agent_id = 50;
    // int tmp_task_id = 0;
    // if (env->task_pool.find(tmp_task_id) != env->task_pool.end())
    // {
    //     int tmp_cost = SchedulerUtils::evaluateCost(tmp_agent_id, tmp_task_id, env);
    //     int tmp_agent_loc = env->curr_states[tmp_agent_id].location;
    //     int tmp_agent_dir = env->curr_states[tmp_agent_id].orientation;
    //     int tmp_task_loc = env->task_pool[tmp_task_id].locations[0];
    //     std::pair<int, int> tmp_task_xy = std::make_pair(tmp_task_loc / env->rows, tmp_task_loc % env->cols);
    //     std::pair<int, int> tmp_agent_xy = std::make_pair(tmp_agent_loc / env->rows, tmp_agent_loc % env->cols);
    //     std::cout << "tmp_agent_id: " << tmp_agent_id << " tmp_task_id: " << tmp_task_id << std::endl;
    //     std::cout << "agent location (x, y): " << tmp_agent_xy.first << " " << tmp_agent_xy.second << " agent direction: " << tmp_agent_dir << std::endl;
    //     std::cout << "task location (x, y): " << tmp_task_xy.first << " " << tmp_task_xy.second << std::endl;
    //     std::cout << "tmp_cost: " << tmp_cost << std::endl;
    // }
    // コストマトリックスの次元を確認
    std::cout << "Agents count: " << agents_ids.size() << std::endl;
    std::cout << "Tasks count: " << task_ids.size() << std::endl;
    std::cout << "Free agents: " << free_agents.size() << std::endl;
    std::cout << "Starting agents: " << starting_agents.size() << std::endl;
    std::cout << "Executing agents: " << executing_agents.size() << std::endl;

    std::unordered_map<int, int> final_assignment;
    SchedulerUtils::Assignment assignment;
    if (true || free_agents.size() != 0)
    {
        // ハンガリアン法で解を求める
        Hungarian::Matrix solver(cost_matrix, total_agents >= task_ids.size(), high_cost_agents);
        // SchedulerUtils::Hungarian::Matrix solver(cost_matrix, total_agents >= task_ids.size(), high_cost_agents);
        assignment = solver.solve();
        // 少し待つ
        // std::cout << "assignment: " << assignment.size() << std::endl;
        // for (const auto &[agent_idx, task_idx] : assignment)
        // {
        //     std::cout << "agent_idx: " << agent_idx << " task_idx: " << task_idx << std::endl;
        // }
        //

        // 結果をfinal_assignmentに反映する際の安全性チェックを追加
        for (const auto &[agent_idx, task_idx] : assignment)
        {
            // 範囲チェックを必ず行う
            if (agent_idx >= agents_ids.size() || task_idx >= task_ids.size())
            {
                std::cout << "Invalid index detected - agent_idx: " << agent_idx
                          << " (max: " << agents_ids.size() - 1
                          << "), task_idx: " << task_idx
                          << " (max: " << task_ids.size() - 1 << ")" << std::endl;
                continue;
            }

            // エージェントIDの取得
            int agent_id = agents_ids[agent_idx];

            // agent_idの有効性チェック
            if (agent_id < 0 || agent_id >= env->num_of_agents)
            {
                std::cout << "Invalid agent_id: " << agent_id << std::endl;
                continue;
            }

            // global_free_agentsの存在チェック
            if (std::find(SchedulerUtils::global_available_agents.begin(),
                          SchedulerUtils::global_available_agents.end(),
                          agent_id) != SchedulerUtils::global_available_agents.end())
            {
                if (task_idx < task_ids.size()) // 追加の安全性チェック
                {
                    final_assignment[agent_id] = task_ids[task_idx];
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < task_ids.size(); i++)
        {
            final_assignment[i] = task_ids[i];
        }
    }

    // final_assignmentの内容を確認するデバッグ出力を追加
    // タスク割り当てが変更された数をカウント
    int changed_count = 0;
    for (const auto &[agent_id, task_id] : final_assignment)
    {
        // std::cout << "agent_id: " << agent_id << " task_id: " << task_id << std::endl;
        if (env->curr_task_schedule[agent_id] != -1 && env->curr_task_schedule[agent_id] != task_id)
        {
            changed_count++;
        }
        proposed_schedule[agent_id] = task_id;
    }
    std::cout << "changed_count: " << changed_count << std::endl;
    // std::cout << "proposed_schedule: " << proposed_schedule.size() << std::endl;
    // for (const auto &task_id : proposed_schedule)
    // {
    //     std::cout << task_id << " ";
    // }
    std::cout << std::endl;
    // free agentsに含まれるagentがfinal_assignmentに含まれていないagentを表示
    // std::cout << "not in final_assignment free agents: " << free_agents.size() << std::endl;
    // for (int agent_id : free_agents)
    // {
    //     if (final_assignment.find(agent_id) == final_assignment.end())
    //     {
    //         std::cout << agent_id << " ";
    //     }
    // }
    std::cout << std::endl;
}