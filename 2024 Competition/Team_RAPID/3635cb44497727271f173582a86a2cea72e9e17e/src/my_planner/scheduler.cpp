#include "my_planner/scheduler.h"
#include "util/HeuristicTable.h"

namespace MyPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
static const int threshold_robots = 33;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    // MyPlanner::init_heuristics(env);
    mt.seed(0);
    return;
}

int hungarian_rect(const std::vector<std::vector<int>>& cost_matrix,
                   std::vector<int>& row_assignment) {
    int n = cost_matrix.size();       // 行数(机器人数)
    if (n == 0) {
        row_assignment.clear();
        return 0;
    }
    int m = cost_matrix[0].size();    // 列数(任务数)
    if (m == 0) {
        row_assignment.assign(n, -1);
        return 0;
    }

    // 这里默认 n <= m
    // 如果 n > m，你需要补齐列或转置，这里仅演示 n<=m 的场景
    std::vector<int> u(n+1), v(m+1), p(m+1), way(m+1);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        std::vector<int> minv(m+1, INT_MAX);
        std::vector<bool> used(m+1, false);
        int j0 = 0;
        do {
            used[j0] = true;
            int i0 = p[j0], delta = INT_MAX, j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    int cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // p[j] 表示 列 j 匹配到的 行
    // row_assignment[i] 表示行 i 匹配到的列
    row_assignment.assign(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            int i = p[j];
            row_assignment[i - 1] = j - 1;
        }
    }
    return -v[0]; // 作为最小成本
}

void schedule_plan(int time_limit,
                   std::vector<int>& proposed_schedule,
                   SharedEnvironment* env,
                   const std::shared_ptr<HeuristicTable>& HT)
{
    auto endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    // 收集新自由机器人、任务加入全局集合
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    // free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // 智能体未到达任务的第一个位置，可以重新分配
    // std::cout << "New tasks size: " << env->new_tasks.size() << std::endl;
    if (env->new_tasks.size() < 33) {
        for (auto task: env->new_tasks) {
            // std::cout << "Task " << task << " locations size: " << env->task_pool[task].locations.size() << std::endl;
            bool add = false;
            int agent_loc, cur_task, cur_cost, new_task, new_cost;
            new_task = env->task_pool[task].locations.at(0);
            for (int k = 0; k < proposed_schedule.size(); ++k) {
                // std::cout << "Agent " << k << " task " << proposed_schedule[k] << std::endl;
                if (proposed_schedule[k] != -1) {
                    int t_id = proposed_schedule[k];
                    if (env->task_pool[t_id].idx_next_loc == 0) {
                        agent_loc = env->curr_states[k].location;
                        cur_task = env->task_pool[t_id].locations.at(0);
                        cur_cost = HT->get(agent_loc, cur_task);
                        new_cost = HT->get(agent_loc, new_task);
                        if (new_cost < cur_cost) {
                            proposed_schedule[k] = task;
                            free_tasks.insert(t_id);
                            add = true;
                            break;
                            // std::cout << "Reassign task " << task << " to agent " << k << std::endl;
                        }
                    }
                }
            }
            if (!add) {
                free_tasks.insert(task);
            }
        }
    }
    else {
        for (auto task: env->new_tasks) {
            free_tasks.insert(task);
        }
    }


    if (free_agents.empty() || free_tasks.empty()) {
        return;
    }

    // 转成有序向量
    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::vector<int> tasks(free_tasks.begin(), free_tasks.end());

    int m = agents.size(); // 机器人数量
    int n = tasks.size();  // 任务数量

    if (m == 0 || n == 0) {
        return;
    }

    // 当 m < threshold_robots 使用矩形匈牙利，否则使用贪心
    if (m < threshold_robots) {
        // 构造 m x n cost 矩阵
        std::vector<std::vector<int>> cost_matrix(m, std::vector<int>(n, 0));
        for (int i = 0; i < m; ++i) {
            int agent_id = agents[i];
            int agent_loc = env->curr_states[agent_id].location;
            for (int j = 0; j < n; ++j) {
                int task_id = tasks[j];
                int task_loc = env->task_pool[task_id].locations[0];
                cost_matrix[i][j] = HT->get(agent_loc, task_loc);
            }
        }
        // 如果 m > n，需要你补齐或转置，这里假设 m <= n 即可

        // 调用矩形匈牙利算法
        std::vector<int> row_assignment;
        int total_cost = hungarian_rect(cost_matrix, row_assignment);

        // 应用结果
        for (int i = 0; i < m; ++i) {
            int agent_id = agents[i];
            int col_idx = row_assignment[i];
            if (col_idx >= 0 && col_idx < n) {
                int task_id = tasks[col_idx];
                proposed_schedule[agent_id] = task_id;
                free_agents.erase(agent_id);
                free_tasks.erase(task_id);
            } else {
                proposed_schedule[agent_id] = -1;
            }
        }
        // std::cout << "[RectHungarian] robots=" << m << " tasks=" << n
        //           << " total_cost=" << total_cost << std::endl;
    }
    else {
        // 贪心
        int greedy_cost = 0, count = 0;
        for (int i = 0; i < m; ++i) {
            if (std::chrono::steady_clock::now() > endtime) {
                break;
            }
            int agent_id = agents[i];
            int best_task = -1;
            int best_cost = INT_MAX;
            int agent_loc = env->curr_states[agent_id].location;

            // 遍历所有剩余任务找到最小代价
            for (auto tid : free_tasks) {
                if ((count++) % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                int tloc = env->task_pool[tid].locations[0];
                int cst = HT->get(agent_loc, tloc);
                if (cst < best_cost) {
                    best_cost = cst;
                    best_task = tid;
                }
            }
            if (best_task != -1) {
                proposed_schedule[agent_id] = best_task;
                greedy_cost += best_cost;
                free_agents.erase(agent_id);
                free_tasks.erase(best_task);
            } else {
                proposed_schedule[agent_id] = -1;
            }
        }
        // std::cout << "[Greedy] robots=" << m << " tasks=" << n << " greedy_cost=" << greedy_cost << std::endl;
    }
}
}   // namespace MyPlanner
