#include "scheduler.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <Logger.h>
#include <thread> 
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <omp.h>
#include <cmath>
#include <chrono>



namespace MyPlanner {
std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_set<int> free_agents_all;
std::unordered_set<int> free_tasks_all;
// static bool first_call = true;
// extern Logger* logger;
// int algorithm_id = 0;
// int agents_id=0;



// 计算块编号
int getBlockId(int x, int y, int b, int cols, int rows) {
    int blocks_per_row = (cols + b - 1) / b;
    int blocks_per_col = (rows + b - 1) / b;
    return (y / b) * blocks_per_row + (x / b);
}

// 动态计算块大小
int computeBlockSize(int totalTasks, int totalAgents, int cols, int rows) {
    int total_cells = cols * rows;
    int total = totalTasks + totalAgents;
    
    if (total == 0) return INT_MAX;
    if (total_cells < total) return 1;
    
    int b = static_cast<int>(sqrt(total_cells / total));
    return std::max(b, 1);
}


// // 动态计算块大小 加阈值
// int computeBlockSize(int totalTasks, int totalAgents, int cols, int rows,SharedEnvironment* env) {
//     int total_cells = cols * rows;
//     int total = totalTasks + totalAgents;
    
//     if (total == 0) return INT_MAX;
//     if (total_cells < total) return 1;

//     int a=std::min(cols/2,rows/2);
//     int b = static_cast<int>(sqrt(total_cells / total));
//     if (b<3) b=3;
//     else if(b>a) b=a;
//     logger->log_info("block size: "+std::to_string(b),env->curr_timestep);
//     return b;
// }

// // 桶排序版本的任务排序
// void sortTasksByDistance(std::vector<int>& tasks, int agent_loc, SharedEnvironment* env) {
//     if (tasks.empty()) return;

//     int max_dist = 0;
//     std::vector<int> distances(tasks.size());
//     for (size_t i = 0; i < tasks.size(); ++i) {
//         int dist = MyPlanner::get_h(env, agent_loc, 
//                        env->task_pool[tasks[i]].locations[0]);
//         distances[i] = dist;
//         max_dist = std::max(max_dist, dist);
//     }

//     typedef boost::container::small_vector<int, 4> Bucket;
//     std::vector<Bucket> buckets(max_dist + 1);
//     for (size_t i = 0; i < tasks.size(); ++i) {
//         buckets[distances[i]].push_back(tasks[i]);
//     }

//     tasks.clear();
//     for (auto& bucket : buckets) {
//         if (!bucket.empty()) {
//             tasks.insert(tasks.end(), bucket.begin(), bucket.end());
//         }
//     }
// }




void sortTasksByDistance(std::vector<int>& tasks, int agent_loc, SharedEnvironment* env) {
    if (tasks.empty()) return;

    int max_dist = 0;
    std::vector<int> distances(tasks.size());
    for (size_t i = 0; i < tasks.size(); ++i) {
        int dist = MyPlanner::get_h(env, agent_loc, 
                       env->task_pool[tasks[i]].locations[0]);
        distances[i] = dist;
        max_dist = std::max(max_dist, dist);
    }

    std::vector<std::vector<int>> buckets(max_dist + 1);
    for (size_t i = 0; i < tasks.size(); ++i) {
        buckets[distances[i]].push_back(tasks[i]);
    }

    tasks.clear();
    for (auto& bucket : buckets) {
        if (!bucket.empty()) {
            tasks.insert(tasks.end(), bucket.begin(), bucket.end());
        }
    }
}

// 获取有效相邻块
std::vector<int> getValidNeighbors(int block_id, int b, int cols, int rows) {
    int blocks_per_row = (cols + b - 1) / b;
    int blocks_per_col = (rows + b - 1) / b;
    int x_block = block_id % blocks_per_row;
    int y_block = block_id / blocks_per_row;
    std::vector<int> neighbors;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = x_block + dx;
            int ny = y_block + dy;
            if (nx >= 0 && nx < blocks_per_row && ny >= 0 && ny < blocks_per_col) {
                neighbors.push_back(ny * blocks_per_row + nx);
            }
        }
    }
    return neighbors;
}

// 改进后的分块匹配算法
void blockBasedMatching(std::unordered_set<int>& free_agents, 
    std::unordered_set<int>& free_tasks,
    std::vector<int>& proposed_schedule, 
    int time_limit, 
    SharedEnvironment* env) {
    
    TimePoint endtime = std::chrono::steady_clock::now() + 
        std::chrono::milliseconds(time_limit);
    int cols = env->cols;
    int rows = env->rows;

    if (free_tasks.empty() || free_agents.empty()) return;

    // 动态计算块尺寸
    // int b = computeBlockSize(free_tasks.size(), free_agents.size(), cols, rows,env);
    int b = computeBlockSize(free_tasks.size(), free_agents.size(), cols, rows);
    b = std::min(b, std::max(cols, rows));

    // 构建块映射
    std::unordered_map<int, std::vector<int>> block_task_map;
    for (int task : free_tasks) {
        int loc = env->task_pool[task].locations[0];
        int x = loc % cols, y = loc / cols;
        block_task_map[getBlockId(x, y, b, cols, rows)].push_back(task);
    }

    std::unordered_map<int, std::vector<int>> block_agent_map;
    for (int agent : free_agents) {
        int loc = env->curr_states[agent].location;
        int x = loc % cols, y = loc / cols;
        block_agent_map[getBlockId(x, y, b, cols, rows)].push_back(agent);
    }

    // 阶段1：块内分配
    int processed_blocks = 0;
    for (auto& [block_id, agents] : block_agent_map) {
        if (std::chrono::steady_clock::now() > endtime) break;
        processed_blocks++;

        // 收集本块及相邻块任务
        std::vector<int> tasks = block_task_map[block_id];
        for (int neighbor : getValidNeighbors(block_id, b, cols, rows)) {
            auto it = block_task_map.find(neighbor);
            if (it != block_task_map.end()) {
                tasks.insert(tasks.end(), it->second.begin(), it->second.end());
            }
        }
        
        // 若仍无任务则添加全局剩余任务
        if (tasks.empty()) {
            tasks.insert(tasks.end(), free_tasks.begin(), free_tasks.end());
        }

        // 为每个智能体分配最近任务
        for (int agent : agents) {
            if (free_agents.find(agent) == free_agents.end()) continue;

            int agent_loc = env->curr_states[agent].location;
            sortTasksByDistance(tasks, agent_loc, env);

            for (int task : tasks) {
                if (free_tasks.count(task)) {
                    proposed_schedule[agent] = task;
                    free_tasks.erase(task);
                    free_agents.erase(agent);
                    break;
                }
            }
        }
    }

    // 阶段2：全局剩余分配（处理跨块未分配情况）
    if (!free_agents.empty() && !free_tasks.empty()) {
        std::vector<int> remaining_agents(free_agents.begin(), free_agents.end());
        std::vector<int> global_tasks(free_tasks.begin(), free_tasks.end());

        for (int agent : remaining_agents) {
            if (std::chrono::steady_clock::now() > endtime) break;
            if (free_agents.find(agent) == free_agents.end()) continue;

            int agent_loc = env->curr_states[agent].location;
            sortTasksByDistance(global_tasks, agent_loc, env);

            for (auto it = global_tasks.begin(); it != global_tasks.end();) {
                if (free_tasks.count(*it)) {
                    proposed_schedule[agent] = *it;
                    free_tasks.erase(*it);
                    free_agents.erase(agent);
                    global_tasks.erase(it);
                    break;
                } else {
                    ++it;
                }
            }
        }
    }

    // logger->log_info("Processed blocks: " + std::to_string(processed_blocks), env->curr_timestep);
}


std::vector<int> hungarian_algorithm(const std::vector<std::vector<int>>& cost_matrix) {
    int n = cost_matrix.size();  // 获取矩阵的行数
    int m = cost_matrix[0].size();  // 获取矩阵的列数

 
    // 初始化辅助数组
    // u和v分别用于存储行和列的顶标
    // p用于存储最终匹配的结果
    // way用于记录增广路径
    std::vector<int> u(n + 1, 0), v(m + 1, 0), p(m + 1, 0), way(m + 1, 0);

    // 对每一行进行处理
    for (int i = 1; i <= n; ++i) {
        std::vector<int> minv(m + 1, std::numeric_limits<int>::max());  // 存储当前行每个列的最小值
        std::vector<bool> used(m + 1, false);  // 标记列是否已被使用
        int j0 = 0;
        p[0] = i;  // 初始化路径起点

        // 开始寻找增广路径
        while (true) {
            used[j0] = true;  // 标记当前列已被使用
            int i0 = p[j0], delta = std::numeric_limits<int>::max(), j1 = 0;
 
            // 寻找未被使用的列中，最小值的列
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    int cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
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
            // 更新u和v数组
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if (p[j0] == 0) break;  // 如果找到增广路径，退出循环
        }


        // 更新路径
        while (j0 != 0) {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        }
    }


    // 构建结果
    std::vector<int> result(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            result[p[j] - 1] = j - 1;
        }
    }

    return result;
}


// std::vector<std::vector<int>> calculate_cost_matrix(const std::vector<int>& agents, const std::vector<int>& tasks, SharedEnvironment* env) {
//     int num_agents = agents.size();
//     int num_tasks = tasks.size();
//     std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks, INT_MAX));
   
//         for (int i = 0; i < num_agents; ++i) {
//             int agent = agents[i];
//             int c_loc = env->curr_states.at(agent).location;
//             for (int j = 0; j < num_tasks; ++j) {
//                 int task_id = tasks[j];
//                 int dist = 0;
//                 int loc = c_loc;
//                 for (int t_loc : env->task_pool[task_id].locations) {
//                     dist += MyPlanner::get_h(env, loc, t_loc);
//                     loc = t_loc;
//                 }
//                 cost_matrix[i][j] = dist;
//             }
//         } 
//     return cost_matrix;
// }

//  距离矩阵(只计算到第一个目标点的距离)
    std::vector<std::vector<int>> calculate_cost_matrix(
        std::vector<int>& agents,
        std::vector<int>& tasks,
        SharedEnvironment* env) 
    {
    
        const size_t num_agents = agents.size();
        const size_t num_tasks = tasks.size();
        std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks));
        // #pragma omp parallel for
        for (size_t i = 0; i < num_agents; ++i) {
            for (size_t j = 0; j < num_tasks; ++j) {
                cost_matrix[i][j] = get_h(env, env->curr_states[agents[i]].location, env->task_pool[tasks[j]].locations[0]);  
            }
        }
    
        return cost_matrix;
    }
void validateProposedSchedule(std::vector<int>& proposed_schedule) {
    std::unordered_set<int> assigned_tasks; // 用于记录已分配的任务

    for (size_t i = 0; i < proposed_schedule.size(); ++i) {
        int task = proposed_schedule[i];

        // 如果任务未被分配过，则标记为已分配
        if (task != -1 && assigned_tasks.find(task) == assigned_tasks.end()) {
            assigned_tasks.insert(task);
        }
        // 如果任务已被分配过，则将当前分配置为 -1（无效）
        else if (task != -1) {
            proposed_schedule[i] = -1;
        }
    }
}

        // 处理空闲集合
void handle_free_sets(SharedEnvironment* env) {
    if (!env) {
        return;
    }

    free_agents_all.clear();
    // free_agents.clear();
    // free_agents_all.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    for (int a = 0; a < env->num_of_agents; a++) {
        if (env->curr_task_schedule[a] == -1||(env->curr_task_schedule[a] != -1 && 
            env->task_pool[env->curr_task_schedule[a]].idx_next_loc == 0 &&
            env->curr_states[a].location != env->task_pool[env->curr_task_schedule[a]].get_next_loc() 
           )) {
            free_agents_all.insert(a);
        }
        // if (env->curr_task_schedule[a] == -1) {
        //     free_agents.insert(a);
        // }
    }

    free_tasks_all.clear();
    // free_tasks.clear();
    // free_tasks_all.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (const auto& task : env->task_pool) {
        if (task.second.agent_assigned == -1 || 
            (task.second.agent_assigned != -1 && task.second.idx_next_loc == 0)) {
            free_tasks_all.insert(task.first);
        }
        // if (task.second.agent_assigned == -1) {
        //     free_tasks.insert(task.first);
        // }
    }
}

void update_schedule(const std::vector<int>& agents, const std::vector<int>& tasks, const std::vector<int>& assignment, std::vector<int>& proposed_schedule) {
    for (int i = 0; i < agents.size(); ++i) {
        int agent = agents[i];
        int task_idx = assignment[i];
        if (task_idx >= 0 && task_idx < tasks.size()) {
            proposed_schedule[agent] = tasks[task_idx];
        } else {
            proposed_schedule[agent] = -1;
        }
    }
}


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env) {
    MyPlanner::init_heuristics(env);
    mt.seed(0);  
    return;
}

void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    clock_t start = clock();
    
    handle_free_sets(env);
    int num_agents = free_agents.size();
    int num_tasks = free_tasks.size();

    int num_agents_all = free_agents_all.size();
    int num_tasks_all = free_tasks_all.size();
    std::vector<int> agents;
    std::vector<int> tasks;
    std::vector<int> assignment;

    // logger->log_info("num_agents: " + std::to_string(num_agents) + ", num_tasks: " + std::to_string(num_tasks),env->curr_timestep);
    if (num_agents_all == 0 || num_tasks_all == 0) {
        return;
    }
    if(env->num_of_agents<350){

        agents.assign(free_agents_all.begin(), free_agents_all.end());
        tasks.assign(free_tasks_all.begin(), free_tasks_all.end());
        std::vector<std::vector<int>> cost_matrix(num_agents_all, std::vector<int>(num_tasks_all, INT_MAX));
        // cost_matrix = calculate_cost_matrix(agents, tasks, env);
        TimePoint cost_matrix_time = std::chrono::steady_clock::now();
        cost_matrix = calculate_cost_matrix(agents, tasks, env);
        // logger->log_info("cost_matrix used: "+std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cost_matrix_time).count()) + "ms",env->curr_timestep);
        assignment = hungarian_algorithm(cost_matrix);
        // algorithm_id=0;
        // agents_id=1;
        // update_schedule(agents, tasks, assignment, proposed_schedule);
        for (int i = 0; i < num_agents_all; ++i) {
            int agent = agents[i];
            int task_idx = assignment[i];
            if (task_idx >= 0 && task_idx < num_tasks_all) {
                proposed_schedule[agent] = tasks[task_idx];
                free_agents_all.erase(agent);
                free_tasks_all.erase(tasks[task_idx]);
            } else {
                proposed_schedule[agent] = -1;
                }
        }
   
    }
    else{
        
        if(num_agents_all*num_tasks_all>200000){
            if(num_agents==0){
                return;
            }
                std::unordered_set<int> free_agents_greedy(free_agents_all.begin(),free_agents_all.end());
                std::unordered_set<int> free_tasks_greedy(free_tasks_all.begin(),free_tasks_all.end());
                blockBasedMatching(free_agents_greedy,free_tasks_greedy,proposed_schedule,time_limit,env);
                validateProposedSchedule(proposed_schedule);
                // algorithm_id=3;
                // agents_id=0;
             
        }
        else{
            agents.assign(free_agents_all.begin(), free_agents_all.end());
            tasks.assign(free_tasks_all.begin(), free_tasks_all.end());
            std::vector<std::vector<int>> cost_matrix(num_agents_all, std::vector<int>(num_tasks_all, INT_MAX));
            // cost_matrix = calculate_cost_matrix(agents, tasks, env);
            TimePoint cost_matrix_time = std::chrono::steady_clock::now();
            cost_matrix = calculate_cost_matrix(agents, tasks, env);
            // logger->log_info("cost_matrix_all used: "+std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cost_matrix_time).count()) + "ms",env->curr_timestep);
            assignment = hungarian_algorithm(cost_matrix);
            // algorithm_id=1;
            // agents_id=1;
            // update_schedule(agents, tasks, assignment, proposed_schedule);
            for (int i = 0; i < num_agents_all; ++i) {
            int agent = agents[i];
            int task_idx = assignment[i];
            if (task_idx >= 0 && task_idx < num_tasks_all) {
                proposed_schedule[agent] = tasks[task_idx];
                free_agents_all.erase(agent);
                free_tasks_all.erase(tasks[task_idx]);
            } 
            else {
                proposed_schedule[agent] = -1;
            }
            }     
    }
    
    }
    
   
    return;

    
}

}