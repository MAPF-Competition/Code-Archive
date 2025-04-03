// hungarian scheduler
#include "scheduler.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <Logger.h>


namespace MyPlanner {
std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_set<int> free_agents_all;
std::unordered_set<int> free_tasks_all;
static bool first_call = true;
extern Logger* logger;
int algorithm_id = 0;
int agents_id=0;

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


std::vector<int> greedy(const std::vector<std::vector<int>>& cost_matrix) {
    int n = cost_matrix.size();
    int m = cost_matrix[0].size();
    std::vector<int> result(n, -1);
    std::vector<bool> used(m, false);

    for (int i = 0; i < n; ++i) {
        int min_cost = std::numeric_limits<int>::max();
        int min_index = -1;
        for (int j = 0; j < m; ++j) {
            if (!used[j] && cost_matrix[i][j] < min_cost) {
                min_cost = cost_matrix[i][j];
                min_index = j;
            }
        }
        if (min_index != -1) {
            result[i] = min_index;
            used[min_index] = true;
        }
    }
    return result;
}

std::vector<std::vector<int>> calculate_cost_matrix(const std::vector<int>& agents, const std::vector<int>& tasks, SharedEnvironment* env) {
    int num_agents = agents.size();
    int num_tasks = tasks.size();
    std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks, INT_MAX));

    for (int i = 0; i < num_agents; ++i) {
        int agent = agents[i];
        int c_loc = env->curr_states.at(agent).location;
        for (int j = 0; j < num_tasks; ++j) {
            int task_id = tasks[j];
            int dist = 0;
            int loc = c_loc;
            for (int t_loc : env->task_pool[task_id].locations) {
                dist += MyPlanner::get_h(env, loc, t_loc);
                loc = t_loc;
            }
            cost_matrix[i][j] = dist;
        }
    }

    return cost_matrix;
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
    free_agents.clear();
    free_tasks.clear();
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    free_agents_all.clear();
    free_agents_all.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    // Add agents that are assigned tasks but haven't reached the first task point
    for (int a = 0; a < env->num_of_agents; a++) {
        if (env->curr_task_schedule[a] != -1 && env->curr_states[a].location != env->task_pool[env->curr_task_schedule[a]].get_next_loc() && env->task_pool[env->curr_task_schedule[a]].idx_next_loc==0) {
            free_agents_all.insert(a);
        }
        if(env->curr_task_schedule[a] == -1 ){
            free_agents.insert(a);
        }
    }
    free_tasks_all.clear();
    free_tasks_all.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (auto& task : env->task_pool) {
        if (task.second.agent_assigned==-1||task.second.agent_assigned != -1 && task.second.idx_next_loc == 0) {
            free_tasks_all.insert(task.first);
        }
        if(task.second.agent_assigned==-1){
            free_tasks.insert(task.first);
        }
    }

    int num_agents = free_agents.size();
    int num_tasks = free_tasks.size();

    int num_agents_all = free_agents_all.size();
    int num_tasks_all = free_tasks_all.size();
    std::vector<int> agents;
    std::vector<int> tasks;
    std::vector<int> assignment;

    logger->log_info("num_agents: " + std::to_string(num_agents) + ", num_tasks: " + std::to_string(num_tasks),env->curr_timestep);
    if (num_agents_all == 0 || num_tasks_all == 0) {
        // for (int agent : free_agents_all) {
        //     proposed_schedule[agent] = -1;
        // }
        return;
    }
    if(env->num_of_agents<300){
        agents.assign(free_agents_all.begin(), free_agents_all.end());
        tasks.assign(free_tasks_all.begin(), free_tasks_all.end());
        std::vector<std::vector<int>> cost_matrix(num_agents_all, std::vector<int>(num_tasks_all, INT_MAX));
        cost_matrix = calculate_cost_matrix(agents, tasks, env);
        assignment = hungarian_algorithm(cost_matrix);
        algorithm_id=1;
        agents_id=1;
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
        // clock_t start_h = clock();
    }
    else{
        
        if(num_agents_all*num_tasks_all>200000){
            if(num_agents==0){
                return;
            }
            agents.assign(free_agents.begin(), free_agents.end());
            tasks.assign(free_tasks.begin(), free_tasks.end());
            std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks, INT_MAX));
            cost_matrix = calculate_cost_matrix(agents, tasks, env);
            if(first_call){
                assignment = greedy(cost_matrix);
                algorithm_id=0;
                agents_id=0;
            }
            else{
                assignment = hungarian_algorithm(cost_matrix);
                algorithm_id=1;
                agents_id=0;
            }
             

            // update_schedule(agents, tasks, assignment, proposed_schedule);
            for (int i = 0; i < num_agents; ++i) {
                int agent = agents[i];
                int task_idx = assignment[i];
                if (task_idx >= 0 && task_idx < num_tasks) {
                    proposed_schedule[agent] = tasks[task_idx];
                    free_agents.erase(agent);
                    free_tasks.erase(tasks[task_idx]);
                } else {
                    proposed_schedule[agent] = -1;
                }
            }
            first_call = false; 
        }
        else{
            agents.assign(free_agents_all.begin(), free_agents_all.end());
            tasks.assign(free_tasks_all.begin(), free_tasks_all.end());
            std::vector<std::vector<int>> cost_matrix(num_agents_all, std::vector<int>(num_tasks_all, INT_MAX));
            cost_matrix = calculate_cost_matrix(agents, tasks, env);
            assignment = hungarian_algorithm(cost_matrix);
            algorithm_id=1;
            agents_id=1;
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
    // update_schedule(agents, tasks, assignment, proposed_schedule);
    #ifndef NDEBUG
    cout << "Time Usage: " << ((float)(clock() - start)) / CLOCKS_PER_SEC << endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: " << env->new_tasks.size() << endl;
    cout << "free agents all: " << num_agents_all << " free tasks all: " <<num_tasks_all << endl;
    cout << "free agents: " << num_agents << " free tasks: " << num_tasks << endl;
    #endif
    string algorithm_name;
    if(algorithm_id==0){
     algorithm_name = "greedy";
    } else{
        algorithm_name = "hungarian";
    }
    string agents_name;
    if(agents_id==0){
        agents_name = "free";
    }   else{
        agents_name = "all";
    }

    logger->log_info("Free agents: " + std::to_string(num_agents) + "  Free agents used: " + std::to_string(num_tasks), env->curr_timestep);
    logger->log_info("All agents: " + std::to_string(num_agents_all) + "   All agents used: " + std::to_string(num_tasks_all), env->curr_timestep);
    logger->log_info("The algorithm used is: " + algorithm_name + "   agents used: "    + agents_name,env->curr_timestep);
    return;

    
}

}
