#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 *
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit
 * and adjust for a specified tolerance to account for potential timing errors.
 * It ensures that initialization does not exceed the allocated time.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
int CELL_SIZE = 2;
 std::unordered_map<int, int> task_urgency;
class HungarianAlgorithm {
public:
    // 使用匈牙利算法解决任务分配问题
    double Solve(std::vector<std::vector<double>>& cost_matrix, std::vector<int>& assignment) {
        int n = cost_matrix.size();
        if (n == 0) return 0.0;
        int m = cost_matrix[0].size();
        if (m == 0) return 0.0;

        bool transpose = n > m;
        if (transpose) {
            std::swap(n, m);
            std::vector<std::vector<double>> temp(m, std::vector<double>(n));
            for (int i = 0; i < m; ++i)
                for (int j = 0; j < n; ++j)
                    temp[i][j] = cost_matrix[j][i];
            cost_matrix.swap(temp);
        }

        std::vector<double> u(n+1, 0);
        std::vector<double> v(m+1, 0);
        std::vector<int> p(m+1, 0);
        std::vector<int> way(m+1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(m+1, std::numeric_limits<double>::max());
            std::vector<bool> used(m+1, false);
            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = std::numeric_limits<double>::max();
                int j1 = 0;
                for (int j = 1; j <= m; ++j) {
                    if (!used[j]) {
                        double cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
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
            } while (j0 != 0);
        }

        assignment.assign(n, -1);
        for (int j = 1; j <= m; ++j) {
            if (p[j] != 0) {
                assignment[p[j]-1] = j-1;
            }
        }

        if (transpose) {
            std::vector<int> temp_assignment(m, -1);
            for (int i = 0; i < n; ++i) {
                if (assignment[i] != -1) {
                    temp_assignment[assignment[i]] = i;
                }
            }
            assignment.swap(temp_assignment);
        }

        return -v[0];
    }
};
bool is_central(int loc, SharedEnvironment* env) {
    int center_row = env->rows / 2;
    int center_col = env->cols / 2;
    int row = loc / env->cols;
    int col = loc % env->cols;
    // 中心区域半径设为地图尺寸的1/4
    int radius = std::min(env->rows, env->cols) / 4;
    return std::abs(row - center_row) <= radius && 
           std::abs(col - center_col) <= radius;
}

// 辅助函数：判断是否在狭窄的外围通道
bool is_narrow(int loc, SharedEnvironment* env) {
    int row = loc / env->cols;
    int col = loc % env->cols;
    // 距离边缘3格以内的区域视为狭窄
    return row <= 5 || row >= env->rows-6 ||
           col <= 5 || col >= env->cols-6;
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

/**
 * Plans a task schedule within a specified time limit.
 *
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 *
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */
// 全局变量，用于动态权重调整
std::unordered_map<int, int> task_assignments;
 std::unordered_map<int, int> location_congestion;
 // 任务区域映射
    std::unordered_map<int, int> task_regions;
    std::unordered_map<int, int> agent_regions;

    // 计算任务所在的区域
    int get_region(int location, SharedEnvironment* env) {
        return location / env->cols;  // 按行划分区域
    }

void TaskScheduler::schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
   if(env->map_name[0]=='r'){
     if(free_agents.size()<110){        
          for (auto it = task_assignments.begin(); it != task_assignments.end(); ) {
        int task_id = it->first;
        int agent_id = it->second;
        if (env->task_pool.count(task_id) && env->task_pool[task_id].idx_next_loc == 0) {
            free_tasks.insert(task_id); // 将任务重新加入空闲任务集合
            free_agents.insert(agent_id); // 将代理重新加入空闲代理集合
            it = task_assignments.erase(it); // 从任务分配中移除
        } else {
            ++it;
        }
    }
    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::vector<int> tasks(free_tasks.begin(), free_tasks.end());
    int num_agents = agents.size();
    int num_tasks = tasks.size();

    if (num_agents == 0 || num_tasks == 0) {
        for (int agent_id : free_agents) {
            proposed_schedule[agent_id] = -1;
        }
        return;
    }

    std::vector<std::vector<double>> cost_matrix(num_agents, std::vector<double>(num_tasks, 0.0));

    for (int i = 0; i < num_agents; ++i) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int agent_id = agents[i];
        for (int j = 0; j < num_tasks; ++j) {
            if (std::chrono::steady_clock::now() > endtime) break;
            int task_id = tasks[j];
            if (!env->task_pool.count(task_id)) continue;

            const Task& task = env->task_pool.at(task_id);
            int agent_loc = env->curr_states.at(agent_id).location;

            double total_dist = (DefaultPlanner::get_h(env, env->curr_states.at(agent_id).location, env->task_pool[task_id].locations[0]))*6;
            int current_loc = agent_loc;
            int a = 0;
            for (const auto& step : task.locations) {
                a += DefaultPlanner::get_h(env, current_loc, step);
                current_loc = step;
            }
             if((5000 - env->curr_timestep)<a){
                continue;
            }
          
            if((5000 - (env->curr_timestep+5))<a){
               a+=10;
            }
             total_dist+=a;
             int start_loc = task.locations[0];
            int end_loc = task.locations[task.locations.size() - 1];
            int grid_x = (start_loc / env->cols) / CELL_SIZE;
            int grid_y = (start_loc % env->cols) / CELL_SIZE;
            int egrid_x = (end_loc / env->cols) / CELL_SIZE;
            int egrid_y = (end_loc % env->cols) / CELL_SIZE;
            int density_penalty = 0;
            int density_penaltyend = 0;
             for (const auto& other : env->task_pool) {
                if (other.first == task_id) continue;
                int other_start = other.second.locations[0];
                int other_grid_x = (other_start / env->cols) / CELL_SIZE;
                int other_grid_y = (other_start % env->cols) / CELL_SIZE;
                if (abs(other_grid_x - grid_x) <= 1 && abs(other_grid_y - grid_y) <= 1) {
                    density_penalty++;
                }
                if (abs(other_grid_x - egrid_x) <= 1 && abs(other_grid_y - egrid_y) <= 1){
                     density_penaltyend++;
                }
            }
            // int density_penalty1 = 0;
            // for (int a = 0; a < env->num_of_agents; ++a) {
            //     int loc = env->curr_states[a].location;
            //     int agent_grid_x = (loc / env->cols) / CELL_SIZE;
            //     int agent_grid_y = (loc % env->cols) / CELL_SIZE;
            //     if (abs(agent_grid_x - grid_x) <= 1 && abs(agent_grid_y - grid_y) <= 1) {
            //         density_penalty1++;
            //     }
            // }
           
            double score =  -(0.9 * total_dist - 0.1 * density_penalty+0.2*density_penaltyend);
            cost_matrix[i][j] = -score;
        }
    }

    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    hungarian.Solve(cost_matrix, assignment);

    for (int i = 0; i < num_agents; ++i) {
        if (assignment[i] != -1 && assignment[i] < num_tasks) {
            int agent_id = agents[i];
            int task_id = tasks[assignment[i]];
            if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
                task_assignments[task_id] = agent_id;
                proposed_schedule[agent_id] = task_id;
                free_agents.erase(agent_id);
                free_tasks.erase(task_id);
            }
        }
    }

    for (int agent_id : free_agents) {
        proposed_schedule[agent_id] = -1;
    }
     }
    else{
        int selected_task, task_makespan, dist, agent_loc;
    clock_t start = clock();
     using TaskAssignment = std::tuple<int, int, int>; // (agent_id, task_id, makespan)
    std::priority_queue<TaskAssignment, std::vector<TaskAssignment>, std::greater<>> assignment_queue;
    // Precompute heuristics for all agent-task pairs
    for (int agent_id : free_agents) {
        for (int task_id : free_tasks) {
            if (std::chrono::steady_clock::now() > endtime) break;

            dist = (DefaultPlanner::get_h(env, env->curr_states.at(agent_id).location, env->task_pool[task_id].locations[0]))*6;
            agent_loc = env->curr_states.at(agent_id).location;
           for (int loc : env->task_pool[task_id].locations) {
               dist += DefaultPlanner::get_h(env, agent_loc, loc);
               agent_loc = loc;
           }
            assignment_queue.emplace(dist, agent_id, task_id);
        }
    }

    // Assign tasks from the priority queue while respecting timing constraints
    while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
        auto [makespan, agent_id, task_id] = assignment_queue.top();
        assignment_queue.pop();

        if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
            proposed_schedule[agent_id] = task_id;
            free_agents.erase(agent_id);
            free_tasks.erase(task_id);
        }
    }

    // Unassigned agents remain idle
    for (int agent_id : free_agents) {
        proposed_schedule[agent_id] = -1;
    }
    }
          return;
    }
    if(env->map_name[0]=='P' || env->map_name[0]=='b'){
 if(free_agents.size()<110){
       for (auto it = task_assignments.begin(); it != task_assignments.end(); ) {
        int task_id = it->first;
        int agent_id = it->second;
        if (env->task_pool.count(task_id) && env->task_pool[task_id].idx_next_loc == 0) {
            free_tasks.insert(task_id); // 将任务重新加入空闲任务集合
            free_agents.insert(agent_id); // 将代理重新加入空闲代理集合
            it = task_assignments.erase(it); // 从任务分配中移除
        } else {
            ++it;
        }
      }
   // 动态拥堵计算（包含周边3x3区域）
    std::unordered_map<int, int> congestion_map;
    
    // 统一的任务分配策略
    using TaskAssignment = std::tuple<double, int, int>;
    std::priority_queue<TaskAssignment> assignment_queue;
    
    for (int agent_id : free_agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        
        for (int task_id : free_tasks) {
            if (!env->task_pool.count(task_id)) continue;
            
            const Task& task = env->task_pool.at(task_id);
            int agent_loc = env->curr_states.at(agent_id).location;
            
            // 计算基础路径成本
            double base_cost = (DefaultPlanner::get_h(env, agent_loc, task.locations[0]))*8;
            int current_loc = agent_loc;
            int a = 0;
            for (const auto& step : task.locations) {
                a += DefaultPlanner::get_h(env, current_loc, step);
                current_loc = step;
            }
            if((5000 - env->curr_timestep)<a){
                continue;
            }
            //宽限期,施加惩罚
            if((5000 - (env->curr_timestep+5))<a){
               a+=100;
            }
            base_cost+=a;
              // base_cost-=task_urgency[task_id];
            // 区域特征加权
            double region_weight = 1.0;
            for (const auto& loc : task.locations) {
                if (is_central(loc, env)) {
                    region_weight *= 1.5;  // 中心区域权重提升50%
                    break;
                }
            }
            
            // 狭窄区域惩罚
            double narrow_penalty = 0.0;
            for (const auto& loc : task.locations) {
                if (is_narrow(loc, env)) {
                    narrow_penalty += 50.0; // 每个狭窄点增加惩罚 4999
                }
            }
            
            // 拥堵感知系数
            double congestion_factor = 0.0;
            for (const auto& loc : task.locations) {
                congestion_factor += congestion_map[loc];
            }
            congestion_factor /= task.locations.size();
            
            // 综合评分公式
            double score = -(0.4 * base_cost * region_weight + 
                           0.3 * congestion_factor +
                           0.3 * narrow_penalty);
            
            assignment_queue.emplace(score, agent_id, task_id);
        }
    }

    // 执行任务分配
    while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
        auto [score, agent_id, task_id] = assignment_queue.top();
        assignment_queue.pop();

        if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
             task_assignments[task_id] = agent_id;
            proposed_schedule[agent_id] = task_id;
            free_agents.erase(agent_id);
            free_tasks.erase(task_id);
        }
    }

    // 处理未分配agent
    for (int agent_id : free_agents) {
        proposed_schedule[agent_id] = -1;
    }
     }
     else{
        for (int t_id : free_tasks) {
             task_urgency[t_id]++; // 未分配的任务，急迫程度+1
        }
        std::unordered_set<int>::iterator it = free_agents.begin();
        int counts = 0;
        while (it != free_agents.end())
        {
            if (std::chrono::steady_clock::now() > endtime) break;
            int i = *it;
            int min_task_i = -1;
            int min_task_makespan = INT_MAX;
            counts = 0;
            for (int t_id : free_tasks)
            {
                if((counts%20==0)&&std::chrono::steady_clock::now() > endtime){
                break;
                }
                int dist = 0;
                int c_loc = env->curr_states.at(i).location;
                for (int loc : env->task_pool[t_id].locations)
                {
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }
                 dist-=task_urgency[t_id];
                if (dist < min_task_makespan)
                {
                    min_task_i = t_id;
                    min_task_makespan = dist;
                }
                counts++;
            }
            if (min_task_i != -1)
            {
                proposed_schedule[i] = min_task_i;
                it = free_agents.erase(it);
                free_tasks.erase(min_task_i);
            }
            else
            {
                proposed_schedule[i] = -1;
                it++;
            }
        }
    }
    return;
}
     if(env->map_name[0]=='s' || env->map_name[0]=='w'){
      for (int t_id : free_tasks) {
             task_urgency[t_id]++; // 未分配的任务，急迫程度+1
        }
        std::unordered_set<int>::iterator it = free_agents.begin();
        int counts = 0;
        while (it != free_agents.end())
        {
            if (std::chrono::steady_clock::now() > endtime) break;
            int i = *it;
            int min_task_i = -1;
            int min_task_makespan = INT_MAX;
            counts = 0;
            for (int t_id : free_tasks)
            {
                if((counts%20==0)&&std::chrono::steady_clock::now() > endtime){
                break;
                }
                int dist = 0;
                int c_loc = env->curr_states.at(i).location;
                for (int loc : env->task_pool[t_id].locations)
                {
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }
                 if((5000 - env->curr_timestep)<dist){
                continue;
               }
           
            if((5000 - (env->curr_timestep+5))<dist){
               dist+=100;
            }
            
                 dist-=task_urgency[t_id];
                if (dist < min_task_makespan)
                {
                    min_task_i = t_id;
                    min_task_makespan = dist;
                }
                counts++;
            }
            if (min_task_i != -1)
            {
                proposed_schedule[i] = min_task_i;
                it = free_agents.erase(it);
                free_tasks.erase(min_task_i);
            }
            else
            {
                proposed_schedule[i] = -1;
                it++;
            }
        }
      }
}

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    // Allow at most half of the time_limit for scheduling
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
     // Use at most half of time_limit to compute schedule, -10 for timing error tolerance
    TaskScheduler::schedule_plan(limit, proposed_schedule, env);
}
