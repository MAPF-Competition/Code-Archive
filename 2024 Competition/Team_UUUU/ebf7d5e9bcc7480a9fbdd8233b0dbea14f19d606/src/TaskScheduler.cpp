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
 std::unordered_map<int, int> task_urgency;
 int CELL_SIZE = 2;
 extern void sigint_handler(int);
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
    return row <= 3 || row >= env->rows-4 ||
           col <= 3 || col >= env->cols-4;
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

void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    // 
     
      for (int t_id : free_tasks) {
             task_urgency[t_id]++; // 未分配的任务，急迫程度+1
        }
     if(free_agents.size()<110){
      if(env->map_name[0]!='w'&& env->map_name[0]!='s'){
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
            double base_cost = 0 ;
            int current_loc = agent_loc;
            for (const auto& step : task.locations) {
                base_cost += DefaultPlanner::get_h(env, current_loc, step);
                current_loc = step;
            }
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
    
}

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    // Allow at most half of the time_limit for scheduling
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
     // Use at most half of time_limit to compute schedule, -10 for timing error tolerance
    schedule_plan(limit, proposed_schedule, env);
}
