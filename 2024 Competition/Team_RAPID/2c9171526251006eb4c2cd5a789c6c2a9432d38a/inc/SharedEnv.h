#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"
#include <unordered_map>


typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;
typedef std::unordered_map<int, Task> TaskPool;

class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;

    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;   //为每个机器人存储目标位置信息

    int curr_timestep = 0;
    //存储所有智能体当前状态的向量，每个 State 包括位置、时间步和方向等信息。
    vector<State> curr_states;

    //任务池，存储所有已揭示但未完成的任务，键为任务 ID，值为任务对象
    TaskPool task_pool; // task_id -> Task
    //存储在当前时间步中新揭示的任务的 ID 列表。
    vector<int> new_tasks; // task ids of tasks that are newly revealed in the current timestep
    //存储当前时间步中任务完成后释放的智能体 ID。
    vector<int> new_freeagents; // agent ids of agents that are newly free in the current timestep
    //当前任务调度结果，保存每个智能体对应分配到的任务 ID（例如，索引对应智能体 ID，值为分配的任务 ID）。
    vector<int> curr_task_schedule; // the current scheduler, agent_id -> task_id


    //记录规划或初始化函数开始执行时的时间点，便于各个模块计算所花费的时间，从而满足时间限制要求。
    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    SharedEnvironment(){}
};
