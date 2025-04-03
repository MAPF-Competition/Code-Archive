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
void TaskScheduler::initialize(int preprocess_time_limit)
{
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 5 * 4 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    schedule_initialize(limit, env);
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

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit / 5 * 4 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    schedule_plan(limit, proposed_schedule, env);
}

void TaskScheduler::schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
{
    init_heuristics(env, preprocess_time_limit);
    mt.seed(0);
    re_assign_time = 0;
    if (env->map_name == "sortation_large.map" || env->map_name == "warehouse_large.map")
    {
        re_assign_num = re_assign_threshold = 10;
    }
    else if (env->map_name == "Paris_1_256.map")
    {
        re_assign_num = 1;
        re_assign_threshold = 5;
    }
    else if (env->map_name == "brc202d.map")
    {
        re_assign_num = 1;
        re_assign_threshold = 5;
    }
    else if (env->map_name == "random-32-32-20.map")
    {
        re_assign_num = 1;
        re_assign_threshold = 5;
    }
    return;
}

void TaskScheduler::init_heuristic(int map_size, DefaultPlanner::HeuristicTable &ht)
{
    ht.htable.clear();
    ht.htable.resize(map_size, MAX_TIMESTEP);
}

int TaskScheduler::get_h(int source, int target)
{
    return DefaultPlanner::global_heuristictable.at(target).htable[source];
}

int TaskScheduler::cal_task_cost(int task_id)
{
    int task_cost = 0;
    for (int i = 0; i < env->task_pool[task_id].locations.size() - 1; i++)
    {
        task_cost += get_h(env->task_pool[task_id].locations[i], env->task_pool[task_id].locations[i + 1]);
    }
    return task_cost;
}

void TaskScheduler::cal_heuristic(int devNum, SharedEnvironment *env, std::vector<int> &cal_loc)
{
    int *h = cudaScheduler.FGDC(devNum, cal_loc);
    int idx_s = 0;
    for (int i = 0; i < cal_loc.size(); i++)
    {
        int idx_e = idx_s + env->map.size();
        DefaultPlanner::global_heuristictable.at(cal_loc[i]).htable.assign(h + idx_s, h + idx_e);
        DefaultPlanner::global_heuristictable.at(cal_loc[i]).htable[cal_loc[i]] = 0;
        global_heuristictable[loc_dict[cal_loc[i]]].resize(state_num);
        for (int j = 0; j < state_num; j++)
        {
            global_heuristictable[loc_dict[cal_loc[i]]][j] = h[idx_s + loc_dict_rever[j]];
        }
        idx_s = idx_e;
    }
    free(h);
}

void TaskScheduler::init_heuristics(SharedEnvironment *env, int preprocess_time_limit)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(preprocess_time_limit);
    clock_t total_time = clock();
    cudaScheduler.initDevice(device_id, env);
    int *neighborsHost = cudaScheduler.initNeighbors();
    DefaultPlanner::global_heuristictable.resize(env->map.size());
    DefaultPlanner::global_neighbors.resize(env->rows * env->cols);

    loc_dict.resize(env->map.size(), -1);
    state_num = 0;
    for (int loc = 0; loc < env->map.size(); loc++)
    {
        if (env->map[loc] == 0)
        {
            loc_dict[loc] = state_num;
            loc_dict_rever.push_back(loc);
            state_num++;
            int neighbor_idx = 4 * loc;
            for (int i = 0; i < 4; i++)
            {
                if (neighborsHost[neighbor_idx + i] > -1)
                {
                    DefaultPlanner::global_neighbors[loc].push_back(neighborsHost[neighbor_idx + i]);
                }
            }
        }
    }
    free(neighborsHost);
    cudaScheduler.setStateNum(state_num, env);
    cout << "State Num: " << state_num << endl;

    long unsigned int cal_size = (1ULL << 28) / env->map.size();
    global_heuristictable.resize(state_num);
    std::vector<int> cal_loc;
    cal_loc.resize(cal_size);
    long unsigned int idx = 0;
    for (int i = 0; i < env->map.size(); i++)
    {
        if (env->map[i] == 0)
        {
            init_heuristic(env->map.size(), DefaultPlanner::global_heuristictable.at(i));
            cal_loc[idx] = i;
            idx++;
            if (idx == cal_size)
            {
                cal_heuristic(device_id, env, cal_loc);
                idx = 0;
            }
        }
    }
    if (idx > 0)
    {
        cal_loc.resize(idx);
        cal_heuristic(device_id, env, cal_loc);
    }
    cout << "Init Time Usage: " << ((float)(clock() - total_time)) / CLOCKS_PER_SEC << endl;
}

void TaskScheduler::task2cuda(SharedEnvironment *env, int t_id)
{
    std::vector<int> h_map = global_heuristictable.at(loc_dict[env->task_pool[t_id].locations[0]]);
    task_cost_map[t_id] = cal_task_cost(t_id);
    cudaScheduler.task2cuda(device_id, t_id, h_map, task_cost_map[t_id]);
}

void TaskScheduler::agent2cuda(SharedEnvironment *env, int a_id)
{
    int agent_loc = loc_dict[env->curr_states.at(a_id).location];
    cudaScheduler.agent2cuda(device_id, a_id, agent_loc);
}

void TaskScheduler::schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout << "schedule plan limit" << time_limit << endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cuda_wo_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    cuda_wo_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    if (!pre_schedule.empty())
    {
        for (int a_id : env->new_freeagents)
        {
            task_cost_map.erase(pre_schedule[a_id]);
            cudaScheduler.freeTaskId(pre_schedule[a_id]);
        }
    }

    int timeout_detected = 0;
    clock_t start = clock();
    for (auto it = cuda_wo_tasks.begin(); it != cuda_wo_tasks.end();)
    {
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int t_id = *it;
        task2cuda(env, t_id);
        it = cuda_wo_tasks.erase(it);
    }

    for (auto it = cuda_wo_agents.begin(); it != cuda_wo_agents.end();)
    {
        if (timeout_detected % 20 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        timeout_detected++;
        int a_id = *it;
        agent2cuda(env, a_id);
        it = cuda_wo_agents.erase(it);
    }

    re_assign_time += 1;
    if (re_assign_time % re_assign_num == 0)
    {
        agent_task_map.clear();
        agent_cost_map.clear();
        task_agent_map.clear();
        for (int a_id = 0; a_id < env->num_of_agents; a_id++)
        {
            if (timeout_detected % 20 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            timeout_detected++;
            int t_id = proposed_schedule[a_id];
            if (free_agents.find(a_id) == free_agents.end() && env->task_pool[t_id].idx_next_loc == 0)
            {
                agent2cuda(env, a_id);
                proposed_schedule[a_id] = -1;
                free_agents.insert(a_id);
                free_tasks.insert(t_id);
                agent_task_map[a_id] = t_id;
                agent_cost_map[a_id] = get_h(env->curr_states.at(a_id).location, env->task_pool[t_id].locations[0]) + task_cost_map[t_id];
                task_agent_map[t_id] = a_id;
            }
        }
    }

    if (!free_agents.empty())
    {
        int taskNum = free_tasks.size();
        int agentNum = free_agents.size();
        std::vector<int> freeTasksID(free_tasks.begin(), free_tasks.end());
        std::vector<int> agentsHost(free_agents.begin(), free_agents.end());
        thrust::host_vector<int> minValuesHost = cudaScheduler.schedule(device_id, agentsHost, freeTasksID);
        for (int i : minValuesHost)
        {
            if (timeout_detected % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            timeout_detected++;
            int a_id = agentsHost[(int)(i / taskNum)];
            if (free_agents.find(a_id) == free_agents.end())
            {
                continue;
            }
            int t_id = freeTasksID[(int)(i % taskNum)];
            if (free_tasks.find(t_id) == free_tasks.end())
            {
                continue;
            }
            if (agent_task_map.find(a_id) == agent_task_map.end())
            {
                int reassign_cost = get_h(env->curr_states.at(a_id).location, env->task_pool[t_id].locations[0]) + task_cost_map[t_id];
                int minus = agent_cost_map[a_id] - reassign_cost;
                if (minus < re_assign_threshold && minus >= 0 && free_tasks.find(agent_task_map[a_id]) != free_tasks.end())
                {
                    proposed_schedule[a_id] = agent_task_map[a_id];
                    free_agents.erase(a_id);
                    free_tasks.erase(agent_task_map[a_id]);
                    continue;
                }
            }
            proposed_schedule[a_id] = t_id;
            free_agents.erase(a_id);
            free_tasks.erase(t_id);
        }
        pre_schedule = proposed_schedule;
    }

#ifndef NDEBUG
    cout << "Time Usage: " << ((float)(clock() - start)) / CLOCKS_PER_SEC << endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: " << env->new_tasks.size() << endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
    return;
}
