#include "CUDAScheduler.h"

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
void CUDAScheduler::initialize(int preprocess_time_limit)
{
  // give at most half of the entry time_limit to scheduler;
  //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
  int limit = preprocess_time_limit / 10 * 7 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
  DefaultPlanner::schedule_initialize(limit, env);
  re_assign_time = 0;
  // re_assign_num = env->re_assign_num;
  // re_assign_threshold = env->re_assign_threshold;
  if (env->map_name == "sortation_large.map")
  {
    large_map = true;
    much_agents = true;
    re_assign_num = 10;
    re_assign_threshold = 20;
  }
  else if (env->map_name == "warehouse_large.map")
  {
    large_map = true;
    much_agents = true;
    re_assign_num = 10;
    re_assign_threshold = 20;
  }
  else if (env->map_name == "brc202d.map")
  {
    large_map = false;
    much_agents = true;
    re_assign_num = 1;
    re_assign_threshold = 40;
  }
  else if (env->map_name == "Paris_1_256.map")
  {
    large_map = false;
    much_agents = true;
    re_assign_num = 1;
    re_assign_threshold = 50;
  }
  else if (env->map_name == "random-32-32-20.map")
  {
    re_assign_num = 1;
    re_assign_threshold = 20;
    large_map = false;
  }
  return;
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

void CUDAScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
  // give at most half of the entry time_limit to scheduler;
  //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
  int limit = time_limit / 10 * 7 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
  schedule_plan(limit, proposed_schedule, env);
  // DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
}

int CUDAScheduler::cal_task_cost(int task_id)
{
  int task_cost = 0;
  for (int i = 0; i < env->task_pool[task_id].locations.size() - 1; i++)
  {
    task_cost += cudaHeuristics->get_h(env->task_pool[task_id].locations[i], env->task_pool[task_id].locations[i + 1]);
  }
  return task_cost;
}

void CUDAScheduler::schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
  // use at most half of time_limit to compute schedule, -10 for timing error tolerance
  // so that the remainning time are left for path planner
  TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
  // cout<<"schedule plan limit" << time_limit <<endl;

  // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
  free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
  free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
  clock_t start = clock();

  TaskPool task_pool = env->task_pool;
  curr_states = env->curr_states;

  if (!pre_schedule.empty())
  {
    for (int a_id : env->new_freeagents)
    {
      task_cost_map.erase(pre_schedule[a_id]);
      cudaHeuristics->freeTaskId(pre_schedule[a_id]);
      pre_schedule[a_id] = -1;
    }
  }
  else
  {
    loc_dict = &cudaHeuristics->loc_dict;
    loc_dict_rever = &cudaHeuristics->loc_dict_rever;
    agent_num = env->num_of_agents;
    pre_schedule.resize(agent_num, -1);
    task_size1 = task_pool[0].locations.size() - 1;
  }

  large_schedule(endtime, proposed_schedule);
  // if (much_agents)
  // {
  //   large_schedule(endtime, proposed_schedule);
  // }
  // else
  // {
  //   small_schedule(endtime, proposed_schedule);
  // }

#ifndef NDEBUG
  cout << "Time Usage: " << ((float)(clock() - start)) / CLOCKS_PER_SEC << endl;
  cout << "new free agents: " << env->new_freeagents.size() << " new tasks: " << env->new_tasks.size() << endl;
  cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
  return;
}

void CUDAScheduler::assign(int a_id, int t_id, std::vector<int> &proposed_schedule,
                           std::unordered_set<int> &assigned_agents, std::unordered_set<int> &assigned_tasks,
                           std::unordered_map<int, int> &agent_task_map, std::unordered_map<int, int> &agent_cost_map,
                           std::unordered_map<int, int> &task_agent_map)
{
  assert(assigned_agents.find(a_id) == assigned_agents.end());
  assert(assigned_tasks.find(t_id) == assigned_tasks.end());
  proposed_schedule[a_id] = t_id;
  pre_schedule[a_id] = t_id;
  free_agents.erase(a_id);
  free_tasks.erase(t_id);
  assigned_agents.insert(a_id);
  assigned_tasks.insert(t_id);
  if (!agent_task_map.empty() && agent_task_map.find(a_id) != agent_task_map.end())
  {
    task_agent_map.erase(agent_task_map[a_id]);
    agent_task_map.erase(a_id);
    agent_cost_map.erase(a_id);
  }
  if (!task_agent_map.empty() && task_agent_map.find(t_id) != task_agent_map.end())
  {
    int pre_a_id = task_agent_map[t_id];
    agent_task_map.erase(pre_a_id);
    agent_cost_map.erase(pre_a_id);
    task_agent_map.erase(t_id);
  }
}

void CUDAScheduler::large_schedule(TimePoint &endtime, std::vector<int> &proposed_schedule)
{
  TaskPool task_pool = env->task_pool;
  curr_states = env->curr_states;
  if (!env->new_tasks.empty())
  {
    std::vector<int> map_ids(env->new_tasks.size());
    std::vector<int> task_locs(env->new_tasks.size());
    std::vector<int> task_cost(env->new_tasks.size());
    int task_idx = 0;
    for (auto t_id : env->new_tasks)
    {
      task_cost_map[t_id] = cal_task_cost(t_id);
      map_ids[task_idx] = cudaHeuristics->insertTaskId(t_id);
      task_locs[task_idx] = loc_dict->at(task_pool[t_id].locations[0]);
      task_cost[task_idx] = task_cost_map[t_id];
      task_idx++;
    }
    cudaHeuristics->task2cuda(map_ids, task_locs, task_cost);
  }

  re_assign_time += 1;
  std::unordered_map<int, int> agent_task_map, agent_cost_map, task_agent_map;
  std::unordered_map<int, int> unfinished_agent_task_map;
  std::vector<int> unfinished_agent, unfinished_agent_cost;
  std::vector<int> agent_trans_ids(agent_num), agent_trans_locs(agent_num);
  int agent_idx = 0;
  bool reassign = re_assign_time % re_assign_num == 0;
  // bool reassign = (!free_agents.empty() || re_assign_time % re_assign_num == 0);
  // re_assign_time = reassign ? 0 : re_assign_time;
  if (reassign && reassign_flag)
  {
    for (int a_id = 0; a_id < agent_num; a_id++)
    {
      int loc = curr_states.at(a_id).location;
      if (free_agents.find(a_id) != free_agents.end())
      {
        agent_trans_ids[agent_idx] = a_id;
        agent_trans_locs[agent_idx] = loc_dict->at(loc);
        agent_idx++;
        continue;
      }
      int t_id = proposed_schedule[a_id];
      assert(t_id != -1);
      if (task_pool[t_id].idx_next_loc == 0 && loc != task_pool[t_id].locations[0])
      {
        proposed_schedule[a_id] = -1;
        pre_schedule[a_id] = -1;

        free_agents.insert(a_id);
        free_tasks.insert(t_id);
        task_agent_map[t_id] = a_id;
        agent_task_map[a_id] = t_id;
        agent_cost_map[a_id] = task_cost_map[t_id] + cudaHeuristics->get_h(loc, task_pool[t_id].locations[0]);

        agent_trans_ids[agent_idx] = a_id;
        agent_trans_locs[agent_idx] = loc_dict->at(loc);
        agent_idx++;
      }
    }
  }
  else
  {
    for (int a_id : free_agents)
    {
      agent_trans_ids[agent_idx] = a_id;
      agent_trans_locs[agent_idx] = loc_dict->at(curr_states.at(a_id).location);
      agent_idx++;
    }
  }
  agent_trans_ids.resize(agent_idx);

  if (!agent_trans_ids.empty())
  {
    cudaHeuristics->agent2cuda(agent_trans_ids, agent_trans_locs);
  }

  int timeout_detected = 0;
  if (!free_agents.empty())
  {
    std::unordered_set<int> assigned_agents, assigned_tasks;
    int taskNum = free_tasks.size();
    int agentNum = free_agents.size();
    std::vector<int> freeTasksID(free_tasks.begin(), free_tasks.end());
    std::vector<int> agentsHost(free_agents.begin(), free_agents.end());
    // auto [minIndicesHost, minValuesHost] = cudaHeuristics->schedule(agentsHost, freeTasksID, unfinished_agent_cost);
    std::vector<int> *minIndicesHost = cudaHeuristics->schedule(agentsHost, freeTasksID, unfinished_agent_cost);

    for (int i : *minIndicesHost)
    {
      if (timeout_detected % 5 == 0 && std::chrono::steady_clock::now() > endtime)
      {
        free(minIndicesHost);
        break;
      }
      if (free_agents.empty())
      {
        free(minIndicesHost);
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
      // int cuda_cost = minValuesHost[i];
      // int host_cost = cudaHeuristics->get_h(env->curr_states.at(a_id).location, env->task_pool[t_id].locations[0]) + task_cost_map[t_id];
      // assert(cuda_cost == host_cost);
      if (agent_task_map.find(a_id) == agent_task_map.end())
      {
        int reassign_cost = cudaHeuristics->get_h(env->curr_states.at(a_id).location, env->task_pool[t_id].locations[0]) + task_cost_map[t_id];
        int minus = agent_cost_map[a_id] - reassign_cost;
        if (minus < re_assign_threshold && minus >= 0 && free_tasks.find(agent_task_map[a_id]) != free_tasks.end())
        {
          assign(a_id, agent_task_map[a_id], proposed_schedule, assigned_agents, assigned_tasks, agent_task_map, agent_cost_map, task_agent_map);
          continue;
        }
      }
      assign(a_id, t_id, proposed_schedule, assigned_agents, assigned_tasks, agent_task_map, agent_cost_map, task_agent_map);
    }
  }
  if (free_agents.empty())
  {
    reassign_flag = true;
  }
  assert(free_tasks.size() - free_agents.size() == agent_num * 0.5);
}
