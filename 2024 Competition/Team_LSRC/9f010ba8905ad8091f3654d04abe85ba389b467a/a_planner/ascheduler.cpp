/*
case of scheduling of N agents & M tasks in map (y,x)

* constraints
  N <= 5000
  M <= 7500 (numTaskReveals==1.5?)
  E: average number of errands for a task

* required:
  1. group tasks and make it proper order in it
  2. assign each group of tasks to each agent

* method:
  1. check if a task is global -> assign differently : O(ME)
    - need to build determination rule between local & global

  2. build a graph to get connected components
    - There is a connection if one end point is near to the other start point: O(M log M)
    - Maybe, SCC can be base component: O(V+E)~=O(M)
    - compute cost for each component,
      then split & merge components for proper size (to get similar makespan)
      : O(ME)

  3. assign each component to the nearest located agent : O(N)
*/

#include "ascheduler.h"
#include "scheduler.h"
// #undef NDEBUG

namespace APlanner
{

std::mt19937 mt;
std::unordered_set<int> global_tasks;
std::unordered_map<int, std::deque<int>> cached_tasks;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::vector<int> free_tasks_list;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
  if (env->map_name=="brc202d.map") {
    DefaultPlanner::schedule_initialize(preprocess_time_limit, env);
    return;
  }

  // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
  APlanner::init_heuristics(env);
  mt.seed(0);
  free_tasks_list.clear();
  free_tasks_list.resize(1000);
}

bool check_task_global(Task& t)
{
  // TODO: implementation
  return false;
}

int check_tasks_dist_square(SharedEnvironment* env, int t1, int t2)
{
  // compute L2-norm distance with static threshold
  int t1_end_idx = env->task_pool[t1].locations.back();
  int t2_start_idx = env->task_pool[t2].locations.front();

  // int r1 = t1_end_idx / rows;
  // int c1 = t1_end_idx % rows;
  // int r2 = t2_start_idx / rows;
  // int c2 = t2_start_idx % rows;
  // int dist_sq = (r1 - r2) * (r1 - r2) + (c1 - c2) * (c1 - c2);
  // return dist_sq;

  int dist = APlanner::get_h(env, t1_end_idx, t2_start_idx);
  if (dist < MAX_TIMESTEP) dist *= dist;

  return dist;
}

void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env)
{
  if (env->map_name=="brc202d.map") {
    DefaultPlanner::schedule_plan(time_limit, proposed_schedule, env);
    return;
  }

  // use at most half of time_limit to compute schedule, -10 for timing error tolerance
  // so that the remainning time are left for path planner
  TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

  // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
  free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
  free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

  int min_task_i, min_task_makespan, dist, c_loc, count;
  clock_t start = clock();
  // choose global tasks first
  for (auto it = free_tasks.begin(); it != free_tasks.end();)
  {
    if (check_task_global(env->task_pool[*it]))
    {
      global_tasks.insert(*it);
      it = free_tasks.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // build connection of graph
  //  * currently implemented naively as O(M^2),
  //	  which need to be optimized with line sweep algorithm
  //  * To get stable performance in computation resources,
  //      "M" should be limited to some upper limit value
  int M = std::min(1000, (int)free_tasks.size());
  int max_cache_size = 0;
  if (!free_tasks.empty() && !free_agents.empty())
  {
    max_cache_size = free_tasks.size() / free_agents.size();
  }
#ifndef NDEBUG
  std::cout << "max_cache_size: " << max_cache_size << '\n';
#endif
  auto ii = free_tasks.begin();
  for (int i = 0; i < M; ++i)
  {
    free_tasks_list[i] = *ii;
    ii++;
  }

  scc_graph graph(M);

  // This value should be tuned w.r.t. map & agent distribution
  int dist_sq_threshold = 16;

  unordered_map<std::string, int> dist_table = {
      {"random-32-32-20.map", 1},
      {"Paris_1_256.map", 64},
      {"brc202d.map", 81},
      {"sortation_large.map", 4},
      {"warehouse_large.map", 4},
  };
  auto dist_sq_pick = dist_table.find(env->map_name);
  if (dist_sq_pick != dist_table.end())
  {
    dist_sq_threshold = dist_sq_pick->second;
  }

#ifndef NDEBUG
  std::cout << "dist_sq_threshold: " << dist_sq_threshold << '\n';
#endif

  for (int i = 0; i < M; ++i)
  {
    for (int j = 0; j < M; ++j)
    {
      if (check_tasks_dist_square(env, free_tasks_list[i], free_tasks_list[j]) <= dist_sq_threshold)
      {
        graph.add_edge(i, j);
      }
    }
  }

  // build scc from graph
  graph.scc();

#ifndef NDEBUG
  std::cout << "size of graph_groups: " << graph.g_grps.size() << '\n';
  std::cout << "size of cached_tasks: " << cached_tasks.size() << '\n';
#endif

  // TODO: split & merge components

  // assign each component to the nearest located agent
  // 	- this also implemented naive O(M*N), need to be optimized
  auto it = free_agents.begin();
  while (it != free_agents.end())
  {
    if (std::chrono::steady_clock::now() > endtime)
    {
      break;
    }
    int i = *it;
    int min_task_k = -1;
    int min_task_id = -1;
    int min_task_dist = INT_MAX;
    int count = 0;
    int ag_loc = env->curr_states.at(i).location;
    // int agy = ag_loc / env->rows;
    // int agx = ag_loc % env->rows;

    // choose cached task assignment
    auto cached = cached_tasks.find(i);
    if (cached != cached_tasks.end())
    {
      int t_id = cached->second.front();
      cached->second.pop_front();

      if (cached->second.empty())
      {
        cached_tasks.erase(i);
#ifndef NDEBUG
        std::cout << "cached(" << i << ") is now empty!!\n";
#endif
      }

#ifndef NDEBUG
      std::cout << "choose cached task assignment for agent(" << i << ")->(" << t_id << ")\n";
#endif
      proposed_schedule[i] = t_id;
      it = free_agents.erase(it);
      continue;
    }

    for (int k = 0; k < M; ++k)
    {
      // skip for already assigned task
      if (free_tasks.count(free_tasks_list[k]) == 0) continue;

      // check for timeout every 10 task evaluations
      if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
      {
        min_task_id = -1;
        break;
      }

      // iterate over the locations (errands) of the task to compute the makespan to finish the task
      // makespan: the time for the agent to complete all the errands of the task t_id in order
      int t_id = free_tasks_list[k];
      int t_start_loc = env->task_pool[t_id].locations.front();
      // int ly = t_start_loc / env->rows;
      // int lx = t_start_loc % env->rows;
      // int dist = (agy - ly) * (agy - ly) + (agx - lx) * (agx - lx);
      int dist = APlanner::get_h(env, ag_loc, t_start_loc);

      // update the new minimum makespan
      if (dist < min_task_dist)
      {
        min_task_k = k;
        min_task_id = t_id;
        min_task_dist = dist;
      }
      count++;
    }

    if (min_task_id != -1)
    {
      proposed_schedule[i] = min_task_id;

      int gid = graph.g_ids[min_task_k];
      if (graph.g_grps[gid].size() > 1)
      {
        // caching linked tasks together
        auto paths = graph.find_walk(min_task_k, max_cache_size);
        if (paths.size() > 1)
        {
#ifndef NDEBUG
          std::cout << "caching linked tasks together for agent#" << i << ":";
#endif
          for (auto p : paths)
          {
            // skip for initial assigned task
            if (p == min_task_k) continue;

#ifndef NDEBUG
            std::cout << ' ' << free_tasks_list[p];
#endif
            cached_tasks[i].push_back(free_tasks_list[p]);
            free_tasks.erase(free_tasks_list[p]);
          }
#ifndef NDEBUG
          std::cout << '\n';
#endif
        }
      }
      free_tasks.erase(min_task_id);
      it = free_agents.erase(it);
    }
    else
    {
      proposed_schedule[i] = -1;
      ++it;
    }
  }

  // std::cout << "task assigned: ";
  // int sz = proposed_schedule.size();
  // for (int i = 0; i < sz; ++i)
  // {
  //   if (proposed_schedule[i] < 0) continue;
  //   std::cout << i << "->" << proposed_schedule[i] << ", ";
  // }
  // std::cout << '\n';

#ifndef NDEBUG
  std::cout << "Time Usage: " << ((float)(clock() - start)) / CLOCKS_PER_SEC << endl;
  std::cout << "new free agents: " << env->new_freeagents.size() << ", new tasks: " << env->new_tasks.size() << endl;
  std::cout << "free agents: " << free_agents.size() << ", free tasks: " << free_tasks.size() << endl;
#endif
}
}  // namespace APlanner
