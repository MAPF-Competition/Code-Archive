#pragma once
#include "Tasks.h"
#include "SharedEnv.h"

#include "CUDAHeuristics.h"

#include <future>
#include <mutex>

class CUDAScheduler
{
public:
  SharedEnvironment *env;
  bool large_map, much_agents;

  // { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
  CUDAScheduler(SharedEnvironment *env, CUDAHeuristics *cuda_heuristics)
      : env(env), cudaHeuristics(cuda_heuristics)
  {
    orienttation_map.resize(4);
    orienttation_map[0] = Orientation::RIGHT;
    orienttation_map[1] = Orientation::DOWN;
    orienttation_map[2] = Orientation::LEFT;
    orienttation_map[3] = Orientation::UP;
  };
  CUDAScheduler() { env = new SharedEnvironment(); };
  virtual ~CUDAScheduler() { delete env; };
  virtual void initialize(int preprocess_time_limit);
  virtual void plan(int time_limit, std::vector<int> &proposed_schedule);

  void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);

private:
  CUDAHeuristics *cudaHeuristics;

  std::vector<int> *loc_dict, *loc_dict_rever;
  vector<State> curr_states;
  std::vector<Orientation> orienttation_map;
  int task_size1, agent_num;

  bool reassign_flag = false;
  bool unfinished_flag = false;
  std::unordered_map<int, int> task_cost_map;
  std::unordered_set<int> free_agents, free_tasks;

  int re_assign_time, re_assign_num, re_assign_threshold;
  std::vector<int> pre_schedule;

  std::vector<std::vector<int>> disMatrix;

  void assign(int a_id, int t_id, std::vector<int> &proposed_schedule,
       std::unordered_set<int> &assigned_agents, std::unordered_set<int> &assigned_tasks,
       std::unordered_map<int, int> &agent_task_map, std::unordered_map<int, int> &agent_cost_map,
       std::unordered_map<int, int> &task_agent_map);
  int cal_task_cost(int task_id);
  void large_schedule(TimePoint &endtime, std::vector<int> &proposed_schedule);
  void small_schedule(TimePoint &endtime, std::vector<int> &proposed_schedule);
};