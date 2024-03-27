#include <MAPFPlanner.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <execution>
#include <limits>
#include <random>

void MAPFPlanner::initialize(int preprocess_time_limit) {
  const std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  read_config();

  switch (config.map_proc_type) {
    case Configuration::MapProcType::Simple:
      proc_simple_map();
      break;
    case Configuration::MapProcType::Limited:
      proc_limited_map();
      break;
    case Configuration::MapProcType::Special:
      proc_special_map();
      break;
  }

  if (config.export_map) export_map();

  next_locs.resize(env->num_of_agents);
  robots.resize(env->map.size());
  paths.resize(env->map.size());
  wait_times.resize(env->num_of_agents, 0);
  prev_moves.resize(env->num_of_agents);

  const std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now();
  std::cout
      << "Init Time = "
      << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()
      << "[s]" << std::endl;
  std::cout << "Init Time = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  cout << "planner initialize done" << endl;
}

void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
  const std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  actions = std::vector<Action>(env->curr_states.size(), Action::W);

  cout << "Plan path for agents" << endl;
  if (config.reuse_path) {
    plan_with_simple_a_star_and_reuse();
  } else {
    plan_with_simple_a_star();
  }
  const std::chrono::steady_clock::time_point routes_planned =
      std::chrono::steady_clock::now();

  cout << "Resolve conflicts" << endl;
  resolve_conflicts();
  const std::chrono::steady_clock::time_point all_conflicts_resolved =
      std::chrono::steady_clock::now();

  handle_waiting_robots();
  for (int i = 0; i < env->num_of_agents; i++) {
    actions[i] = next_locs[i].action;
  }

  const std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now();
  std::cout << "Rout planning = "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   routes_planned - begin)
                   .count()
            << "[ms]" << std::endl;
  std::cout << "All conflicts resolved = "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   all_conflicts_resolved - begin)
                   .count()
            << "[ms]" << std::endl;

  std::cout
      << "Plan Time = "
      << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()
      << "[s]" << std::endl;
  std::cout << "Plan Time = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
}
