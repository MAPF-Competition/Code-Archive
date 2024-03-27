#include "../inc/instance.hpp"
#include "SharedEnv.h"
#include "graph.hpp"
#include <vector>

Instance::Instance(Graph *g, SharedEnvironment *env)
    : G(g), starts(Config()), goals(Config()), real_goals(Config()),
      N(env->num_of_agents) {
  for (int i = 0; i < env->num_of_agents; i++) {
    auto start = env->curr_states[i].location;
    starts.push_back(G->U[start]);
    auto end = env->goal_locations[i].front().first;
    goals.push_back(G->U[end]);
    real_goals.push_back(G->U[end]);
  }
  if (N <= 3000) {
    time_window = 10;
  } else {
    time_window = 5;
  }
}

Instance::Instance(Graph *g, SharedEnvironment *env, std::vector<int> _goals)
    : G(g), starts(Config()), goals(Config()), real_goals(Config()),
      N(env->num_of_agents) {
  for (int i = 0; i < env->num_of_agents; i++) {
    auto start = env->curr_states[i].location;
    starts.push_back(G->U[start]);
    auto end = env->goal_locations[i].front().first;
    goals.push_back(G->U[_goals[i]]);
    real_goals.push_back(G->U[end]);
  }
  if (N <= 3000) {
    time_window = 10;
  } else {
    time_window = 5;
  }
}
