/*
 * instance definition
 */
#pragma once
#include <deque>
#include <random>

#include "SharedEnv.h"
#include "graph.hpp"
#include "utils.hpp"

struct Instance {
  Graph* G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  Config real_goals;   // goal configuration
  const uint N;   // number of agents
  uint time_window;

  // For MAPF-COMPETITION
  Instance(Graph* g, SharedEnvironment* env);
  Instance(Graph* g, SharedEnvironment* env, std::vector<int> _goals);
  ~Instance() {}
};

// solution: a sequence of configurations
using Solution = std::deque<Config>;
