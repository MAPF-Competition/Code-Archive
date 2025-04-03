/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct LACAM_PREFIX_Instance {
  LACAM_PREFIX_Graph *G;       // graph
  LACAM_PREFIX_Config starts;  // initial configuration
  LACAM_PREFIX_Config goals;   // goal configuration
  const uint N;   // number of agents
  bool delete_graph_after_used;

    LACAM_PREFIX_Instance(const std::vector<int>& map, int width, int height,
           const uint _N = 1) : G(new LACAM_PREFIX_Graph(map, width, height)), starts(LACAM_PREFIX_Config()), goals(LACAM_PREFIX_Config()), N(_N) {};
           
  LACAM_PREFIX_Instance(LACAM_PREFIX_Graph *_G, const LACAM_PREFIX_Config &_starts, const LACAM_PREFIX_Config &_goals, uint _N);
  LACAM_PREFIX_Instance(const std::string &map_filename,
           const std::vector<int> &start_indexes,
           const std::vector<int> &goal_indexes);
  // for MAPF benchmark
  LACAM_PREFIX_Instance(const std::string &scen_filename, const std::string &map_filename,
           const int _N = 1);
  // random instance generation
  LACAM_PREFIX_Instance(const std::string &map_filename, const int _N = 1,
           const int seed = 0);
  ~LACAM_PREFIX_Instance();

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using LACAM_PREFIX_Solution = std::vector<LACAM_PREFIX_Config>;
