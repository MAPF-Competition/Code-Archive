/*
 * Implementation of SUO
 *
 * references:
 * Optimizingspaceutilizationformoreeffective multi-robot path planning.
 * Shuai D Han and Jingjin Yu.
 * In Proceedings of IEEE International Conference on Robotics and Automation
 * (ICRA). 2022.
 */
#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "utils.hpp"

struct Scatter {
  const LACAM_PREFIX_Instance *ins;
  const Deadline *deadline;
  std::mt19937 MT;
  const int verbose;
  const int N;
  const int V_size;
  const int T;  // makespan lower bound
  LACAM_PREFIX_DistTable *D;
  const int cost_margin;
  int sum_of_path_length;

  // outcome
  std::vector<LACAM_PREFIX_Path> paths;
  // agent, vertex-id, next vertex
  std::vector<std::unordered_map<int, LACAM_PREFIX_Vertex *>> scatter_data;

  // collision data
  LACAM_PREFIX_CollisionTable CT;

  void construct();

  Scatter(const LACAM_PREFIX_Instance *_ins, LACAM_PREFIX_DistTable *_D, const Deadline *_deadline,
          const int seed = 0, int _verbose = 0, int _cost_margin = 2);
};
