/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct LACAM_PREFIX_DistTable {
  const int K;  // number of vertices
  std::vector<std::vector<int>>
      table;  // distance table, index: agent-id & vertex-id
  std::vector<std::queue<LACAM_PREFIX_Vertex *>> OPEN;  // search queue

  int get(const int i, const int v_id);   // agent, vertex-id
  int get(const int i, const LACAM_PREFIX_Vertex *v);  // agent, vertex

  LACAM_PREFIX_DistTable(const LACAM_PREFIX_Instance &ins);
  LACAM_PREFIX_DistTable(const LACAM_PREFIX_Instance *ins);

  void setup(const LACAM_PREFIX_Instance *ins);  // initialization
};
