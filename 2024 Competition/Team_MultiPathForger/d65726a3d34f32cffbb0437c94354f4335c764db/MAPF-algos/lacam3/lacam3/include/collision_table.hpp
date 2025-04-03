/*
 * fast collision checking, used in SUO and refinner
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct LACAM_PREFIX_CollisionTable {
  // vertex, time, agents
  std::vector<std::vector<std::vector<int>>> body;
  std::vector<std::vector<int>> body_last;
  int collision_cnt;
  int N;

  LACAM_PREFIX_CollisionTable(const LACAM_PREFIX_Instance *ins);
  ~LACAM_PREFIX_CollisionTable();

  int getCollisionCost(const LACAM_PREFIX_Vertex *v_from, const LACAM_PREFIX_Vertex *v_to,
                       const int t_from);
  void enrollPath(const int i, LACAM_PREFIX_Path &path);
  void clearPath(const int i, LACAM_PREFIX_Path &path);
  void shrink();
};
