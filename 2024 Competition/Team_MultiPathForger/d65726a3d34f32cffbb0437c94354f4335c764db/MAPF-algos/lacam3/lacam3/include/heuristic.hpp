/*
 * heuristic definition
 */

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"

struct Heuristic {
  const LACAM_PREFIX_Instance *ins;
  LACAM_PREFIX_DistTable *D;

  Heuristic(const LACAM_PREFIX_Instance *_ins, LACAM_PREFIX_DistTable *_D);
  int get(const LACAM_PREFIX_Config &C);
};
