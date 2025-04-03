#include "../include/heuristic.hpp"

Heuristic::Heuristic(const LACAM_PREFIX_Instance *_ins, LACAM_PREFIX_DistTable *_D) : ins(_ins), D(_D) {}

int Heuristic::get(const LACAM_PREFIX_Config &Q)
{
  auto cost = 0;
  for (size_t i = 0; i < ins->N; ++i) cost += D->get(i, Q[i]);
  return cost;
}
