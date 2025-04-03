/*
 * implementation of LACAM_PREFIX_PIBT
 *
 * references:
 * Priority Inheritance with Backtracking for Iterative Multi-agent LACAM_PREFIX_Path
 * Finding. Keisuke Okumura, Manao Machida, Xavier Défago & Yasumasa Tamura.
 * Artificial Intelligence (AIJ). 2022.
 */
#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "scatter.hpp"
#include "utils.hpp"

struct LACAM_PREFIX_PIBT {
  const LACAM_PREFIX_Instance *ins;
  std::mt19937 MT;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  LACAM_PREFIX_DistTable *D;

  // specific to LACAM_PREFIX_PIBT
  const int NO_AGENT;
  std::vector<int> occupied_now;                // for quick collision checking
  std::vector<int> occupied_next;               // for quick collision checking
  std::vector<std::array<LACAM_PREFIX_Vertex *, 5>> C_next;  // next location candidates
  std::vector<float> tie_breakers;              // random values, used in LACAM_PREFIX_PIBT

  // swap, used in the LaCAM* paper
  bool flg_swap;

  // scatter
  Scatter *scatter;

  LACAM_PREFIX_PIBT(const LACAM_PREFIX_Instance *_ins, LACAM_PREFIX_DistTable *_D, int seed = 0, bool _flg_swap = true,
       Scatter *_scatter = nullptr);
  ~LACAM_PREFIX_PIBT();

  bool set_new_config(const LACAM_PREFIX_Config &Q_from, LACAM_PREFIX_Config &Q_to,
                      const std::vector<int> &order);
  bool funcPIBT(const int i, const LACAM_PREFIX_Config &Q_from, LACAM_PREFIX_Config &Q_to);
  int is_swap_required_and_possible(const int ai, const LACAM_PREFIX_Config &Q_from,
                                    LACAM_PREFIX_Config &Q_to);
  bool is_swap_required(const int pusher, const int puller,
                        LACAM_PREFIX_Vertex *v_pusher_origin, LACAM_PREFIX_Vertex *v_puller_origin);
  bool is_swap_possible(LACAM_PREFIX_Vertex *v_pusher_origin, LACAM_PREFIX_Vertex *v_puller_origin);
};
