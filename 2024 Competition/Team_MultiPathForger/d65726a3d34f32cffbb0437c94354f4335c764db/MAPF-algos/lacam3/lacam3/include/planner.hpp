/*
 * Implementation of LaCAM*
 *
 * references:
 * LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding.
 * Keisuke Okumura.
 * Proc. AAAI Conf. on Artificial Intelligence (AAAI). 2023.
 *
 * Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding.
 * Keisuke Okumura.
 * Proc. Int. Joint Conf. on Artificial Intelligence (IJCAI). 2023.
 *
 * Engineering LaCAM*: Towards Real-Time, Large-Scale, and Near-Optimal
 * Multi-Agent Pathfinding. Keisuke Okumura. Proc. Int. Conf. on Autonomous
 * Agents and Multiagent Systems. 2024.
 */
#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "heuristic.hpp"
#include "hnode.hpp"
#include "instance.hpp"
#include "pibt.hpp"
#include "refiner.hpp"
#include "scatter.hpp"
#include "translator.hpp"
#include "utils.hpp"

struct LACAM_PREFIX_Planner {
  const LACAM_PREFIX_Instance *ins;
  const Deadline *deadline;
  const int seed;
  std::mt19937 MT;
  const int verbose;
  const int depth;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  LACAM_PREFIX_DistTable *D;
  bool delete_dist_table_after_used;

  // heuristic
  Heuristic *heuristic;

  // scatter (SUO)
  Scatter *scatter;

  // configuration generator
  std::vector<LACAM_PREFIX_PIBT *> pibts;

  // for refiner
  int seed_refiner;
  std::list<std::future<LACAM_PREFIX_Solution>> refiner_pool;

  // for search utils
  std::deque<LACAM_PREFIX_HNode *> OPEN;
  std::unordered_map<LACAM_PREFIX_Config, LACAM_PREFIX_HNode *, ConfigHasher> EXPLORED;
  LACAM_PREFIX_HNode *H_init;  // start node
  LACAM_PREFIX_HNode *H_goal;  // goal node

  // parameters
  static bool FLG_SWAP;  // whether to use swap technique in LACAM_PREFIX_PIBT
  static bool
      FLG_STAR;  // whether to refine solutions after initial solution discovery
  static bool FLG_MULTI_THREAD;
  static int SCATTER_MARGIN;  // used in SUO
  static int PIBT_NUM;  // number of LACAM_PREFIX_PIBT run, i.e., Monte-Carlo configuration
                        // generator
  static bool FLG_REFINER;  // whether to use refiners
  static int REFINER_NUM;   // number of refiners
  static bool
      FLG_SCATTER;  // whether to use space utilization optimization (SUO)
  static float RANDOM_INSERT_PROB1;  // inserting the start node
  static float RANDOM_INSERT_PROB2;  // inserting a node after finding the goal
  static bool FLG_RANDOM_INSERT_INIT_NODE;
  static float RECURSIVE_RATE;
  static double RECURSIVE_TIME_LIMIT;

  // for logging
  static int CHECKPOINTS_DURATION;
  static std::string MSG;

  int search_iter;
  int time_initial_solution;
  int cost_initial_solution;
  std::vector<int> checkpoints;

  LACAM_PREFIX_Planner(const LACAM_PREFIX_Instance *_ins, int _verbose = 0,
          const Deadline *_deadline = nullptr, int _seed = 0,
          int _depth = 0,          // used in recursive LaCAM
          LACAM_PREFIX_DistTable *_D = nullptr  // used in recursive LaCAM
  );
  ~LACAM_PREFIX_Planner();
  LACAM_PREFIX_Solution solve();
  bool set_new_config(LACAM_PREFIX_HNode *S, LACAM_PREFIX_LNode *M, LACAM_PREFIX_Config &Q_to);
  LACAM_PREFIX_HNode *create_highlevel_node(const LACAM_PREFIX_Config &Q, LACAM_PREFIX_HNode *parent);
  void rewrite(LACAM_PREFIX_HNode *H_from, LACAM_PREFIX_HNode *H_to);
  int get_edge_cost(const LACAM_PREFIX_Config &C1, const LACAM_PREFIX_Config &C2);
  LACAM_PREFIX_Solution backtrack(LACAM_PREFIX_HNode *H);
  void apply_new_solution(const LACAM_PREFIX_Solution &plan);
  void set_scatter();
  void set_pibt();
  void set_refiner();
  LACAM_PREFIX_Solution get_refined_plan(const LACAM_PREFIX_Solution &plan_origin);
  void update_checkpoints();
  void logging();
};
