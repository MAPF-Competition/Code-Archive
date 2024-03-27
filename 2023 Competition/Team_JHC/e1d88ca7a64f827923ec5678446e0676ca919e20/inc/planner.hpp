/*
 * LaCAM algorithm
 */
#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

// low-level search node
struct LConstraint {
  std::vector<int> who;
  Vertices where;
  const int depth;
  LConstraint();
  LConstraint(LConstraint *parent, int i, Vertex *v); // who and where
  ~LConstraint();
};

// high-level search node
struct Node {
  const Config C;
  int depth;
  Node *parent;

  // for low-level search
  std::vector<std::vector<int>> pathes;
  std::vector<int> conflicts;
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<LConstraint *> search_tree;

  Node(Config _C, DistTable &D, Node *_parent = nullptr);
  ~Node();
};
using Nodes = std::vector<Node *>;

// PIBT agent
struct Agent {
  const int id;
  Vertex *v_now;  // current location
  Vertex *v_next; // next location
  Agent(int _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent *>;

// next location candidates, for saving memory allocation
using Candidates = std::vector<std::array<Vertex *, 5>>;

struct Planner {
  const Instance *ins;
  std::mt19937 *MT;
  const int verbose;

  // solver utils
  const int N; // number of agents
  const int V_size;
  DistTable D;
  Candidates C_next;               // next location candidates
  std::vector<float> tie_breakers; // random values, used in PIBT
  Agents A;
  Agents occupied_now;  // for quick collision checking
  Agents occupied_next; // for quick collision checking

  Planner(const Instance *_ins, std::mt19937 *_MT, int _verbose = 0);
  Solution solve(int deadline, vector<int> skip_ids);
  bool get_new_config(Node *S, LConstraint *M);
  bool funcPIBT(Agent *ai);
};

// main function
Solution solve(const Instance &ins, const int verbose = 0,
               std::mt19937 *MT = nullptr, int deadline = 1000, vector<int> skip_ids = {});
