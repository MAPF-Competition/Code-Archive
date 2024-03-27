﻿#pragma once
#include "ConstraintTable.h"
#include "Instance.h"

class LLNode  // low-level node
{
 public:
  State state;
  int g_val = 0;
  int h_val = 0;
  LLNode* parent = nullptr;
  int num_of_conflicts = 0;
  bool in_openlist = false;
  bool wait_at_goal = false;  // the action is to wait at the goal vertex or not. This is used for
                              // >lenghth constraints
  bool is_goal = false;

  // the following is used to comapre nodes in the OPEN list
  struct compare_node {
    // returns true if n1 > n2 (note -- this gives us *min*-heap).
    bool operator()(const LLNode* n1, const LLNode* n2) const {
      if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
        if (n1->h_val == n2->h_val) {
          return rand() % 2 == 0;  // break ties randomly
        }
        return n1->h_val >=
               n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
      }
      return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
    }
  };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest
      // g-val)

  // the following is used to compare nodes in the FOCAL list
  struct secondary_compare_node {
    bool operator()(const LLNode* n1, const LLNode* n2) const  // returns true if n1 > n2
    {
      if (n1->num_of_conflicts == n2->num_of_conflicts) {
        if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
          if (n1->h_val == n2->h_val) {
            return rand() % 2 == 0;  // break ties randomly
          }
          return n1->h_val >=
                 n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
        }
        return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        // break ties towards smaller f_vals (prefer shorter solutions)
      }
      return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

  LLNode() {}

  LLNode(State state, int g_val, int h_val, LLNode* parent, int num_of_conflicts)
      : state(state),
        g_val(g_val),
        h_val(h_val),
        parent(parent),
        num_of_conflicts(num_of_conflicts) {}

  LLNode(const LLNode& other) { copy(other); }

  ~LLNode() = default;

  void copy(const LLNode& other) {
    state = other.state;
    g_val = other.g_val;
    h_val = other.h_val;
    parent = other.parent;
    num_of_conflicts = other.num_of_conflicts;
    wait_at_goal = other.wait_at_goal;
    is_goal = other.is_goal;
  }

  inline int getFVal() const { return g_val + h_val; }
};

std::ostream& operator<<(std::ostream& os, const LLNode& node);

class SingleAgentSolver {
 public:
  uint64_t accumulated_num_expanded = 0;
  uint64_t accumulated_num_generated = 0;
  uint64_t accumulated_num_reopened = 0;
  uint64_t num_runs = 0;

  int num_collisions = -1;
  double runtime_build_CT = 0;   // runtimr of building constraint table
  double runtime_build_CAT = 0;  // runtime of building conflict avoidance table

  State start_state;
  int goal_location;
  vector<int> my_heuristic;  // this is the precomputed heuristic for this agent
  int compute_heuristic(int from,
                        int to) const  // compute admissible heuristic between two locations
  {
    return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
  }
  void compute_heuristics();

  const Instance& instance;

  virtual Path findPath(const ConstraintTable& constraint_table) = 0;  // return the path
  void findMinimumSetofColldingTargets(vector<int>& goal_table, set<int>& A_target);

  virtual int getTravelTime(int start, int end, const ConstraintTable& constraint_table,
                            int upper_bound) = 0;

  virtual string getName() const = 0;

  list<int> getNextLocations(int curr) const;  // including itself and its neighbors
  list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }
  uint64_t getNumExpanded() const { return num_expanded; }
  // int getStartLocation() const {return instance.start_states[agent]; }
  // int getGoalLocation() const {return instance.goal_locations[agent]; }

  SingleAgentSolver(const Instance& instance, int agent)
      : instance(instance),  // agent(agent),
        start_state(instance.start_states[agent]),
        goal_location(instance.goal_locations[agent]) {
//    compute_heuristics();
  }

  virtual ~SingleAgentSolver() = default;

  void reset() {
    if (num_generated > 0) {
      accumulated_num_expanded += num_expanded;
      accumulated_num_generated += num_generated;
      accumulated_num_reopened += num_reopened;
      num_runs++;
    }
    num_expanded = 0;
    num_generated = 0;
    num_reopened = 0;
  }

 protected:
  uint64_t num_expanded = 0;
  uint64_t num_generated = 0;
  uint64_t num_reopened = 0;
  int min_f_val;  // minimal f value in OPEN
  double w = 1;   // suboptimal bound

  int get_DH_heuristic(int from, int to) const {
    return abs(my_heuristic[from] - my_heuristic[to]);
  }
};
