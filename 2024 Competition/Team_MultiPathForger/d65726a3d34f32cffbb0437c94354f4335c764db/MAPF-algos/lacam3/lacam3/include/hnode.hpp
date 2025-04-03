/*
 * high-level node of LaCAM
 */

#pragma once

#include "dist_table.hpp"
#include "lnode.hpp"

// high-level search node
struct LACAM_PREFIX_HNode;
struct CompareHNodePointers {  // for determinism
  bool operator()(const LACAM_PREFIX_HNode *lhs, const LACAM_PREFIX_HNode *rhs) const;
};

struct LACAM_PREFIX_HNode {
  static int COUNT;

  const LACAM_PREFIX_Config C;
  LACAM_PREFIX_HNode *parent;
  std::set<LACAM_PREFIX_HNode *, CompareHNodePointers> neighbor;

  // value
  int g;
  int h;
  int f;

  // for low-level search
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<LACAM_PREFIX_LNode *> search_tree;

  LACAM_PREFIX_HNode(LACAM_PREFIX_Config _C, LACAM_PREFIX_DistTable *D, LACAM_PREFIX_HNode *_parent = nullptr, int _g = 0,
        int _h = 0);
  ~LACAM_PREFIX_HNode();

  LACAM_PREFIX_LNode *get_next_lowlevel_node(std::mt19937 &MT);
};
using HNodes = std::vector<LACAM_PREFIX_HNode *>;

std::ostream &operator<<(std::ostream &os, const LACAM_PREFIX_HNode *H);
