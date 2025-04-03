/*
 * low-level node of LaCAM
 */

#pragma once
#include "graph.hpp"

// low-level search node
struct LACAM_PREFIX_LNode {
  static int COUNT;

  std::vector<int> who;
  Vertices where;
  const int depth;
  LACAM_PREFIX_LNode();
  LACAM_PREFIX_LNode(LACAM_PREFIX_LNode *parent, int i, LACAM_PREFIX_Vertex *v);  // who and where
  ~LACAM_PREFIX_LNode();
};
