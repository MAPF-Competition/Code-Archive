/*
 * graph definition
 */
#pragma once
#include "SharedEnv.h"
#include "utils.hpp"
#include <cstddef>

struct Vertex {
  const int id;    // index for V in Graph
  const int index; // index for U (width * y + x) in Graph
  std::vector<Vertex *> neighbor;
  std::vector<Vertex *> neighbor4;

  Vertex(int _id, int _index);
};
using Vertices = std::vector<Vertex *>;
using Config = std::vector<Vertex *>; // locations for all agents

struct Graph {
  Vertices V; // without nullptr
  Vertices U; // with nullptr, i.e., |U| = width * height
  Vertices U4;
  int width;  // grid width
  int height; // grid height
  Graph();
  // Create graph based on the environment
  Graph(SharedEnvironment *env);
  ~Graph();

  int size() const; // the number of vertices, |V|
};

bool is_same_config(
    const Config &C1,
    const Config &C2); // check equivalence of two configurations
bool any_goal_same_config(const Config &C1, const Config &C2,
                          vector<int> skip_ids);

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config &C) const;
};

std::ostream &operator<<(std::ostream &os, const Vertex *v);