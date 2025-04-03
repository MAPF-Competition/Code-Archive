/*
 * graph definition
 */
#pragma once
#include "utils.hpp"

struct LACAM_PREFIX_Vertex {
  const int id;     // index for V in LACAM_PREFIX_Graph
  const int index;  // index for U (width * y + x) in LACAM_PREFIX_Graph
  const int x;
  const int y;
  std::vector<LACAM_PREFIX_Vertex *> neighbor;

  LACAM_PREFIX_Vertex(int _id, int _index, int _x, int _y);
};
using Vertices = std::vector<LACAM_PREFIX_Vertex *>;
using LACAM_PREFIX_Config = std::vector<LACAM_PREFIX_Vertex *>;  // locations for all agents
using LACAM_PREFIX_Path = std::vector<LACAM_PREFIX_Vertex *>;    // path
using Paths = std::vector<LACAM_PREFIX_Path>;

struct LACAM_PREFIX_Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  LACAM_PREFIX_Graph();
  LACAM_PREFIX_Graph(const std::vector<int>& map, int width, int height);
  LACAM_PREFIX_Graph(const std::string &filename);  // taking map filename
  ~LACAM_PREFIX_Graph();

  int size() const;  // the number of vertices, |V|
};

inline int manhattanDist(LACAM_PREFIX_Vertex *a, LACAM_PREFIX_Vertex *b)
{
  return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

bool is_same_config(
    const LACAM_PREFIX_Config &C1,
    const LACAM_PREFIX_Config &C2);  // check equivalence of two configurations

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const LACAM_PREFIX_Config &C) const;
};

std::ostream &operator<<(std::ostream &os, const LACAM_PREFIX_Vertex *v);
std::ostream &operator<<(std::ostream &os, const LACAM_PREFIX_Config &Q);
std::ostream &operator<<(std::ostream &os, const Paths &paths);
