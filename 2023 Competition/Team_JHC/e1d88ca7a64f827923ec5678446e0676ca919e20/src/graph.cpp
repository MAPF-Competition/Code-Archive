#include "../inc/graph.hpp"
#include "SharedEnv.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

Vertex::Vertex(int _id, int _index)
    : id(_id), index(_index), neighbor(Vertices()) {}

Graph::Graph() : V(Vertices()), width(0), height(0) {}
Graph::~Graph() {
  for (auto &v : V)
    if (v != nullptr)
      delete v;
  V.clear();
}

// to load graph
static const std::regex r_height = std::regex(R"(height\s(\d+))");
static const std::regex r_width = std::regex(R"(width\s(\d+))");
static const std::regex r_map = std::regex(R"(map)");

Graph::Graph(SharedEnvironment *env)
    : V(Vertices()), width(env->cols), height(env->rows) {
  U = Vertices(width * height, nullptr);
  U4 = Vertices();
  // Loop env->map and create vertices
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; x++) {
      auto index = width * y + x;
      if (env->map[index] == 1)
        continue; // obstacle
      auto v = new Vertex(V.size(), index);
      V.push_back(v);
      U[index] = v;
    }
  }

  // create edges
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr)
        continue;
      // left
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr)
          v->neighbor.push_back(u);
      }
      // right
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr)
          v->neighbor.push_back(u);
      }
      // up
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr)
          v->neighbor.push_back(u);
      }
      // down
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr)
          v->neighbor.push_back(u);
      }

      if (v->neighbor.size() > 2) {
        U4.push_back(v);
      }
    }
  }

  auto manhattan_distance = [this](int index1, int index2) {
    return abs(index1 % width - index2 % width) +
           abs(index1 / width - index2 / width);
  };

  // loop all U4 vertices and find the nearest vertices
  // store them in neighbor4
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr)
        continue;
      // sort u4 by manhattan distance to v
      std::sort(U4.begin(), U4.end(),
                [v, &manhattan_distance](const Vertex *u1, const Vertex *u2) {
                  return manhattan_distance(u1->index, v->index) <
                         manhattan_distance(u2->index, v->index);
                });

      // add the nearest vertices to v->neighbor4
      // v->neighbor4 = U4;
      auto size = min((unsigned long)20, U4.size());
      for (int i = 0; i < size; ++i) {
        v->neighbor4.push_back(U4[i]);
      }
    }
  }
}

int Graph::size() const { return V.size(); }

bool is_same_config(const Config &C1, const Config &C2) {
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    if (C1[i]->id != C2[i]->id)
      return false;
  }
  return true;
}

bool any_goal_same_config(const Config &C1, const Config &C2,
                          vector<int> skip_ids) {
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    if (std::find(skip_ids.begin(), skip_ids.end(), i) != skip_ids.end())
      continue;
    if (C1[i]->id == C2[i]->id)
      return true;
  }
  return false;
}

uint ConfigHasher::operator()(const Config &C) const {
  uint hash = C.size();
  for (auto &v : C) {
    hash ^= v->id + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  return hash;
}

std::ostream &operator<<(std::ostream &os, const Vertex *v) {
  os << v->index;
  return os;
}
