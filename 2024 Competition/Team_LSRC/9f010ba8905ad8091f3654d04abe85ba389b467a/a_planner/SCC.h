#ifndef APLANNER_SCC_H
#define APLANNER_SCC_H

#include <iostream>
#include <vector>

namespace APlanner
{
template <class E>
struct csr
{
  std::vector<int> start;
  std::vector<E> elist;
  csr() = default;
  csr(int n, const std::vector<std::pair<int, E>>& edges)
      : start(n + 1), elist(edges.size())
  {
    for (auto e : edges)
    {
      start[e.first + 1]++;
    }
    for (int i = 1; i <= n; i++)
    {
      start[i] += start[i - 1];
    }
    auto counter = start;
    for (auto e : edges)
    {
      elist[counter[e.first]++] = e.second;
    }
  }
};

struct scc_graph
{
  int _n;
  struct edge
  {
    int to;
  };
  csr<edge> g_;
  std::vector<std::pair<int, edge>> edges;
  std::vector<int> g_ids;
  std::vector<std::vector<int>> g_grps;
  std::vector<int> g_ord;

  explicit scc_graph(int n) : _n(n), g_ids(n), g_ord(n, -1)
  {
    edges.clear();
  }

  int num_vertices() { return _n; }

  void add_edge(int from, int to)
  {
    assert(0 <= from && from < _n);
    assert(0 <= to && to < _n);
    edges.push_back({from, {to}});
  }

  int scc_ids()
  {
    g_ = csr<edge>(_n, edges);
    int now_ord = 0, group_num = 0;
    std::vector<int> visited, low(_n), ord(_n, -1);
    visited.reserve(_n);
    auto dfs = [&](auto self, int v) -> void
    {
      low[v] = ord[v] = now_ord++;
      visited.push_back(v);
      for (int i = g_.start[v]; i < g_.start[v + 1]; i++)
      {
        auto to = g_.elist[i].to;
        if (ord[to] == -1)
        {
          self(self, to);
          low[v] = std::min(low[v], low[to]);
        }
        else
        {
          low[v] = std::min(low[v], ord[to]);
        }
      }
      if (low[v] == ord[v])
      {
        while (true)
        {
          int u = visited.back();
          visited.pop_back();
          ord[u] = _n;
          g_ids[u] = group_num;
          if (u == v) break;
        }
        group_num++;
      }
    };
    for (int i = 0; i < _n; i++)
    {
      if (ord[i] == -1) dfs(dfs, i);
    }
    for (auto& x : g_ids)
    {
      x = group_num - 1 - x;
    }
    return group_num;
  }

  void scc()
  {
    int group_num = scc_ids();
    std::vector<int> counts(group_num);
    for (auto x : g_ids) counts[x]++;
    g_grps.clear();
    g_grps.resize(group_num);
    for (int i = 0; i < group_num; i++)
    {
      g_grps[i].reserve(counts[i]);
    }
    for (int i = 0; i < _n; i++)
    {
      g_grps[g_ids[i]].push_back(i);
    }
  }

  std::vector<int> find_walk(int idx, int max_cache_size)
  {
    // computing scc need to be done prior
    if (g_ids[idx] < 0 || g_grps[g_ids[idx]].size() < 2) return {idx};

    // check maxsize cut
    if (max_cache_size < 1) return {idx};

    // this is done by just get any path (not hamiltonian path)
    std::vector<int> visited;
    int now_ord = 0;
    int v = idx;
    while (g_ord[v] == -1 && now_ord <= max_cache_size)
    {
      g_ord[v] = now_ord++;
      visited.push_back(v);
      for (int i = g_.start[v]; i < g_.start[v + 1]; i++)
      {
        auto to = g_.elist[i].to;
        if (g_ord[to] == -1 && g_grps[g_ids[v]] == g_grps[g_ids[to]])
        {
          v = to;
          break;
        }
      }
    }

    return visited;
  }
};

}  // namespace APlanner

#endif