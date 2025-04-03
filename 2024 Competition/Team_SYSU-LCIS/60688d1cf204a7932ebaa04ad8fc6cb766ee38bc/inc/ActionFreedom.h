#pragma once
#include <thread>

#include "Types.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "ActionModel.h"

struct AStarNode
{
  AStarNode(int loc, int orientation, int time_step, float g_cost, float h_cost, std::shared_ptr<AStarNode> parent)
      : loc(loc), orientation(orientation), time_step(time_step), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost + h_cost), parent(parent) {}
  int loc, orientation, time_step;
  float g_cost, h_cost, f_cost;
  int tie_breaker = rand();
  std::shared_ptr<AStarNode> parent;
  bool operator<(const AStarNode &other) const
  {
    if (f_cost == other.f_cost)
    {
      return tie_breaker > other.tie_breaker;
    }
    return f_cost < other.f_cost;
  }
};

struct AStarNodeComparator
{
  bool operator()(const std::shared_ptr<AStarNode> &lhs, const std::shared_ptr<AStarNode> &rhs) const
  {
    if (lhs->f_cost == rhs->f_cost)
    {
      if (lhs->h_cost == rhs->h_cost)
      {
        return lhs->tie_breaker > rhs->tie_breaker;
      }
      return lhs->h_cost < rhs->h_cost;
    }
    return lhs->f_cost < rhs->f_cost;
  }
};

struct GraphEdge
{
  GraphEdge()
  {
    edge = std::array<int, 2>{-1, -1};
    isValid = false;
  };
  GraphEdge(int src, int dst)
  {
    edge = std::array<int, 2>{src, dst};
    isValid = true;
  }
  bool isValid;
  std::array<int, 2> edge;

  void reset()
  {
    edge = std::array<int, 2>{-1, -1};
    isValid = false;
  }

  bool operator==(const GraphEdge &other) const
  {
    if (!isValid || !other.isValid)
    {
      return false;
    }
    return (edge[0] == other.edge[0] && edge[1] == other.edge[1]) ||
           (edge[0] == other.edge[1] && edge[1] == other.edge[0]);
  };
};

namespace std
{
  template <>
  struct hash<GraphEdge>
  {
    size_t operator()(const GraphEdge &graphEdge) const
    {
      if (!graphEdge.isValid)
      {
        return std::size_t{0};
      }
      std::size_t hashValue = 0;
      std::hash<int> hashInt;
      int minVertex = std::min(graphEdge.edge[0], graphEdge.edge[1]);
      int maxVertex = std::max(graphEdge.edge[0], graphEdge.edge[1]);
      hashValue = hashInt(minVertex) ^ (hashInt(maxVertex) << 1);
      return hashValue;
    }
  };
};

struct Occupy
{
  Occupy() : loc(-1), orientation(-1), edge(), children(0) {};
  Occupy(int loc, int orientation) : loc(loc), orientation(orientation) {};
  Occupy(Occupy &&other) noexcept
      : loc(other.loc), orientation(other.orientation), edge(std::move(other.edge)), children(std::move(other.children))
  {
    other.loc = 0;         // 或者其他适当的默认值
    other.orientation = 0; // 或者其他适当的默认值
    other.children.clear();
  };

  Occupy &operator=(Occupy &&other) noexcept
  {
    if (this != &other) // 防止自我赋值
    {
      // 转移其他对象的成员
      loc = other.loc;
      orientation = other.orientation;
      edge = std::move(other.edge);
      children = std::move(other.children);

      // 将其他对象的成员设置为默认值
      other.loc = 0;         // 或者其他适当的默认值
      other.orientation = 0; // 或者其他适当的默认值
      other.children.clear();
    }
    return *this;
  }

  ~Occupy()
  {
    for (auto child : children)
    {
      delete child;
    }
  }

  GraphEdge *edge;
  int loc, orientation;
  std::vector<Occupy *> children;
};

class ActionFreedom
{
public:
  int agent_num;
  std::unordered_map<int, int> curr_cell_constraints;
  std::unordered_map<GraphEdge, int> next_edge_constraints;
  std::array<int, 4> moves;
  std::vector<int> freedom;
  std::vector<Occupy *> occupy_list;
  SharedEnvironment *env;
  ActionFreedom(SharedEnvironment *env)
      : env(env), agent_num(env->num_of_agents), freedom(env->num_of_agents), occupy_list(env->num_of_agents)
  {
    moves[0] = 1;
    moves[1] = env->cols;
    moves[2] = -1;
    moves[3] = -env->cols;
  };

  void init();
  void worker(int agent_id);
  void get_child(Occupy *root);

private:
};