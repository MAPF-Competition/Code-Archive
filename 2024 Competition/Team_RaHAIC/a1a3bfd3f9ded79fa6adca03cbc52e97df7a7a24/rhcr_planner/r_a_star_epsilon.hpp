#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "r_neighbor.hpp"
#include "r_planresult.hpp"


namespace RHCR_Planner{



template <typename State2, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State2> >
class AStarEpsilon {
 public:
  AStarEpsilon(Environment& environment, float w)
      : m_env(environment), m_w(w) {}

  

  bool search(const State2& startState,
              PlanResult<State2, Action, Cost>& solution,
              int maxPathLength = -1) {  //添加最大路径长度参数 -hzj
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = 0;


    openSet_t openSet;
    focalSet_t
        focalSet;  // subset of open nodes that are within suboptimality bound
    std::unordered_map<State2, fibHeapHandle_t> stateToHeap;
    std::unordered_set<State2> closedSet;
    std::unordered_map<State2, std::tuple<State2, Action, Cost, Cost>>
        cameFrom;

    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), 0, 0));

    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;

    focalSet.push(handle);

    std::vector<Neighbor<State2, Action, int> > neighbors;
    neighbors.reserve(10);

    Cost bestFScore = (*handle).fScore;

    while (!openSet.empty()) {
        Cost oldBestFScore = bestFScore;
        bestFScore = openSet.top().fScore;
        if (bestFScore > oldBestFScore) {
          auto iter = openSet.ordered_begin();
          auto iterEnd = openSet.ordered_end();
          for (; iter != iterEnd; ++iter) {
            Cost val = iter->fScore;
            if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
              const Node& n = *iter;
              focalSet.push(n.handle);
            }
            if (val > bestFScore * m_w) {
              break;
            }
          }
        }

      auto currentHandle = focalSet.top();
      Node current = *currentHandle;
      m_env.onExpandNode(current.state, current.fScore, current.gScore);
      
      // 判断是否到达终点或达到路径长度限制 -hzj
      if (m_env.isSolution(current.state) || 
            (maxPathLength > 0 && current.state.timestep >= maxPathLength)) {

        // 从当前open集合中找到时间步内代价最小的状态 -hzj
        // Node bestNode = current;
        // auto iter1 = openSet.ordered_begin();
        // auto iterEnd = openSet.ordered_end();

        // // 只检查前N个节点，避免完整遍历
        // const int MAX_CHECK_NODES = 3;
        // int checked = 0;
    
        // for (; iter1 != iterEnd && checked < MAX_CHECK_NODES; ++iter1, ++checked) {
        //      const Node& n = *iter1;
        //     if (n.state.timestep == maxPathLength && n.fScore < bestNode.fScore) {
        //        bestNode = n;
        //     }
        // }

        solution.states.clear();
        solution.actions.clear();
        //auto iter = cameFrom.find(current.state); 修改为bestNode -hzj
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, 0));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());

        // solution.cost = current.gScore; 修改为bestNode -hzj
        solution.cost = current.gScore;
        solution.fmin = openSet.top().fScore;

        return true;
      }

      focalSet.pop();
      openSet.erase(currentHandle);
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);

      // traverse neighbors
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);

      for (const Neighbor<State2, Action, int>& neighbor : neighbors) {
        // 如果超过最大路径长度，不考虑这个邻居 -hzj
        if (maxPathLength > 0 && neighbor.state.timestep > maxPathLength) {
            continue;
        }

        if (closedSet.find(neighbor.state) == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          if (iter == stateToHeap.end()) {  // Discover a new node
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
            Cost focalHeuristic =
                current.focalHeuristic +
                m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore);
            auto handle = openSet.push(
                Node(neighbor.state, fScore, tentative_gScore, focalHeuristic));
            (*handle).handle = handle;
            if (fScore <= bestFScore * m_w) {
              focalSet.push(handle);
            }
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
          } else {
            auto handle = iter->second;
            if (tentative_gScore >= (*handle).gScore) {
              continue;
            }
            Cost last_gScore = (*handle).gScore;
            Cost last_fScore = (*handle).fScore;
            Cost delta = last_gScore - tentative_gScore;
            (*handle).gScore = tentative_gScore;
            (*handle).fScore -= delta;
            openSet.increase(handle);
            m_env.onDiscover(neighbor.state, (*handle).fScore,
                             (*handle).gScore);
            if ((*handle).fScore <= bestFScore * m_w &&
                last_fScore > bestFScore * m_w) {
              focalSet.push(handle);
            }
          }

          cameFrom.erase(neighbor.state);
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
        }
      }
    }

    return false;
  }

 private:
  struct Node;

  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;

  struct Node {
    Node(const State2& state, Cost fScore, Cost gScore, Cost focalHeuristic)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          focalHeuristic(focalHeuristic) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore
      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
      return os;
    }

    State2 state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;

    fibHeapHandle_t handle;
  };

  struct compareFocalHeuristic {
    bool operator()(const fibHeapHandle_t& h1,
                    const fibHeapHandle_t& h2) const {
      // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
      // Path Finding" by Cohen et. al.)
      // 1. lowest focalHeuristic
      // 2. lowest fScore
      // 3. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;

      } else if ((*h1).fScore != (*h2).fScore) {
        return (*h1).fScore > (*h2).fScore;
      } else {
        return (*h1).gScore < (*h2).gScore;
      }
    }
  };


  typedef typename boost::heap::d_ary_heap<
      fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;


 private:
  Environment& m_env;
  float m_w;
};

}  // namespace RHCR_Planner
