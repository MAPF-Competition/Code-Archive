

#if !defined(SIPP_H)
#define SIPP_H

#include <queue>
#include <set>
#include <vector>
#include <unordered_set>

#include "ActionModel.h"

namespace sipp {

constexpr int MAX_TIME = 500000;

using Interval = std::pair<int, int>;  // [start, end]

struct NodeConstraint {  // location occpuied at timestamp t
  NodeConstraint(int time, int loc) : t(time), location(loc) {}
  int t;
  int location;
};

struct EdgeConstraint {
  EdgeConstraint(int time, int from, int to) : t(time), u(from), v(to) {}
  int t;
  int u;
  int v;
  bool operator<(const EdgeConstraint &other) const {
    return std::tie(t, u, v) < std::tie(other.t, other.u, other.v);
  }
};

struct SIPPState : public State {
  int g;                        // f_ = g_ + heu
  double f;
  int conflicts;
  std::shared_ptr<SIPPState> pre;  // parent SIPPState
  std::pair<int,int> interval;
  SIPPState() : State(-1, -1, -1) {
            f = -1;
            g = -1;
            interval = {-1, -1};
  }
  SIPPState(int location, int orientation, int timestep = -1,
            std::shared_ptr<SIPPState> _pre = nullptr, int begin = -1, int end = -1)
      : State(location, timestep, orientation),pre(_pre) {
            f = -1;
            g = -1;
            interval = {begin, end};
  }

  bool operator==(const SIPPState &other) const {
    return location == other.location && orientation == other.orientation && interval == other.interval;
  }

};

using SIPPStatePtr = std::shared_ptr<SIPPState>;

State getState(SIPPStatePtr n);
// 针对Open集合，f小的在前，相同f时g大的在前
struct CompareSIPPStatePtrOpen {  
  bool operator()(const SIPPStatePtr &a, const SIPPStatePtr &b) const {
    if (a->f != b->f) return a->f < b->f;
    if (a->g != b->g) return b->g < a->g; 
    if (a->interval != b->interval) return a->interval < b->interval;
    return a < b;
  }
};
// 针对Focal优先队列，conflicts少的在前，相同conflicts时f小的在前
struct CompareSIPPStatePtrFocal {  
  bool operator()(const SIPPStatePtr &a, const SIPPStatePtr &b) const {
    if (a->conflicts != b->conflicts) return a->conflicts > b->conflicts;
    if (a->f != b->f) return a->f > b->f;
    if (a->g != b->g) return b->g > a->g; 
    if (a->interval != b->interval) return a->interval > b->interval;
    return a > b; 
  }
};

// top: SIPPState with smallest conflicts
using SIPPStatePtrPQ = std::priority_queue<SIPPStatePtr,std::vector<SIPPStatePtr>,CompareSIPPStatePtrFocal>;

struct SIPPStateHash {
    size_t operator()(const SIPPState& state) const {
        size_t hash1 = std::hash<uint16_t>()(state.location);
        size_t hash2 = std::hash<uint8_t>()(state.orientation);
        size_t hash3 = std::hash<uint16_t>()(state.interval.first);
        size_t hash4 = std::hash<uint16_t>()(state.interval.second);
        return hash1 ^ (hash2 << 1) ^ (hash3 << 2) ^ (hash4 << 3); ;
    }
};

bool operator<(const SIPPState &lhs, const SIPPState &rhs);
std::ostream &operator<<(std::ostream &os, const SIPPState &state);
std::ostream &operator<<(std::ostream &os, const EdgeConstraint &con);
std::ostream &operator<<(std::ostream &os, const NodeConstraint &con);

struct EdgeConstraintHash {
    std::size_t operator()(const EdgeConstraint& e) const {
      //选质数进行hash计算，64bit机器上应该不会溢出
        constexpr std::size_t prime1 = 8011;
        constexpr std::size_t prime2 = 250007;
        return e.t +
               prime1 * e.u +
               prime1 * prime2 * e.v;
    }
};

// struct EdgeConstraintEqual {//用于处理hash冲突情况
//     bool operator()(const EdgeConstraint& lhs, const EdgeConstraint& rhs) const {
//         return lhs.t == rhs.t && lhs.u == rhs.u && lhs.v == rhs.v;
//     }
// };

class SIPP {
 private:
  // collision intervals for added node constraints
  std::unordered_map<int, std::vector<Interval> > collision_intervals_map;
  std::vector<NodeConstraint> raw_node_cons;
  std::set<EdgeConstraint> edge_constraints_set;
  

  std::unordered_map<int,int> path_heatmap;
  std::list<int>
      additional_obstacles;  // treat other agents' start location as obstacles

  // map data
  int rows;
  int cols;
  std::vector<int> map;
  const std::vector<vector<int> >& costmaps;

  std::set<SIPPStatePtr,CompareSIPPStatePtrOpen> open;
  std::unordered_map<SIPPState, int,SIPPStateHash> close;  // state and g value
  SIPPStatePtrPQ focal;

  double focal_w = 1.2;
  double min_f_val;
  double focal_bound;

  int horizon = INT_MAX;


  std::vector<std::string> backtrace_debug;


 public:
  SIPP(const std::vector<int> &_map, int _rows, int _cols,
       const std::vector<vector<int>> &_costmaps)
      : map(_map), rows(_rows), cols(_cols), costmaps(_costmaps) {}

  ~SIPP() {}

 public:
  std::vector<State> Search(
      std::pair<int, int> start, std::pair<int, int> goal,
      const std::vector<NodeConstraint> &ncons,
      const std::vector<EdgeConstraint> &econs, const std::list<int> &obs,
      const std::unordered_map<int, std::vector<int>> &other_paths);

  void printSuccessors(const SIPPStatePtr& s,const std::vector<SIPPStatePtr>& succs) const ;

  void SetHorizon(int h){horizon = h;}

  void dump_debug();
 private:
  void addNodeConstraints(const std::vector<NodeConstraint> &cons);

  void addEdgeConstraints(const std::vector<EdgeConstraint> &cons);

  bool validateMove(int loc /* new */, int loc2 /* from */);

  std::vector<pair<int, State> > getValidMoves(SIPPStatePtr cur_state);

  std::vector<SIPPStatePtr> getSuccessors(SIPPStatePtr cur,
      const std::unordered_map<int, std::vector<int>> &other_paths);

  std::vector<State> reconstructPath(SIPPStatePtr start, SIPPStatePtr goal);

  double heuristic(int loc1, int loc2);

  void updateFocalList();

  void printIntervals(const std::vector<sipp::Interval> &intervals) {
        std::cout << " intervals: ";
        for (auto &i : intervals) {
          std::cout << "\t[" << i.first << "," << i.second << "], ";
        }
        std::cout << std::endl;
  }
  void checkPathWithEdgeConstraints(const std::vector<State> &path,
                                    const std::set<EdgeConstraint> &cons);

  void checkPathWithNodeConstraints(const std::vector<State> &path,
                                    const std::vector<NodeConstraint> &cons);

  std::vector<Interval> getSafeIntervals(const int location);

};

}  // namespace sipp

#endif  // SIPP_H
