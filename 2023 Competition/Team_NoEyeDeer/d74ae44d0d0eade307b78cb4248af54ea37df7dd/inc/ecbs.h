#if !defined(ECBS_H)
#define ECBS_H

#include <unordered_map>

#include <sipp.h>
#include "omp.h"

namespace cbs {
enum ConflictType {
  C_NODE,
  C_SWAP
};

inline std::string ConflictType2Str(ConflictType in) {
  return (in == C_NODE) ? "vertex" : "edge";
}

struct Conflict {
  Conflict()
      : time1(-1),
        time2(-1),
        id1(-1),
        id2(-1),
        location1(-1),
        location2(-1),
        type(C_NODE) {}
  Conflict(int t1, int t2, int _id1, int _id2, int loc1, int loc2,
           ConflictType _type)
      : time1(t1),
        time2(t2),
        id1(_id1),
        id2(_id2),
        location1(loc1),
        location2(loc2),
        type(_type) {}

  ConflictType type;
  int location1;
  int location2;  // same as location1 when C_NODE
  int id1;
  int id2;
  int time1;
  int time2; 
};

std::ostream& operator<<(std::ostream& os, const Conflict& con);

inline bool compareConflicts(const Conflict& a, const Conflict& b) {
  if (a.time1 != b.time1) {
    return a.time1 < b.time1; // 时间升序
  }  
  if (a.time2 != b.time2) {
    return a.time2 < b.time2; // 时间升序
  }
  if (a.id1 != b.id1) {
    return a.id1 < b.id1; // id1 升序
  }
  return a.id2 < b.id2; // id2 升序
}

#define DIJKSTRA_START 0
#define SIPP_START 1
struct ECBSPath {
  ECBSPath() {}
  ECBSPath(int _id, const std::vector<State>& v) : id(_id), vertices(v) {
    cost = vertices.size();
  }
  std::vector<State> vertices;
  int id;  // agent to which this path belongs
  int cost;
};

struct ECBSNode {
  std::map<int, std::vector<sipp::NodeConstraint>> ncons = {};
  std::map<int, std::vector<sipp::EdgeConstraint>> econs = {};
  std::vector<ECBSPath> paths = {};
  int cost = 0;
  std::shared_ptr<ECBSNode> parent = nullptr;
  int conflict_num = 0;

  double g, h, f;

  void UpdateCost() {  // SIC
    cost = 0;
    for (const auto& it : paths) {
      cost += it.cost;
    }
    g = cost;
    f = g + h;
  }

  friend std::ostream& operator<<(std::ostream& os, const ECBSNode& n) {
    os << "(cost:" << n.cost << ", g:" << n.g << ", h:" << n.h << ", f:" << n.f
       << ")";
    return os;
  }
};

using ECBSNodePtr = std::shared_ptr<ECBSNode>;
struct CompareECBSNodePtrOpen {
  bool operator()(const ECBSNodePtr& a, const ECBSNodePtr& b) const {
    return std::tie(a->f, a->g, a) < std::tie(b->f, b->g, b);
  }
};

struct CompareECBSNodePtrFocal {
  bool operator()(const ECBSNodePtr& a, const ECBSNodePtr& b) const {
    return std::tie(a->conflict_num, a->f) > std::tie(b->conflict_num, b->f);
  }
};

using ECBSNodePtrPQ = std::priority_queue<ECBSNodePtr, std::vector<ECBSNodePtr>,
                                          CompareECBSNodePtrFocal>;

class ECBSSolver {
 private:
  double focal_w = 1.2;  // >1.0

  std::set<ECBSNodePtr, CompareECBSNodePtrOpen> open;
  ECBSNodePtrPQ focal;
  std::vector<int> map;
  int rows, cols;
  const std::vector<vector<int>>& costmaps;
  const std::vector<vector<int>>& predecessors;

  int horizon = INT_MAX;  // only consider conflicts within horizon

 public:
  ECBSSolver(const std::vector<int>& _map, int _rows, int _cols,
             const std::vector<vector<int>>& _costmaps,
             const std::vector<vector<int>>& _predecessors)
      : map(_map),
        rows(_rows),
        cols(_cols),
        costmaps(_costmaps),
        predecessors(_predecessors) {}

  void setHorizon(int h) { horizon = h; }

  /// @brief create SoftObstacles pertains to agent_id's other paths, send it to
  /// low level
  /// @param n
  /// @param agent_id
  /// @return
  std::unordered_map<int, std::vector<int>> paths2SoftObstacles(
      const std::vector<ECBSPath>& paths, int agent_id);

  std::unordered_map<int, std::vector<int>> paths2SoftObstacles(
      const std::vector<std::list<State>>& paths, int agent_id);

  std::list<State> SearchOnePath(const std::pair<State, State>& task,
                                 const std::vector<std::list<State>>& all_paths,
                                 const vector<State>& cur_states, int agent_id,
                                 const int time_limit = 100);

  std::vector<std::list<State>> Solve(
      const std::vector<std::pair<State, State>>& tasks, int time_limit = 100,
      const std::vector<std::pair<State, State>>& next_tasks = {});

 private:
  void updateFocal();

  // get path from Dijkstra precomputed info
  Path getPathFromPredecessors(State start, State end);

  void constructRootNode(const std::vector<std::pair<State, State>>& tasks);

  // 返回的数组下标就是对应机器人的ID
  std::vector<std::list<State>> constructPaths(
      const std::vector<ECBSPath>& sol);

  void getConflict(const Path& path1, const Path& path2, int id1, int id2,
                   std::vector<Conflict>& con);

  std::vector<Conflict> getConflicts(
      const std::vector<State>& candidate,
      const std::vector<std::list<State>>& all_paths,int agent_id);

  // return first confict, or empty
  std::vector<Conflict> getConflicts(const ECBSNodePtr p);

  // treat other neighbor agents as static obstacles
  std::list<int> checkNeighborHasAgents(std::pair<State, State> task, 
                                        const std::set<int>& all_start_locs);
};

}  // namespace cbs
#endif  // ECBS_H
