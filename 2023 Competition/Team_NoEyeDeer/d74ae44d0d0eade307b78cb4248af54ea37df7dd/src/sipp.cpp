#include "sipp.h"

#include <functional>
#include <unordered_map>

#include "threadpool.h"

namespace sipp {


std::ostream& operator<<(std::ostream& os, const EdgeConstraint& con) {
  os << std::dec << "(" << con.t << "," << con.u << "," << con.v << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const NodeConstraint& con) {
  os << "(" << con.t << "," << con.location << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const SIPPState& state) {
  os << std::dec << "( loc:" << state.location << ",ori:" << state.orientation
     << ",intv:[" << state.interval.first << "," << state.interval.second
     << "],g:" << state.g << ",f:" << state.f << ",time: " << state.timestep
     << ")";
  return os;
}

void checkInterval(const Interval& in) {
  if (in.first > in.second || in.first < 0 || in.second < 0 ||
      in.first > MAX_TIME || in.second > MAX_TIME) {
    std::cout << in.first << "," << in.second << std::endl;
    throw std::runtime_error("intervals[i].first > intervals[i].second");
  }
}
void SIPP::checkPathWithNodeConstraints(
    const std::vector<State>& path, const std::vector<NodeConstraint>& cons) {
  for (auto con : cons) {
    if (con.t >= path.size()) continue;
    if (path.at(con.t).location == con.location) {
      std::cout << path << std::endl;
      std::cout << path[con.t] << std::endl;
      throw std::runtime_error("checkPathWithNodeConstraints");
    }
  }
}
void SIPP::checkPathWithEdgeConstraints(const std::vector<State>& path,
                                        const std::set<EdgeConstraint>& cons) {
  for (auto con : cons) {
    if (con.t + 1 >= path.size()) continue;
    if ((path.at(con.t).location == con.u) &&
        (path.at(con.t + 1).location == con.v)) {
      std::cout << path << std::endl;
      std::cout << path[con.t] << "->" << path[con.t + 1] << std::endl;
      throw std::runtime_error("checkPathWithEdgeConstraints");
    }
  }
}

std::vector<State> SIPP::reconstructPath(SIPPStatePtr start,
                                         SIPPStatePtr goal) {
  SIPPStatePtr cur = goal;
  std::vector<SIPPStatePtr> path;
  path.emplace_back(cur);
  while (cur->pre) {
    cur = cur->pre;
    path.emplace_back(cur);
  }
#ifdef DEBUG
  std::cout << "reconstructPath:" << std::endl;
  for (int i = 0; i < path.size(); i++) {
    // path[i].timestep = i;
    std::cout << "\t" << *path[i] << std::endl;
  }
#endif
  std::reverse(path.begin(), path.end());

  std::vector<State> newPath;
  for (size_t i = 0; i < path.size(); i++) {
    newPath.emplace_back(path[i]->location, path[i]->g, path[i]->orientation);

    // 检查是否需要在当前节点等待
    if (i < path.size() - 1 && path[i + 1]->g > path[i]->g + 1) {
      // 在当前节点创建等待节点，直到 g 值满足条件
      int waitTime = path[i + 1]->g - path[i]->g - 1;
      for (int w = 0; w < waitTime; ++w) {
        newPath.emplace_back(path[i]->location, path[i]->g + w + 1,
                             path[i]->orientation);
      }
    }
  }
#ifdef DEBUG
  std::cout << "refine path:" << std::endl;
  for (int i = 0; i < newPath.size(); i++) {
    // path[i].timestep = i;
    std::cout << "\t" << newPath[i] << std::endl;
  }
#endif
  checkPathWithNodeConstraints(newPath, raw_node_cons);
  checkPathWithEdgeConstraints(newPath, edge_constraints_set);

  return newPath;
}

#define USE_COSTMAP 0

double SIPP::heuristic(int loc1, int loc2) {
#if USE_COSTMAP
  // search heuristic map
  return (costmaps)[loc2][loc1];
#else
  int loc1_x = loc1 / this->cols;
  int loc1_y = loc1 % this->cols;
  int loc2_x = loc2 / this->cols;
  int loc2_y = loc2 % this->cols;
  return (abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y));
#endif
}

bool SIPP::validateMove(int loc /* new */, int loc2 /* from */) {
  int loc_x = loc / this->cols;
  int loc_y = loc % this->cols;

  if (loc_x >= this->rows || loc_y >= this->cols || this->map[loc] == 1)
    return false;

  if (std::find(additional_obstacles.begin(), additional_obstacles.end(),
                loc) != additional_obstacles.end()) {
    return false;
  }

  int loc2_x = loc2 / this->cols;
  int loc2_y = loc2 % this->cols;
  if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1) return false;
  return true;
}

/**
 * int: time to execute
 * State: next state
 */
vector<pair<int, State>> SIPP::getValidMoves(SIPPStatePtr cur_state) {
  vector<pair<int, State>> neighbors;
  int location = cur_state->location;
  int direction = cur_state->orientation;
  // forward
  int candidates[4] = {location + 1, location + this->cols, location - 1,
                       location - this->cols};
  int forward = candidates[direction];
  int new_direction = direction;
  if (forward >= 0 && forward < this->map.size() &&
      validateMove(forward, location))
    neighbors.emplace_back(
        make_pair(1, State(forward, cur_state->timestep + 1, new_direction)));
  // turn left
  new_direction = direction - 1;
  if (new_direction == -1) new_direction = 3;
  neighbors.emplace_back(
      make_pair(1, State(location, cur_state->timestep + 1, new_direction)));
  // turn right
  new_direction = direction + 1;
  if (new_direction == 4) new_direction = 0;
  neighbors.emplace_back(
      make_pair(1, State(location, cur_state->timestep + 1, new_direction)));
  // wait
  // neighbors.emplace_back(make_pair(
  //     1, State(location, cur_state->timestep + 1, direction)));
  return neighbors;
}

// get safe intervals pertains to VERTEX constraints
std::vector<Interval> SIPP::getSafeIntervals(const int location) {
  std::vector<Interval> intervals = {};
  // 计算s的所有安全的时间间隔
  if (collision_intervals_map.find(location) ==
      collision_intervals_map.end()) {  // s位置没有会发生碰撞的区间
    intervals.emplace_back(0, MAX_TIME);
  } else {  // s存在某些区间有碰撞
    std::pair<int, int> temp = {0, MAX_TIME};
    const auto& collision_intervals = collision_intervals_map[location];
    for (const auto& colli_intv : collision_intervals) {
      temp.second = colli_intv.first - 1;
      intervals.push_back(temp);
      /**
       * { 0 , colli_intv.first - 1} , {colli_intv.first,
       * colli_intv.second}, {colli_intv.second+1, MAX_TIME}
       */
      temp.first = colli_intv.second + 1;
    }
    temp.second = MAX_TIME;
    intervals.push_back(temp);
  }

  return intervals;
}

std::vector<SIPPStatePtr> SIPP::getSuccessors(
    SIPPStatePtr cur,
    const std::unordered_map<int, std::vector<int>>& other_paths) {
  // std::ostringstream stream;
  // APPEND_TO_STREAM_AND_CLEAR(stream, backtrace_debug,
  //                            "SIPP getSuccessors cur " << *cur);

#ifdef DEBUG
  std::cout << "SIPP getSuccessors cur " << *cur << std::endl;
  std::cout << "SIPP getSuccessors in " << this << std::endl;
  std::cout << "SIPP getSuccessors edge_constraints_set size "
            << edge_constraints_set.size() << std::endl;
#endif
  std::vector<SIPPStatePtr> succs;
  vector<pair<int, State>> valid_moves = getValidMoves(cur);
  for (const auto& m : valid_moves) {
    std::vector<Interval> intervals = getSafeIntervals(m.second.location);
#ifdef DEBUG
    std::cout << " before econs ";
    printIntervals(intervals);
#endif
    /* split interval for edge constraints
      边约束：T时刻，不准从U到V，（到达V的时间为T+1）
      V的一个安全间隔为 [first, second]
      此安全区域变化情况（以T+1劈开）：
      a. first < T+1 < second :  [ first, T ],  [ T+2, second ]
      b. first = T+1 < second :  [ T+2, second ]
      c. first < T+1 = second :  [ first, T ]
      d. first = T+1 = second :  [ ] 删除
      e. 其它情况： [ first, second ] 不变
    */
    bool has_edge_con = false;
    int cur_intv_end = cur->interval.second;
    for (const auto& econ : edge_constraints_set) {
      if (econ.u == cur->location && econ.v == m.second.location) {
        has_edge_con = true;
        // cur's safe interval should be truncated due to edge conflict
        cur_intv_end = std::min(econ.t - 1,cur_intv_end);
        for (size_t i = 0; i < intervals.size(); ++i) {
          int first = intervals[i].first;
          int second = intervals[i].second;
          int pivot = econ.t + 1;
          // int pivot = econ.t;
          if (first < pivot && pivot < second) {
            intervals[i] = {first, econ.t};
            intervals.insert(intervals.begin() + i + 1, {pivot + 1, second});
            ++i;
          } else if (first == pivot && pivot < second) {
            intervals[i] = {pivot + 1, second};
          } else if (first < pivot && pivot == second) {
            intervals[i] = {first, econ.t};
          } else if (first == pivot && pivot == second) {
            intervals.erase(intervals.begin() + i);
            i--;
          } else {
            continue;
          }
        }
      }
    }

#ifdef DEBUG
    std::cout << " after econs ";
    printIntervals(intervals);
    std::cout<<"cur_intv_end "<<cur_intv_end<<std::endl;
#endif
    for (const auto& intv : intervals) {  // for each safe interval i in cfg
      // "earliest arrival time at cfg during interval i with no collisions"

      SIPPStatePtr next_state =
          std::make_shared<SIPPState>(m.second.location, m.second.orientation,
                                      cur->g + m.first, cur, 0, MAX_TIME);
      next_state->g = cur->g + m.first;
      next_state->interval = intv;
      
#ifdef DEBUG
      std::cout << " ** valid next_state: " << getState(next_state)
                << std::endl;
#endif
      // APPEND_TO_STREAM_AND_CLEAR(stream, backtrace_debug, " ** valid next_state: " << *next_state);
      if (intv.second < next_state->g) {
        continue;
      }

      if (next_state->g < intv.first) {
        next_state->g = intv.first;
      }
      
      if (next_state->g > cur_intv_end + 1) {
        // unable to reach this state safely
        continue;
      }

      checkInterval(intv);

      // check this interval has soft obstacles, ref:LNS2-2022
      next_state->conflicts = cur->conflicts;
      if (other_paths.find(next_state->location) != other_paths.end()) {
        const auto& occu_times = other_paths.at(next_state->location);
        for (const auto& t : occu_times) {
          if (intv.first <= t && t <= intv.second) {
            next_state->conflicts++;
            break;
          }
        }
      }
#ifdef DEBUG
      std::cout << " SIPP succs push_back: " << *next_state << std::endl;
#endif
      // APPEND_TO_STREAM_AND_CLEAR(
      //     stream, backtrace_debug,
      //     " SIPP succs push_back: " << *next_state);
      succs.push_back(next_state);
    }
  }  // end for move

  return succs;
}

/// @brief 将 点约束 转换成 碰撞区间 并存储到成员变量
/// @param cons
void SIPP::addNodeConstraints(const std::vector<NodeConstraint>& cons) {
  raw_node_cons = cons;
  for (const auto& con : cons) {
    std::cout << "    addNodeConstraints " << con << std::endl;
#ifdef DEBUG
    std::cout << "    addNodeConstraints " << con << std::endl;
#endif
    if (collision_intervals_map.find(con.location) ==
        collision_intervals_map.end()) {
      collision_intervals_map[con.location] = {{con.t, con.t}};
    } else {
      auto collision_intervals = collision_intervals_map[con.location];
      for (int i = 0; i < collision_intervals.size(); i++) {
        if (con.t + 1 < collision_intervals[i].first) {
          collision_intervals.insert(collision_intervals.begin() + i,
                                     {con.t, con.t});
          break;
        } else if (con.t + 1 == collision_intervals[i].first) {
          collision_intervals[i].first = con.t;
          break;
        } else if ((collision_intervals[i].second + 1) == con.t) {
          collision_intervals[i].second = con.t;
          break;
        } else if (i + 1 == collision_intervals.size()) {
          collision_intervals.push_back({con.t, con.t});
          break;
        }
      }

      // Sort and remove duplicates
      std::sort(collision_intervals.begin(), collision_intervals.end());
      collision_intervals.erase(
          std::unique(collision_intervals.begin(), collision_intervals.end()),
          collision_intervals.end());

      // Merge overlapping intervals
      for (size_t i = 0; i < collision_intervals.size() - 1;) {
        if (collision_intervals[i].second + 1 >=
            collision_intervals[i + 1].first) {
          collision_intervals[i].second = std::max(
              collision_intervals[i].second, collision_intervals[i + 1].second);
          collision_intervals.erase(collision_intervals.begin() + i + 1);
        } else {
          ++i;
        }
      }

      collision_intervals_map.at(con.location) = collision_intervals;
    }
  }
}

/// @brief 边约束将存到成员变量，每次搜索时进行检查
/// @param cons
void SIPP::addEdgeConstraints(const std::vector<EdgeConstraint>& cons) {
  for (auto& con : cons) {
    std::cout << "    addEdgeConstraints " << con << std::endl;
#ifdef DEBUG
    std::cout << "    addEdgeConstraints " << con << std::endl;
#endif
    edge_constraints_set.insert(con);
  }
}

void SIPP::printSuccessors(const SIPPStatePtr& s,
                           const std::vector<SIPPStatePtr>& succs) const {
  std::cout << *s << " Successors: " << std::endl;
  for (const auto& it : succs) {
    std::cout << "\t" << *it << std::endl;
  }
}

struct SIPPStateCmp {
  bool operator()(const SIPPState& lhs, const SIPPState& rhs) const {
    return std::tie(lhs.f, rhs.g, lhs.interval, lhs.location, lhs.orientation) <
           std::tie(rhs.f, lhs.g, rhs.interval, rhs.location, rhs.orientation);
  }
};

State getState(SIPPStatePtr n) {
  return State(n->location, n->timestep, n->orientation);
}

/// @brief
/// @param start
/// @param goal
/// @param ncons node constraints
/// @param econs edge constraints
/// @param obs
/// @param other_paths  treat other paths' vertices as soft obstacles.
/// format: { location - [time] }
/// @return
std::vector<State> SIPP::Search(
    std::pair<int, int> start, std::pair<int, int> goal,
    const std::vector<NodeConstraint>& ncons,
    const std::vector<EdgeConstraint>& econs, const std::list<int>& obs,
    const std::unordered_map<int, std::vector<int>>& other_paths) {
  // backtrace_debug.clear();
  collision_intervals_map.clear();
  open.clear();
  close.clear();
  focal = SIPPStatePtrPQ();
  additional_obstacles = obs;
  addNodeConstraints(ncons);
  addEdgeConstraints(econs);
#ifdef DEBUG
  std::cout << "sipp collision_intervals_map: ";
  for (auto ci : collision_intervals_map) {
    std::cout << ci.first << "\t: ";
    for (auto v : ci.second) {
      std::cout << "[" << v.first << "," << v.second << "] ";
    }
    std::cout << std::endl;
  }
#endif
  SIPPStatePtr s_start = std::make_shared<SIPPState>(start.first, start.second,
                                                     0, nullptr, 0, MAX_TIME);

  // get s_start's safe interval
  s_start->interval = getSafeIntervals(s_start->location).at(0);
  //

  s_start->g = 0;
  s_start->f = heuristic(s_start->location, goal.first);
  s_start->conflicts = 0;  // assume start no conflits

  open.insert(s_start);
  min_f_val = open.begin()->get()->f;
  focal_bound = min_f_val * focal_w;
  focal.push(s_start);

  int cnt = 0;
  while (!focal.empty()) {
    SIPPStatePtr s = focal.top();
    focal.pop();
    open.erase(s);
    close.insert({*s, s->g});
    if (s->location == goal.first) {  // path found
      return reconstructPath(s_start, s);
    }

    std::vector<SIPPStatePtr> successors = getSuccessors(s, other_paths);
#ifdef DEBUG
    printSuccessors(s, successors);
#endif
    std::vector<SIPPStatePtr> focal_to_push;
    for (auto sprime : successors) {
#ifdef DEBUG
      if (close.find(*sprime) == close.end()) {
        std::cout << "successor " << *sprime << " not in close, insert to open"
                  << std::endl;
      } else {
        std::cout << "successor " << *sprime
                  << " IS in close, sprime->g: " << sprime->g
                  << ", close.at(*sprime):" << close.at(*sprime) << std::endl;
      }
#endif
      if (close.find(*sprime) == close.end() ||
          sprime->g < close.at(*sprime)) {  // not visited or is better
        close[*sprime] = sprime->g;
        sprime->timestep = s->timestep + 1;
        sprime->f = sprime->g + heuristic(s->location, goal.first);
        open.insert(sprime);
        focal_to_push.push_back(sprime);
      }
    }

    // std::cout << "sipp successors.size(): " << successors.size() <<
    // std::endl;
    for (const auto& it : focal_to_push) {
      if (it->f <= open.begin()->get()->f * focal_w) {
        focal.push(it);
      }
    }
    updateFocalList();
  }
  std::vector<State> path;
  std::cout << "Path not found" << std::endl;
  return path;
}

void SIPP::dump_debug() {
  std::cout << "------------ dump_debug -------------" << std::endl;
  for (auto s : backtrace_debug) {
    std::cout << " *** " << s << std::endl;
  }
}

void SIPP::updateFocalList() {
  if (open.empty()) return;
  double new_min_f_val = open.begin()->get()->f;
  if (new_min_f_val == min_f_val) return;
  SIPPStatePtrPQ().swap(focal);
  min_f_val = new_min_f_val;
  focal_bound = min_f_val * focal_w;

#if DEBUG
  std::cout << " == SIPP updateFocalList ==" << std::endl;
  std::cout << "\tnew_min_f_val: " << new_min_f_val << std::endl;
  std::cout << "\tmin_f_val: " << min_f_val << std::endl;
#endif
  for (const auto& n : open) {
    if (n->f <= focal_w * new_min_f_val) {
      focal.push(n);
    } else {
      break;
    }
  }
}

}  // namespace sipp
