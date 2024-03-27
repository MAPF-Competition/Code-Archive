#include "sipp.h"

using namespace sipp;

#include <iostream>
#include <list>
#include <set>
#include <unordered_map>
#include <vector>

void checkInterval(sipp::Interval& in) {
  if (in.first > in.second) {
    std::cout << in.first << "," << in.second << std::endl;
    throw std::runtime_error("intervals[i].first > intervals[i].second");
  }
}

int main() {
  std::unordered_map<int, std::vector<sipp::Interval>> collision_intervals_map =
      {{0, {}}, {1, {}}, {2, {}}, {3, {}}, {4, {{1,1},{3,3},{7,8}}},
       {5, {}}, {6, {}}, {7, {}}, {8, {}}};
  std::set<EdgeConstraint>
      edge_constraints_set = {{1, 3, 4}};   

  SIPPStatePtr cur = std::make_shared<SIPPState>(3, 0, 1, nullptr);
  cur->g = 0;
  SIPPStatePtr next_state = std::make_shared<SIPPState>(4, 0, 2, cur);
  next_state->g = cur->g + 1;

  // 计算next_state的所有安全的时间间隔
  std::vector<sipp::Interval> intervals = {};
  if (collision_intervals_map.find(next_state->location) ==
      collision_intervals_map.end()) {  // next_state位置没有会发生碰撞的区间
    intervals.push_back({next_state->g, MAX_TIME});
  } else {  // next_state存在某些区间有碰撞，计算next_state的所有safe interval
    std::pair<int, int> temp = {0, MAX_TIME};
    auto collision_intervals = collision_intervals_map[next_state->location];
    for (auto colli_intv : collision_intervals) {
      temp.second = colli_intv.first - 1;
      checkInterval(temp);
      intervals.push_back(temp);
      /**
       * { 0 , colli_intv.first - 1} , {colli_intv.first, colli_intv.second},
       * {colli_intv.second+1, MAX_TIME}
       */
      temp.first = colli_intv.second + 1;
    }
    temp.second = MAX_TIME;
    checkInterval(temp);
    intervals.push_back(temp);
  }

  std::cout << "Safe intervals after vertex con:\n";
  for (auto& interval : intervals) {
    std::cout << "[" << interval.first << ", " << interval.second << "]\n";
  }

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
  for (auto econ : edge_constraints_set) {
    if (econ.u != cur->location || econ.v != next_state->location) continue;
    for (size_t i = 0; i < intervals.size(); ++i) {
      checkInterval(intervals[i]);
      int first = intervals[i].first;
      int second = intervals[i].second;
      int pivot = econ.t + 1;
      // int pivot = econ.t;
      if (first < pivot && pivot < second) {
        intervals[i] = {first, econ.t};
        intervals.insert(intervals.begin() + i + 1, {pivot + 1, second});
        checkInterval(intervals[i]);
        ++i;
        checkInterval(intervals[i]);
      } else if (first == pivot && pivot < second) {
        intervals[i] = {pivot + 1, second};
        checkInterval(intervals[i]);
      } else if (first < pivot && pivot == second) {
        intervals[i] = {first, econ.t};
        checkInterval(intervals[i]);
      } else if (first == pivot && pivot == second) {
        intervals.erase(intervals.begin() + i);
        i--;
      } else {
        continue;
      }
    }
  }

  // 输出安全间隔进行验证
  std::cout << "Safe intervals after edge con:\n";
  for (auto& interval : intervals) {
    std::cout << "[" << interval.first << ", " << interval.second << "]\n";
  }

  return 0;
}
