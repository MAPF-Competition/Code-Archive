#include "ActionModel.h"
#include "common.h"
#include "instance.hpp"
#include <MAPFPlanner.h>
#include <assert.h>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <lacam.hpp>
#include <map>
#include <random>
#include <tuple>
#include <vector>

struct AstarNode {
  int location;
  int direction;
  int f, g, h;
  AstarNode *parent;
  int t = 0;
  bool closed = false;
  AstarNode(int _location, int _direction, int _g, int _h, AstarNode *_parent)
      : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h),
        parent(_parent) {}
  AstarNode(int _location, int _direction, int _g, int _h, int _t,
            AstarNode *_parent)
      : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h),
        t(_t), parent(_parent) {}
};

struct cmp {
  bool operator()(AstarNode *a, AstarNode *b) {
    if (a->f == b->f)
      return a->g <= b->g;
    else
      return a->f > b->f;
  }
};

void MAPFPlanner::initialize(int preprocess_time_limit) {
  auto t_s = Time::now();
  G = new Graph(env);
  int seed = 0;
  MT = new std::mt19937(seed);
#ifndef NDEBUG
  time_step = -1;
  last_set_time_step = 0;
  last_set_line = 0;
#endif
  unsolved = false;
  auto duration =
      std::chrono::duration_cast<std::chrono::seconds>(Time::now() - t_s)
          .count();
  cout << "planner initialize done in " << duration << " sec" << endl;
}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit, vector<Action> &actions) {
#ifndef NDEBUG
  time_step++;
#endif
  actions = std::vector<Action>(env->curr_states.size(), Action::W);
  if (time_limit == 0 or unsolved) {
    return;
  }

  // TODO: deal with empty goal location
  bool replan = false;
  if (ins == nullptr or sol.size() < 2) {
    replan = true;
  } else {
    for (int i = 0; i < env->num_of_agents; i++) {
      if (env->goal_locations[i].empty()) {
        cout << "empty goal location" << endl;
      }
      auto goal = env->goal_locations[i].front().first;
      auto _goal = ins->real_goals[i]->index;
      if (goal != _goal) {
        replan = true;
        break;
      }
    }
  }
#ifndef NDEBUG
  // Store starts and ends
  // std::vector<int> _starts;
  // std::vector<int> _ends;
  // std::vector<int> _goals;
  std::vector<tuple<int, int>> _starts;
  std::vector<tuple<int, int>> _ends;
  std::vector<tuple<int, int>> _goals;
  for (int i = 0; i < env->num_of_agents; i++) {
    // _starts.push_back(env->curr_states[i].location);
    // _ends.push_back(env->goal_locations[i].front().first);
    // if (ins != nullptr) {
    //   _goals.push_back(ins->goals[i]->index);
    // }
    // push row and col
    _starts.push_back(make_tuple(env->curr_states[i].location / env->cols,
                                 env->curr_states[i].location % env->cols));
    _ends.push_back(
        make_tuple(env->goal_locations[i].front().first / env->cols,
                   env->goal_locations[i].front().first % env->cols));
    if (ins != nullptr) {
      _goals.push_back(make_tuple(ins->goals[i]->index / env->cols,
                                  ins->goals[i]->index % env->cols));
    }
  }
#endif
  if (replan) {
    // if (sol.size() < 2) {
    if (sol.empty()) {
      cout << "sol empty" << endl;
    }
    std::vector<int> goals;
    for (int i = 0; i < env->num_of_agents; i++) {
      auto goal = env->goal_locations[i].front().first;
      goals.push_back(goal);
    }
    bool overlap = true;
    std::vector<int> unsolvable_agents;
    std::vector<int> skip_agents;
    while (overlap) {
      // Check if the goals are overlap
      // which means some goal is the same with other
      std::map<int, vector<int>> goal_map;
      for (int i = 0; i < env->num_of_agents; i++) {
        auto goal = goals[i];
        if (goal_map.find(goal) == goal_map.end()) {
          goal_map[goal] = vector<int>{i};
        } else {
          goal_map[goal].push_back(i);
        }
      }
      // Iterate over the goal_map
      // If goals are overlapped
      // only plan for the goal that has lowest reveal time
      overlap = false;
      for (auto it = goal_map.begin(); it != goal_map.end(); it++) {
        if (it->second.size() == 1) {
          continue;
        }
        overlap = true;
#ifndef NDEBUG
        cout << "Overlap: ";
        for (auto agent : it->second) {
          cout << agent << " ";
        }
        cout << endl;
#endif
        int min_reveal_time = INT_MAX;
        int min_agent = -1;
        for (auto agent : it->second) {
          if (env->goal_locations[agent].front().second < min_reveal_time or
              env->curr_states[agent].location ==
                  env->goal_locations[agent].front().first) {
            min_reveal_time = env->goal_locations[agent].front().second;
            min_agent = agent;
          }
        }
        for (auto agent : it->second) {
          if (agent == min_agent) {
            continue;
          }
          auto cur_loc = env->curr_states[agent].location;
          if (find(goals.begin(), goals.end(), cur_loc) == goals.end()) {
            goals[agent] = cur_loc;
            unsolvable_agents.push_back(agent);
            skip_agents.push_back(agent);
            continue;
          }
          // find neighbor4 in G and set it as goal
          auto n4 = G->U[cur_loc]->neighbor4;
          // make sure the goal is not the same with other goals
          // if it is the same, then find the next neighbor4
          int goal = -1;
          for (auto n : n4) {
            if (find(goals.begin(), goals.end(), n->index) == goals.end()) {
              goal = n->index;
              break;
            }
          }
#ifndef NDEBUG
          // if (goal == -1) {
          //   for (auto goal : goals) {
          //     cout << goal << " ";
          //   }
          //   cout << endl;
          // }
#endif
          if (goal == -1) {
            goal = env->curr_states[agent].location;
            unsolvable_agents.push_back(agent);
            goals[agent] = goal;
          } else {
            goals[agent] = goal;
          }
          skip_agents.push_back(agent);
        }
      }
      for (auto agent : unsolvable_agents) {
        for (int i = 0; i < env->num_of_agents; i++) {
          if (i == agent) {
            continue;
          }
          if (goals[i] == env->curr_states[agent].location) {
            goals[i] = env->curr_states[i].location;
            unsolvable_agents.push_back(i);
          }
        }
      }
    }
#ifndef NDEBUG
    if (overlap) {
      cout << "After overlap: ";
      for (auto goal : goals) {
        cout << goal << " ";
      }
      cout << endl;
    }
    std::map<int, vector<int>> _goal_map;
    for (int i = 0; i < env->num_of_agents; i++) {
      auto goal = goals[i];
      if (_goal_map.find(goal) == _goal_map.end()) {
        _goal_map[goal] = vector<int>{i};
      } else {
        _goal_map[goal].push_back(i);
      }
    }
    // Assert there is no overlap
    for (auto it = _goal_map.begin(); it != _goal_map.end(); it++) {
      assert(it->second.size() == 1);
    }
#endif

    ins = new Instance(G, env, goals);
    auto _sol = solve(*ins, 0, MT, 1000, skip_agents);
    if (_sol.size() <= 1) {
      if (_sol.empty()) {
        unsolved = true;
        return;
      }
#ifndef NDEBUG
      last_set_time_step = time_step;
      last_set_line = 243;
#endif
      sol = _sol;
      _sol.pop_front();
      return;
    }
#ifndef NDEBUG
    last_set_time_step = time_step;
    last_set_line = 247;
#endif
    sol = _sol;
  }

  bool match_direction = true;
  for (int i = 0; i < env->num_of_agents; i++) {
    if (env->goal_locations[i].empty()) {
      // TODO: deal with empty goal location, agent i may need to move away
      continue;
    }
    auto cur_loc = env->curr_states[i].location;
    auto last = (*(sol.begin()))[i]->index;
    assert(last == cur_loc);
    auto next = (*(sol.begin() + 1))[i]->index;
    if (last == next) {
      continue;
    }
    // TODO neighour with direction, not location
    // if direction not match next location, rotate and set match_direction to
    // false
    int ori = -1;
    if (last == next - 1) {
      ori = 0;
    } else if (last == next + 1) {
      ori = 2;
    } else if (last == next - env->cols) {
      ori = 1;
    } else if (last == next + env->cols) {
      ori = 3;
    }
    assert(ori != -1);

    auto cur_ori = env->curr_states[i].orientation;
    if (ori != cur_ori) {
      match_direction = false;
      // 0:east, 1:south, 2:west, 3:north
      // 0 - 1 == -1, 1 -> 0
      if (ori - cur_ori == -1 || ori - cur_ori == 3) {
        actions[i] = Action::CCR;
      } else {
        actions[i] = Action::CR;
      }
    }
  }
  if (match_direction) {
    for (int i = 0; i < env->num_of_agents; i++) {
      if (env->goal_locations[i].empty()) {
        // TODO: deal with empty goal location, agent i may need to move away
        continue;
      }
      auto cur_loc = env->curr_states[i].location;
      auto last = (*(sol.begin()))[i]->index;
      assert(last == cur_loc);
      auto next = (*(sol.begin() + 1))[i]->index;
      if (last == next) {
        continue;
      }
      actions[i] = Action::FW;
    }
    sol.pop_front();
  }

  // for (int i = 0; i < env->num_of_agents; i++) {
  //   list<pair<int, int>> path;
  //   if (env->goal_locations[i].empty()) {
  //     path.push_back(
  //         {env->curr_states[i].location, env->curr_states[i].orientation});
  //   } else {
  //     path = single_agent_plan(env->curr_states[i].location,
  //                              env->curr_states[i].orientation,
  //                              env->goal_locations[i].front().first);
  //   }
  //   if (path.front().first != env->curr_states[i].location) {
  //     actions[i] = Action::FW; // forward action
  //   } else if (path.front().second != env->curr_states[i].orientation) {
  //     int incr = path.front().second - env->curr_states[i].orientation;
  //     if (incr == 1 || incr == -3) {
  //       actions[i] = Action::CR; // C--counter clockwise rotate
  //     } else if (incr == -1 || incr == 3) {
  //       actions[i] = Action::CCR; // CCR--clockwise rotate
  //     }
  //   }
  // }
  return;
}

list<pair<int, int>> MAPFPlanner::single_agent_plan(int start, int start_direct,
                                                    int end) {
  list<pair<int, int>> path;
  priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
  unordered_map<int, AstarNode *> all_nodes;
  unordered_set<int> close_list;
  AstarNode *s = new AstarNode(start, start_direct, 0,
                               getManhattanDistance(start, end), nullptr);
  open_list.push(s);
  all_nodes[start * 4 + start_direct] = s;

  while (!open_list.empty()) {
    AstarNode *curr = open_list.top();
    open_list.pop();
    close_list.emplace(curr->location * 4 + curr->direction);
    if (curr->location == end) {
      while (curr->parent != NULL) {
        path.emplace_front(make_pair(curr->location, curr->direction));
        curr = curr->parent;
      }
      break;
    }
    list<pair<int, int>> neighbors =
        getNeighbors(curr->location, curr->direction);
    for (const pair<int, int> &neighbor : neighbors) {
      if (close_list.find(neighbor.first * 4 + neighbor.second) !=
          close_list.end())
        continue;
      if (all_nodes.find(neighbor.first * 4 + neighbor.second) !=
          all_nodes.end()) {
        AstarNode *old = all_nodes[neighbor.first * 4 + neighbor.second];
        if (curr->g + 1 < old->g) {
          old->g = curr->g + 1;
          old->f = old->h + old->g;
          old->parent = curr;
        }
      } else {
        AstarNode *next_node =
            new AstarNode(neighbor.first, neighbor.second, curr->g + 1,
                          getManhattanDistance(neighbor.first, end), curr);
        open_list.push(next_node);
        all_nodes[neighbor.first * 4 + neighbor.second] = next_node;
      }
    }
  }
  for (auto n : all_nodes) {
    delete n.second;
  }
  all_nodes.clear();
  return path;
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {
  int loc1_x = loc1 / env->cols;
  int loc1_y = loc1 % env->cols;
  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2) {
  int loc_x = loc / env->cols;
  int loc_y = loc % env->cols;

  if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
    return false;

  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1)
    return false;
  return true;
}

list<pair<int, int>> MAPFPlanner::getNeighbors(int location, int direction) {
  list<pair<int, int>> neighbors;
  // forward
  int candidates[4] = {location + 1, location + env->cols, location - 1,
                       location - env->cols};
  int forward = candidates[direction];
  int new_direction = direction;
  if (forward >= 0 && forward < env->map.size() &&
      validateMove(forward, location))
    neighbors.emplace_back(make_pair(forward, new_direction));
  // turn left
  new_direction = direction - 1;
  if (new_direction == -1)
    new_direction = 3;
  neighbors.emplace_back(make_pair(location, new_direction));
  // turn right
  new_direction = direction + 1;
  if (new_direction == 4)
    new_direction = 0;
  neighbors.emplace_back(make_pair(location, new_direction));
  neighbors.emplace_back(make_pair(location, direction)); // wait
  return neighbors;
}
