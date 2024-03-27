#include <MAPFPlanner.h>

#include <boost/asio.hpp>

constexpr int MAX_PLAN_IN_TIMESTEP = 3000;

void MAPFPlanner::plan_with_simple_a_star() {
  for (int i = 0; i < env->map.size(); i++) {
    robots[i].reset();
  }

  boost::asio::thread_pool pool;

  for (int i = 0; i < env->num_of_agents; i++) {
    robots[env->curr_states[i].location].cur = i;
    if (env->goal_locations[i].empty()) {
      robots[env->curr_states[i].location].stay = true;
      next_locs[i].loc = env->curr_states[i].location;
      next_locs[i].dir = env->curr_states[i].orientation;
      next_locs[i].action = Action::W;
    } else {
      boost::asio::post(pool, [this, i]() {
        const int next = get_next_location_with_a_star(
            env->curr_states[i].location, env->goal_locations[i].front().first);
        set_next_locations_for_robot(i, next);
      });
    }
  }

  pool.join();
}

void MAPFPlanner::plan_with_simple_a_star_and_reuse() {
  for (int i = 0; i < env->map.size(); i++) {
    robots[i].reset();
  }

  std::srand(std::time(nullptr));

  int now_planned = 0;

  boost::asio::thread_pool pool;

  for (int i = 0; i < env->num_of_agents; i++) {
    robots[env->curr_states[i].location].cur = i;

    if (!paths[i].empty() &&
        env->curr_states[i].location == paths[i].front().loc &&
        env->curr_states[i].orientation == paths[i].front().dir) {
      prev_moves[i] = paths[i].front();
      paths[i].pop_front();
    }

    if (!paths[i].empty()) {
      set_next_locations_for_robot(i, paths[i].front());
    } else {
      if (env->goal_locations[i].empty() ||
          MAX_PLAN_IN_TIMESTEP < now_planned) {
        robots[env->curr_states[i].location].stay = true;
        next_locs[i].loc = env->curr_states[i].location;
        next_locs[i].dir = env->curr_states[i].orientation;
        next_locs[i].action = Action::W;
      } else {
        now_planned++;
        boost::asio::post(pool, [this, i]() {
          const int next = create_path_for_robot_with_a_star(
              i, env->curr_states[i].location,
              env->goal_locations[i].front().first);
          if (!paths[i].empty()) {
			      set_next_locations_for_robot(i, paths[i].front());
          }
        });
      }
    }
  }

  pool.join();
}

void MAPFPlanner::set_next_locations_for_robot(const int robot,
                                               const int next) {
  const int curr = env->curr_states[robot].location;
  const int ori = env->curr_states[robot].orientation;
  const Move next_move = get_move(curr, ori, next);
  next_locs[robot] = next_move;
  if (curr == next_move.loc) {
    robots[curr].stay = true;
  } else {
    switch (next_move.dir) {
      case EAST:
        robots[next_move.loc].nbr[WEST] = robot;
        break;
      case SOUTH:
        robots[next_move.loc].nbr[NORTH] = robot;
        break;
      case WEST:
        robots[next_move.loc].nbr[EAST] = robot;
        break;
      case NORTH:
        robots[next_move.loc].nbr[SOUTH] = robot;
        break;
    }
  }
}

void MAPFPlanner::set_next_locations_for_robot(const int robot,
                                               const Move next_move) {
  const int curr = env->curr_states[robot].location;
  next_locs[robot] = next_move;
  if (curr == next_move.loc) {
    robots[curr].stay = true;
  } else {
    switch (next_move.dir) {
      case EAST:
        robots[next_move.loc].nbr[WEST] = robot;
        break;
      case SOUTH:
        robots[next_move.loc].nbr[NORTH] = robot;
        break;
      case WEST:
        robots[next_move.loc].nbr[EAST] = robot;
        break;
      case NORTH:
        robots[next_move.loc].nbr[SOUTH] = robot;
        break;
    }
  }
}

int MAPFPlanner::get_next_location_with_a_star(int start, int end) {
  std::vector<int> g;
  g.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> h;
  h.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> f;
  f.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> p;
  p.resize(env->map.size(), -1);
  std::vector<bool> c;
  c.resize(env->map.size(), false);
  auto vert_before = [&f](int a, int b) { return f[a] > f[b]; };
  std::priority_queue<int, std::vector<int>, decltype(vert_before)> open{
      vert_before};
  open.push(start);
  g[start] = 0;

  while (!open.empty()) {
    const int q = open.top();
    open.pop();
    if (c[q]) continue;

    for (int i = 0; neighbours[q][i] != -1; ++i) {
      const int idx = neighbours[q][i];

      if (end == idx) {
        p[idx] = q;
        int curr = idx;
        while (curr != -1 && p[curr] != start && p[curr] != -1) {
          curr = p[curr];
        }
        return curr;
      }

      int cur_g = 0;

      const auto [prev_i, prev_j] = get_row_col_idx(p[q]);
      const auto [cur_i, cur_j] = get_row_col_idx(q);
      const auto [next_i, next_j] = get_row_col_idx(idx);
      const int prev_step_i = cur_i - prev_i;
      const int prev_step_j = cur_j - prev_j;
      const int next_step_i = next_i - cur_i;
      const int next_step_j = next_j - cur_j;

      if (prev_step_i == next_step_i && prev_step_j == next_step_j)
        cur_g = 1;
      else if (prev_step_i == -next_step_i || prev_step_j == -next_step_j)
        cur_g = 3;
      else
        cur_g = 2;

      cur_g += g[q];
      h[idx] = get_manhattan_distance(idx, end);

      if (g[idx] > cur_g) {
        g[idx] = cur_g;
        f[idx] = cur_g + h[idx];
        p[idx] = q;
        open.push(idx);
      }
    }

    c[q] = true;
  }

  return start;
}

int MAPFPlanner::create_path_for_robot_with_a_star(const int robot,
                                                   const int start,
                                                   const int end) {
  std::vector<int> g;
  g.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> h;
  h.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> f;
  f.resize(env->map.size(), std::numeric_limits<int>::max());
  std::vector<int> p;
  p.resize(env->map.size(), -1);
  std::vector<bool> c;
  c.resize(env->map.size(), false);
  auto vert_before = [&f](int a, int b) { return f[a] > f[b]; };
  std::priority_queue<int, std::vector<int>, decltype(vert_before)> open{
      vert_before};
  open.push(start);
  g[start] = 0;

  while (!open.empty()) {
    const int q = open.top();
    open.pop();
    if (c[q]) continue;

    for (int i = 0; neighbours[q][i] != -1; ++i) {
      const int idx = neighbours[q][i];

      if (end == idx) {
        paths[robot].clear();
        std::list<int> path;
        p[idx] = q;
        int curr = idx;
        while (curr != -1 && p[curr] != start && p[curr] != -1) {
          path.push_front(curr);
          curr = p[curr];
        }
        path.push_front(curr);

        int start_loc = env->curr_states[robot].location;
        int start_dir = env->curr_states[robot].orientation;
        Move cur = get_move(start_loc, start_dir, start);
        for (auto step : path) {
          while (cur.loc != step) {
            cur = get_move(cur.loc, cur.dir, step);
            paths[robot].push_back(cur);
          }
        }

        return curr;
      }

      int cur_g = 0;

      const auto [prev_i, prev_j] = get_row_col_idx(p[q]);
      const auto [cur_i, cur_j] = get_row_col_idx(q);
      const auto [next_i, next_j] = get_row_col_idx(idx);
      const int prev_step_i = cur_i - prev_i;
      const int prev_step_j = cur_j - prev_j;
      const int next_step_i = next_i - cur_i;
      const int next_step_j = next_j - cur_j;

      if (prev_step_i == next_step_i && prev_step_j == next_step_j)
        cur_g = 1;
      else if (prev_step_i == -next_step_i || prev_step_j == -next_step_j)
        cur_g = 3;
      else
        cur_g = 2;

      cur_g += g[q];
      h[idx] = get_manhattan_distance(idx, end);

      if (g[idx] > cur_g) {
        g[idx] = cur_g;
        f[idx] = cur_g + h[idx];
        p[idx] = q;
        open.push(idx);
      }
    }

    c[q] = true;
  }

  return start;
}

MAPFPlanner::Move MAPFPlanner::get_move(const int loc, const int dir,
                                        const int next) const {
  const int east = loc + 1;
  const int south = loc + env->cols;
  const int west = loc - 1;
  const int north = loc - env->cols;
  switch (dir) {
    case EAST: {
      if (next == east) return {east, EAST, Action::FW};
      if (next == south) return {loc, SOUTH, Action::CR};
      if (next == west) return {loc, SOUTH, Action::CR};
      if (next == north) return {loc, NORTH, Action::CCR};
    } break;
    case SOUTH: {
      if (next == east) return {loc, EAST, Action::CCR};
      if (next == south) return {south, SOUTH, Action::FW};
      if (next == west) return {loc, WEST, Action::CR};
      if (next == north) return {loc, EAST, Action::CCR};
    } break;
    case WEST: {
      if (next == east) return {loc, SOUTH, Action::CCR};
      if (next == south) return {loc, SOUTH, Action::CCR};
      if (next == west) return {west, WEST, Action::FW};
      if (next == north) return {loc, NORTH, Action::CR};
    } break;
    case NORTH: {
      if (next == east) return {loc, EAST, Action::CR};
      if (next == south) return {loc, EAST, Action::CR};
      if (next == west) return {loc, WEST, Action::CCR};
      if (next == north) return {north, NORTH, Action::FW};
    } break;
    default:
      break;
  }
  return {loc, dir, Action::W};
}

int MAPFPlanner::get_manhattan_distance(int loc1, int loc2) const {
  const auto [loc1_x, loc1_y] = get_row_col_idx(loc1);
  const auto [loc2_x, loc2_y] = get_row_col_idx(loc2);
  return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}
