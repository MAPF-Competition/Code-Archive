#include <MAPFPlanner.h>

void MAPFPlanner::resolve_conflicts() {
  std::srand(std::time(nullptr));
  //std::srand(0);

  // cout << "Resolve stay conflicts" << endl;
  for (int i = 0; i < env->map.size(); i++) {
    if (robots[i].stay) {
      resolve_reserve(robots[i].cur);
    }
  }

  // cout << "Resolve rest of the conflicts" << endl;
  for (int i = 0; i < env->map.size(); i++) {
    in_loop.clear();
    resolve_in_location(i, i);
  }
}

void MAPFPlanner::resolve_reserve(const int robot) {
  const int loc = next_locs[robot].loc;
  if (robots[loc].resolved) return;
  for (int i = 0; i < robots[loc].nbr.size(); i++) {
    const int r2 = robots[loc].nbr[i];
    if (robot != r2 && r2 != -1) {
      const int l2 = env->curr_states[r2].location;
      const int o = env->curr_states[r2].orientation;
      next_locs[r2].loc = l2;
      next_locs[r2].dir = o;
      next_locs[r2].action = Action::W;
      robots[l2].stay = true;
      robots[loc].nbr[i] = -1;
      resolve_reserve(r2);
    }
  }
  robots[loc].resolved = true;
}

bool MAPFPlanner::resolve_in_location(const int loc, const int start) {
  if (robots[loc].resolved) return false;
  if (-1 != robots[loc].cur && robots[loc].stay) {
    resolve_reserve(robots[loc].cur);
  } else if (-1 != robots[loc].cur) {
    const int next_loc = next_locs[robots[loc].cur].loc;
    if (-1 != robots[next_loc].cur &&
        loc == next_locs[robots[next_loc].cur].loc) {
      resolve_edge_conflict(loc, next_loc);
    } else {
      if (in_loop.count(next_loc)) {
        resolve_reserve(robots[loc].cur);
        return true;
      } else {
        in_loop.insert(loc);
        const bool loop = resolve_in_location(next_loc, start);
        in_loop.erase(loc);
        if (!loop) {
          if (!robots[loc].stay) {
            reserve_random(loc);
          }
        } else {
          resolve_reserve(robots[loc].cur);
        }
        return loop;
      }
    }
  } else {
    reserve_random(loc);
  }
  return false;
}

void MAPFPlanner::reserve_random(const int loc) {
  std::vector<int> race;
  race.reserve(4);
  for (int i = 0; i < robots[loc].nbr.size(); i++) {
    if (-1 != robots[loc].nbr[i]) {
      race.push_back(i);
      if (Action::FW == prev_moves[robots[loc].nbr[i]].action) {
        race.push_back(i);
      }
    }
  }
  if (!race.empty()) {
    const int r = std::rand() % race.size();
    resolve_reserve(robots[loc].nbr[race[r]]);
  }
}

void MAPFPlanner::resolve_edge_conflict(const int loc1, const int loc2) {
  const int r1 = robots[loc1].cur;
  const int r2 = robots[loc2].cur;

  next_locs[r1].loc = loc1;
  next_locs[r1].dir = env->curr_states[r1].orientation;
  next_locs[r1].action = Action::W;
  robots[loc1].stay = true;

  next_locs[r2].loc = loc2;
  next_locs[r2].dir = env->curr_states[r2].orientation;
  next_locs[r2].action = Action::W;
  robots[loc2].stay = true;

  robots[loc1].nbr[(next_locs[r2].dir + 2) % 4] = -1;
  robots[loc2].nbr[(next_locs[r1].dir + 2) % 4] = -1;

  resolve_reserve(r1);
  resolve_reserve(r2);
}

void MAPFPlanner::handle_waiting_robots() {
  for (int robot = 0; robot < env->num_of_agents; ++robot) {
    if (next_locs[robot].action == Action::W) {
      wait_times[robot]++;
      if (wait_times[robot] > config.max_wait_time) {
        wait_times[robot] = 0;
        paths[robot].clear();
        step_in_available_dir(robot);
      }
    }
  }
}

int MAPFPlanner::get_step_loc(const int loc, const int dir) {
  auto [x, y] = get_row_col_idx(loc);
  switch (dir) {
    case EAST:
      return y < env->cols - 1 ? loc + 1 : -1;
    case SOUTH:
      return x < env->rows - 1 ? loc + env->cols : -1;
    case WEST:
      return y > 0 ? loc - 1 : -1;
    case NORTH:
      return x > 0 ? loc - env->cols : -1;
    default:
      return -1;
  }
}

void MAPFPlanner::step_in_available_dir(const int robot) {
  const int loc = env->curr_states[robot].location;
  const int dir = env->curr_states[robot].orientation;
  auto set_step = [this](const int robot, const int loc, const int next_dir,
                         const Action action) -> bool {
    if (const int next = get_step_loc(loc, next_dir);
        next != -1 && env->map[next] == 0 && robots[next].stay == false &&
        robots[next].nbr[0] == -1 && robots[next].nbr[1] == -1 &&
        robots[next].nbr[2] == -1 && robots[next].nbr[3] == -1) {
      next_locs[robot].loc = loc;
      next_locs[robot].dir = next_dir;
      next_locs[robot].action = action;
      robots[next].nbr[(next_dir + 2) % 4] = robot;
      paths[robot].push_back(get_move(loc, next_dir, next));
      return true;
    }
    return false;
  };

  // Can move to clockwise turn location
  if (set_step(robot, loc, (dir + 1) % 4, Action::CR)) return;

  // Can move to counter clockwise location
  if (set_step(robot, loc, (dir + 3) % 4, Action::CCR)) return;

  // Can move back
}
