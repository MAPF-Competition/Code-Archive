#include <MAPFPlanner.h>

void MAPFPlanner::proc_simple_map() {
  neighbours.resize(env->map.size());
  components.resize(env->map.size(), -1);

  for (int loc = 0; loc < env->map.size(); loc++) {
    if (1 == env->map[loc]) {
      neighbours[loc][0] = -1;
    } else {
      const auto [x, y] = get_row_col_idx(loc);
      int n_count = 0;

      if (y + 1 < env->cols && 1 != env->map[loc + 1]) {
        neighbours[loc][n_count++] = loc + 1;
      }

      if (y > 0 && 1 != env->map[loc - 1]) {
        neighbours[loc][n_count++] = loc - 1;
      }

      if (x + 1 < env->rows && 1 != env->map[loc + env->cols]) {
        neighbours[loc][n_count++] = loc + env->cols;
      }

      if (x > 0 && 1 != env->map[loc - env->cols]) {
        neighbours[loc][n_count++] = loc - env->cols;
      }

      neighbours[loc][n_count] = -1;
    }
  }

  SCC();
}

void MAPFPlanner::proc_limited_map() {
  std::vector<bool> unreachable;
  unreachable.resize(env->map.size(), true);
  neighbours.resize(env->map.size());
  components.resize(env->map.size(), -1);

  for (int loc = 0; loc < env->map.size(); loc++) {
    if (1 == env->map[loc]) {
      neighbours[loc][0] = -1;
    } else {
      const auto [x, y] = get_row_col_idx(loc);
      std::array<int, 5> possible = {-1, -1, -1, -1, -1};
      size_t n_count = 0;
      size_t p_count = 0;

      if (y + 1 < env->cols && 1 != env->map[loc + 1]) {
        if (x % 2 == 0) {
          neighbours[loc][n_count++] = loc + 1;
          unreachable[loc + 1] = false;
        } else {
          possible[p_count++] = loc + 1;
          unreachable[loc] = false;
        }
      }

      if (y > 0 && 1 != env->map[loc - 1]) {
        if (x % 2 == 1 || unreachable[loc - 1]) {
          neighbours[loc][n_count++] = loc - 1;
          unreachable[loc - 1] = false;
        } else {
          possible[p_count++] = loc - 1;
          unreachable[loc] = false;
        }
      }

      if (x + 1 < env->rows && 1 != env->map[loc + env->cols]) {
        if (y % 2 == 0) {
          neighbours[loc][n_count++] = loc + env->cols;
          unreachable[loc + env->cols] = false;
        } else {
          possible[p_count++] = loc + env->cols;
          unreachable[loc] = false;
        }
      }

      if (x > 0 && 1 != env->map[loc - env->cols]) {
        if (y % 2 == 1 || unreachable[loc - env->cols]) {
          neighbours[loc][n_count++] = loc - env->cols;
          unreachable[loc - env->cols] = false;
        } else {
          possible[p_count++] = loc - env->cols;
          unreachable[loc] = false;
        }
      }

      if (!n_count && p_count) {
        neighbours[loc][n_count++] = possible[0];
      }
      neighbours[loc][n_count] = -1;
    }
  }

  SCC();
  merge_components();
}

void MAPFPlanner::SCC() {
  std::vector<int> disc;
  disc.resize(env->map.size(), -1);
  std::vector<int> low;
  low.resize(env->map.size(), -1);
  std::vector<bool> on_stack;
  on_stack.resize(env->map.size(), false);
  std::stack<int> st;
  int time = 0;

  for (int u = 0; u < env->map.size(); u++) {
    if (-1 == disc[u] && !env->map[u]) {
      SCC_util(u, time, components, disc, low, st, on_stack);
    }
  }
}

void MAPFPlanner::SCC_util(int u, int& time, std::vector<int>& comp,
                           std::vector<int>& disc, std::vector<int>& low,
                           std::stack<int>& st, std::vector<bool>& on_stack) {
  disc[u] = low[u] = ++time;
  st.push(u);
  on_stack[u] = true;

  for (int i = 0; neighbours[u][i] != -1; i++) {
    const int v = neighbours[u][i];
    if (-1 == disc[v]) {
      SCC_util(v, time, comp, disc, low, st, on_stack);
      low[u] = std::min(low[u], low[v]);
    } else if (on_stack[v]) {
      low[u] = std::min(low[u], disc[v]);
    }
  }

  int w = 0;
  if (low[u] == disc[u]) {
    while (st.top() != u) {
      w = st.top();
      comp[w] = low[u];
      on_stack[w] = false;
      st.pop();
    }
    w = st.top();
    comp[w] = low[u];
    on_stack[w] = false;
    st.pop();
  }
}

void MAPFPlanner::merge_components() {
  for (int loc = 0; loc < env->map.size(); loc++) {
    if (env->map[loc]) continue;

    const auto [x, y] = get_row_col_idx(loc);

    auto addEdge = [&](const int loc, const int loc2) {
      if (components[loc] != components[loc2]) {
        int i = 0;
        bool found = false;
        while (-1 != neighbours[loc][i]) {
          if (loc2 == neighbours[loc][i]) found = true;
          i++;
        }
        if (!found) {
          neighbours[loc][i] = loc2;
          neighbours[loc][i + 1] = -1;
          SCC();
        }
      }
    };

    if (y + 1 < env->cols && 1 != env->map[loc + 1]) {
      addEdge(loc, loc + 1);
    }

    if (y > 0 && 1 != env->map[loc - 1]) {
      addEdge(loc, loc - 1);
    }

    if (x + 1 < env->rows && 1 != env->map[loc + env->cols]) {
      addEdge(loc, loc + env->cols);
    }

    if (x > 0 && 1 != env->map[loc - env->cols]) {
      addEdge(loc, loc - env->cols);
    }
  }
}

void MAPFPlanner::proc_special_map() {
  switch (env->map.size()) {
    case 70000:  // Warehouse
      {
        int wall = 0;
        for (int i = 0; i < env->map.size(); i++) {
          if (env->map[i]) {
            wall++;
          }
        }
        switch (wall) {
          case 31414:  // Warehouse
            proc_special_warehouse(130, 491, 7, 7, 3, 4);
            break;
          case 15680:  // Sortation
            proc_special_warehouse(133, 493, 5, 5, 2, 2);
            break;
          default:
            proc_limited_map();
            break;
        }
      }
      break;
    case 1024:  // Random
      if (env->num_of_agents >= 600) {
        proc_special_random();
        break;
      }
    case 65536:   // City
    case 254930:  // Game
    default:
      proc_limited_map();
      break;
  }
}

void MAPFPlanner::proc_special_warehouse(const int b_se_x, const int b_se_y,
                                         const int b_nw_x, const int b_nw_y,
                                         const int x_step, const int y_step) {
  std::vector<bool> unreachable;
  unreachable.resize(env->map.size(), true);
  neighbours.resize(env->map.size());
  components.resize(env->map.size(), -1);

  for (int loc = 0; loc < env->map.size(); loc++) {
    if (1 == env->map[loc]) {
      neighbours[loc][0] = -1;
    } else {
      const auto [x, y] = get_row_col_idx(loc);
      std::array<int, 5> possible = {-1, -1, -1, -1, -1};
      size_t n_count = 0;
      size_t p_count = 0;

      if (b_se_x >= x && b_nw_x <= x && b_se_y >= y && b_nw_y <= y) {
        if (y + 1 < env->cols && 1 != env->map[loc + 1]) {
          if ((x / x_step) % 2 == 0) {
            neighbours[loc][n_count++] = loc + 1;
            unreachable[loc + 1] = false;
          } else {
            possible[p_count++] = loc + 1;
            unreachable[loc] = false;
          }
        }

        if (y > 0 && 1 != env->map[loc - 1]) {
          if ((x / x_step) % 2 == 1 || unreachable[loc - 1]) {
            neighbours[loc][n_count++] = loc - 1;
            unreachable[loc - 1] = false;
          } else {
            possible[p_count++] = loc - 1;
            unreachable[loc] = false;
          }
        }

        if (x + 1 < env->rows && 1 != env->map[loc + env->cols]) {
          if ((y / y_step) % 2 == 0) {
            neighbours[loc][n_count++] = loc + env->cols;
            unreachable[loc + env->cols] = false;
          } else {
            possible[p_count++] = loc + env->cols;
            unreachable[loc] = false;
          }
        }

        if (x > 0 && 1 != env->map[loc - env->cols]) {
          if ((y / y_step) % 2 == 1 || unreachable[loc - env->cols]) {
            neighbours[loc][n_count++] = loc - env->cols;
            unreachable[loc - env->cols] = false;
          } else {
            possible[p_count++] = loc - env->cols;
            unreachable[loc] = false;
          }
        }
      } else {
        if (y + 1 < env->cols && 1 != env->map[loc + 1]) {
          if (x % 2 == 0) {
            neighbours[loc][n_count++] = loc + 1;
            unreachable[loc + 1] = false;
          } else {
            possible[p_count++] = loc + 1;
            unreachable[loc] = false;
          }
        }

        if (y > 0 && 1 != env->map[loc - 1]) {
          if (x % 2 == 1 || unreachable[loc - 1]) {
            neighbours[loc][n_count++] = loc - 1;
            unreachable[loc - 1] = false;
          } else {
            possible[p_count++] = loc - 1;
            unreachable[loc] = false;
          }
        }

        if (x + 1 < env->rows && 1 != env->map[loc + env->cols]) {
          if (y % 2 == 0) {
            neighbours[loc][n_count++] = loc + env->cols;
            unreachable[loc + env->cols] = false;
          } else {
            possible[p_count++] = loc + env->cols;
            unreachable[loc] = false;
          }
        }

        if (x > 0 && 1 != env->map[loc - env->cols]) {
          if (y % 2 == 1 || unreachable[loc - env->cols]) {
            neighbours[loc][n_count++] = loc - env->cols;
            unreachable[loc - env->cols] = false;
          } else {
            possible[p_count++] = loc - env->cols;
            unreachable[loc] = false;
          }
        }
      }

      if (!n_count && p_count) {
        neighbours[loc][n_count++] = possible[0];
      }
      neighbours[loc][n_count] = -1;
    }
  }

  SCC();
  merge_components();
}

void MAPFPlanner::proc_special_random() {
  neighbours.resize(env->map.size());
  std::vector<int> dirs = {
      1,  6,  4,  4,  4,  6,  4,  4,  4,  4,  16, 2,  4,  4,  6,  4,  4,  16,
      2,  4,  4,  16, 2,  16, 2,  4,  4,  4,  4,  4,  4,  4,  16, 2,  4,  8,
      16, 2,  16, 16, 2,  12, 4,  4,  4,  4,  4,  6,  12, 4,  4,  16, 8,  16,
      10, 4,  4,  16, 1,  1,  1,  3,  2,  8,  2,  6,  8,  8,  1,  1,  3,  1,
      3,  1,  1,  1,  1,  8,  16, 1,  8,  8,  4,  4,  12, 4,  4,  16, 16, 1,
      9,  8,  16, 1,  2,  8,  2,  5,  8,  8,  8,  16, 2,  16, 3,  1,  1,  1,
      1,  9,  1,  1,  3,  8,  16, 2,  4,  4,  4,  6,  4,  12, 9,  1,  2,  16,
      2,  8,  2,  16, 16, 8,  9,  1,  1,  1,  3,  1,  3,  1,  1,  8,  16, 16,
      2,  16, 1,  1,  1,  1,  1,  2,  16, 8,  8,  16, 2,  1,  1,  8,  2,  6,
      4,  8,  8,  8,  16, 2,  6,  16, 8,  16, 1,  8,  16, 16, 3,  1,  9,  1,
      1,  8,  16, 2,  4,  12, 8,  16, 2,  8,  16, 16, 2,  1,  8,  8,  9,  8,
      16, 2,  4,  16, 16, 1,  8,  8,  4,  4,  4,  4,  12, 16, 2,  4,  4,  6,
      8,  9,  8,  16, 2,  8,  4,  16, 2,  16, 8,  8,  8,  16, 1,  6,  6,  4,
      4,  12, 4,  12, 4,  4,  4,  4,  12, 4,  4,  16, 16, 3,  9,  8,  16, 2,
      4,  16, 8,  4,  3,  1,  9,  8,  8,  4,  16, 2,  6,  6,  4,  4,  4,  4,
      4,  4,  16, 8,  6,  4,  14, 4,  4,  4,  16, 8,  16, 1,  1,  2,  8,  8,
      1,  2,  16, 8,  16, 8,  4,  6,  2,  2,  4,  8,  4,  6,  4,  12, 4,  16,
      2,  16, 2,  9,  1,  1,  1,  9,  4,  16, 16, 2,  8,  8,  16, 2,  16, 8,
      1,  8,  8,  2,  2,  1,  9,  8,  16, 2,  16, 8,  8,  2,  4,  4,  4,  8,
      16, 8,  8,  8,  16, 2,  6,  6,  8,  12, 2,  6,  4,  12, 8,  8,  8,  2,
      1,  3,  1,  8,  16, 1,  3,  9,  8,  2,  16, 8,  16, 8,  16, 8,  8,  9,
      1,  2,  2,  2,  16, 8,  2,  4,  16, 8,  8,  8,  8,  2,  16, 2,  16, 8,
      16, 16, 2,  16, 8,  6,  4,  12, 4,  12, 1,  8,  8,  8,  16, 2,  2,  2,
      4,  12, 2,  16, 1,  9,  9,  8,  8,  2,  4,  4,  4,  12, 6,  4,  4,  4,
      4,  2,  16, 8,  16, 8,  9,  8,  8,  8,  16, 2,  1,  2,  16, 8,  2,  1,
      8,  16, 16, 16, 8,  2,  4,  4,  4,  4,  2,  16, 2,  4,  12, 1,  2,  9,
      1,  8,  8,  16, 8,  12, 6,  6,  4,  1,  1,  8,  2,  9,  1,  1,  1,  1,
      9,  2,  16, 8,  4,  12, 2,  16, 2,  16, 8,  16, 2,  8,  16, 8,  9,  1,
      1,  1,  2,  2,  8,  1,  1,  10, 3,  8,  16, 1,  1,  8,  16, 3,  1,  1,
      1,  9,  3,  1,  2,  16, 8,  1,  1,  10, 16, 8,  8,  16, 16, 16, 2,  2,
      9,  8,  16, 8,  8,  16, 1,  9,  1,  1,  1,  2,  16, 16, 16, 8,  2,  16,
      2,  16, 8,  8,  16, 8,  16, 9,  8,  16, 16, 2,  6,  2,  8,  16, 16, 16,
      16, 2,  14, 4,  4,  4,  4,  6,  6,  4,  16, 8,  3,  1,  3,  1,  8,  9,
      4,  16, 1,  8,  16, 2,  6,  6,  1,  3,  8,  16, 16, 16, 2,  4,  3,  1,
      3,  2,  16, 2,  1,  8,  16, 8,  4,  16, 1,  2,  16, 8,  16, 1,  8,  8,
      16, 1,  2,  4,  16, 2,  8,  4,  6,  4,  2,  16, 2,  16, 1,  1,  1,  1,
      3,  9,  1,  8,  16, 1,  9,  3,  1,  9,  1,  9,  9,  8,  16, 16, 2,  16,
      1,  5,  8,  16, 1,  8,  2,  2,  4,  4,  4,  4,  12, 16, 2,  16, 2,  6,
      4,  12, 4,  4,  4,  4,  16, 8,  12, 16, 16, 16, 1,  2,  16, 16, 16, 2,
      4,  12, 2,  2,  16, 8,  16, 8,  8,  16, 2,  2,  6,  2,  16, 8,  4,  8,
      8,  9,  4,  16, 9,  1,  1,  3,  1,  3,  1,  1,  3,  2,  16, 8,  2,  3,
      1,  9,  1,  8,  8,  16, 2,  2,  2,  1,  2,  16, 8,  8,  9,  8,  16, 1,
      9,  8,  16, 8,  16, 2,  16, 16, 1,  3,  1,  8,  2,  3,  1,  1,  1,  1,
      9,  1,  3,  1,  3,  1,  1,  1,  8,  9,  8,  16, 1,  8,  8,  16, 2,  16,
      2,  4,  16, 2,  16, 2,  1,  8,  2,  1,  3,  1,  1,  1,  9,  1,  1,  3,
      1,  3,  1,  1,  1,  8,  16, 1,  8,  1,  8,  4,  12, 4,  4,  16, 2,  12,
      16, 2,  8,  16, 2,  16, 1,  2,  1,  1,  8,  8,  16, 2,  16, 1,  1,  1,
      1,  1,  1,  9,  1,  10, 16, 16, 2,  4,  12, 4,  4,  14, 4,  4,  8,  16,
      1,  2,  16, 1,  8,  16, 8,  8,  16, 3,  3,  1,  3,  1,  1,  1,  1,  1,
      1,  1,  3,  3,  2,  2,  4,  4,  8,  2,  5,  1,  9,  4,  16, 3,  2,  16,
      16, 1,  12, 8,  4,  6,  4,  16, 3,  1,  8,  16, 9,  1,  10, 16, 2,  2,
      2,  1,  3,  8,  8,  2,  8,  16, 16, 16, 16, 1,  3,  1,  2,  16, 8,  4,
      4,  4,  16, 1,  5,  1,  8,  16, 8,  16, 8,  16, 2,  2,  2,  16, 2,  8,
      9,  1,  9,  2,  16, 16, 16, 16, 2,  16, 1,  2,  16, 16, 8,  16, 2,  16,
      16, 1,  9,  1,  9,  2,  16, 2,  4,  1,  3,  1,  1,  9,  8,  16, 8,  6,
      4,  4,  16, 16, 1,  1,  1,  1,  1,  1,  9,  1,  9,  1,  1,  9,  8,  16,
      8,  4,  16, 1,  1,  1,  1,  1,  1,  1,  9,  4,  16, 1,  1,  8};

  for (int loc = 0; loc < neighbours.size(); loc++) {
    if (1 == env->map[loc]) {
      neighbours[loc][0] = -1;
    } else {
      const auto [x, y] = get_row_col_idx(loc);
      int n_count = 0;

      if (y + 1 < env->cols && 1 != env->map[loc + 1] && dirs[loc] & 1) {
        neighbours[loc][n_count++] = loc + 1;
      }

      if (y > 0 && 1 != env->map[loc - 1] && dirs[loc] & 4) {
        neighbours[loc][n_count++] = loc - 1;
      }

      if (x + 1 < env->rows && 1 != env->map[loc + env->cols] &&
          dirs[loc] & 2) {
        neighbours[loc][n_count++] = loc + env->cols;
      }

      if (x > 0 && 1 != env->map[loc - env->cols] && dirs[loc] & 8) {
        neighbours[loc][n_count++] = loc - env->cols;
      }

      neighbours[loc][n_count] = -1;
    }
  }
}
