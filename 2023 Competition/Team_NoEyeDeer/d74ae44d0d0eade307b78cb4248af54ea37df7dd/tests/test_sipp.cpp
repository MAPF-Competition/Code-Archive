#include <iostream>

#include "sipp.h"

void getCostMap(const Grid& grid, vector<vector<int>>& costmaps) {
  costmaps.resize(grid.map.size());

  for (int i = 0; i < grid.map.size(); i++) {
    std::vector<int> distances(std::vector<int>(grid.map.size(), INT32_MAX));
    std::vector<int> pred(std::vector<int>(grid.map.size(), -1));
    int goal = i;
    if (grid.map[i] == 1) {
      continue;
    }
    std::queue<int> q;
    q.push(goal);
    distances[goal] = 0;
    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!q.empty()) {
      auto current = q.front();
      auto current_cord =
          std::pair<int, int>{current / grid.cols, current % grid.cols};
      q.pop();
      for (auto dir : directions) {
        int newRow = current_cord.first + dir.first;
        int newCol = current_cord.second + dir.second;
        int newLoc = newRow * grid.cols + newCol;
        if (newRow >= 0 && newRow < grid.rows && newCol >= 0 &&
            newCol < grid.cols && distances[newLoc] == INT32_MAX &&
            grid.map[newLoc] == 0) {
          distances[newLoc] = distances[current] + 1;
          q.push(newLoc);
        }
      }
    }
    costmaps[i] = distances;
  }
}

std::vector<sipp::NodeConstraint> path2cons(
    const std::vector<sipp::SIPPState>& path) {
  std::vector<sipp::NodeConstraint> ret;
  for (auto it : path) {
    ret.emplace_back(sipp::NodeConstraint(it.timestep, it.location));
  }
  return ret;
}

const std::map<int, std::string> DIRECTION = {
    {0, "east"}, {1, "south"}, {2, "west"}, {3, "north"}};

void checkPathWithNodeConstraints(
    const std::vector<State>& path,
    const std::vector<sipp::NodeConstraint>& cons) {
  if (path.empty()) return;
  for (auto con : cons) {
    if (path[con.t].location == con.location) {
      std::cout << path << std::endl;
      std::cout << path[con.t] << std::endl;
    }
    assert(path[con.t].location != con.location);
  }
}

void checkPathWithEdgeConstraints(
    const std::vector<State>& path,
    const std::vector<sipp::EdgeConstraint>& cons) {
  for (auto con : cons) {
    if ((path[con.t].location == con.u) &&
        (path[con.t + 1].location == con.v)) {
      std::cout << path << std::endl;
      std::cout << path[con.t] << "->" << path[con.t + 1] << std::endl;
    }
    assert((path[con.t].location != con.u) ||
           (path[con.t + 1].location != con.v));
  }
}

int TestNodeConstraints(const std::string& map_directory) {
  std::cout << " ***************************************** " << std::endl;
  std::cout << " |||||||||||| TestNodeConstraints ||||||| " << std::endl;
  std::cout << " ***************************************** " << std::endl;
  // 创建一个3x3的简单网格
  Grid grid(map_directory + "/empty-3-3.map");
  // 初始化SIPP
  vector<vector<int>> costmaps;
  getCostMap(grid, costmaps);
  sipp::SIPP planner(grid.map, grid.rows, grid.cols, costmaps);
  /*
  Grid:
  0  1  2
  3  4  5
  6  7  8
  */

  std::pair<int, int> start{0, 0};
  std::pair<int, int> goal{8, 0};

  // NodeConstraint: {time, location}
  std::vector<sipp::NodeConstraint> ncons{};
  std::vector<State> path = planner.Search(start, goal, ncons, {}, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first,0, start.second));
  checkPathWithNodeConstraints(path, ncons);

  ncons = {{1, 0}};
  path = planner.Search(start, goal, ncons, {}, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithNodeConstraints(path, ncons);

  ncons = {{1, 1}, {3, 2}, {3, 6}, {5, 7}, {6, 5}, {6, 7}};
  path = planner.Search(start, goal, ncons, {}, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithNodeConstraints(path, ncons);

  ncons = {{1, 1}, {3, 6}, {6, 7}};
  path = planner.Search(start, goal, ncons, {}, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithNodeConstraints(path, ncons);

  ncons = {{1, 1}, {3, 6}, {6, 7}, {5, 5}};
  path = planner.Search(start, goal, ncons, {}, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithNodeConstraints(path, ncons);

  return 0;
}

int TestEdgeConstraints(const std::string& map_directory) {
  std::cout << " ***************************************** " << std::endl;
  std::cout << " |||||||||||| TestEdgeConstraints ||||||| " << std::endl;
  std::cout << " ***************************************** " << std::endl;
  // 创建一个3x3的简单网格
  Grid grid(map_directory + "/empty-3-3.map");
  // 初始化SIPP
  vector<vector<int>> costmaps;
  getCostMap(grid, costmaps);
  sipp::SIPP planner(grid.map, grid.rows, grid.cols, costmaps);
  /*
  Grid:
  0  1  2
  3  4  5
  6  7  8
  */
  std::pair<int, int> start{0, 0};
  std::pair<int, int> goal{8, 0};

  std::vector<sipp::EdgeConstraint> econs{};
  std::vector<State> path = planner.Search(start, goal, {}, econs, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithEdgeConstraints(path, econs);

  // EdgeConstraint: { time, from, to }
  econs = {{1, 1, 2}};
  path = planner.Search(start, goal, {}, econs, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithEdgeConstraints(path, econs);

  econs = {{0, 0, 1}, {1, 1, 2}, {1, 1, 2}, {1, 1, 2}};
  path = planner.Search(start, goal, {}, econs, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithEdgeConstraints(path, econs);

  econs = {{1, 1, 2}, {3, 4, 7}};
  path = planner.Search(start, goal, {}, econs, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithEdgeConstraints(path, econs);

  econs = {{1, 1, 2}, {3, 4, 7}, {5, 7, 8}};
  path = planner.Search(start, goal, {}, econs, {}, {});
  std::cout << "Start at (" << start.first << "," << start.second
            << ") path:" << std::endl
            << path << std::endl
            << std::endl;
  // path.insert(path.begin(), State(start.first, 0,start.second));
  checkPathWithEdgeConstraints(path, econs);

  return 0;
}

int main(int argc, char** argv) {
  TestNodeConstraints(std::string(argv[1]));
  TestEdgeConstraints(std::string(argv[1]));
  return 0;
}