#pragma once
#include "SharedEnv.h"
#include "customcommon.h"

// Currently only works for undirected unweighted 4-nighbor grids
class Instance {
 public:
  int window;
  int num_of_cols;
  int num_of_rows;
  int map_size;
  SharedEnvironment *env;
  vector<State> start_states;
  vector<int> goal_locations;

  Instance() = default;

  Instance(SharedEnvironment *env, int window);

  void initialize();

  void printAgents() const;

  string getMapFile() const { return map_fname; };
  vector<State> getStarts() const { return start_states; };
  vector<int> getGoals() const { return goal_locations; };

  inline bool isObstacle(int loc) const { return my_map[loc]; }

  inline bool validMove(int curr, int next) const {
    if (next < 0 || next >= map_size) return false;
    if (my_map[next]) return false;
    return getManhattanDistance(curr, next) < 2;
  }

  list<int> getNeighbors(int curr) const;
  list<State> getNeighbors(State curr_state) const;
  int computeNextLocation(const State &curr) const;
  int computeNextDirectionLeft(const State &curr) const;
  int computeNextDirectionRight(const State &curr) const;

  inline int linearizeCoordinate(int row, int col) const { return (this->num_of_cols * row + col); }
  inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
  inline int getColCoordinate(int id) const { return id % this->num_of_cols; }

  inline pair<int, int> getCoordinate(int id) const {
    return make_pair(id / this->num_of_cols, id % this->num_of_cols);
  }

  inline int getCols() const { return num_of_cols; }

  inline int getManhattanDistance(int loc1, int loc2) const {
    int loc1_x = getRowCoordinate(loc1);
    int loc1_y = getColCoordinate(loc1);
    int loc2_x = getRowCoordinate(loc2);
    int loc2_y = getColCoordinate(loc2);
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
  }

  static inline int getManhattanDistance(const pair<int, int> &loc1, const pair<int, int> &loc2) {
    return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
  }

  int getDegree(int loc) const {
    assert(loc >= 0 && loc < map_size && !my_map[loc]);
    int degree = 0;
    if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols]) degree++;
    if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols]) degree++;
    if (loc % num_of_cols > 0 && !my_map[loc - 1]) degree++;
    if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1]) degree++;
    return degree;
  }

  int getDefaultNumberOfAgents() const { return num_of_agents; }
  string getInstanceName() const { return agent_fname; }

  void savePaths(const string &file_name, const vector<Path *> &paths) const;

  bool validateSolution(const vector<Path *> &paths, int sum_of_costs,
                        int num_of_colliding_pairs) const;

 private:
  // int moves_offset[MOVE_COUNT];
  vector<int> my_map;
  string map_fname;
  string agent_fname;

  int num_of_agents;

  bool nathan_benchmark = true;

  bool loadMap();

  void printMap() const;

  void saveMap() const;

  bool loadAgents();

  void saveAgents() const;

  void saveNathan() const;

  void generateConnectedRandomGrid(int rows, int cols, int obstacles);

  // initialize new [rows x cols] map with random obstacles
  void generateRandomAgents(int warehouse_width);

  bool addObstacle(int obstacle);  // add this obsatcle only if the map is still connected
  bool isConnected(int start, int goal);

  // run BFS to find a path between start and goal, return true if a path exists.

  int randomWalk(int loc, int steps) const;

  // Class  SingleAgentSolver can access private members of Node
  friend class SingleAgentSolver;
};
