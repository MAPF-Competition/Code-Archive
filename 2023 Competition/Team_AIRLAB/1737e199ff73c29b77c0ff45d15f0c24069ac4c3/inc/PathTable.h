#pragma once
#include "customcommon.h"

#define NO_AGENT -1

class PathTable {
 public:
  int window;
  int makespan = 0;
  vector<vector<int>>
      table;  // this stores the collision-free paths, the value is the id of the agent
  vector<vector<int>> goals;
  int num_of_agents;
  // this stores the goal locatons of the paths: key is the location, while value is the timestep
  // when the agent reaches the goal
  void reset() {
    auto map_size = table.size();
    table.clear();
    table.resize(map_size);
    goals.assign(map_size, vector<int>(num_of_agents, MAX_COST));
    makespan = 0;
  }

  void insertPath(int agent_id, const Path& path);

  void deletePath(int agent_id, const Path& path);

  bool constrained(int from, int to, int to_time) const;

  void get_agents(set<int>& conflicting_agents, int loc) const;

  void get_agents(set<int>& conflicting_agents, int neighbor_size, int loc) const;

  void getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to,
                            int to_time) const;
  ;

  int getHoldingTime(int location, int earliest_timestep) const;

  explicit PathTable(int window, int num_of_agents, int map_size = 0)
      : window(window),
        table(map_size),
        goals(map_size, vector<int>(num_of_agents, MAX_COST)),
        num_of_agents(num_of_agents) {}
};

class PathTableWC  // with collisions
{
 public:
  int window;
  int makespan = 0;
  vector<vector<list<int>>> table;  // this stores the paths, the value is the id of the agent
  vector<vector<int>> goals;
  int num_of_agents;
  // this stores the goal locatons of the paths: key is the location, while value is the timestep
  // when the agent reaches the goal
  void reset() {
    auto map_size = table.size();
    table.clear();
    table.resize(map_size);
    goals.assign(map_size, vector<int>(num_of_agents, MAX_COST));
    makespan = 0;
  }

  void insertPath(int agent_id, const Path& path);

  void insertPath(int agent_id);

  void deletePath(int agent_id);

  const Path* getPath(int agent_id) const { return paths[agent_id]; }

  int getFutureNumOfCollisions(int loc, int time) const;

  // return #collisions when the agent waiting at loc starting from time forever
  int getNumOfCollisions(int from, int to, int to_time) const;

  bool hasCollisions(int from, int to, int to_time) const;

  bool hasEdgeCollisions(int from, int to, int to_time) const;

  int getLastCollisionTimestep(int location) const;

  // return the agent who reaches its target target_location before timestep earliest_timestep
  int getAgentWithTarget(int target_location, int latest_timestep) const;

  void clear();

  explicit PathTableWC(int window, int map_size = 0, int num_of_agents = 0)
      : window(window),
        table(map_size),
        goals(map_size, vector<int>(num_of_agents, MAX_COST)),
        paths(num_of_agents, nullptr),
        num_of_agents(num_of_agents) {}

 private:
  vector<const Path*> paths;
};
