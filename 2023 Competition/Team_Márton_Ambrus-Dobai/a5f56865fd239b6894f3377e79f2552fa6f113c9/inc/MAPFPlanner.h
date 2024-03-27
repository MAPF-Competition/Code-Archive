#pragma once
#include <ctime>
#include <stack>
#include <unordered_set>

#include "ActionModel.h"
#include "SharedEnv.h"

class MAPFPlanner {
 public:
  SharedEnvironment* env;

  explicit MAPFPlanner(SharedEnvironment* env) : env(env), env_raii(env){};
  MAPFPlanner() : env(new SharedEnvironment()), env_raii(env){};
  // virtual ~MAPFPlanner() { delete env; };

  virtual void initialize(int preprocess_time_limit);

  // Return next states for all agents
  virtual void plan(int time_limit, std::vector<Action>& actions);

 private:
  std::unique_ptr<SharedEnvironment> env_raii;

  struct Loc {
    int row, col;
  };
  inline Loc get_row_col_idx(const int loc) const {
    return {loc / env->cols, loc % env->cols};
  };

  // Runtime configurations
  struct Configuration {
    enum MapProcType { Simple, Limited, Special };
    bool reuse_path = true;
    bool export_map = false;
    bool use_random = true;
    MapProcType map_proc_type = Special;
    int max_wait_time = 4;
    std::string map_file_name = "map.json";
  };
  Configuration config;
  void read_config();

  // Debug utilities
  int get_cell_type(int loc);
  void write_map(std::ostream& os);
  void export_map();

  // Map preprocess functions
  std::vector<std::array<int, 5>> neighbours;
  void proc_simple_map();
  void proc_limited_map();
  void proc_special_map();
  void proc_special_warehouse(const int b_se_x, const int b_se_y,
                              const int b_nw_x, const int b_nw_y,
                              const int x_step, const int y_step);
  void proc_special_random();

  // Tarjan's Algorithm for Strongly Connected Components
  std::vector<int> components;
  void SCC();
  void SCC_util(int u, int& time, std::vector<int>& comp,
                std::vector<int>& disc, std::vector<int>& low,
                std::stack<int>& st, std::vector<bool>& on_stack);
  void merge_components();

  // Robot locations
  enum DIR { EAST = 0, SOUTH = 1, WEST = 2, NORTH = 3 };
  struct RobotsInLocation {
    bool resolved = false;  // True if the conflicts are resolved here
    bool stay = false;      // True if the robot don't want to move away
    int cur = -1;           // The robot currently in the location
    std::array<int, 4> nbr = {
        -1, -1, -1, -1};  // Neighbours want to step here, index with DIR enums!

    void reset() {
      resolved = false;
      stay = false;
      cur = -1;
      nbr.fill(-1);
    };
  };
  std::vector<RobotsInLocation> robots;  // robots in the location
  struct Move {
    int loc, dir;
    Action action;
  };
  std::vector<Move> next_locs;  // planned next location for the robot
  void set_next_locations_for_robot(const int robot, const int next);
  void set_next_locations_for_robot(const int robot, const Move next_move);

  // Path planners
  std::vector<std::list<Move>> paths;
  void plan_with_simple_a_star();
  void plan_with_simple_a_star_and_reuse();
  int get_next_location_with_a_star(int start, int end);
  int create_path_for_robot_with_a_star(const int robot, const int start,
                                        const int end);
  Move get_move(const int loc, const int dir, const int next) const;
  int get_manhattan_distance(int loc1, int loc2) const;

  // Conflict resolution
  std::unordered_set<int> in_loop;
  void resolve_conflicts();
  void resolve_reserve(const int robot);  // Robot can reserve it's next place
  bool resolve_in_location(
      const int loc, const int start);  // Resolve conflicts on given location,
                                        // returns true if in a loop
  void reserve_random(const int loc);   // Resolve location for a random robot
  void resolve_edge_conflict(const int loc1, const int loc2);

  // Waiting agents
  std::vector<int> wait_times;
  void handle_waiting_robots();
  void step_in_available_dir(const int robot);
  int get_step_loc(const int loc, const int dir);

  // Prioritize forward motion
  std::vector<Move> prev_moves;
};
