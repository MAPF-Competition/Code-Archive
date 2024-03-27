#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <unordered_set>
#include <set>
// #include "tsl/hopscotch_set.h"
#include <bitset>

enum class Algorithm {Long, Short, Lazy, Bobo, SAT, Test, Bully, Game, Comb, NoAlgorithm};
enum class InstanceMap {City, Random, Sortation, Warehouse, Game, Unknown};

struct Parameters {
    Algorithm algorithm = Algorithm::NoAlgorithm; // 0: Lazy; 1: SAT; ... 
    bool halt = false;
    std::map<std::string, double> values; // map for barriers, path_bound, time_bound, congestion, must_have_moved...
};

namespace std {
    template <> struct hash<std::pair<int,int>> {
        size_t operator()(std::pair<int,int> x) const {
            return hash<long long int>()((long long int)x.first + ((long long int) x.second << 32));
        }
    };
}

class Planner
{
public:
    Planner();

public:
    /// @brief Position of an agent, defined by
    /// -  a location: an integer encoding a pixel in the grid
    /// - a direction: (0:east, 1:south, 2:west, 3:north)
    /// - a timestep
    struct Position
    {
        int location, direction, time;
        Position(int _location=0, int _direction=0, int _time=0):
            location(_location), direction(_direction), time(_time) {}
        Position(const State & state, int _time):
            location(state.location), direction(state.orientation), time(_time) {}

        Action action(const Position &prev) const {
            if (location != prev.location)
                return Action::FW;
            if (direction == prev.direction) {
                return Action::W;
            }
            int incr = direction - prev.direction;
            if (incr == 1 || incr == -3) // a rotation is encoded as an integer: (0:east, 1:south, 2:west, 3:north)
                return Action::CR;
            else
                return Action::CCR;
        }

        bool operator==(const Position& other) const {
            return location == other.location && direction == other.direction && time == other.time;
        }
        
        void print() const {
            std::cout << "{location: " << location << ", direction: " << direction << ", time: " << time << "}" << std::endl;
        }
    };

    /**
     * @brief The AstarNode struct
     * Node for the A* search algorithm
     */
    struct AstarNode {
        int location;
        int direction;
        int time;
        float f; // sum of g and h
        float g; // cost of the current path
        int g2; // length of the current path
        int h; // estimate of the cost of the remaining path
        AstarNode* parent;
        bool closed = false;
        AstarNode(int _location,int _direction, int _time, float _g, int _h, AstarNode* _parent, int _g2 = -1)
            : location(_location)
            , direction(_direction)
            , time(_time), f(_g+_h)
            , g(_g)
            , g2(_g2)
            , h(_h)
            , parent(_parent) {
            if(g2 == -1) g2 = g;
        }
        void print(int ncols) const {
            const int x = location % ncols;
            const int y = location / ncols;
            cout << " (x:" << x << ", y:" << y << ", dir:" << direction << ", t:" << time << ")";
        }
    };

    void init(SharedEnvironment* _env, const Parameters & parameters);
    bool check_moves(const std::vector<Action> moves) const;
    std::string map() const;

protected:
    void get_actions(std::vector<Action> & actions) const;
    std::vector<Position> neighbors(int agent, const Position &position, std::list<int> & conflicts) const;
    void register_path(int agent);
    void unregister_path(int agent);
    int choose_conflict(const std::vector<int> & conflicts, const std::vector<int> & enqueued) const;
    void read_barriers();
    void read_directions();
    bool hit_barrier(int location, int direction) const;
    bool safe_move(int location, int direction) const;
    int unsafe_move(int location, int direction) const;
    std::vector<Planner::Position> straight_path(int loc1, int loc2) const;
    void write_paths_json(int bound = 20) const;
    void reset_passing();
    std::vector<int> agent_at(int location, int time) const;
    Position position_of(int agent, int time) const;

    // Distances
    int manhattan_distance(int loc1, int loc2) const;
    void compute_location_indices();
    inline bool closer_east(int location, int end) const { return m_direction_map.at(m_index_of_location.at(end)).at(m_index_of_location.at(location)) & 1; }
    inline bool closer_south(int location, int end) const { return m_direction_map.at(m_index_of_location.at(end)).at(m_index_of_location.at(location)) & 2; }
    int distance(int loc1, int loc2) const;
    int distance_diff(int loc1, int loc2, int end) const;
    int distance_sortation(int loc1, int loc2) const;
    int distance_warehouse(int loc1, int loc2) const;
    void read_distance_sites();
    void make_labels(const std::vector<int> & sites);
    void make_directions(bool write=false);
    void write_directions_map();
    std::vector<char> compute_directions(int location) const;
    std::vector<int> compute_distance(int location) const;
    std::vector<int> compute_distance_with_turns(int location) const;
    std::vector<int> location_neighbors(int loc) const;
    void make_distance_map_json() const;
    void write_distance_map_json(int location) const;
    bool test_directions(int location);

    // Other
    int encoding(const Position & pos) const;
    Planner::Position move(const Position & pos, const Action &action) const;
    int move_location(int location, int direction) const;
    void print_elapsed_time() const;
    bool halt() const;
    void print_path(int i) const;
    void check() const;
    inline bool obstacle(int location) const { return env->map.at(location) != 0; }
    inline int task(int agent) const { return env->goal_locations.at(agent).front().first; }
    void update_tasks_done(std::vector<Action> & actions);
    double time_diff(const std::chrono::high_resolution_clock::time_point & start) const;
    int direction(int loc1, int loc2) const;

protected:
    SharedEnvironment* env;
    InstanceMap m_map = InstanceMap::Unknown;
    int m_timestep; // current timestep. I prefer to ignore env->curr_timestep because it can change in the middle of a function 
    int m_tasks_done;
    std::chrono::high_resolution_clock::time_point m_start_time;
    bool m_halt = false; // True for stopping computing actions
    int m_barriers = 0; // 0: no barriers, 1: always, >1: selective
    int m_must_have_moved = 0;
    int m_num_of_agents;

    std::vector<std::vector<Position>> m_paths; // current computed paths for each agent
    std::vector<int> m_index_of_location; // m_index_of_location[loc] position of the pixels loc among the non-obstacle pixels
    std::vector<int> m_location_at_index; // the opposite
    std::vector<std::vector<char>> m_direction_map;  // m_direction_map[i][j] directions of move to go from m_location_at_index[j] to m_location_at_index[i]
    std::vector<std::list<int>> m_passing; // m_passing[loc] = agents passing by a location
    std::unordered_map<pair<int,int>,std::vector<int>> m_passing_hash; // m_passing_hash[location, time] = agents at this space-time point
    std::unordered_map<int,std::vector<int>> m_ending_hash; // m_ending_hash[loc] = agents finishing their current path at location loc
    std::vector<std::vector<bool>> m_barrier;
    std::vector<std::vector<int>> m_sites_distance; // pairwise distances between sites
    std::vector<int> m_sites_label; // m_sites_label[loc] = label of the closest site to loc
    std::set<int> m_idle_locations; // loc in m_idle_locations if I cannot reach it without passing by a bottleneck
    std::vector<int> m_recent; // Number of agents in a position in last 15 steps
    int ppwc_calls = 0;
};
