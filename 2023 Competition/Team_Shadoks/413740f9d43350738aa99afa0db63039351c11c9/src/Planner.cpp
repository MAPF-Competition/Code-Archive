#include "Planner.h"
#include "nlohmann/json.hpp"

#include <random>
#include <fstream>  // ofstream

// #undef NDEBUG // @todo Comment this
#include <cassert>


Planner::Planner() : env(nullptr) {}

void Planner::init(SharedEnvironment* _env, const Parameters & _parameters) {
    env = _env;
    m_num_of_agents = env->num_of_agents;
    cout << "\nPlanning " << m_num_of_agents << " agents for the map " << env->map_name << endl;
    m_timestep = 0;
    m_tasks_done = 0;
    m_paths.resize(m_num_of_agents);
    m_passing.resize(env->cols*env->rows);
    srand(0);
    m_halt = _parameters.halt;
    if (_parameters.values.count("barriers") > 0)
        m_barriers = static_cast<int>(_parameters.values.at("barriers"));
    if (_parameters.values.count("must_have_moved") > 0)
        m_must_have_moved = static_cast<int>(_parameters.values.at("must_have_moved"));
    if (env->map_name == "brc202d.map") m_map = InstanceMap::Game;
    else if (env->map_name == "warehouse_large.map") m_map = InstanceMap::Warehouse;
    else if (env->map_name == "warehouse_small.map") m_map = InstanceMap::Warehouse;
    else if (env->map_name == "sortation_large.map") m_map = InstanceMap::Sortation;
    else if (env->map_name == "random-32-32-20.map") m_map = InstanceMap::Random;
    else if (env->map_name == "Paris_1_256.map")     m_map = InstanceMap::City;
    else m_map = InstanceMap::Unknown;
    compute_location_indices();
    read_barriers();
    
    // Choose one among these two functions
    // read_directions();   // (1) Read a directions file from assets/
    make_directions(false);  // (2) Compute the directions and write them to a file (so that we can read it the next time)

    read_distance_sites();
    m_start_time = std::chrono::high_resolution_clock::now();

    // write_distance_map_json(62436); // @debug writing a JSON file with the distance map
}


/**
 * @brief Planner::compute_location_indices
 * Compute the vectors m_index_of_location and m_location_at_index;
 */
void Planner::compute_location_indices() {
    m_index_of_location.clear();
    m_index_of_location.resize(env->cols*env->rows, -1);
    m_location_at_index.clear();
    int n_pixels = 0;
    for (int i = 0; i < env->cols*env->rows; i++) {
        if (!obstacle(i)) {
            m_index_of_location.at(i) = n_pixels;
            m_location_at_index.push_back(i);
            n_pixels++;
        }
    }
    cout << "There are " << n_pixels << " obstacle-free pixels" << endl;
}


void Planner::read_barriers() {
    // Reset the distance map
    m_barrier.clear();
    m_barrier.resize(4);
    // Read the metadata
    // std::string fn = env->file_storage_path + "/barriers.json";
    std::string fn = "assets/barriers.json";
    std::ifstream f(fn);
    if (!f.is_open()) std::cerr << "Error opening file " << fn << endl;
    nlohmann::json data = nlohmann::json::parse(f);
    std::vector<int> idle = data[env->map_name]["idle"].get<std::vector<int>>();
    for (int loc : idle)
        m_idle_locations.insert(loc);

    if (m_barriers == 0) {
        cout << "Barriers are ignored" << endl;
        return;
    }

    std::vector<int> locations;
    locations = data[env->map_name]["E"].get<std::vector<int>>();
    m_barrier[0].resize(env->cols*env->rows, false);
    for (int loc : locations)
        m_barrier[0].at(loc) = true;
    locations = data[env->map_name]["S"].get<std::vector<int>>();
    m_barrier[1].resize(env->cols*env->rows, false);
    for (int loc : locations)
        m_barrier[1].at(loc) = true;
    locations = data[env->map_name]["W"].get<std::vector<int>>();
    m_barrier[2].resize(env->cols*env->rows, false);
    for (int loc : locations)
        m_barrier[2].at(loc) = true;
    locations = data[env->map_name]["N"].get<std::vector<int>>();
    m_barrier[3].resize(env->cols*env->rows, false);
    for (int loc : locations)
        m_barrier[3].at(loc) = true;
    cout << "Barriers read from " << fn << endl;
}

/// @brief A move from a location towards a direction is allowed
/// @param location 
/// @param direction 
/// @return True if the move is not allowed because it hits a barrier
bool Planner::hit_barrier(int location, int direction) const {
    if (m_barriers == 0)
        return false;

    if(m_recent.size() && m_recent.at(location) >= m_barriers - 1)
        return false;

    return m_barrier.at(direction).at(location);
}


/// @brief Compute all the valid positions after a given position of an agent
/// @param agent Index of the current agent
/// @param position Its current position
/// @param conflicts Conflicts encountered
/// @return A list of valid positions
std::vector<Planner::Position> Planner::neighbors(int agent, const Planner::Position &position, std::list<int> & conflicts) const {
    std::vector<Planner::Position> neighbors;
    std::vector<Planner::Position> positions = {
        move(position, Action::FW), // @note We do not care if it hits a barrier
        move(position, Action::CR),
        move(position, Action::CCR),
        move(position, Action::W)
    };

    const int t = position.time;
    for (const Planner::Position & next : positions) {
        if (next.location < 0 || obstacle(next.location))
            continue;
        bool valid = true;

        std::unordered_set<int> relevant_agents;

        
        for(int tt = t; tt <= t+1; tt++)
            if(m_passing_hash.count(std::make_pair(next.location,tt)))
                for(int i : m_passing_hash.at(std::make_pair(next.location,tt)))
                    if (i != agent)
                        relevant_agents.insert(i);

        if(m_ending_hash.count(next.location))
            for(int i : m_ending_hash.at(next.location))
                if (i != agent && m_paths.at(i).back().time >= t - m_must_have_moved)
                    relevant_agents.insert(i);

        for (int i : relevant_agents) {
            if (m_paths.at(i).size() > t+1-m_timestep) {
                if (m_paths.at(i).at(t+1-m_timestep).location == next.location) {
                    // cout << "hits a" << i << endl;
                    valid = false;
                    conflicts.push_back(i);
                    break;
                } else if (m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location == position.location) {
                    // cout << "crosses a" << i << endl;
                    valid = false;
                    conflicts.push_back(i);
                    break;
                }
            } else if (m_paths.at(i).size() > t-m_timestep) {
                const Position & other = m_paths.at(i).at(t-m_timestep);
                if (next.location == other.location && (next.direction - other.direction + 4) % 4 == 2) {
                    // cout << "invade facing a" << i << endl;
                    valid = false;
                    conflicts.push_back(i);
                    break;
                }
            }
            if (m_paths.at(i).size() <= t+1-m_timestep && m_paths.at(i).back().location == next.location) {
                // cout << "hits a" << i << endl;
                valid = false;
                conflicts.push_back(i);
                break;
            }
        }
        if (valid) {
            neighbors.push_back(next);
        }
    }

    return neighbors;
}


/// @brief Compute all the valid positions after a given position of an agent
/// @param agent Index of the current agent
/// @param position Its current position
/// @param conflicts Conflicts encountered
/// @return A list of valid positions
/// @note I do not understand how, but this is slower than the other function
// std::list<Planner::Position> Planner::neighbors(int agent, const Position &position, std::list<int> & conflicts) const {
//     std::list<Position> neighbors;
//     const int t = position.time;
//     Position next(-1, 0, 0);
//     if (!hit_barrier(position.location, position.direction))
//         next = move(position, Action::FW);
//     if (next.location >= 0 && !obstacle(next.location)) {
//         bool valid = true;
//         for (int i : m_passing.at(next.location)) { // for all robots passing by the new location
//             if (i != agent) {
//                 if (m_paths.at(i).size() > t+1-m_timestep) {
//                     // hits the planned path of another agent
//                     if (m_paths.at(i).at(t+1-m_timestep).location == next.location) {
//                         // cout << "hits a" << i << endl;
//                         conflicts.push_back(i);
//                         valid = false;
//                         break;
//                     } else if (m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location == position.location) {
//                         // cout << "crosses a" << i << endl;
//                         conflicts.push_back(i);
//                         valid = false;
//                         break;
//                     }
//                 } else if (m_paths.at(i).size() > t-m_timestep) {
//                     // We go to a neighbor that is facing us. Do not remove this test
//                     const Position & other = m_paths.at(i).at(t-m_timestep);
//                     if (next.location == other.location && (next.direction - other.direction + 4) % 4 == 2) {
//                         // cout << "invade facing a" << i << endl;
//                         conflicts.push_back(i);
//                         valid = false;
//                         break;
//                     }
//                 }
//             }
//         }
//         if (valid) {
//             neighbors.push_back(next);
//         }
//     }
//     // Other actions
//     next = move(position, Action::CR);
//     bool valid = true;
//     for (int i : m_passing.at(next.location)) { // for all robots passing by the new location
//         if (i != agent && m_paths.at(i).size() > t+1-m_timestep && m_paths.at(i).at(t+1-m_timestep).location == next.location) {
//             // cout << "hits a" << i << endl;
//             conflicts.push_back(i);
//             valid = false;
//             break;
//         }
//     }
//     if (valid) {
//         neighbors.push_back(move(position, Action::W));
//         neighbors.push_back(move(position, Action::CR));
//         neighbors.push_back(move(position, Action::CCR));
//     }
//     return neighbors;
// }


// Tell if I can move forward
bool Planner::safe_move(int location, int direction) const {
    const Position pos(location, direction);
    const Position next = move(pos, Action::W);
    if (next.location < 0)
        return false;
    if (obstacle(next.location))
        return false;
    return true;
}


/// @brief Get the adjacent location along a direction, without any validation
/// @param location Current location in the map
/// @param direction A direction to move: -1 (do not move), 0 (east), 1 (south)...
/// @return The next location. It returns -1 if it exists the map boundaries
int Planner::unsafe_move(int location, int direction) const {
    switch (direction) {
        case 0: return location + 1;
        case 1: return location + env->cols;
        case 2: return location - 1;
        case 3: return location - env->cols;
    }
    return location;
}

/**
 * @brief choose_conflict
 * @param conflicts List of conflicting agents
 * @param enqueued Counter
 * @return The agent with the least counter. If there is equality, pick one at **random**.
 * This is very important to avoid loops.
 */
int Planner::choose_conflict(const std::vector<int> & conflicts, const std::vector<int> & enqueued) const {
    int best_a = conflicts.at(0);
    int best_value = enqueued.at(best_a) + (0.1*std::rand() / RAND_MAX) - 0.05;
    for (int a : conflicts) {
        const double val = enqueued.at(a) + (0.1*std::rand() / RAND_MAX) - 0.05;
        if (val < best_value) {
            best_a = a;
            best_value = val;
        }
    }
    return best_a;
}

/**
 * @brief Planner::register_path Update the m_passing
 * @param agent The index of an agent
 */
void Planner::register_path(int agent) {
    for (const Position & pos : m_paths.at(agent)) {
        // m_passing.at(pos.location).push_back(agent);
        m_passing_hash[make_pair(pos.location, pos.time)].push_back(agent);
    }

    if(m_paths.at(agent).size())
        m_ending_hash[m_paths.at(agent).back().location].push_back(agent);
}

void Planner::unregister_path(int agent) {
    for (const Position & pos : m_paths.at(agent)) {
        auto &v = m_passing_hash[make_pair(pos.location, pos.time)];
        v.erase(std::remove(v.begin(), v.end(), agent), v.end());
    }

    if(m_paths.at(agent).size()) {
        auto &v = m_ending_hash[m_paths.at(agent).back().location];
        v.erase(std::remove(v.begin(), v.end(), agent), v.end());
    }
}


/**
 * @brief Planner::get_actions
 * Get actions from the current paths
 */
void Planner::get_actions(std::vector<Action> & actions) const {
    for (int a = 0; a < m_num_of_agents; a++) {
        if (m_paths.at(a).size() > 1) {
            const Position &first = m_paths.at(a).at(0);
            const Position &second = m_paths.at(a).at(1);
            actions.at(a) = second.action(first);
        } else {
            actions.at(a) = Action::W;
        }
    }
}



// std::vector<Planner::Position> Planner::straight_path(int loc1, int loc2) const {
//     std::vector<Position> path;
//     int cur_loc = loc1;
//     while (cur_loc != loc2) {
//         const int x = cur_loc % env->cols;
//         const int y = cur_loc / env->cols;
//         if (closer_east(cur_loc)) {
//             path.push_back(Position(cur_loc, 0));
//             cur_loc += 1;
//         } else if (closer_south(cur_loc)) {
//             path.push_back(Position(cur_loc, 1));
//             cur_loc += env->cols;
//         } else if (x-1 >= 0 && !closer_east(cur_loc-1)) {
//             path.push_back(Position(cur_loc, 2));
//             cur_loc -= 1;
//         }
//         else if (y-1 >= 0 && !closer_south(cur_loc-env->cols)) {
//             path.push_back(Position(cur_loc, 3));
//             cur_loc -= env->cols;
//         } else {
//             // We are in a different connected component
//             path.clear();
//             return path;
//             cout << "ERROR in straight_path(" << loc1 << "," << loc2 << ") at location " << cur_loc << endl;
//         }
//     }
//     return path;
// }

void Planner::write_paths_json(int bound) const {
    // write JSON file
    nlohmann::json data;
    data["teamSize"] = m_num_of_agents;
    data["start"] = nlohmann::json::array();
    const std::vector<std::string> directions = { "E", "S", "W", "N"};
    for (int i = 0; i < m_num_of_agents; i++) {
        const int x = env->curr_states[i].location % env->cols;
        const int y = env->curr_states[i].location / env->cols;
        nlohmann::json cur_start = nlohmann::json::array({y, x, directions.at(env->curr_states[i].orientation)});
        data["start"].push_back(cur_start);
    }
    data["makespan"] = bound;
    data["plannerPaths"] = nlohmann::json::array();
    for (int i = 0; i < m_num_of_agents; i++) {
        std::string moves = "";
        for (int t = 0; t < bound; t++) {
            if (t+1 < m_paths.at(i).size()) {
                Action move = m_paths.at(i).at(t+1).action(m_paths.at(i).at(t));
                if (move == Action::FW)         moves += "F,";
                else if (move == Action::W)     moves += "W,";
                else if (move == Action::CR)    moves += "R,";
                else if (move == Action::CCR)   moves += "C,";
            } else {
                moves += "W,";
            }
        }
        data["plannerPaths"].push_back(moves);
    }
    data["tasks"] = nlohmann::json::array();
    for (int i = 0; i < m_num_of_agents; i++) {
        const int location = env->goal_locations.at(i).front().first;
        const int x = location % env->cols;
        const int y = location / env->cols;
        nlohmann::json cur_task = nlohmann::json::array({i, y, x});
        data["tasks"].push_back(cur_task);
    }
    data["events"] = nlohmann::json::array();
    for (int i = 0; i < m_num_of_agents; i++) {
        nlohmann::json cur_event = nlohmann::json::array({i, 0, "assigned"});
        data["events"].push_back(nlohmann::json::array({cur_event}));
    }
    data["errors"] = nlohmann::json::array();

    std::string fn = "log/current-paths-t" + std::to_string(m_timestep) + ".json";
    std::ofstream file(fn);
    file << data.dump(2);
    file.close();
}


void Planner::reset_passing() {
    m_passing.clear();
    m_passing.resize(env->cols*env->rows);
    for (int a = 0; a < m_num_of_agents; a++) {
        register_path(a);
    }
}


int Planner::manhattan_distance(int loc1, int loc2) const {
    const int loc1_y = loc1 / env->cols;
    const int loc1_x = loc1 % env->cols;
    const int loc2_y = loc2 / env->cols;
    const int loc2_x = loc2 % env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


/// @brief Read all the directions map in memory
/// @note This is for testing, I am not sure that I can handle them. Note that we do not compute the distances here!
void Planner::read_directions() {
    if (m_barriers == 0 && (m_map == InstanceMap::Sortation || m_map == InstanceMap::Warehouse)) {
        return;
    }
    m_direction_map.resize(m_location_at_index.size());

    // Open file
    std::string dat_fn = "direction-" + map(); // names of the binary file
    if (m_barriers == 1) // Changed to use normal file for > 1
        dat_fn += "-barriers";
    dat_fn = env->file_storage_path + "/" + dat_fn + ".dat";
    // dat_fn = "assets/" + dat_fn + ".dat";
    std::ifstream file(dat_fn, std::ios::binary | std::ios::in);
    if (!file.is_open()) std::cout << "Error opening file " << dat_fn << endl;
    
    const std::streamsize n_bytes = ceil(1.0 * m_location_at_index.size() / 4); // each distance map is encoded with 2 bits per pixel
    for (std::size_t i = 0; i < m_direction_map.size(); i++) {
        const std::streampos pos = i * n_bytes;
        file.seekg(pos, std::ios::beg); // move to initial position
        std::vector<char> buffer(n_bytes);
        file.read(reinterpret_cast<char*>(buffer.data()), n_bytes);
        if (!file) cout << "Error reading " << dat_fn << endl;
        m_direction_map.at(i).assign(m_location_at_index.size(), 0);
        // Decode the binary file
        for (int loc_index = 0; loc_index < m_location_at_index.size(); loc_index++) {
            const int j = loc_index / 4;
            const int k = loc_index % 4;
            m_direction_map.at(i).at(loc_index) = (buffer[j] >> (2*k)) & 3;
        }
    }
    cout << "Directions for all " << m_direction_map.size() << " locations read" << endl;
}


/// @brief Approximate (pseudo-)distance between two locations
/// @param loc1 Start location
/// @param loc2 End location
/// @return Estimate of the distance using the sites
int Planner::distance(int loc1, int loc2) const {
    if (m_barriers != 1 && m_map == InstanceMap::Sortation) // Changed for >1
        return distance_sortation(loc1, loc2);
    if (m_barriers != 1 && m_map == InstanceMap::Warehouse) // Changed for >1
        return distance_warehouse(loc1, loc2);
    const int label1 = m_sites_label.at(loc1);
    const int label2 = m_sites_label.at(loc2);
    return m_sites_distance.at(label1).at(label2);
}


/// @brief The difference dist(loc2, end) - dist(loc1, end).
/// @param loc1 First location
/// @param loc2 Second location
/// @param end The target location
/// @return The difference dist(loc2, end) - dist(loc1, end).
/// @note gf there are barriers, then it is not true that the difference
/// between two adjacent pixels is {+1,-1} and hence we need to precompute the
/// whole distance map (m_distance_map)
int Planner::distance_diff(int loc1, int loc2, int end) const {
    if (loc1 == loc2)
        return 0;

    if (m_barriers != 1) {
        if (m_map == InstanceMap::Sortation)
            return distance_sortation(loc2, end) - distance_sortation(loc1, end);
        if (m_map == InstanceMap::Warehouse)
            return distance_warehouse(loc2, end) - distance_warehouse(loc1, end);
    }
   if (loc1 < loc2) {
       if (loc1 + 1 == loc2) { // Move East
           return closer_east(loc1, end) ? -1 : 1;
       } else { // Move South
           return closer_south(loc1, end) ? -1 : 1;
       }
   } else {
       if (loc2 + 1 == loc1) { // Move West
           return closer_east(loc2, end) ? 1 : -1;
       } else { // Move North
           return closer_south(loc2, end) ? 1 : -1;
       }
   }
   return 0;
}

int Planner::distance_sortation(int loc1, int loc2) const {
    if (obstacle(loc1) || obstacle(loc2)) {
        return -1;
    }
    int dist = manhattan_distance(loc1, loc2);
    if (loc1 == loc2)
        return dist;

    const int x1 = loc1 % env->cols;
    const int y1 = loc1 / env->cols;
    const int x2 = loc2 % env->cols;
    const int y2 = loc2 / env->cols;
    const int xmin = min(x1, x2);
    const int xmax = max(x1, x2);
    const int ymin = min(y1, y2);
    const int ymax = max(y1, y2);

    if (6 <= xmin && xmax <= 492 && ymax >= 6 && ymin <= 132 && x1 == x2 && x1%2 == 0)
        return dist + 2;
    if (6 <= ymin && ymax <= 132 && xmax >= 6 && xmin <= 492 && y1 == y2 && y1%2 == 0)
        return dist + 2;
    return dist;
}

int Planner::distance_warehouse(int loc1, int loc2) const {
    if (obstacle(loc1) || obstacle(loc2)) {
        return -1;
    }
    int dist = manhattan_distance(loc1, loc2);
    if (loc1 == loc2)
        return dist;

    const int x1 = loc1 % env->cols;
    const int y1 = loc1 / env->cols;
    const int x2 = loc2 % env->cols;
    const int y2 = loc2 / env->cols;
    const int xmin = min(x1, x2);
    const int xmax = max(x1, x2);
    const int ymin = min(y1, y2);
    const int ymax = max(y1, y2);
    // middle blocks, vertical axis
    if (8 <= xmin && xmax <= 490 && ymax >= 8 && ymin <= 129 && y1 != y2) {
        if (max(x1%4, x2%4) <= 2 && x1/4 == x2/4 && !(x1%4 == 1 && x2%4 == 1))
            return dist + 2;
        if (x1 == x2 && x1%4 == 1)
            return dist + 4;
    }
    // middle blocks, horizontal axis
    if (8 <= ymin && ymax <= 129 && xmax >= 8 && xmin <= 490 && y1%3 != 1 && y2%3 != 1 && (y1+1)/3 == (y2+1)/3 && x1 != x2)
        return dist + 2;
    if (abs(x1 - x2) >= 4) {
        if (ymax == 2) return dist + 2;
        if (ymax == 1) return dist + 4;
        if (ymax == 0) return dist + 6;
        if (ymin == 137) return dist + 2;
        if (ymin == 138) return dist + 4;
        if (ymin == 139) return dist + 6;
    }
    if (abs(y1 - y2) >= 4 && ymin <= 127) {
        if (xmax == 2) return dist + 2;
        if (xmax == 1) return dist + 4;
        if (xmax == 0) return dist + 6;
    }
    if (abs(y1 - y2) >= 4 && ymax >= 12) {
        if (xmin == 497) return dist + 2;
        if (xmin == 498) return dist + 4;
        if (xmin == 499) return dist + 6;
    }
    return dist;
}


/// @brief Read the sites, label their neighborhoods and compute the (pseudo-)distances
void Planner::read_distance_sites() {
    // Changed for > 1
    if (m_barriers != 1 && (m_map == InstanceMap::Sortation || m_map == InstanceMap::Warehouse)) {
        return;
    }

    // Reset the distance map
    m_sites_distance.clear();
    m_sites_label.clear();
    m_sites_label.resize(env->cols*env->rows, -1);

    // Read the sites
    std::string json_fn = "assets/sites.json";
    std::ifstream f(json_fn);
    if (!f.is_open()) std::cerr << "Error opening file " << json_fn << endl;
    nlohmann::json data = nlohmann::json::parse(f);
    const std::vector<int> sites = data[env->map_name]["sites"].get<std::vector<int>>();
    cout << "Reading " << sites.size() << " sites in the map" << endl;
    make_labels(sites);

    // @debug Remove this when it works
    // for (int i = 0; i < env->cols*env->rows; i++) {
    //     if (!obstacle(i)) {
    //         assert(test_directions(i));
    //     }
    // }

    // Make the (pseudo) distances. Note that they are not symmetric because of the barriers
    assert(m_sites_distance.empty());
    m_sites_distance.clear();
    m_sites_distance.resize(sites.size());
    for (std::size_t i = 0; i < sites.size(); i++) {
        m_sites_distance.at(i).assign(sites.size(), -1);
    }
    for (int i = 0; i < sites.size(); i++) {
        // cout << "Reading " << sites.size() << " sites in the map: " << i << "\r" << std::flush;
        const std::vector<int> cur_dist_map = compute_distance_with_turns(sites.at(i));
        for (int j = 0; j < sites.size(); j++) {
            m_sites_distance.at(j).at(i) = cur_dist_map.at(sites.at(j));
        }
    }
    cout << "Distance approximation computed" << endl;
}


/// @brief Build the vector m_label
/// @details Given a list of sites, we make a labeling m_label such that m_label[i] = closest site to location i
void Planner::make_labels(const std::vector<int> & sites) {
    std::queue<int> queue;
    std::vector<int> prev(env->cols*env->rows, -1);
    std::vector<int> dist(env->cols*env->rows, -1);
    for (std::size_t i = 0; i < sites.size(); i++) {
        assert(!obstacle(sites.at(i)));
        queue.push(sites.at(i));
        m_sites_label.at(sites.at(i)) = i;
        dist.at(sites.at(i)) = 0;
    }
    while (!queue.empty()) {
        const int cur_loc = queue.front(); queue.pop();
        const std::vector<int> neighbors = location_neighbors(cur_loc);
        for (int next_loc : neighbors) {
            int next_dist = 0; // distance for next_loc given the current path
            if (dist.at(cur_loc) == 0) {
                // The neighbors of the location are at distance 1 (if they are facing the location)
                next_dist = 1;
            } else {
                assert(prev.at(cur_loc) >= 0);
                if (next_loc - cur_loc == cur_loc - prev.at(cur_loc)) {
                    next_dist = dist.at(cur_loc) + 1;
                } else {
                    next_dist = dist.at(cur_loc) + 2;
                }
            }
            if (dist.at(next_loc) < 0 || next_dist < dist.at(next_loc)) {
                dist.at(next_loc) = next_dist;
                prev.at(next_loc) = cur_loc;
                m_sites_label.at(next_loc) = m_sites_label.at(cur_loc);
                queue.push(next_loc);
            }
        }
    }
}



// Write the binary files with the directions for each pixel to reach a location

/// @brief Write the arrays of directions (m_directions_map) to a binary file so that
/// we can read it instead of computing the directions
/// @note We assume that the direction have been computed
void Planner::write_directions_map() {
    std::string filename = "direction-" + map(); // names of the binary file
    if (m_barriers > 0)
        filename += "-barriers";
    filename += ".dat";
    std::ofstream file;
    file.open(filename, std::ios::binary | std::ios::out);
    cout << "Writing directions for " << env->map_name << endl;
    const int n_pixels = m_location_at_index.size();
    for (const std::vector<char> & cur_dir : m_direction_map) {
        // Put 4 codes into a char
        const int row_size = ceil(1.0 * n_pixels / 4);
        char row[row_size];
        for (int j = 0; j < row_size; j++) {
            char byte = 0;
            for (int k = 0; k < 4; k++) {
                if (4*j+k < n_pixels) {
                    byte |= (cur_dir.at(4*j+k)) << 2*k;
                }
            }
            row[j] = byte;
        }
        file.write(reinterpret_cast<char*>(row), sizeof(row));
    }
    file.close();
    cout << "Directions map for " << env->map_name << " written in " << filename << endl;
}


/// @brief Compute the directions for each location of the map
void Planner::make_directions(bool write) {
    m_direction_map.resize(m_location_at_index.size());
    const int n_pixels = m_location_at_index.size();
    for (int i = 0; i < n_pixels; i++) {
        m_direction_map.at(i) = compute_directions(m_location_at_index.at(i));
        if (i % 1000 == 0)
            cout << "Directions from location with index " << i << "/" << n_pixels << " done" << endl;
    }
    if (write) {
        write_directions_map();
    }
}

/// @brief Compute the compressed list of directions.
/// @param location A map location
/// @return A vector directions such that directions[i] = directions of next pixel in the shortest path to location from m_location_at_index[i]
std::vector<char> Planner::compute_directions(int location) const {
    std::vector<int> dist = compute_distance(location);
    std::vector<char> directions(m_location_at_index.size(), 0);
    for (std::size_t i = 0; i < m_location_at_index.size(); i++) {
        const int cur_loc = m_location_at_index.at(i);
        const int x = cur_loc % env->cols;
        const int y = cur_loc / env->cols;
        char code = 0;
        int next = cur_loc + 1;
        if (x+1 < env->cols && dist.at(next) >= 0 && dist.at(next) < dist.at(cur_loc))
            code += 1;
        next = cur_loc + env->cols;
        if (y+1 < env->rows && dist.at(next) >= 0 && dist.at(next) < dist.at(cur_loc))
            code += 2;
        directions.at(i) = code;
    }
    return directions;
}



/// @brief Compute distances to a location
/// @param location Target location
/// @return A vector with the distance for each location. This distances takes rotations
/// into account. For an exact evaluation of a distance from a position to a locations, 
/// you must add +1 or +2 so that the position faces the next location.
std::vector<int> Planner::compute_distance(int location) const {
    std::vector<int> dist(env->rows*env->cols, -1);
    if (obstacle(location))
        return dist;
    std::queue<int> queue;
    dist.at(location) = 0;
    queue.push(location);
    while (!queue.empty()) {
        int loc = queue.front(); queue.pop();
        std::vector<int> neighbors = location_neighbors(loc);
        for (int neighbor : neighbors) {
            if (dist.at(neighbor) < 0 || dist.at(loc) + 1 < dist.at(neighbor)) {
                dist.at(neighbor) = dist.at(loc) + 1;
                queue.push(neighbor);
            }
        }
    }
    return dist;
}



/// @brief Compute distances to a location taking turns into account
/// @param location Target location
/// @return A vector with the distance for each location. This distances takes rotations
/// into account. For an exact evaluation of a distance from a position to a locations, 
/// you must add +1 or +2 so that the position faces the next location.
std::vector<int> Planner::compute_distance_with_turns(int location) const {
    std::vector<int> dist(env->rows*env->cols, -1);
    if (obstacle(location))
        return dist;
    std::queue<int> queue;
    std::vector<int> prev(env->cols*env->rows, -1); // prev[l] previous position of the shortest path to location l
    dist.at(location) = 0;
    queue.push(location);
    while (!queue.empty()) {
        const int cur_loc = queue.front(); queue.pop();
        const std::vector<int> neighbors = location_neighbors(cur_loc);
        for (int next_loc : neighbors) {
            int next_dist = 0; // distance for next_loc given the current path
            if (dist.at(cur_loc) == 0) {
                // The neighbors of the location are at distance 1 (if they are facing the location)
                next_dist = 1;
            } else {
                assert(prev.at(cur_loc) >= 0);
                if (next_loc - cur_loc == cur_loc - prev.at(cur_loc)) {
                    next_dist = dist.at(cur_loc) + 1;
                } else {
                    next_dist = dist.at(cur_loc) + 2;
                }
            }
            if (dist.at(next_loc) < 0 || next_dist < dist.at(next_loc)) {
                dist.at(next_loc) = next_dist;
                prev.at(next_loc) = cur_loc;
                queue.push(next_loc);
            }
        }
    }
    // Check the existence of a path
    // for (std::size_t i = 0; i < dist.size(); i++) {
    //     if (!obstacle(i) && dist.at(i) < 0) {
    //         cout << "No path from " << i << " to " << location << endl;
    //     }
    // }
    return dist;
}

/**
 * @brief Planner::location_neighbors
 * Obtain the neighbors of a given location
 * @param location The start location
 */
std::vector<int> Planner::location_neighbors(int loc) const {
    const int x = loc % env->cols;
    const int y = loc / env->cols;
    std::vector<int> neighbors;
    if (x+1 < env->cols && !obstacle(loc+1) && !hit_barrier(loc+1, 2))
        neighbors.push_back(loc+1);
    if (y+1 < env->rows && !obstacle(loc+env->cols) && !hit_barrier(loc+env->cols, 3))
        neighbors.push_back(loc+env->cols);
    if (x-1 >= 0 && !obstacle(loc-1) && !hit_barrier(loc-1, 0))
        neighbors.push_back(loc-1);
    if (y-1 >= 0 && !obstacle(loc-env->cols) && !hit_barrier(loc-env->cols, 1))
        neighbors.push_back(loc-env->cols);
    return neighbors;
}


/// @brief Write the distance mas to a JSON file
/// @note This only works for the Random instances, otherwise the file is way too large
void Planner::make_distance_map_json() const {
    // Compute distances
    cout << "Computing directions for " << env->map_name << endl;
    std::vector<std::vector<int>> distances;
    int max_dist = 0;
    for (int i = 0; i < env->cols*env->rows; i++) {
        const std::vector<int> cur_dist = compute_distance(i);
        // const std::vector<int> cur_dist = compute_distance_with_turns(i);
        max_dist = max(max_dist, *max_element(std::begin(cur_dist), std::end(cur_dist)));
        distances.push_back(cur_dist);
    }
    // write JSON file
    nlohmann::json data;
    data["distances"] = distances;
    std::ofstream file("distances.json");
    file << data << endl;
    file.close();
    cout << "Distance map written to distances.json. The largest distance is " << max_dist << endl;
}


/// @brief Write the distance map to a location
/// so that we can open it with distance-viewer
void Planner::write_distance_map_json(int location) const {
    std::vector<std::vector<int>> distances(1);
    distances.at(0) = compute_distance(location);
    // distances.at(0) = compute_distance_with_turns(location);
    // write JSON file
    nlohmann::json data;
    data["distances"] = distances;
    std::ofstream file("cur_distance_map.json");
    file << data << endl;
    file.close();
    cout << "cur_distance_map.json written" << endl;
}


/// @brief Check if the directions read are the directions that I wrote
/// @param location A location in the map
/// @return True if everything is fine
/// @note I did this because I had some strange errors
// bool Planner::test_directions(int location) {
//     read_direction_map(location);
//     std::vector<char> directions = compute_directions(location);
//     // cout << m_direction_map.size() << " " << directions.size();
//     assert(directions.size() == m_location_at_index.size());
//     assert(m_direction_map.size() == m_index_of_location.size());
//     for (int i = 0; i < env->cols*env->rows; i++) {
//         if (obstacle(i)) {
//             if (m_direction_map.at(i) != 0) {
//                 cout << "An obstacle has a direction" << endl;
//                 return false;
//             }
//         } else if (m_direction_map.at(i) != directions.at(m_index_of_location.at(i))) {
//             cout << "Error in Planner::test_directions(" << location << "): m_direction_map[" << i << "] = " 
//             << static_cast<int>(m_direction_map.at(i)) << " != " << static_cast<int>(directions.at(m_index_of_location.at(i))) << endl;
//             return false;
//         }
//     }
//     return true;
// }


/**
 * @brief Planner::encoding
 * @param pos A position
 * @return An integer encoding a position
 */
int Planner::encoding(const Position & pos) const {
    const int npixels = env->cols*env->rows;
    const int t = pos.time - m_timestep;
    return 4 * (npixels * t + pos.location) + pos.direction;
}

Planner::Position Planner::move(const Position & pos, const Action &action) const {
    Position next(pos.location, pos.direction, pos.time + 1);
    if (action == Action::FW) {
        const int x = pos.location % env->cols;
        const int y = pos.location / env->cols;
        if (pos.direction == 0) {
            if (x+1 < env->cols) next.location++;
            else next.location = -1;
        } else if (pos.direction == 1) {
            if (y+1 < env->rows) next.location += env->cols;
            else next.location = -1;
        } else if (pos.direction == 2) {
            if (x-1 >= 0) next.location--;
            else next.location = -1;
        } else if (pos.direction == 3) {
            if (y-1 >= 0) next.location -= env->cols;
            else next.location = -1;
        } else {
            next.location = -1;
        }
    } else if (action == Action::CR) {
        next.direction = (next.direction+1) % 4;
    } else if (action == Action::CCR) {
        next.direction = (next.direction+3) % 4;
    }
    return next;
}

/// @brief Print the current timestep, number of tasks and elapsed time
void Planner::print_elapsed_time() const {
    std::cout << "<t:" << m_timestep <<  "(" << env->curr_timestep << "); tasks:" << m_tasks_done << "; " << time_diff(m_start_time) << "s>" << endl;
}


bool Planner::halt() const {
    if (m_halt)
        return true;
    // if (!(m_map == InstanceMap::Game || m_map == InstanceMap::Random))
    //     return true;

    return false;
}

void Planner::print_path(int i) const {
    cout << "Path for a" << i << ":";
    for (const Position pos : m_paths.at(i)) {
        const int x = pos.location % env->cols;
        const int y = pos.location / env->cols;
        cout << " (x:" << x << ", y:" << y << ", dir:" << pos.direction << ", t:" << pos.time << ")";
    }
    cout << endl;
}

void Planner::check() const {
    // assert(false); // To check that the asserts are working
    for (int a = 0; a < m_num_of_agents; a++) {
        assert(m_paths.at(a).front().location == env->curr_states[a].location);
        assert(m_paths.at(a).front().direction == env->curr_states[a].orientation);
        assert(m_paths.at(a).front().time == m_timestep);
        assert(!obstacle(env->curr_states[a].location));
    }
}

/// @brief Check if the planned moves are valid. I do this skip the next computations so that the program
/// finishes as soon as possible
/// @param moves The moves of the agents
/// @return True if the moves are valid
bool Planner::check_moves(const std::vector<Action> moves) const {
    std::vector<int> next_location(m_num_of_agents, -1); // location of each robot after the action
    std::vector<int> agent_at_loc_before(env->cols*env->rows, -1);
    std::vector<int> agent_at_loc_after (env->cols*env->rows, -1);
    bool valid = true;
    // Compute the next location
    for (int a = 0; a < m_num_of_agents; a++) {
        const Position cur_pos(env->curr_states.at(a), m_timestep);
        const Position next_pos = move(cur_pos, moves.at(a));
        if (next_pos.location < 0) {
            cout << "Error (Planner::check_moves): a" << a << " exits the map" << endl;
            // return false;
            valid = false;
        }
        else if (obstacle(next_pos.location)) {
            cout << "Error (Planner::check_moves): a" << a << " goes onto the obstacle at location " << next_pos.location << endl;
            // return false;
            valid = false;
        }
        next_location.at(a) = next_pos.location;
        agent_at_loc_before.at(cur_pos.location) = a;
        if (agent_at_loc_after.at(next_pos.location) >= 0) {
            cout << "Error (Planner::check_moves): vertex collision between a" << a << " and a" << agent_at_loc_after.at(next_pos.location) << endl;
            // return false;
            valid = false;
        }
        agent_at_loc_after.at(next_pos.location) = a;
    }
    for (int a = 0; a < m_num_of_agents; a++) {
        const int a2 = agent_at_loc_before.at(next_location.at(a)); // a2 = agent at the location where a is going
        if (a2 >= 0 && a2 != a && env->curr_states.at(a).location == next_location.at(a2)) {
            cout << "Error (Planner::check_moves): edge collision between a" << a << " and a" << a2 << endl;
            // return false;
            valid = false;
        }
    }
    return valid;
}


void Planner::update_tasks_done(std::vector<Action> & actions) {
    for (int a = 0; a < m_num_of_agents; a++) {
        const Position cur_pos(env->curr_states.at(a), 0);
        const Position next = move(cur_pos, actions.at(a));
        if (next.location == task(a)) {
            m_tasks_done++;
        }
    }
}

std::string Planner::map() const {
    if (env->map_name == "brc202d.map")
        return "game";
    if (env->map_name == "warehouse_large.map")
        return "warehouse";
    if (env->map_name == "sortation_large.map")
        return "sortation";
    if (env->map_name == "random-32-32-20.map")
        return "random";
    if (env->map_name == "Paris_1_256.map")
        return "city";
    return "";
}


double Planner::time_diff(const std::chrono::high_resolution_clock::time_point & start) const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start).count();
}

/// @brief Get the direction of move between two locations
/// @param loc1 
/// @param loc2 
/// @return 
int Planner::direction(int loc1, int loc2) const {
    if (loc2 == loc1) return -1;
    if (loc2 == loc1 + 1) return 0;
    if (loc2 == loc1 + env->cols) return 1;
    if (loc2 == loc1 - 1) return 2;
    if (loc2 == loc1 - env->cols) return 3;
    assert(false);
    return -1;
}


/// @brief Agent(s) at a given location and time
/// @param location 
/// @param time 
/// @return The index of the agent(s) at the location and time, or -1 if there is none.
/// @details We look if there is a planned agent passing by that location and time, or if there is 
/// an agent that finishes its path at that location in the interval [time, time + m_must_have_moved]. Note that we can have both.
/// For instance, we plan an agent a1 and then a2 (which finishes its task). The plan of a2 can end at a location, who is occupied by a1 a second later. Hence, 
/// this location has *two* agents.
/// @note We can find the same agent several times in the data structures m_passing_hash and m_ending_hash
std::vector<int> Planner::agent_at(int location, int time) const {
    std::vector<int> agents;
    // agent passing
    if (m_passing_hash.contains(std::make_pair(location, time))) {
        for (int a : m_passing_hash.at(std::make_pair(location, time))) {
            if (m_paths.at(a).size() > time - m_timestep && m_paths.at(a).at(time - m_timestep).location == location) {
                agents.push_back(a);
            }
        }
    }
    // agents that recently finished a task
    if (m_ending_hash.contains(location)) {
        for (int a : m_ending_hash.at(location)) {
            if (m_paths.at(a).back().location == location) {
                const int end_time = m_paths.at(a).back().time;
                if (end_time <= time && time <= end_time + m_must_have_moved + 1) { // we add 1 to m_must_have_moved: otherwise agents always block
                    agents.push_back(a);
                }
            }
        }
    }
    return agents;
}


/// @brief Position of an agent at a given time
/// @param agent 
/// @param time 
/// @note The agent remains at its position for a few steps at the end of its path
/// @return 
Planner::Position Planner::position_of(int agent, int time) const {
    const int end_time = m_paths.at(agent).back().time;
    if (time <= end_time) {
        return m_paths.at(agent).at(time - m_timestep);
    }
    assert(end_time <= time && time <= end_time + m_must_have_moved + 1);
    return m_paths.at(agent).back();
}
