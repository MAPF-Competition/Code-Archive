#include <PlannerLazy.h>
#include "nlohmann/json.hpp"
#include <unordered_set>

#include <random>
#include <fstream>  // ofstream

PlannerLazy::PlannerLazy() : Planner() {}


void PlannerLazy::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    for (auto &[k,v] : parameters.values)
        cout << k << " = " << v << endl;
    m_path_bound = parameters.values.at("path_bound");
    m_congestion = parameters.values.at("congestion");
    if (parameters.values.count("safety_gap") > 0)
        m_safety_gap = (parameters.values.at("safety_gap") > 0);

    cout << "PlannerLazy(barriers: " << m_barriers
        << ", must_have_moved: " << m_must_have_moved
        << ", path_bound: " << m_path_bound
        << ", congestion: " << m_congestion
        << ", safety_gap: " << std::boolalpha << m_safety_gap << ") initialized" << endl;
}

/**
 * @brief PlannerLazy::plan
 * @param actions
 */
void PlannerLazy::plan(vector<Action> & actions, double timeLeft) {
    actions = std::vector<Action>(m_num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;
    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
    const bool has_moved = update_paths();
    
    // Process the queue
    double elapsed_time = 0;
    int count = 0;

    while (/*count < m_num_of_agents &&*/ has_moved && timeLeft - elapsed_time > .075) { // @debug Change this
        const int a = m_queue.front();
        m_queue.pop_front();
        m_queue.push_back(a);
        const Position cur_pos(env->curr_states[a], m_timestep);
        if (env->goal_locations.at(a).empty()) { // if no goal, do not move, that is, keep the same location and orientation. I do not think that this may happen
            cout << "Warning: agent a" << a << " has no task" << endl;
        } else {
            // int path_bound = m_path_bound;
            // int path_bound = std::min(scount / m_num_of_agents + 5, m_path_bound);
            int path_bound = m_path_bound;
            if(m_path_bound == -1)
                path_bound = std::max(scount / m_num_of_agents - env->curr_timestep, 6);
            else if(scount < m_num_of_agents)
                path_bound = 1 + path_bound/3; // First time they are calculated

            if(count == 0)
                cout << "path_bound = " << path_bound << std::endl;

            unregister_path(a);
            plan_path(a, cur_pos, task(a), path_bound, timeLeft - elapsed_time - .06);
            register_path(a);
        }
        // if (m_queue.size() != m_num_of_agents) {
        //     for (int x : m_queue)
        //         cout << x << " ";
        //     cout << endl;
        // }
        assert(m_queue.size() == m_num_of_agents);
        elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();
        count++;
        scount++;
        // write_paths_json();
    }

    cout << "Time before post-processing: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count() << endl;

    cout << "-- " << count << " agents planned" << endl;
    get_actions(actions);
    avoid_wait_collisions(actions);
    update_tasks_done(actions);
    print_elapsed_time();
    // write_paths_json();
    // if (!check_moves(actions))
    //     m_halt = true;
    m_timestep++;
    cout << "Time before return: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count() << endl;
    return;
}

void PlannerLazy::plan_path(int agent, const Position &start, int end, int path_bound, double timeLeft) {
//    cout << "Planning the path of a" << agent << " from " << start.location << " to " << end << endl;
    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();

    if (env->map.at(start.location) == 1) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
    if (env->map.at(end) == 1) cout << "Error: agent a" << agent << " is assigned to location " << end << " which is an obstacle" << endl;
    // assert(m_paths.at(agent).front() == start);
    if (!(m_paths.at(agent).front() == start)) {
        cout << "a" << agent << " t:" << m_timestep << endl;
        m_paths.at(agent).front().print();
        start.print();
    }

    std::vector<Position> path;
    std::priority_queue<std::tuple<float,float,int,AstarNode*>, vector<std::tuple<float,float,int,AstarNode*>>> open_list;
    std::unordered_map<long long int, AstarNode*> all_nodes; // nodes indexed by location
    std::unordered_set<long long int> close_list; // set of done nodes
    std::vector<AstarNode*> bestFound;
    AstarNode* s = new AstarNode(start.location, start.direction, start.time, 0, 0, nullptr); // start node
    open_list.push(std::make_tuple(-s->f, -s->g, 0, s));
    all_nodes[encoding(start)] = s;
    bool timeout = false;

    while (!open_list.empty()) {
        auto f_g_node = open_list.top(); open_list.pop(); // get the next node
        AstarNode* curr = std::get<3>(f_g_node);
        if(curr->closed)
            continue; // This is an outdated version that should be ignored
        curr->closed = true;
        const Position end_pos(curr->location, curr->direction, curr->time);
        close_list.emplace(encoding(end_pos)); // mark it as done
        double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();

        if(curr->g2 > bestFound.size()) {
            bestFound.push_back(curr);
            if(curr->g2 > path_bound) // We should stop here because of bound
                break;
        }
        if (curr->location == end) { // Goal achieved
            bestFound.push_back(curr); // Maybe not unnecessary
            break;
        }
        if(elapsed_time > timeLeft) { // It's taking too long
            timeout = true;
            break;
        }

        // otherwise, try every next step possible and add the node to the open list
        auto neighborhood = neighbors_v4(agent, Position(curr->location, curr->direction, curr->g2 + m_timestep));
        for (const Position &neighbor : neighborhood) {
            if (close_list.contains(encoding(neighbor))) // the node is already done
                continue;
            if (all_nodes.contains(encoding(neighbor))) { // the node is already in the list: update its value
                AstarNode* old = all_nodes[encoding(neighbor)];
                if (curr->g + 1.0 + congestion(neighbor.location) < old->g) {
                    old->g = curr->g + 1.0 + congestion(neighbor.location);
                    old->f = old->h + old->g;
                    old->parent = curr;
                    old->g2 = curr->g2 + 1;
                    open_list.push(std::make_tuple(-old->f,
                                                   -old->g,
                                                   2*(neighbor.location != curr->location) + (neighbor.direction != curr->direction),
                                                   old)); // Added duplicate
                }
            } else { // create the node
                AstarNode* next_node = new AstarNode(
                    neighbor.location,
                    neighbor.direction,
                    curr->time + 1,
                    curr->g + 1.0 + congestion(neighbor.location),
                    curr->h + distance_diff(curr->location, neighbor.location, end),
                    curr,
                    curr->g2 + 1
                );
                open_list.push(std::make_tuple(-next_node->f,
                                               -next_node->g,
                                               2*(neighbor.location != curr->location) + (neighbor.direction != curr->direction),
                                               next_node));
                all_nodes[encoding(neighbor)] = next_node;
            }
        }
    }

    // if we find the goal, trace back the path
    if(bestFound.size()) {
        AstarNode* curr = bestFound.back();
        while (curr->parent != NULL) {
            const Position pos(curr->location, curr->direction, curr->time);
            path.push_back(pos);
            curr = curr->parent;
        }
        // append the new path
        m_paths.at(agent).clear();
        m_paths.at(agent).push_back(start);
        for (auto it = path.crbegin(); it != path.crend(); ++it)
            m_paths.at(agent).push_back(*it);
    }
    else {
        // cout << "Warning: could not find a path for a" << agent << " at t:" << m_timestep << endl;
    }

    for (auto n : all_nodes) {
        delete n.second;
    }

    if(timeout)
        move_to_front(agent);
}




/// @brief Remove the first position of all the computed paths.
/// At timestep 0, we set the initial position instead
/// @note I know that sometimes PlannerLazy::plan() finishes and the moves are ignored, so I check that here
bool PlannerLazy::update_paths() {
    bool has_moved = true;
    if (m_timestep == 0) {
        // Add the initial position for all
        m_queue.resize(m_num_of_agents, 0);
        std::iota(m_queue.begin(), m_queue.end(), 0);
        for (int a = 0; a < m_num_of_agents; a++) {
            const Position first_pos(env->curr_states.at(a), m_timestep);
            m_paths.at(a).push_back(first_pos);
        }
    } else {
        // Verify that each robot has moved
        for (int a = 0; a < m_num_of_agents && has_moved; a++) {
            const Position start(env->curr_states[a], m_timestep);
            if (m_paths.at(a).size() > 1 && start != m_paths.at(a).at(1)) {
                has_moved = false;
            }
        }
        if (has_moved) {
            // Remove the initial position for all
            for (int a = 0; a < m_num_of_agents; a++) {
                if (m_paths.at(a).size() > 1) {
                    m_paths.at(a).erase(m_paths.at(a).begin());
                } else {
                    assert(m_paths.at(a).size() == 1);
                    m_paths.at(a).at(0).time = m_timestep;
                }
                const Position start(env->curr_states[a], m_timestep);
                if (!(m_paths.at(a).front() == start)) {
                    cout << "a" << a << " t:" << m_timestep << " path and curr_state:" << endl;
                    m_paths.at(a).front().print();
                    start.print();
                }
            }
        } else {
            // Push the timesteps later
            cout << "-- Warning: The planned actions were ignored" << endl;
            for (int a = 0; a < m_num_of_agents; a++) {
                for (auto it = m_paths.at(a).begin(); it != m_paths.at(a).end(); ++it) {
                    it->time++;
                }
            }
        }
    }
    // Reset m_passing
    m_passing_hash.clear();
    m_ending_hash.clear();

    m_recent.clear();
    m_recent.resize(env->cols*env->rows);

    for (int a = 0; a < m_num_of_agents; a++) {
        register_path(a);
        for(int t = 0; t < 15 && t < m_paths.at(a).size(); t++)
            m_recent.at(m_paths.at(a).at(t).location)++;
    }

    cout << "Traffic (out of 15): min="
            << *min_element(m_recent.begin(), m_recent.end())
            << " max="
            << *max_element(m_recent.begin(), m_recent.end())
            << endl;

    return has_moved;
}

float PlannerLazy::congestion(int location) {
    if(m_congestion == 0.0)
        return 0.0;
    return m_recent.at(location) / m_congestion;
}


void PlannerLazy::move_to_front(int a) {
    m_queue.erase(std::remove(m_queue.begin(), m_queue.end(), a), m_queue.end()); // @note Maybe unnecessary
    m_queue.push_front(a);
}


/**
 * @brief PlannerLazy::get_moves Get the moves given the current state of the paths
 * Also, if an agent reaches its task, we enqueue it.
 * @param plan
 */
void PlannerLazy::get_actions(std::vector<Action> & actions) {
    for (int a = 0; a < m_num_of_agents; a++) {
        if (m_paths.at(a).size() > 1) {
            const Position &first = m_paths.at(a).at(0);
            const Position &second = m_paths.at(a).at(1);
            actions.at(a) = second.action(first);
            if (second.location == task(a)) {
                move_to_front(a);
            }
        } else {
            actions.at(a) = Action::W;
        }
    }
}

// void PlannerLazy::sort_by_distance(std::queue<int> & queue) {
//     std::vector<int> temp;
//     while (!queue.empty()) {
//         temp.push_back(queue.front());
//         queue.pop();
//     }
//     // Compute approximate distances
//      std::vector<int> dist(m_num_of_agents);
//      for (int a = 0; a < m_num_of_agents; a++) {
//          const int source = env->curr_states[a].location;
//          const int target = task(a);
//          // read_distance_map(target);
//          dist.at(a) = manhattan_distance(source, target);
//      }
//     // Sort by distance
//     std::sort(temp.begin(), temp.end(), [&dist](int a, int b) {
//         return dist.at(a) < dist.at(b);
//     });
//     for (int x : temp) {
//         queue.push(x);
//     }
// }


/**
 * @brief Planner::neighbors Compute all the valid positions after a given position of an agent
 * @param agent Index of the current agent
 * @param position Its current position
 * @return A list of valid positions
 */
vector<Planner::Position> PlannerLazy::neighbors(int agent, const Position &position) const {
    std::vector<Position> neighbors;
    std::vector<Position> positions = {
        move(position, Action::FW),
        move(position, Action::CR),
        move(position, Action::CCR),
        move(position, Action::W)
    };

    const int t = position.time;
    for (const Position & next : positions) {
        if (next.location < 0 || obstacle(next.location))
            continue;
        bool valid = true;

        std::unordered_set<int> relevant_agents;
        // Agents passing by next.location at time t and t+1
        for(int tt = t; tt <= t+1; tt++)
            if(m_passing_hash.contains(std::make_pair(next.location,tt)))
                for(int i : m_passing_hash.at(std::make_pair(next.location,tt)))
                    if (i != agent)
                        relevant_agents.insert(i);
        // Agents ending at next.location soon enough
        if(m_ending_hash.contains(next.location))
            for(int i : m_ending_hash.at(next.location))
                if (i != agent && m_paths.at(i).back().time >= t - m_must_have_moved)
                    relevant_agents.insert(i);

        for (int i : relevant_agents) {
            if (m_paths.at(i).size() > t+1-m_timestep) {
                if (m_paths.at(i).at(t+1-m_timestep).location == next.location) {
                    // cout << "hits a" << i << endl;
                    valid = false;
                    break;
                } else if (m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location == position.location) {
                    // cout << "crosses a" << i << endl;
                    valid = false;
                    break;
                }
                // sticking
                else if (m_safety_gap && m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location - m_paths.at(i).at(t-m_timestep).location == next.location - position.location) {
                    // cout << "sticks to a" << i << endl;
                    valid = false;
                    break;
                }
            } else if (m_paths.at(i).size() > t-m_timestep) {
                const Position & other = m_paths.at(i).at(t-m_timestep);
                if (next.location == other.location && (next.direction - other.direction + 4) % 4 == 2) {
                    // cout << "invade facing a" << i << endl;
                    valid = false;
                    break;
                }
            }
            if (m_paths.at(i).size() <= t+1-m_timestep &&
                m_paths.at(i).size() + m_must_have_moved + m_timestep > t &&
                m_paths.at(i).back().location == next.location) {
                // cout << "hits a" << i << endl;
                valid = false;
                break;
            }
        }
        if (valid) {
            neighbors.push_back(next);
        }
    }

    return neighbors;
}

std::vector<Planner::Position> PlannerLazy::neighbors_v4(int agent, const Position &position) const {
    std::vector<Planner::Position> neighbors;

    const int t = position.time;

    int fw_loc = move(position, Action::FW).location;
    if (fw_loc >= 0 && !obstacle(fw_loc)) {   // Conflicts when movin forward
        bool valid = true;
        if(m_ending_hash.contains(fw_loc)) {
            for(int ca : m_ending_hash.at(fw_loc)) {
                assert(ca != agent);
                if (m_paths[ca].back().time <= t+1 && m_paths[ca].back().time + m_must_have_moved >= t + 1 && m_paths[ca].back().location == fw_loc) {
                    valid = false;
                    break;
                }
            }
        }

        if(m_passing_hash.contains(std::make_pair(fw_loc, t+1))) {
            if(m_passing_hash.at(std::make_pair(fw_loc, t+1)).size()) {
                assert(m_passing_hash.at(std::make_pair(fw_loc, t+1)).at(0) != agent);
                valid = false;
            }
        }

        // Edge conflicts
        if(valid && m_passing_hash.contains(std::make_pair(fw_loc, t))) {
            for(int ca : m_passing_hash.at(std::make_pair(fw_loc, t))) {
                assert(ca != agent);
                if(m_paths.at(ca).size() > t + 1 - m_timestep && m_paths.at(ca).at(t + 1 - m_timestep).location == position.location) {
                    assert(m_paths.at(ca).at(t - m_timestep).location == fw_loc);
                    valid = false;
                    break;
                }
            }
        }

        if(valid)
            neighbors.push_back(move(position, Action::FW));
    }

    { // Conflicts when staying in the same location
        bool valid = true;
        int stay_loc = position.location;
        if(m_ending_hash.contains(stay_loc)) {
            for(int ca : m_ending_hash.at(stay_loc)) {
                assert(ca != agent);
                if (m_paths[ca].back().time <= t+1 && m_paths[ca].back().time + m_must_have_moved >= t + 1 && m_paths[ca].back().location == stay_loc) {
                    valid = false;
                    break;
                }
            }
        }
        if(valid && m_passing_hash.contains(std::make_pair(stay_loc, t+1))) {
            if(m_passing_hash.at(std::make_pair(stay_loc, t+1)).size()) {
                    assert(m_passing_hash.at(std::make_pair(stay_loc, t+1)).at(0) != agent);
                    assert(m_paths.at(ca).size() > t+1-m_timestep && m_paths.at(ca).at(t+1-m_timestep).location == stay_loc);
                    valid = false;
            }
        }

        if(valid) {
            const std::vector<Planner::Position> stay_positions = {
                move(position, Action::CR),
                move(position, Action::CCR),
                move(position, Action::W)
            };
            for(const Position &pos: stay_positions)
                neighbors.push_back(pos);
        }
    }

    return neighbors;
}




/// @brief Alternative function for the neighbors. It should be easier to understand and more efficient
/// @param agent Index of the current agent
/// @param position Its current position
/// @return 
std::vector<Planner::Position> PlannerLazy::neighbors_v2(int agent, const Position &position) const {
    std::vector<Position> neighbors;
    std::vector<Position> positions = {
        move(position, Action::FW),
        move(position, Action::CR),
        move(position, Action::CCR),
        move(position, Action::W)
    };

    const int t = position.time;
    for (const Position & next : positions) {
        if (next.location < 0 || obstacle(next.location))
            continue;
        bool valid = true;

        std::unordered_set<int> relevant_agents;
        std::vector<int> agents;
        agents = agent_at(next.location, t);
        for (int a : agents) {
            if (a >= 0 && a != agent) relevant_agents.insert(a);
        }
        agents = agent_at(next.location, t+1);
        for (int a : agents) {
            if (a >= 0 && a != agent) relevant_agents.insert(a);
        }      
        for (int i : relevant_agents) {
            if (m_paths.at(i).size() > t+1-m_timestep) {
                if (m_paths.at(i).at(t+1-m_timestep).location == next.location) {
                    // cout << "hits a" << i << endl;
                    valid = false;
                    break;
                } else if (m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location == position.location) {
                    // cout << "crosses a" << i << endl;
                    valid = false;
                    break;
                }
                // sticking
                else if (m_safety_gap && m_paths.at(i).at(t-m_timestep).location == next.location && m_paths.at(i).at(t+1-m_timestep).location - m_paths.at(i).at(t-m_timestep).location == next.location - position.location) {
                    // cout << "sticks to a" << i << endl;
                    valid = false;
                    break;
                }
            } else if (m_paths.at(i).size() > t-m_timestep) {
                const Position & other = m_paths.at(i).at(t-m_timestep);
                if (next.location == other.location && (next.direction - other.direction + 4) % 4 == 2) {
                    // cout << "invade facing a" << i << endl;
                    valid = false;
                    break;
                }
            }
            if (m_paths.at(i).size() <= t+1-m_timestep &&
                m_paths.at(i).size() + m_must_have_moved + m_timestep > t &&
                m_paths.at(i).back().location == next.location) {
                // cout << "hits a" << i << endl;
                valid = false;
                break;
            }
        }
        if (valid) {
            neighbors.push_back(next);
        }
    }

    return neighbors;
}


/// @brief Alternative function for the neighbors. It should be easier to understand and more efficient
/// @param agent Index of the current agent
/// @param position Its current position
/// @return 
std::vector<Planner::Position> PlannerLazy::neighbors_v3(int agent, const Position &position) const {
    std::vector<Position> neighbors;
    // @note We do not check the barriers
    const Position next_fw = move(position, Action::FW);
    if (valid_fw(agent, position, next_fw)) {
        neighbors.push_back(next_fw);
    }
    const Position next_cr = move(position, Action::CR);
    if (valid_cr(agent, position, next_cr)) {
        neighbors.push_back(next_cr);
        neighbors.push_back(move(position, Action::CCR));
        neighbors.push_back(move(position, Action::W));
    }
    return neighbors;
}


bool PlannerLazy::valid_fw(int agent, const Position & cur_pos, const Position & next_pos) const {
    if (next_pos.location < 0 || obstacle(next_pos.location))
        return false;
    const int t = cur_pos.time;
    std::vector<int> agents;
    agents = agent_at(next_pos.location, t+1);
    for (int a : agents) {
        if (a != agent) {
            // cout << "hits a" << a << endl;
            return false;
        }
    }
    agents = agent_at(cur_pos.location, t+1);
    for (int a : agents) {
        if (a != agent && position_of(a, t).location == next_pos.location) {
            // cout << "crosses a" << a << endl;
            return false;
        }
    }
    agents = agent_at(next_pos.location, t);
    for (int a : agents) {
        if (a != agent && (next_pos.direction - position_of(a, t).direction + 4) % 4 == 2) {
            // cout << "invades facing a" << a << endl;
            return false;
        }
    }
    return true;
}


bool PlannerLazy::valid_cr(int agent, const Position & cur_pos, const Position & next_pos) const {
    const int t = cur_pos.time;
    const std::vector<int> agents = agent_at(cur_pos.location, t+1);
    for (int a : agents) {
        if (a != agent) {
            // cout << "hits a" << a << endl;
            return false;
        }
    }
    return true;
}


/**
 * @brief PlannerLazy::avoid_collisions
 * We check if there are agents that are going to collide and stop them.
 * This happens when an agent reaches its task and a previously planned agent passes by the same location.
 * See @brief PlannerLazy.
 * @param actions Moves to do in this timestep
 */

void PlannerLazy::avoid_wait_collisions(vector<Action> & actions) {
    std::unordered_set<int> waitingLocations;
    std::unordered_map<int,int> movingAgents; // key: nextLocation, value: agent
    for(int a = 0; a < m_num_of_agents; a++) {
        const Action &act = actions.at(a);
        if(act == Action::W) {
            int location = m_paths.at(a).at(0).location;
            waitingLocations.insert(location);
        }
        else if(act == Action::FW) {
            movingAgents[m_paths.at(a).at(1).location] = a;
        }
    }

    std::vector<int> invadingAgents; // Agents that we force to wait

    for(std::pair<int,int> pos_a : movingAgents) {
        int nextLocation = pos_a.first;
        int a = pos_a.second;
        while(a != -1 && waitingLocations.contains(nextLocation)) {
            // Agent a can no longer move, it will wait at previousLocation instead
            invadingAgents.push_back(a);
            movingAgents[nextLocation] = -1; // to represent erased without invalidating iterators
            int previousLocation = m_paths.at(a).at(0).location;
            waitingLocations.insert(previousLocation);
            // Check if another agent moves towards where agent a now waits
            if(movingAgents.contains(previousLocation)) {
                // Call this other agent a now and repeat
                a = movingAgents.at(previousLocation);
                nextLocation = previousLocation;
            }
            else
                break;
        }
    }

    for(int invading : invadingAgents) {
        cout << "-- collision unplanning a" << invading << endl;
        m_paths.at(invading).erase(m_paths.at(invading).begin() + 1, m_paths.at(invading).end());
        m_paths.at(invading).push_back(move(m_paths.at(invading).front(), Action::W));
        actions.at(invading) = Action::W;
        move_to_front(invading);
    }
}
