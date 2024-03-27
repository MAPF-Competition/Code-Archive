#include "PlannerLong.h"
#include "nlohmann/json.hpp"

#include <random>
#include <fstream>  // ofstream

PlannerLong::PlannerLong() : Planner() {}


void PlannerLong::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    if (parameters.values.count("alpha") > 0)
        m_alpha = parameters.values.at("alpha");
    if (parameters.values.count("beta") > 0)
        m_beta = parameters.values.at("beta");
    if (parameters.values.count("enqueue_bound") > 0)
        m_enqueue_bound = parameters.values.at("enqueue_bound");
    if (parameters.values.count("nb_steps") > 0)
        m_nb_steps = parameters.values.at("nb_steps");
    m_enqueued.resize(m_num_of_agents, 0);
    m_dummy.resize(m_num_of_agents, false);
    cout << "PlannerLong("
        << "alpha: " << m_alpha
        << ", beta: " << m_beta
        << ", enqueue_bound: " << m_enqueue_bound 
        << ", nb_steps: " << m_nb_steps << ") initialized" << endl;
}


/**
 * @brief PlannerLong::plan We plan a short path for each robot using A* search and
 * take the first step for this timestep
 * @param time_limit
 * @param actions
 */


/// @brief Plan a path for each unplanned agent until its task
/// @param time_limit Allowed time
/// @param actions Array of actions to send
void PlannerLong::plan(vector<Action> & actions, double time_limit) {
    m_start_time = std::chrono::high_resolution_clock::now();
    m_time_limit = time_limit;
    actions = std::vector<Action>(m_num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;
    const bool has_moved = update_paths();
    if (!has_moved) {
        get_actions(actions);
        return;
    }
    update_dummy();

    // Put the unplanned agents into the queue
    std::queue<int> queue;
    fill_queue(queue);
    sort_by_distance(queue);

    // Copy current state of the data
    const std::queue<int> backup_queue(queue);
    const std::vector<int> backup_enqueued(m_enqueued);
    const std::vector<std::vector<Position>> backup_paths(m_paths);
    // Best available data
    std::vector<int> best_enqueued(m_enqueued);
    std::vector<std::vector<Position>> best_paths(m_paths);
    double best_val = 1000 * 500; // 200 * 500 should be enough

    m_round = 0;
    while (!backup_queue.empty() && m_round < 10) {
        // Reset the data
        queue = backup_queue;
        m_enqueued = backup_enqueued;
        m_forbidden_position.clear();
        std::copy(backup_paths.begin(), backup_paths.end(), m_paths.begin()); // m_paths <- backup_paths
        m_passing_hash.clear();
        m_ending_hash.clear();
        for (int a = 0; a < m_num_of_agents; a++)
            register_path(a);
        if (try_planning(queue)) {
            const double value = evaluate_paths(true);
            if (value < best_val) {
                best_val = value;
                best_enqueued = m_enqueued;
                std::copy(m_paths.begin(), m_paths.end(), best_paths.begin()); // best_paths <- m_paths
            }
        } else {
            break;
        }
        m_round++;
    }
    m_enqueued = best_enqueued;
    std::copy(best_paths.begin(), best_paths.end(), m_paths.begin()); // m_paths <- best_paths
    get_actions(actions);
    update_tasks_done(actions);
    print_elapsed_time();
    m_timestep++;
    assert(check_moves(actions));
    return;
}



/// @brief Update the planned paths.
/// @details At timestep 0, we set the initial position instead. For the next timesteps,
/// we remove the first position if it was used
bool PlannerLong::update_paths() {
    bool has_moved = true;
    if (m_timestep == 0) {
        // Add the initial position for all
        for (int a = 0; a < m_num_of_agents; a++) {
            const Position first_pos(env->curr_states.at(a), 0);
            m_paths.at(a).push_back(first_pos);
        }
    } else {
        // Verify that each robot has moved
        for (int a = 0; a < m_num_of_agents && has_moved; a++) {
            const Position start(env->curr_states[a], m_timestep);
            if (m_paths.at(a).size() > 1 && !(start == m_paths.at(a).at(1))) {
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
    return has_moved;
}


void PlannerLong::fill_queue(std::queue<int> & queue) {
    for (int a = 0; a < m_num_of_agents; a++) {
        // Enqueue the robots that need a path (even if they are already over their task)
        if (m_paths.at(a).size() == 1) {
            queue.push(a);
            m_enqueued.at(a) = 1;
        } 
        // else if (!m_dummy.at(a)) { // add suboptimal paths
        //     const int start = env->curr_states.at(a).location;
        //     const int end = task(a);
        //     // const double speed = distance(start, end) >
        //     if (m_paths.size() > distance(start, end) + 40) {
        //         queue.push(a);
        //         m_enqueued.at(a)++;
        //     }
        // }
    }
}

/// @brief Find paths for the agents in the queue
/// @param queue Queue of agents without a path
/// @return true if we emptied the queue in time
bool PlannerLong::try_planning(std::queue<int> & queue) {
    int count = 0; // Number of calls to plan_path
    while (!queue.empty()) {
        if (m_round > 0 && time_diff(m_start_time) > m_time_limit - 0.05) {
            cout << "-- Time: " << time_diff(m_start_time) << " ; agents planned: " << count << " (round " << m_round << ", interrupted)" << endl;
            return false;
        }
        int a = queue.front(); queue.pop();
        const Position cur_pos(env->curr_states.at(a), m_timestep);
        if (env->goal_locations.at(a).empty()) { // if no goal, do not move, that is, keep the same location and orientation. I do not think that this may happen
            cout << "Warning: agent a" << a << " has no task" << endl;
            continue;
        }
        const std::vector<int> conflicts = plan_path(a, cur_pos);
        register_path(a);
        for (int ca : conflicts) {
            // cout << " -- a" << a << " unplans a" << ca << " (" << queue.size()+1 << " in queue)\n"; // @debug
            const Position first_pos(env->curr_states.at(ca), m_timestep);
            unregister_path(ca);
            m_paths.at(ca).clear();
            m_paths.at(ca).push_back(first_pos);
            register_path(ca);
            queue.push(ca);
            m_enqueued.at(ca)++;
        }
        count++;
    }
    cout << "-- Time: " << time_diff(m_start_time) << " ; agents planned: " << count << " (round " << m_round << ")" << endl;
    return true;
}


/// @brief Find a path for agent that minimizes the length and the number of conflicts
/// @param agent Index of the agent
/// @param start Start position
/// @return Conflicting agents
std::vector<int> PlannerLong::plan_path(int agent, const Position &start) {

    if (m_dummy.at(agent)) {
        return plan_dummy_path(agent, start);
    }

    const int end = task(agent);
    // Empty the path if necessary. I don't use this
    if (m_paths.at(agent).size() > 1) {
        unregister_path(agent);
        m_paths.at(agent).clear();
        m_paths.at(agent).push_back(start);
        register_path(agent);

    }

    // cout << "Planning the path of a" << agent << " from (" << start.location%env->cols << "," << start.location/env->cols << ") to (" << end%env->cols << "," << end/env->cols << ")" << endl;
    if (obstacle(start.location)) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
    if (obstacle(end)) cout << "Error: agent a" << agent << " is assigned to location " << end << " which is an obstacle" << endl;

    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
    std::priority_queue<std::tuple<float, float, Node*>, std::vector<std::tuple<float, float, Node*>>> open_list; // Nodes to explore
    std::unordered_map<int, Node*> all_nodes; // nodes indexed by location
    std::unordered_set<int> close_list; // set of done nodes
    Node* s = new Node(start, nullptr); // start node
    open_list.push(std::make_tuple(-s->f, -s->g, s));
    all_nodes[encoding(start)] = s;
    bool timeout = false;
    std::vector<int> conflicts;

    while (!open_list.empty()) {
        const auto f_g_node = open_list.top(); open_list.pop(); // get the next node
        Node* curr = std::get<2>(f_g_node);
        if (curr->closed)
            continue; // This is an outdated version that should be ignored
        curr->closed = true;
        const Position end_pos(curr->location, curr->direction, curr->time);
        close_list.emplace(encoding(end_pos)); // mark it as done
        if (curr->location == end) {
            // Get the conflicts
            for (int a = 0; a < m_num_of_agents; a++) {
                if (curr->conflicts.test(a))
                    conflicts.push_back(a);
            }
            // Trace back the path
            std::vector<Position> path;
            while (curr->parent != NULL) {
                const Position pos(curr->location, curr->direction, curr->time);
                path.push_back(pos);
                curr = curr->parent;
            }
            // append the new path
            for (auto it = path.crbegin(); it != path.crend(); ++it)
                m_paths.at(agent).push_back(*it);
            // if we are already over the task, add a second step to block its position
            if (m_paths.at(agent).size() == 1) {
                Position next = m_paths.at(agent).front();
                next.time++;
                m_paths.at(agent).push_back(next);
            }
            forbid_position(agent, conflicts);
            break;
        }
        // otherwise, try every next step possible and add the node to the open list
        const auto neighborhood = neighbors(agent, Position(curr->location, curr->direction, curr->time));
        for (const auto &[neighbor, conflicts] : neighborhood) {
            if (close_list.find(encoding(neighbor)) != close_list.end()) // the node is already done
                continue;
            const auto confs = sum_conflicts(curr, conflicts);
            const double next_g = (curr->time - m_timestep) + m_alpha * confs.first;
            if (all_nodes.find(encoding(neighbor)) != all_nodes.end()) { // the node is already in the list: update its value
                Node* old_node = all_nodes[encoding(neighbor)];
                if (next_g < old_node->g) {
                    old_node->conflicts = confs.second;
                    old_node->conflict_cost = confs.first;
                    old_node->g = next_g;
                    old_node->f = old_node->g + old_node->h;
                    old_node->parent = curr;
                    open_list.push(std::make_tuple(-old_node->f, -old_node->g, old_node)); // Added duplicate
                }
            } else { // create the node
                Node* next_node = new Node(neighbor, curr);
                next_node->conflicts = confs.second;
                next_node->conflict_cost = confs.first;
                next_node->g = next_g;
                next_node->h = curr->h + distance_diff(curr->location, neighbor.location, end);
                next_node->f = next_node->g + next_node->h;
                open_list.push(std::make_tuple(-next_node->f, -next_node->g, next_node));
                all_nodes[encoding(neighbor)] = next_node;
            }
        }
    }
    for (auto n : all_nodes) {
        delete n.second;
    }
    return conflicts;
}


/// @brief Sort the agents in the queue by their distance to the task
/// @param queue 
void PlannerLong::sort_by_distance(std::queue<int> & queue) {
    // Compute approximate distances
    std::vector<std::pair<int, double>> agents;
    while (!queue.empty()) {
        int a = queue.front(); queue.pop();
        const int source = env->curr_states.at(a).location;
        const int target = task(a);
        double dist = distance(source, target);
        agents.push_back(std::make_pair(a, dist));
        if (source == target) // agents that have finished their task have zero priority
            agents.at(a).second = 10000;
    }
    // Sort
    std::sort(agents.begin(), agents.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });
    cout << "-- First agents:";
        for (int i = 0; i < std::min((int)agents.size()-1, 8); i++)
            cout << " a" << agents.at(i).first << ":" << agents.at(i).second;
        if (!agents.empty())
            cout << "... a"  << agents.back().first << ":" << agents.back().second << " (" << agents.size() << ")";
        cout << endl;
    for (std::size_t a = 0; a < agents.size(); a++)
        queue.push(agents.at(a).first);
}



/// @brief Compute all the valid positions after a given position of an agent
/// @param agent Index of the current agent
/// @param position Its current position
/// @return Vector of pairs of valid positions with a conflicting agent or -1 for no conflict
std::vector<std::pair<Planner::Position, std::vector<int>>> PlannerLong::neighbors(int agent, const Position &position) const {
    std::vector<std::pair<Planner::Position, std::vector<int>>> neighbors;
    const Position next_fw = move(position, Action::FW);
    if (valid_move(agent, position, next_fw)) {
        const std::vector<int> conflicts = conflicts_fw(agent, position, next_fw);
        neighbors.push_back(std::make_pair(next_fw, conflicts));
    }
    const Position next_cr = move(position, Action::CR);
    const std::vector<int> conflicts = conflicts_cr(agent, position, next_cr);
    neighbors.push_back(std::make_pair(next_cr, conflicts));
    neighbors.push_back(std::make_pair(move(position, Action::CCR), conflicts));
    neighbors.push_back(std::make_pair(move(position, Action::W), conflicts));
    return neighbors;
}


std::vector<int> PlannerLong::conflicts_fw(int agent, const Position & cur_pos, const Position & next_pos) const {
    if (next_pos.time > m_nb_steps) { // conflicts below the time horizon are ignored
        return std::vector<int>();
    }
    std::unordered_set<int> conflicts;
    const int t = cur_pos.time;
    std::vector<int> agents;
    agents = agent_at(next_pos.location, t+1);
    for (int a : agents) {
        if (a != agent) {
            // cout << "hits a" << a << endl;
            conflicts.insert(a);
        }
    }
    agents = agent_at(cur_pos.location, t+1);
    for (int a : agents) {
        if (a != agent && position_of(a, t).location == next_pos.location) {
            // cout << "crosses a" << a << endl;
            conflicts.insert(a);
        }
    }
    // I think that I do not need this one
    // agents = agent_at(next_pos.location, t);
    // for (int a : agents) {
    //     if (a != agent && (next_pos.direction - position_of(a, t).direction + 4) % 4 == 2) {
    //         // cout << "invades facing a" << a << endl;
    //         conflicts.insert(a);
    //     }
    // }
    return std::vector<int>(conflicts.begin(), conflicts.end());
}

std::vector<int> PlannerLong::conflicts_cr(int agent, const Position & cur_pos, const Position & next_pos) const {
    if (next_pos.time > m_nb_steps) { // conflicts below the time horizon are ignored
        return std::vector<int>();
    }
    std::unordered_set<int> conflicts;
    const int t = cur_pos.time;
    const std::vector<int> agents = agent_at(cur_pos.location, t+1);
    for (int a : agents) {
        if (a != agent) {
            // cout << "hits a" << a << endl;
            conflicts.insert(a);
        }
    }
    return std::vector<int>(conflicts.begin(), conflicts.end());
}

/// @brief Tell if an agent can move (do FW)
/// @param agent 
/// @param cur_pos 
/// @param next_pos 
/// @return 
bool PlannerLong::valid_move(int agent, const Position & cur_pos, const Position & next_pos) const {
    if (next_pos.location < 0 || obstacle(next_pos.location))
        return false;
    const std::tuple<int, int, int> key = std::make_tuple(agent, next_pos.location, next_pos.time);
    if (!m_forbidden_position.empty() && m_forbidden_position.find(key) != m_forbidden_position.end())
        return false;
    return true;

    // Smart detection of conflicts: it did not work
    // const int t = cur_pos.time;
    // int cur_loc = next_pos.location;
    // int cur_a = agent;
    // std::bitset<200> visited;
    // visited.set(agent);
    // while (true) {
    //     const std::vector<int> agents = agent_at(cur_loc, t); // @todo Can I have more than one agent?
    //     if (agents.empty())
    //         return true;
    //     cur_a = agents.front();
    //     if (visited.test(cur_a))
    //         return false;
    //     visited.set(cur_a);
    //     const int dir =  position_of(cur_a, t).direction;
    //     cur_loc = move_location(cur_loc, dir);
    //     if (cur_loc < 0 || obstacle(cur_loc))
    //         return false;
    // }

    // Simply avoid invading a facing agent
    // The next position is outside the map or on an obstacle
    // if (next_pos.location < 0 || obstacle(next_pos.location))
    //     return false;
    // // It goes onto the location of a robot that will not be able to escape
    // const int t = cur_pos.time;
    // const std::vector<int> agents = agent_at(next_pos.location, t);
    // for (int a : agents) {
    //     if (a != agent && (next_pos.direction - position_of(a, t).direction + 4) % 4 == 2) {
    //         // cout << "invades facing a" << a << endl;
    //         return false;
    //     }
    // }
    // return true;
}


/// @brief Comput the path value of a node: sum of conflicts
/// @param curr 
/// @param neighbor 
/// @param conflicts 
/// @return 
std::pair<int, std::bitset<200>> PlannerLong::sum_conflicts(Node * curr, const std::vector<int> & conflicts) const {
    double beta = (m_round == 0) ? 2.0 : m_beta - 0.15 * m_round; // 2.0, 1.6, 1.5, 1.4...
    // const double beta = (m_timestep == 0) ? 2.0 : m_beta;
    if(beta < .1) beta = .1;

    int cost = curr->conflict_cost;
    std::bitset<200> next_conflicts = curr->conflicts;
    for (int a : conflicts) {
        if (!next_conflicts.test(a)) {
            next_conflicts.set(a);
            if (!m_dummy.at(a))
                cost += std::pow(m_enqueued.at(a) + 1, beta);
        }
    }
    return std::make_pair(cost, next_conflicts);
}


/// @brief Detect an inevitable collision and forbid it
/// @param curr 
/// @details Look at the path we computed. If it makes a loop, I forbid a position for the other agent
void PlannerLong::forbid_position(int agent, const std::vector<int> & conflicts) {
    if (m_enqueued.at(agent) < m_enqueue_bound)
        return;
    if (conflicts.empty())
        return;
    const int ca = conflicts.front(); // conflicting agent
    // Find the position to forbid. 
    for (int i = 0; i < m_paths.at(agent).size(); i++) {
        const Position & pos = position_of(agent, i + m_timestep);
        if (pos.location == position_of(ca, i + m_timestep).location) { // @todo I used to have a bug here
            const Position & pos = m_paths.at(agent).at(i);
            m_forbidden_position.insert(std::make_tuple(ca, pos.location, pos.time));
            return;
        }
        if (i+1 < m_paths.at(agent).size() && i+1 < m_paths.at(ca).size() && m_paths.at(agent).at(i  ).location == m_paths.at(ca).at(i+1).location && m_paths.at(agent).at(i+1).location == m_paths.at(ca).at(i  ).location) {
            const Position & pos = m_paths.at(ca).at(i+1);
            m_forbidden_position.insert(std::make_tuple(ca, pos.location, pos.time));
            return;
        }
    }
    assert(false); // We cannot reach this point
}


void PlannerLong::update_dummy() {
    int count = std::count_if(m_dummy.begin(), m_dummy.end(), [](bool value) { return value; });
    for (int a = 0; a < m_num_of_agents; a++) {
        if (!m_dummy.at(a)) {
            const int start = env->curr_states.at(a).location;
            const int end = task(a);
            if (distance(start, end) > m_nb_steps - m_timestep) {
                m_dummy.at(a) = true;
                count++;
                cout << "-- a" << a << " is dummy (" << count << ")" << endl;
            }
        }
    }
}


/// @brief Find a path for agent that minimizes the conflicts, with no task in mind
/// @param agent Index of the agent
/// @param start Start position
/// @return Conflicting agents
/// @note This is almost the same function as plan_paths, there are only a few differences
std::vector<int> PlannerLong::plan_dummy_path(int agent, const Position &start) {
    assert(m_dummy.at(agent));
    // Empty the path if necessary
    if (m_paths.at(agent).size() > 1) {
        unregister_path(agent);
        m_paths.at(agent).clear();
        m_paths.at(agent).push_back(start);
        register_path(agent);
    }

    // cout << "Planning the path of a" << agent << " from (" << start.location%env->cols << "," << start.location/env->cols << ") to (" << end%env->cols << "," << end/env->cols << ")" << endl;
    if (obstacle(start.location)) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
    
    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
    std::priority_queue<std::tuple<float, float, Node*>, std::vector<std::tuple<float, float, Node*>>> open_list; // Nodes to explore
    std::unordered_map<int, Node*> all_nodes; // nodes indexed by location
    std::unordered_set<int> close_list; // set of done nodes
    Node* s = new Node(start, nullptr); // start node
    open_list.push(std::make_tuple(-s->f, -s->g, s));
    all_nodes[encoding(start)] = s;
    bool timeout = false;
    std::vector<int> conflicts;

    while (!open_list.empty()) {
        const auto f_g_node = open_list.top(); open_list.pop(); // get the next node
        Node* curr = std::get<2>(f_g_node);
        if (curr->closed)
            continue; // This is an outdated version that should be ignored
        curr->closed = true;
        const Position end_pos(curr->location, curr->direction, curr->time);
        close_list.emplace(encoding(end_pos)); // mark it as done
        if (curr->time > m_nb_steps) {
            // Get the conflicts
            for (int a = 0; a < m_num_of_agents; a++) {
                if (curr->conflicts.test(a))
                    conflicts.push_back(a);
            }
            // Trace back the path
            std::vector<Position> path;
            while (curr->parent != NULL) {
                const Position pos(curr->location, curr->direction, curr->time);
                path.push_back(pos);
                curr = curr->parent;
            }
            // append the new path
            for (auto it = path.crbegin(); it != path.crend(); ++it)
                m_paths.at(agent).push_back(*it);
            // if we are already over the task, add a second step to block its position
            if (m_paths.at(agent).size() == 1) {
                Position next = m_paths.at(agent).front();
                next.time++;
                m_paths.at(agent).push_back(next);
            }
            forbid_position(agent, conflicts);
            break;
        }

        // otherwise, try every next step possible and add the node to the open list
        const auto neighborhood = neighbors(agent, Position(curr->location, curr->direction, curr->time));
        for (const auto &[neighbor, conflicts] : neighborhood) {
            if (close_list.find(encoding(neighbor)) != close_list.end()) // the node is already done
                continue;
            const auto confs = sum_conflicts(curr, conflicts);
            const double next_g = confs.first; // @note We do not count the length of the path
            if (all_nodes.find(encoding(neighbor)) != all_nodes.end()) { // the node is already in the list: update its value
                Node* old_node = all_nodes[encoding(neighbor)];
                if (next_g < old_node->g) {
                    old_node->conflicts = confs.second;
                    old_node->conflict_cost = confs.first;
                    old_node->g = next_g;
                    old_node->f = old_node->g + old_node->h;
                    old_node->parent = curr;
                    open_list.push(std::make_tuple(-old_node->f, -old_node->g, old_node)); // Added duplicate
                }
            } else { // create the node
                float next_h = 0;
                if (neighbor.location == curr->location && neighbor.direction == curr->direction) {
                    next_h = -1;
                }
                Node* next_node = new Node(neighbor, curr);
                next_node->conflicts = confs.second;
                next_node->conflict_cost = confs.first;
                next_node->g = next_g;
                next_node->h = next_h; // @note dummy distance estimate
                next_node->f = next_node->g + next_node->h;
                open_list.push(std::make_tuple(-next_node->f, -next_node->g, next_node));
                all_nodes[encoding(neighbor)] = next_node;
            }
        }
    }
    for (auto n : all_nodes) {
        delete n.second;
    }
    return conflicts;
}

/// @brief Evaluate the current paths
double PlannerLong::evaluate_paths(bool print) const {
    double val1 = 0, val2 = 0, val3 = 0;
    const double b = 1 - 1.0 * m_num_of_agents / m_location_at_index.size();

    for (const auto & path : m_paths) {
        val1 += path.size();
        val2 += pow(path.size(), b);
        val3 += min(1.0 * path.size(), 25.0);
    }
    if (print)
        cout << "eval: " << val1 << " " << val2 << " " << val3 << endl;
    return val2;
}
