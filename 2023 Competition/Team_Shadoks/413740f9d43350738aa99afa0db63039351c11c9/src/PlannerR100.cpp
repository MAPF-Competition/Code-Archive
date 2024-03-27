#include "PlannerR100.h"
#include "nlohmann/json.hpp"

#include <random>
// #include <fstream>  // ofstream

PlannerR100::PlannerR100() : Planner() {}


void PlannerR100::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);

    m_alpha = parameters.values.at("alpha");
    m_beta = parameters.values.at("beta");
    m_enqueue_bound = parameters.values.at("enqueue_bound");
    m_nb_steps = parameters.values.at("nb_steps");
    m_goodlen = parameters.values.at("goodlen");
    m_erase = parameters.values.at("erase");

    for(auto &[k,v] : parameters.values)
        cout << k << " = " << v << endl;

    m_enqueued.resize(m_num_of_agents, 0);
    m_dummy.resize(m_num_of_agents, false);
    cout << "PlannerR100("
        << "alpha: " << m_alpha
        << ", beta: " << m_beta
        << ", enqueue_bound: " << m_enqueue_bound 
        << ", nb_steps: " << m_nb_steps << ") initialized" << endl;
}

// double PlannerR100::ratePaths() {
//     double lendone = 0;
//     int a = 0;
//     for(auto &path : m_paths) {
//         if(path.size() && path.back().location == task(a)) {
//             lendone += pow(path.size(), .5);
//         }
//
//         a++;
//     }
//
//     return lendone;
// }

double PlannerR100::ratePaths() {
    double ret = 0;
    int a = 0;
    for(auto &path : m_paths) {
        if(path.size() && path.size() < m_goodlen && path.back().location == task(a)) {
            ret += m_goodlen - path.size();
        }

        a++;
    }

    return -ret;
}

/**
 * @brief PlannerR100::plan We plan a short path for each robot using A* search and
 * take the first step for this timestep
 * @param time_limit
 * @param actions
 */
void PlannerR100::plan(vector<Action> & actions) {
    m_start_time  = std::chrono::high_resolution_clock::now();

    actions = std::vector<Action>(m_num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;

    double elapsed_time = 0.0;
    double bestScore = 1e50;
    std::vector<std::vector<Position>> bestPaths;
    std::vector<int> bestenqueued;
    int attempt = 0;

    const bool has_moved = update_paths();
    auto savedPaths = m_paths;
    auto savedenqueued = m_enqueued;

    do {
        if(attempt != 0) {
            m_enqueued = savedenqueued;
            for(int a = 0; a < m_num_of_agents; a++) {
                if(rand() % 10000 > 10000 * m_erase)
                    m_paths.at(a) = savedPaths.at(a);
                else {
                    m_paths.at(a) = std::vector<Position>{Position(env->curr_states.at(a), m_timestep)};
                }
            }
            m_passing_hash.clear();
            m_ending_hash.clear();
            for (int a = 0; a < m_num_of_agents; a++)
                register_path(a);
        }

        update_dummy();
        m_forbidden_position.clear();

        // Put the unplanned agents into the queue
        std::queue<int> queue;
        fill_queue(queue);
        shuffle(queue);

        // Process the queue
        bool valid = true;
        int count = 0;
        while (has_moved && !queue.empty()) {
            int a = queue.front(); queue.pop();
            const Position cur_pos(env->curr_states.at(a), m_timestep);
            if (env->goal_locations.at(a).empty()) { // if no goal, do not move, that is, keep the same location and orientation. I do not think that this may happen
                cout << "Warning: agent a" << a << " has no task" << endl;
                continue;
            }
            double timeAvailable = attempt == 0 ? 10 : .9 - time_diff(m_start_time);
            unregister_path(a);
            const std::vector<int> conflicts = plan_path(a, cur_pos, timeAvailable);
            if(conflicts == std::vector<int>{-1}) {
                valid = false;
                break;
            }
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
        if(!valid)
            break;

        auto curScore = ratePaths();
        // cout << curScore;
        if(curScore < bestScore) {
            bestScore = curScore;
            bestPaths = m_paths;
            bestenqueued = m_enqueued;
            cout << '+' << std::flush;
        }
        else {
            if(attempt < 100)
                cout << '.' << std::flush;
            else if(attempt == 100)
                cout << '*' <<std::flush;
        }


        elapsed_time = time_diff(m_start_time);
        attempt++;
    } while(elapsed_time < .8);

    m_paths = bestPaths;
    m_enqueued = bestenqueued;
    cout << endl;

    get_actions(actions);
    update_tasks_done(actions);
    print_elapsed_time();
    // write_paths_json(); // @debug
    m_timestep++;
    assert(check_moves(actions));
    return;
}


/// @brief Update the planned paths.
/// @details At timestep 0, we set the initial position instead. For the next timesteps,
/// we remove the first position if it was used
bool PlannerR100::update_paths() {
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

    m_passing_hash.clear();
    m_ending_hash.clear();
    for (int a = 0; a < m_num_of_agents; a++)
        register_path(a);

    return has_moved;
}



void PlannerR100::fill_queue(std::queue<int> & queue) {
    // m_enqueued = std::vector<int>(m_num_of_agents, 0);

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

    // New approach
//    for (int a = 0; a < m_num_of_agents; a++) {
//        queue.push(a);
//        enqueued.at(a)++;
//    }
}


/// @brief Find a path for agent that minimizes the length and the number of conflicts
/// @param agent Index of the agent
/// @param start Start position
/// @param timeleft time allowed to use (ignore it)
/// @return Conflicting agents
std::vector<int> PlannerR100::plan_path(int agent, const Position &start, double timeleft) {
    auto plan_start = std::chrono::high_resolution_clock::now();

    if (m_dummy.at(agent)) {
        return plan_dummy_path(agent, start, timeleft);
    }

    const int end = task(agent);
    // Empty the path if necessary
    if (m_paths.at(agent).size() > 1) {
        m_paths.at(agent).clear();
        m_paths.at(agent).push_back(start);
    }

    // cout << "Planning the path of a" << agent << " from (" << start.location%env->cols << "," << start.location/env->cols << ") to (" << end%env->cols << "," << end/env->cols << ")" << endl;
    if (obstacle(start.location)) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
    if (obstacle(end)) cout << "Error: agent a" << agent << " is assigned to location " << end << " which is an obstacle" << endl;

    // std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
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
        if (time_diff(plan_start) > timeleft) { // It's taking too long
            return {-1};
        }

        // otherwise, try every next step possible and add the node to the open list
        const auto neighborhood = neighbors(agent, Position(curr->location, curr->direction, curr->time));
        for (const auto &[neighbor, conflicts] : neighborhood) {
            if (close_list.find(encoding(neighbor)) != close_list.end()) // the node is already done
                continue;
            const auto confs = add_conflicts(curr, conflicts);
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

void PlannerR100::shuffle(std::queue<int> & queue) {
    // Compute approximate distances
    std::vector<std::pair<int, double>> agents;
    while (!queue.empty()) {
        int a = queue.front(); queue.pop();
        const int source = env->curr_states.at(a).location;
        const int target = task(a);
        double dist = rand();
        agents.push_back(std::make_pair(a, dist));
        if (source == target) // agents that have finished their task have zero priority
            agents.at(a).second = 10000;
    }
    // Sort
    std::sort(agents.begin(), agents.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });
    // cout << "-- First agents:";
        // for (int i = 0; i < std::min((int)agents.size()-1, 8); i++)
        //     cout << " a" << agents.at(i).first << ":" << agents.at(i).second;
        // if (!agents.empty())
        //     cout << "... a"  << agents.back().first << ":" << agents.back().second << " (" << agents.size() << ")" << endl;
    for (std::size_t a = 0; a < agents.size(); a++)
        queue.push(agents.at(a).first);
}

/// @brief Sort the agents in the queue by their distance to the task
/// @param queue 
void PlannerR100::sort_by_distance(std::queue<int> & queue) {
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
            cout << "... a"  << agents.back().first << ":" << agents.back().second << " (" << agents.size() << ")" << endl;
    for (std::size_t a = 0; a < agents.size(); a++)
        queue.push(agents.at(a).first);
}



/// @brief Compute all the valid positions after a given position of an agent
/// @param agent Index of the current agent
/// @param position Its current position
/// @return Vector of pairs of valid positions with a conflicting agent or -1 for no conflict
std::vector<std::pair<Planner::Position, std::vector<int>>> PlannerR100::neighbors(int agent, const Position &position) const {
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


std::vector<int> PlannerR100::conflicts_fw(int agent, const Position & cur_pos, const Position & next_pos) const {
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

std::vector<int> PlannerR100::conflicts_cr(int agent, const Position & cur_pos, const Position & next_pos) const {
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
bool PlannerR100::valid_move(int agent, const Position & cur_pos, const Position & next_pos) const {
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
    // std::bitset<100> visited;
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
std::pair<int, std::bitset<100>> PlannerR100::add_conflicts(Node * curr, const std::vector<int> & conflicts) const {
    const float beta = (m_timestep == 0) ? 2.0 : m_beta; // I set a large beta in the first timestep so that it finishes fast
    int cost = curr->conflict_cost;
    std::bitset<100> next_conflicts = curr->conflicts;
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
void PlannerR100::forbid_position(int agent, const std::vector<int> & conflicts) {
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


void PlannerR100::update_dummy() {
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
/// @param timeleft time allowed to use (ignore it)
/// @return Conflicting agents
std::vector<int> PlannerR100::plan_dummy_path(int agent, const Position &start, double timeleft) {
    assert(m_dummy.at(agent));
    // Empty the path if necessary
    if (m_paths.at(agent).size() > 1) {
        m_paths.at(agent).clear();
        m_paths.at(agent).push_back(start);
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
        // if (time_diff(start_time) > timeleft) { // It's taking too long
        //     timeout = true;
        //     break;
        // }

        // otherwise, try every next step possible and add the node to the open list
        const auto neighborhood = neighbors(agent, Position(curr->location, curr->direction, curr->time));
        for (const auto &[neighbor, conflicts] : neighborhood) {
            if (close_list.find(encoding(neighbor)) != close_list.end()) // the node is already done
                continue;
            const auto confs = add_conflicts(curr, conflicts);
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
