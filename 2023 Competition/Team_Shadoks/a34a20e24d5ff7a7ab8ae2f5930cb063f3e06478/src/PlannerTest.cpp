#include "PlannerTest.h"

#include <random>
#include <fstream>  // ofstream


PlannerTest::PlannerTest() : Planner() {}


void PlannerTest::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    if (parameters.values.count("path_bound") > 0)
        m_path_bound = parameters.values.at("path_bound");
    if (parameters.values.count("time_bound") > 0)
        m_time_bound = parameters.values.at("time_bound");
    m_location.resize(env->num_of_agents, std::vector<int>(6, -1));
    m_agent.resize(env->cols*env->rows, std::vector<int>(6, -1));
    m_enqueued.resize(env->num_of_agents, 0);
    m_forward.resize(env->num_of_agents, false);
    m_next_locations.resize(env->num_of_agents);
    cout << "PlannerTest(path_bound: " << m_path_bound << ", time_bound: " << m_time_bound << ") initialized" << endl;
}


/// @brief Plan the next action
/// @param actions 
void PlannerTest::plan(vector<Action> & actions) {
    actions = std::vector<Action>(env->num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;
    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
    reset();
    const std::vector<int> agents = sorted_agents();
    cout << "-- First agents: a" << agents.at(0) << " a" << agents.at(1) << " a" << agents.at(2) << " a" << agents.at(3) << " a" << agents.at(4) 
        << " a" << agents.at(5) << " a" << agents.at(6) << " a" << agents.at(7) << "..." << endl; 
    double elapsed_time = 0;
    int planned_agents = 0; // number of calls to conflict_solver
    int successful_plans = 0; // number of agents that advance in two locations towards their task
    for (int a : agents) {
        // Plan the agent if its task is not in an idle area
        if (m_idle_locations.count(task(a)) == 0) {
            bool success = conflict_solver(a);
            if (success)
                successful_plans++;
        } else {
            // cout << "a" << a << " wants to go to an endpoint" << endl;
        }
        planned_agents++;
        elapsed_time = time_diff(start_time);
        if (elapsed_time >= m_time_bound)
            break;
    }
    set_moves(actions);
    update_tasks_done(actions);
    cout << "-- Time: " << time_diff(start_time) << " ; agents planned: " << planned_agents << " (successful: " << successful_plans << ")" << endl;
    print_elapsed_time();
    // if (!check_moves(actions))
    //     m_halt = true;
    m_timestep++;
	return;
}

/// @brief Try to plan a given agent to make one move towards its task
/// @param agent Index of the agent to plan
/// @return True if we can push it towards its task
bool PlannerTest::conflict_solver(int agent) {
    const int bound = m_path_bound; // bound for stopping looking for an assignment
    const bool reset_count = true; // reset the conflict count in each conflict solver
    // Make a copy of the data in case this solver fails
    std::vector<std::vector<int>> copy_location(m_location.begin(), m_location.end()); // deep copy 
    std::vector<std::vector<int>> copy_agent(m_agent.begin(), m_agent.end()); // deep copy 
    std::vector<int> copy_enqueued(m_enqueued.begin(), m_enqueued.end()); 
    if (reset_count)
        std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // Reset m_enqueued
    assert(m_queue.empty());
    // Find a valid assignment. We stop when we reach one or when an agent is enqueued 100 times
    m_queue.push(agent);
    m_enqueued.at(agent) = 1;
    m_forward.at(agent) = true;
    bool invalid = false; // We do not find a solution
    std::set<int> collisions;
    make_closer_locations(agent);
    int count = 0;
    do {
        const int a = m_queue.front(); m_queue.pop();
        // cout << "a" << a << " (count:" << m_enqueued.at(a) << ") <- queue" << endl;
        const double cur_cost = best_move(a, collisions);
        for (int a2 : collisions) {
            if (m_location.at(a2).front() >= 0) { // if it is not already uplanned (in the queue)
                // cout << "queue <- a" << a2 << " (count:" << m_enqueued.at(a2)+1 << ")" << endl;
                for (int t = 0; t < 6; t++) {
                    const int loc = m_location.at(a2).at(t);
                    if (m_agent.at(loc).at(t) == a2)
                        m_agent.at(loc).at(t) = -1;
                    m_location.at(a2).at(t) = -1;
                }
                m_queue.push(a2);
                if (m_enqueued.at(a2) >= bound) {
                    invalid = true;
                } else {
                    m_enqueued.at(a2)++;
                }
            }
        }
        count++;
    } while (!m_queue.empty() && !invalid);
    // cout << "-- Conflict solver(" << agent << "): iterations=" << count << ", max_enqueued=" << *std::max_element(m_enqueued.begin(), m_enqueued.end()) << endl; // @debug
    if (invalid) {
        // Reset m_queue and the old values
        std::queue<int> empty;
        std::swap(m_queue, empty);
        m_location.assign(copy_location.begin(), copy_location.end()); // deep copy
        m_agent.assign(copy_agent.begin(), copy_agent.end()); // deep copy
        m_enqueued.assign(copy_enqueued.begin(), copy_enqueued.end()); // deep copy
        m_forward.at(agent) = false;
        // cout << "-- a" << agent << " not planned" << endl;
    }
    // cout << "-- path conflict: " << m_location.at(agent).at(0) << " (" << env->curr_states[agent].orientation << ") " << m_location.at(agent).at(1) << " " << m_location.at(agent).at(2) << " " << m_location.at(agent).at(3) << " " << m_location.at(agent).at(4) << " " << m_location.at(agent).at(5) << endl; // @debug
    return !invalid;
}


/// @brief Reset member variables
void PlannerTest::reset() {
    for (int a = 0; a < env->num_of_agents; a++) {
        const int loc = env->curr_states.at(a).location;
        std::fill(m_location.at(a).begin(), m_location.at(a).end(), loc);
        std::fill(m_agent.at(loc).begin(), m_agent.at(loc).end(), a);

    }
    std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // I think that I should keep the same at each call of conflict_solver
    std::fill(m_forward.begin(), m_forward.end(), false);
}


/// @brief Get the next location after an action
/// @param location Current location in the map
/// @param direction A direction to move: -1 (do not move), 0 (east), 1 (south)...
/// @return The next location. It returns -1 if it exists the map boundaries
int PlannerTest::move(int location, int direction) const {
    if (direction == -1)
        return location;
    const int x = location % env->cols;
    const int y = location / env->cols;
    if (direction == 0)
        return (x+1 < env->cols) ? location + 1 : -1;
    if (direction == 1)
        return (y+1 < env->rows) ? location + env->cols : -1;
    if (direction == 2)
        return (x-1 >= 0) ? location - 1 : -1;
    return (y-1 >= 0) ? location - env->cols : -1;
}


/// @brief Find the move for an agent that minimizes the conflicts
/// @param agent Index of the agent to plan
/// @param collisions Vector of (the indices of) the agents that have a collision with the planned move
/// @return Direction of the move
int PlannerTest::best_move(int agent, std::set<int> & collisions) {
    std::priority_queue<SearchNode*, std::vector<SearchNode*>, cmp> open_list; // search queue
    std::unordered_map<int, SearchNode*> all_nodes; // nodes indexed by (location, time) 
    unordered_set<int> close_list; // set of done nodes
    SearchNode* s = new SearchNode(env->curr_states.at(agent).location, 0, std::set<int>(), 0, nullptr); // start node
    open_list.push(s);
    all_nodes[encoding(s->location, s->time)] = s;
    double cost = -1;
    bool path_found = false;
    while (!open_list.empty()) {
        SearchNode* cur_node = open_list.top(); open_list.pop(); // get the next node
        // cur_node->print(); // @debug
        close_list.emplace(encoding(cur_node->location, cur_node->time)); // mark it as done
        if (finished(agent, cur_node)) { // if we find the goal, trace back the path
            cost = cur_node->cost;
            collisions = cur_node->collisions;
            // Update its path
            while (cur_node != NULL) {
                m_location.at(agent).at(cur_node->time) = cur_node->location;
                m_agent.at(cur_node->location).at(cur_node->time) = agent;
                cur_node = cur_node->parent;
            }
            path_found = true;
            // cout << "-- path a" << agent << " (best_move): " << m_location.at(agent).at(0) << " (" << env->curr_states[agent].orientation << ") " << m_location.at(agent).at(1) << " " << m_location.at(agent).at(2) << " " << m_location.at(agent).at(3) << " " << m_location.at(agent).at(4) << " " << m_location.at(agent).at(5) << endl; // @debug
            break;
        }
        if (wrong_path(agent, cur_node))
            continue;
        // otherwise, try every next step possible and add the node to the open list
        std::vector<int> neighborhood = neighbors(agent, cur_node);
        for (int next_loc : neighborhood) {
            if (next_loc < 0)
                continue;
            const int node_code = encoding(next_loc, cur_node->time + 1);
            if (close_list.find(node_code) != close_list.end()) // the node is already done
                continue;
            std::set<int> next_collisions;
            double next_cost = get_collisions(agent, cur_node, next_loc, next_collisions);
            if (next_loc != cur_node->location) { // @note Give priority to moving agents
                // next_cost -= 0.1;
                next_cost -= 1/(cur_node->time+1);
            }
            // Enqueue this node
            if (all_nodes.find(node_code) != all_nodes.end()) { // the node is already in the list: update its value if it is better
                SearchNode* old_node = all_nodes[node_code];
                if (next_cost < old_node->cost) {
                    old_node->cost = next_cost,
                    old_node->collisions = next_collisions,
                    old_node->parent = cur_node;
                }
            } else { // create the node
                SearchNode* next_node = new SearchNode(next_loc, cur_node->time + 1, next_collisions, next_cost, cur_node);
                open_list.push(next_node);
                all_nodes[node_code] = next_node;
            }
        }
    }
    for (auto n : all_nodes) {
        delete n.second;
    }
    all_nodes.clear();
    assert(path_found); // @debug
    return cost;
}


/// @brief Tell if this is a final node in the search of a valid path
/// @param agent Index of the agent that we are planning
/// @param node A search node with a location, time, and other informations
/// @return True if this node finishes the search, that is:
/// - If it is not a forward agent, this is a path of length 5
/// - If it is a forward agent, this is a path of length 5 and the final location is closer to the target or it passes by the task
bool PlannerTest::finished(int agent, const SearchNode* node) const {
    if (node->time < 5)
        return false;
    if (m_forward.at(agent)) {
        const int end = task(agent);
        if (node->location == end || node->parent->location == end || node->parent->parent->location == end || node->parent->parent->parent->location == end || node->parent->parent->parent->parent->location == end)
            return true;
        if (m_next_locations.at(agent).count(node->location) > 0)
            return false;
        const int start_distance = distance(env->curr_states[agent].location, task(agent));
        const int cur_distance = distance(node->location, task(agent));
        return cur_distance < start_distance;
    }
    return true;
}


std::vector<int> PlannerTest::neighbors(int agent, const SearchNode* node) const {
    // Set possible next locations
    const int prev1 = node->location;
    const int direction = env->curr_states.at(agent).orientation;
    std::vector<int> neighs(5, -1); // there are at most 5 neighbors, including the same location
    neighs.at(0) = prev1;
    if (node->time == 0) { // You can only move forward
        neighs.at(1) = move(prev1, direction);
    } else if (node->time == 1) {
        const int prev2 = env->curr_states.at(agent).location;
        neighs.at(1) = move(prev1, direction);
        if (prev1 == prev2) {
            neighs.at(2) = move(prev1, (direction+1)%4);
            neighs.at(3) = move(prev1, (direction+3)%4);
        }
    } else {
        const int prev1 = node->location;
        const int prev2 = node->parent->location;
        const int prev3 = node->parent->parent->location;
        if (prev1 != prev2) { // move forward
            neighs.at(1) = 2*prev1 - prev2; // note that it can exit the map
        } else if (prev2 != prev3) { // move forward or to a side
            for (int dir = 0; dir < 4; dir++) {
                int next = move(prev1, dir);
                if (next != prev3)
                    neighs.at(dir+1) = next;
            }
        } else {
            for (int dir = 0; dir < 4; dir++)
                neighs.at(dir+1) = move(prev1, dir);
        }
    }
    // Check that the moves are ok
    for (int &next_loc : neighs) {
        if (!valid_move(prev1, next_loc)) {
            next_loc = -1;
        }
    }
    return neighs;
}

bool PlannerTest::valid_move(int loc1, int loc2) const {
    // Check that we stay within the map
    if (loc2 < 0) return false;
    if (loc2 >= env->cols*env->rows) return false;
    if (loc2 == loc1 + 1 && loc2 % env->cols == 0) return false;
    if (loc2 == loc1 - 1 && loc1 % env->cols == 0) return false;
    // Check the collision with an obstacle
    if (obstacle(loc2)) return false;
    // Check the collision with a barrier
    if (loc2 == loc1 + 1 && hit_barrier(loc1, 0)) return false;
    if (loc2 == loc1 + env->cols && hit_barrier(loc1, 1)) return false;
    if (loc2 == loc1 - 1 && hit_barrier(loc1, 2)) return false;
    if (loc2 == loc1 - env->cols && hit_barrier(loc1, 3)) return false;
    return true;
}

/// @brief Update the cost and the collisions of this node
/// @param node 
double PlannerTest::get_collisions(int agent, const SearchNode * node, int location, std::set<int> & collisions) const {
    assert(collisions.empty());
    for (int a : node->collisions)
        collisions.insert(a);
    // collisions = node->collisions;
    double cost = node->cost;
    // Vertex collision
    int a1 = m_agent.at(location).at(node->time + 1);
    if (a1 >= 0 && a1 != agent && collisions.count(a1) == 0) {
        collisions.insert(a1);
        cost += pow(m_enqueued.at(a1), 2) + 1;
    }
    // Edge collisions
    if (node->location != location) { // only necessary if the agent is moving
        int a2 = m_agent.at(location).at(node->time);
        if (a2 >= 0 && m_agent.at(node->location).at(node->time+1) == a2 && collisions.count(a2) == 0) {
            collisions.insert(a2);
            cost += m_enqueued.at(a2)*m_enqueued.at(a2) + 1;
        }
    }
    return cost;
}


/// @brief Check if an agent can move from a location to a direction
/// @param location Location in the map
/// @param direction Direction of the move: -1 (do not move), 0 (east), 1 (south)...
/// @return True if the agent can make that move: it remains in the map, it does not go into an obstacle and it respects the barriers
bool PlannerTest::good_move(int location, int direction) const {
    if (direction == -1)
        return true;
    if (hit_barrier(location, direction))
        return false;

    const int x = location % env->cols;
    const int y = location / env->cols;
    if (direction == 0 && x+1 >= env->cols)
        return false;
    if (direction == 1 && y+1 >= env->rows)
        return false;
    if (direction == 2 && x-1 < 0)
        return false;
    if (direction == 3 && y-1 < 0)
        return false;
    const int next_loc = move(location, direction);
    return !obstacle(next_loc);
}


/// @brief Compute the next action for each agent given its next locations.
/// @param plan 
/// @details First, we assign a rotation to the agents that will move to a location that they are 
/// not facing, and put the rest of the robots in a list. Then, we identify the robots that will not
/// be able to advance because their next locations is occupied by a rotation agent. We back propagate
/// from these agents to identify all the agents that cannot move. And thus we move the rest of the agents.
void PlannerTest::set_moves(std::vector<Action> & plan) const {
    for (int a = 0; a < env->num_of_agents; a++) {
        const int cur_dir = env->curr_states.at(a).orientation;
        const int cur_loc = env->curr_states.at(a).location;
        int t = -1;
        for (int i = 1; i < 6 && t < 0; i++) {
            if (m_location.at(a).at(i) != cur_loc)
                t = i;
        }
        if (t < 0) {
            plan.at(a) = Action::W;
        } else if (t == 1) {
            plan.at(a) = Action::FW;
        } else {
            const int dir = direction(cur_loc, m_location.at(a).at(t));
            if (dir == cur_dir)
                plan.at(a) = Action::W;
            else if (dir == (cur_dir+3)%4) {
                plan.at(a) = Action::CCR;
            } else {
                plan.at(a) = Action::CR;
            }
        }
    }
}


/// @brief Sort the agents by distance to their task. We use approximate distances
/// @return The list of agents
std::vector<int> PlannerTest::sorted_agents() const {
    // Compute approximate distances
     std::vector<int> dist(env->num_of_agents, -1);
     for (int a = 0; a < env->num_of_agents; a++) {
         const int source = env->curr_states[a].location;
         const int target = task(a);
         dist.at(a) = distance(source, target);
     }
    // Sort by distance
    std::vector<int> temp(env->num_of_agents);
    std::iota(temp.begin(), temp.end(), 0); // assign values from 0 to temp.size() - 1
    std::sort(temp.begin(), temp.end(), [&dist](int a, int b) {
        return dist.at(a) < dist.at(b);
    });
    return temp;
}


/// @brief Compute the locations that are at least two locations closer to the target that are reachable within 5 steps
/// @param agent 
void PlannerTest::make_closer_locations(int agent) {
    const int cur_loc = env->curr_states[agent].location;
    const int end = task(agent);
    m_next_locations.at(agent).clear();
    if (good_move(cur_loc, 0) && closer_east(cur_loc, end))
        m_next_locations.at(agent).insert(move(cur_loc, 0));
    if (good_move(cur_loc, 1) && closer_south(cur_loc, end))
        m_next_locations.at(agent).insert(move(cur_loc, 1));
    if (good_move(cur_loc, 2) && !closer_east(cur_loc - 1,          end))
        m_next_locations.at(agent).insert(move(cur_loc, 2));
    if (good_move(cur_loc, 3) && !closer_south(cur_loc - env->cols, end))
        m_next_locations.at(agent).insert(move(cur_loc, 3));
}

/// @brief Tell if a path is not good, so that we stop exploring it
/// @details A path is bad if it is too long or if after three actions it does not get closer to its target.
/// @param agent 
/// @param node 
/// @return 
bool PlannerTest::wrong_path(int agent, const SearchNode* node) const {
    if (node->time >= 5)
        return true;
    const int start_distance = distance(env->curr_states[agent].location, task(agent));
    const int cur_distance = distance(node->location, task(agent));
    if (m_forward.at(agent) && node->time >= 3 && cur_distance >= start_distance)
        return true;
    return false;
}