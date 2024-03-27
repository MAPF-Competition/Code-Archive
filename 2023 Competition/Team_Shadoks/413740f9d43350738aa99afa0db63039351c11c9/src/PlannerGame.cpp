#include "PlannerGame.h"

#include <random>
#include <fstream>  // ofstream


PlannerGame::PlannerGame() : Planner() {}


void PlannerGame::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    assert(m_map == InstanceMap::Game);
    if (parameters.values.count("enqueue_bound") > 0)
        m_enqueue_bound = parameters.values.at("enqueue_bound");
    if (parameters.values.count("time_bound") > 0)
        m_time_bound = parameters.values.at("time_bound");
    if (parameters.values.count("agents_bound") > 0)
        m_agents_bound = min(1.0, parameters.values.at("agents_bound"));

    m_location.resize(env->num_of_agents, std::vector<int>(2, -1));
    m_agent.resize(m_location_at_index.size(), std::vector<int>(2, -1));
    m_enqueued.resize(env->num_of_agents, 0);
    m_forward.resize(env->num_of_agents, false);
    m_position.resize(env->num_of_agents);
    m_time_to_move.resize(env->num_of_agents, -1);

    cout << "PlannerGame(enqueue_bound: " << m_enqueue_bound 
    << ", time_bound: " << m_time_bound 
    << ", barriers: " << m_barriers
    << ", agents: " << m_agents_bound << ") initialized" << endl;
}


/// @brief Plan the next action
/// @param actions 
void PlannerGame::plan(vector<Action> & actions) {
    actions = std::vector<Action>(env->num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    reset();
    const std::vector<int> agents = sorted_agents();
    double elapsed_time = 0;
    int planned_agents = 0;
    int successful_plans = 0;
    for (int i = 0; i < m_agents_bound * agents.size(); i++) {
        int a1 = agents.at(i);
        bool success = conflict_solver(a1);
        if (success) {
            successful_plans++;
            update_time_to_move();
        }
        planned_agents++;
        if (time_diff(start_time) >= m_time_bound)
            break;
    }
    set_moves(actions);
    // wide_view(actions);
    update_tasks_done(actions);
    print_elapsed_time();
    cout << "-- Time: " << time_diff(start_time) << " ; agents planned: " << planned_agents << " (successful: " << successful_plans << ")" << endl;
    // if (!check_moves(actions))
    //     m_halt = true;
    m_timestep++;
	return;
}

/// @brief Try to plan a given agent to make one move towards is task
/// @param agent Index of the agent to plan
/// @return True if we can push it towards its task
bool PlannerGame::conflict_solver(int agent) {
    const bool reset_count = false; // reset the conflict count in each conflict solver
    // Make a copy of the data in case this solver fails
    std::vector<std::vector<int>> copy_location(m_location.begin(), m_location.end()); // deep copy
    std::vector<int> copy_enqueued(m_enqueued.begin(), m_enqueued.end()); 
    int old_time = m_time_to_move.at(agent);
    if (reset_count)
        std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // Reset m_enqueued. I removed this because it seems slower.
    assert(m_queue.empty());
    // Find a valid assignment. We stop when we reach one or when an agent is enqueued 100 times
    m_queue.push(agent);
    m_enqueued.at(agent) += 1;
    m_forward.at(agent) = true;
    m_time_to_move.at(agent) = 3; // @note I set it to the maximum now, it will be corrected if the planning is successful
    bool invalid = false; // We do not find a solution
    std::vector<int> collisions;
    int count = 0;
    do {
        const int a = m_queue.front(); m_queue.pop();
        // cout << "a" << a << " (count:" << m_enqueued.at(a) << ") <- queue" << endl;
        const int cur_loc = best_move(a, collisions);
        // cout << "a" << a << ": " << m_location.at(a).at(0) << " -> " << m_location.at(a).at(1) << endl;
        for (int a2 : collisions) {
            // cout << "queue <- a" << a2 << " (count:" << m_enqueued.at(a2)+1 << ")" << endl;
            const int loc = m_location.at(a2).at(1);
            assert(loc >= 0);
            if (get_agent(loc, 1) == a2)
                set_agent(loc, 1, -1);
            m_location.at(a2).at(1) = -1;
            m_queue.push(a2);
            if (m_enqueued.at(a2) >= m_enqueue_bound) {
                invalid = true;
            } else {
                m_enqueued.at(a2)++;
            }
        }
        // validate_correspondance(); // @debug
        count++;
    } while (!m_queue.empty() && !invalid);
    // cout << "-- Conflict solver(" << agent << "): iterations=" << count << ", max_enqueued=" << *std::max_element(m_enqueued.begin(), m_enqueued.end()) << endl; // @debug
    if (invalid) {
        // Reset m_queue and the old values
        std::queue<int> empty;
        std::swap(m_queue, empty);
        m_location.assign(copy_location.begin(), copy_location.end()); // deep copy
        backup_agent();
        m_enqueued.assign(copy_enqueued.begin(), copy_enqueued.end()); // deep copy
        m_forward.at(agent) = false;
        m_time_to_move.at(agent) = old_time;
        // cout << "-- a" << agent << " not planned" << endl;
        return false;
    }
    return true;
}


/// @brief Reset member variables
void PlannerGame::reset() {
    if (m_timestep == 0) {
        std::fill(m_agent.begin(), m_agent.end(), std::vector<int>(2, -1));
        for (int a = 0; a < env->num_of_agents; a++) {
            m_position.at(a) = Position(env->curr_states.at(a), 0);
            const int loc = m_position.at(a).location;
            const int i_loc = m_index_of_location.at(loc);
            std::fill(m_location.at(a).begin(), m_location.at(a).end(), loc);
            std::fill(m_agent.at(i_loc).begin(), m_agent.at(i_loc).end(), a);
        }
        std::fill(m_time_to_move.begin(), m_time_to_move.end(), 0);
    } else {
        std::fill(m_agent.begin(), m_agent.end(), std::vector<int>(2, -1));
        for (int a = 0; a < env->num_of_agents; a++) {
            m_position.at(a) = Position(env->curr_states.at(a), 0);
            m_location.at(a).at(0) = m_position.at(a).location;
            for (int t = 0; t <= 1; t++) {
                const int loc = m_location.at(a).at(t);
                const int i_loc = m_index_of_location.at(loc);
                 m_agent.at(i_loc).at(t) = a;
            }
        }
        update_time_to_move();
    }
    std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // I think that I should keep the same at each call of conflict_solver
    std::fill(m_forward.begin(), m_forward.end(), false);
    
}


/// @brief Get the next location after an action
/// @param location Current location in the map
/// @param direction A direction to move: -1 (do not move), 0 (east), 1 (south)...
/// @return The next location. It returns -1 if it exists the map boundaries
int PlannerGame::move(int location, int direction) const {
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


/// @brief Compute the collisions produced when an agent moves in a direction
/// @param agent Index of the agent that moves
/// @param direction Location where it moves
/// @return Agents that are in collision. There are at most 2
/// @note There can be two collisions: if two agent in a row are coming at you and you are going towards them, 
/// you have an edge collision with one and a vertex collision with the other
/// @todo Can there be more than 1?
std::vector<int> PlannerGame::get_collisions(int agent, int next_location) const {
    std::vector<int> collisions;
    const int cur_location = m_location.at(agent).at(0);
    // vertex collision
    const int a1 = get_agent(next_location, 1);
    if (a1 >= 0 && a1 != agent) {
        collisions.push_back(a1);
    }
    // edge collision
    const int a2 = get_agent(next_location, 0);
    if (a2 >= 0 && m_location.at(a2).at(1) == cur_location) {
        collisions.push_back(a2);
    }
    return collisions;
}


/// @brief Find the move for an agent that minimizes the conflicts
/// @param agent Index of the agent to plan
/// @param collisions Vector of (the indices of) the agents that have a collision with the planned move
/// @return Direction of the move
/// @details I tried to prioritize locations that are in front of the agent, but that did not work well
int PlannerGame::best_move(int agent, std::vector<int> & collisions) {
    int best_location = -1;
    double best_value = -1;
    collisions.clear();
    const std::vector<int> locations = next_locations(agent);
    for (int loc : locations) {
        std::vector<int> cur_collisions = get_collisions(agent, loc);
        double value = 0.1 * steps_to_move(agent, loc); // @note We prefer direct moves
        for (int a : cur_collisions) {
            value += std::pow(m_enqueued.at(a) + 1, 2);
        }
        if (best_value == -1 || value < best_value) {
            best_value = value;
            collisions = cur_collisions;
            best_location = loc;
        }
    }
    // Update data
    const int old_loc = m_location.at(agent).at(1);
    if (old_loc >= 0) { // if the agent was planned
        set_agent(old_loc, 1, -1);
    }
    m_location.at(agent).at(1) = best_location;
    set_agent(best_location, 1, agent);
    return best_location;
}


/// @brief Check if an agent can move from a location to a direction
/// @param location Location in the map
/// @param direction Direction of the move: -1 (do not move), 0 (east), 1 (south)...
/// @return True if the agent can make that move: it remains in the map, it does not go into an obstacle and it respects the barriers
bool PlannerGame::good_move(int location, int direction) const {
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


/// @brief Compute the next move for each agent given its next location.
/// @param plan 
/// @details First, we assign a rotation to the agents that will move to a location that they are 
/// not facing, and put the rest of the robots in a list. Then, we identify the robots that will not
/// be able to advance because their next locations is occupied by a rotation agent. We back propagate
/// from these agents to identify all the agents that cannot move. And thus we move the rest of the agents.
void PlannerGame::set_moves(std::vector<Action> & plan) const {
    // validate_correspondance(); // @debug
    // Find agents that must rotate and agents that are ready to move
    std::vector<int> ready_agents; // agents that face their next location
    for (int a = 0; a < env->num_of_agents; a++) {
        int cur_dir = -1;
        if (m_location.at(a).at(0) != m_location.at(a).at(1)) {
            cur_dir = direction(m_location.at(a).at(0), m_location.at(a).at(1));
        }
        if (cur_dir >= 0) {
            const int delta = (m_position.at(a).direction - cur_dir + 4) % 4;
            if (delta == 0) {
                ready_agents.push_back(a);
            } else if (delta == 1 ||delta == 2) {
                plan.at(a) = Action::CCR;
            } else {
                plan.at(a) = Action::CR;
            }
        }
    }
    // Find robots that can move
    std::vector<int> prev_agent(env->num_of_agents, -1); // prev_agent[a] = agent that is planned to move to the location of a
    std::list<int> waiting_robots; // agents facing its next location that cannot advance (because there is an agent turning in front of them)
    for (int a : ready_agents) {
        const int next_location = m_location.at(a).at(1);
        const int a2 = get_agent(next_location, 0);
        if (a2 >= 0) {
            prev_agent.at(a2) = a;
            if (plan.at(a2) == Action::CR || plan.at(a2) == Action::CCR) {
                waiting_robots.push_back(a);
            }
        }
    }
    // Given the waiting robots, which cannot move, we back propagate to find the agents that are blocked by them
    std::set<int> blocked_agents; // agents that cannot move in this timestep
    for (int a : waiting_robots) {
        int a2 = a;
        do {
            blocked_agents.insert(a2);
            a2 = prev_agent.at(a2);
        } while (a2 >= 0);
    }
    // Move the agents that are not blocked
    for (int a : ready_agents) {
        if (blocked_agents.count(a) == 0) {
            plan.at(a) = Action::FW;
        }
    }
}


/// @brief Get the next location of an agent. We compute the direction that makes this agent closer to its task, or any direction.
/// @param agent The (index of the) agent
/// @return A list of directions: -1 (do not move), 0 (east), 1 (south)...
std::vector<int> PlannerGame::next_locations(int agent) const {
    const int cur_loc = m_location.at(agent).at(0);
    const int end = task(agent);
    std::vector<int> locations;
    if (cur_loc == end) { // @note this can really happen
        locations.push_back(cur_loc);
        return locations;
    }
    std::vector<int> directions;
    if (m_forward.at(agent)) {
        if (m_barriers == 0 && (m_map == InstanceMap::Sortation || m_map == InstanceMap::Warehouse)) {
            if (good_move(cur_loc, 0) && distance_diff(cur_loc, cur_loc + 1,         end) < 0)
                directions.push_back(0);
            if (good_move(cur_loc, 1) && distance_diff(cur_loc, cur_loc + env->cols, end) < 0)
                directions.push_back(1);
            if (good_move(cur_loc, 2) && distance_diff(cur_loc, cur_loc - 1,         end) < 0)
                directions.push_back(2);
            if (good_move(cur_loc, 3) && distance_diff(cur_loc, cur_loc - env->cols, end) < 0)
                directions.push_back(3);
        } else {
            if (good_move(cur_loc, 0) && closer_east(cur_loc, end))
                directions.push_back(0);
            if (good_move(cur_loc, 1) && closer_south(cur_loc, end))
                directions.push_back(1);
            if (good_move(cur_loc, 2) && !closer_east(cur_loc - 1,          end))
                directions.push_back(2);
            if (good_move(cur_loc, 3) && !closer_south(cur_loc - env->cols, end))
                directions.push_back(3);
        }
        assert(!directions.empty());
        for (int direction : directions) {
            if (punctual(agent, direction)) 
                locations.push_back(move(cur_loc, direction));
        }
    } else {
        for (int direction = -1; direction <= 3; direction++) {
            if (good_move(cur_loc, direction) && punctual(agent, direction)) 
                locations.push_back(move(cur_loc, direction));
        }
    }
    return locations;
}


/// @brief Sort the agents by distance to their task. We use approximate distances
/// @return The list of agents
std::vector<int> PlannerGame::sorted_agents() const {
    // Compute approximate distances
    std::vector<int> dist(env->num_of_agents, -1);
    for (int a = 0; a < env->num_of_agents; a++) {
        const int source = m_location.at(a).at(0);
        const int target = task(a);
        dist.at(a) = distance(source, target);
    }
    // for (int d : dist) { cout << d << "; "; } cout << endl; // @debug
    // Sort by distance
    std::vector<int> temp(env->num_of_agents);
    std::iota(temp.begin(), temp.end(), 0); // assign values from 0 to temp.size() - 1
    std::sort(temp.begin(), temp.end(), [&dist](int a, int b) {
        return dist.at(a) < dist.at(b);
    });
    cout << "-- First agents: a" << temp.at(0) << " a" << temp.at(1) << " a" << temp.at(2) << " a" << temp.at(3) << " a" << temp.at(4) 
        << " a" << temp.at(5) << " a" << temp.at(6) << " a" << temp.at(7) << "... a"  << temp.back() << endl; 
    return temp;
}


void PlannerGame::validate_correspondance() const {
    for (int a = 0; a < env->num_of_agents; a++) {
        const int loc1 = m_position.at(a).location;
        assert(m_location.at(a).at(0) == loc1);
        assert(get_agent(loc1, 0) == a);
    }
    for (int t = 0; t <= 1; t++) {
        for (int i = 0; i < env->cols*env->rows; i++) {
            if (get_agent(i, t) >= 0)
                assert(m_location.at(get_agent(i, t)).at(t) == i);
        }
        for (int a = 0; a < env->num_of_agents; a++)
            if (m_location.at(a).at(t) >= 0)
                assert(get_agent(m_location.at(a).at(t), t) == a);
    }
}

int PlannerGame::get_agent(int location, int time) const {
    assert(0 <= location && location < m_index_of_location.size());
    const int i_loc = m_index_of_location.at(location);
    if (i_loc < 0)
        return -1;
    return m_agent.at(i_loc).at(time);
}

void PlannerGame::set_agent(int location, int time, int agent) {
    assert(0 <= location && location < m_index_of_location.size());
    const int i_loc = m_index_of_location.at(location);
    assert(i_loc >= 0);
    m_agent.at(i_loc).at(time) = agent;
}


/// @brief Rebuild the map m_agent 
void PlannerGame::backup_agent() {
    std::fill(m_agent.begin(), m_agent.end(), std::vector<int>(2, -1));
    for (int a = 0; a < env->num_of_agents; a++) {
        for (int t = 0; t <= 1; t++) {
            set_agent(m_location.at(a).at(t), t, a);
        }
    }
}


/// @brief Count the number of steps to move from a position to a neighboring location, that is 1 + the number of rotations
/// @param location Current location
/// @param direction Current direction
/// @param next_location Next location 
/// @return 
/// @note I do not use since this (surprisingly) did not help
int PlannerGame::steps_to_move(int agent, int next_location) const {
    const int location = env->curr_states[agent].location;
    const int direction = env->curr_states[agent].orientation;
    if (location == next_location)
        return 0;
    // Get the direction needed to move
    int next_direction = 0;
    if (next_location == location + env->cols)
        next_direction = 1;
    else if (next_location == location - 1)
        next_direction = 2;
    else if (next_location == location - env->cols)
        next_direction = 3;
    if (next_direction == direction)
        return 1;
    if ((next_direction - direction + 4) % 4 == 2)
        return 3;
    return 2;
}

/// @brief Update m_time_to_move
/// @details The time to move of each agent is the maximum of its time to move to its next location and 
/// the time to move of the agent that is currently at its next location.
void PlannerGame::update_time_to_move() {
    std::fill(m_time_to_move.begin(), m_time_to_move.end(), 0);

    // This is a dumb algorithm: we compute the initial time to move and then we update until stability
    for (int a = 0; a < env->num_of_agents; a++) {
        const int next_location = m_location.at(a).at(1);
        m_time_to_move.at(a) = steps_to_move(a, next_location);
    }
    bool stability = false;
    while (!stability) {
        stability = true;
        for (int a = 0; a < env->num_of_agents; a++) {
            const int next_location = m_location.at(a).at(1);
            int prev_a = get_agent(next_location, 0);
            if (prev_a >= 0 && m_time_to_move.at(prev_a) > m_time_to_move.at(a)) {
                m_time_to_move.at(a) = m_time_to_move.at(prev_a);
                stability = false;
            }
        }
    }
}


/// @brief Tell if an agent moving towards a directin makes a forward agent increase its arrival time
/// @param agent 
/// @param direction 
/// @return 
bool PlannerGame::punctual(int agent, int direction) const {
    int next_location = move(env->curr_states.at(agent).location, direction);
    int cur_time = steps_to_move(agent, next_location);
    if (cur_time <= m_time_to_move.at(agent))
        return true;
    // this direction makes the agent slower to move 
    int prev_agent = agent;
    int cur_agent = get_agent(m_location.at(prev_agent).at(0), 1);
    while (cur_agent >= 0) {
        if (cur_time <= m_time_to_move.at(cur_agent) || cur_agent == agent)
            return true;
        if (m_forward.at(cur_agent))
            return false;
        prev_agent = cur_agent;
        cur_agent = get_agent(m_location.at(cur_agent).at(0), 1);
    }
    return true;
}



void PlannerGame::print_path(int agent) const {
    std::list<int> path;
    path.push_back(agent);
    int next_location = m_location.at(agent).at(1);
    int cur_a = get_agent(next_location, 0);
    while (cur_a >= 0 && path.size() < 50) {
        path.push_back(cur_a);
        next_location = m_location.at(cur_a).at(1);
        cur_a = get_agent(next_location, 0);
    }
    cout << "Path for a" << agent << " starting at location " << m_position.at(agent).location << ":";
    for (int cur_a : path) {
        cout << " a" << cur_a;
        if (m_location.at(cur_a).at(1) == m_location.at(cur_a).at(0) + 1)
            cout << "(E:"; 
        if (m_location.at(cur_a).at(1) == m_location.at(cur_a).at(0) + env->cols)
            cout << "(S:"; 
        if (m_location.at(cur_a).at(1) == m_location.at(cur_a).at(0) - 1)
            cout << "(W:"; 
        if (m_location.at(cur_a).at(1) == m_location.at(cur_a).at(0) - env->cols)
            cout << "(N:";
        cout << m_time_to_move.at(cur_a) << ")";
    }
    cout << endl;
}



/// @brief Turn waiting robots that are facing an obstacle
void PlannerGame::wide_view(std::vector<Action> & actions) {
    // return;
    const bool careful = false;
    for (int a = 0; a < env->num_of_agents; a++) {
        if (actions.at(a) == Action::W && !good_move(env->curr_states.at(a).location, env->curr_states.at(a).orientation)) {
            // Decide to which side we can turn
            if (!careful) {
                const int dir_cr = (env->curr_states.at(a).orientation + 1) % 4;
                if (good_move(env->curr_states.at(a).location, dir_cr)) {
                    actions.at(a) = Action::CR;
                } else {
                    actions.at(a) = Action::CCR;
                }
            } else {
                const int dir_cr  = (env->curr_states.at(a).orientation + 1) % 4;
                const int dir_ccr = (env->curr_states.at(a).orientation + 3) % 4;
                const bool turn_right = good_move(env->curr_states.at(a).location, dir_cr );
                const bool turn_left  = good_move(env->curr_states.at(a).location, dir_ccr);
                if (turn_right && !turn_left) {
                    actions.at(a) = Action::CR;
                } else if (!turn_right && turn_left) {
                    actions.at(a) = Action::CCR;
                } else if (!turn_right && !turn_left) {
                    actions.at(a) = Action::CR;
                }
            }
        }
    }
}
