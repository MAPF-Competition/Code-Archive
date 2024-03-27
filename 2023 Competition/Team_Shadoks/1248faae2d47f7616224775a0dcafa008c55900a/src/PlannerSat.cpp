#include "PlannerSat.h"

#include <random>
#include <fstream>  // ofstream


PlannerSAT::PlannerSAT() : Planner() {}


void PlannerSAT::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    if (parameters.values.count("greedy") > 0)
        m_greedy = (parameters.values.at("greedy") > 0);
    if (parameters.values.count("enqueue_bound") > 0)
        m_enqueue_bound = parameters.values.at("enqueue_bound");
    if (parameters.values.count("time_bound") > 0)
        m_time_bound = parameters.values.at("time_bound");
    if (parameters.values.count("steps") > 0)
        m_nb_steps = static_cast<int>(parameters.values.at("steps"));
    if (parameters.values.count("shots") > 0)
        m_shots = static_cast<int>(parameters.values.at("shots"));
    if (parameters.values.count("agents_bound") > 0)
        m_agents_bound = min(1.0, parameters.values.at("agents_bound"));
        
    m_location.resize(env->num_of_agents, std::vector<int>(2, -1));
    m_agent.resize(m_location_at_index.size(), std::vector<int>(2, -1));
    m_enqueued.resize(env->num_of_agents, 0);
    m_forward.resize(env->num_of_agents, false);
    m_position.resize(env->num_of_agents);
    m_time_to_move.resize(env->num_of_agents, -1);
    cout << "PlannerSAT(enqueue_bound: " << m_enqueue_bound 
    << ", time_bound: " << m_time_bound 
    << ", barriers: " << std::boolalpha << m_barriers
    << ", greedy: " << std::boolalpha << m_greedy
    << ", steps: " << m_nb_steps
    << ", shots: " << m_shots
    << ", agents: " << m_agents_bound << ") initialized" << endl;

    // Testing a different approach
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> distribution(0.0, 0.1);
    m_agent_weight.resize(m_shots);
    for (int j = 0; j < m_agent_weight.size(); j++) 
        for (int a = 0; a < env->num_of_agents; a++)
            m_agent_weight.at(j).push_back(distribution(generator));
}


/// @brief Plan the next action
/// @param actions 
void PlannerSAT::plan(vector<Action> & actions) {
    actions = std::vector<Action>(env->num_of_agents, Action::W); // set the default action of waiting
    if (halt())
        return;
    m_start_time = std::chrono::high_resolution_clock::now();

    int best_dist_sum = 1000000;
    int best_planned_agents = 0;
    int best_successful_plans = 0;
    std::vector<int> best_locations(env->num_of_agents, 0);
    for (int shot = -1; shot < m_shots - 1; shot++) {
        reset();
        // fast_plan();
        double elapsed_time = 0;
        int planned_agents = 0;
        int successful_plans = 0;
        const std::vector<int> agents = sorted_agents(shot);
        for (int i = 0; i < m_agents_bound * agents.size(); i++) {
            const int agent = agents.at(i);
            bool success = push_agent(agent);
            if (success) {
                successful_plans++;
                // if (agent == 127) { print_path(agent); }
                // print_path(agent);
            }
            planned_agents++;
            if (time_diff(m_start_time) >= m_time_bound)
                break;
        }
        const int dist_sum = evaluate_locations();
        cout << "-- shot(" << shot << "): " << time_diff(m_start_time) << ";" << planned_agents << ";" << successful_plans << ";" << dist_sum << endl;
        if (dist_sum < best_dist_sum) {
            best_dist_sum = dist_sum;
            best_successful_plans = successful_plans;
            best_planned_agents = planned_agents;
            for (int a = 0; a < env->num_of_agents; a++)
                best_locations.at(a) = m_location.at(a).at(1);
        }
    }
    // use the best locations
    for (int a = 0; a < env->num_of_agents; a++)
        best_locations.at(a) = m_location.at(a).at(1) = best_locations.at(a);

    if (m_nb_steps == 2 && time_diff(m_start_time) < m_time_bound) {
        const std::vector<std::vector<int>> location = second_plan(); // @note I am testing this
        set_moves_extended(location, actions);
    } else {
        set_moves(actions);
    }
    wide_view(actions); // @note Testing this
    update_tasks_done(actions);
    print_elapsed_time();
    cout << "-- Time: " << time_diff(m_start_time) << " ; agents planned: " << best_planned_agents << " (successful: " << best_successful_plans << ")" << endl;
    // if (!check_moves(actions)) m_halt = true; // @debug
    m_timestep++;
	return;
}


/// @brief Reset member variables
void PlannerSAT::reset() {
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
        m_time_to_move = make_time_to_move();
    }
    std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // I think that I should keep the same at each call of conflict_solver
    std::fill(m_forward.begin(), m_forward.end(), false);
}


/// @brief Sort the agents by distance to their task. We use approximate distances
/// @param shot The short we are doing. shot=-1 means that we do not randomize the distances
/// @return The list of agents
std::vector<int> PlannerSAT::sorted_agents(int shot) const {
    // Compute approximate distances
    std::vector<std::pair<int, double>> agents(env->num_of_agents);
    for (int a = 0; a < env->num_of_agents; a++) {
        const int source = m_location.at(a).at(0);
        const int target = task(a);
        double dist = distance(source, target);
        if (shot >= 0)
            dist += m_agent_weight.at(shot).at(a);
        agents.at(a) = std::make_pair(a, dist);
        if (source == target) // agents that have finished their task have zero priority
            agents.at(a).second = 10000;
    }
    // for (int d : dist) { cout << d << "; "; } cout << endl; // @debug
    // Give higher priority to agents at an endpoint if they are over a task whose agent is close
    for (int a = 0; a < env->num_of_agents; a++) {
        if (agents.at(a).second < 10) {
            const int target = task(a);
            const int a2 = get_agent(target, 0);
            if (m_idle_locations.count(target) > 0 && a2 >= 0) {
                agents.at(a2).second = 0;
            }
        }
    }
    // sort
    std::sort(agents.begin(), agents.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });
    cout << "-- First agents: a" << agents.at(0).first << ":" << agents.at(0).second 
        << " a" << agents.at(1).first << ":" << agents.at(1).second 
        << " a" << agents.at(2).first << ":" << agents.at(2).second 
        << " a" << agents.at(3).first << ":" << agents.at(3).second 
        << " a" << agents.at(4).first << ":" << agents.at(4).second 
        << " a" << agents.at(5).first << ":" << agents.at(5).second 
        << " a" << agents.at(6).first << ":" << agents.at(6).second 
        << " a" << agents.at(7).first << ":" << agents.at(7).second 
        << "... a"  << agents.back().first << ":" << agents.back().second << endl;
    std::vector<int> temp(agents.size());
    for (std::size_t a = 0; a < agents.size(); a++)
        temp.at(a) = agents.at(a).first;
    return temp;
}


/// @brief Find a new location for agent such that
/// 1) it is closer to its task
/// 2) it can move withint a number of steps
/// 3) it does not delay previously planned agents
/// @param agent Agent to plan
/// @param steps Objective number of steps
/// @return True if we succeded
bool PlannerSAT::conflict_solver(int agent, int steps) {
    const bool reset_count = false; // reset the conflict count in each conflict solver
    // Make a copy of the data in case this solver fails
    std::vector<std::vector<int>> copy_location(m_location.begin(), m_location.end()); // deep copy
    std::vector<int> copy_enqueued(m_enqueued.begin(), m_enqueued.end());
    if (reset_count)
        std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // Reset m_enqueued. I removed this because it seems slower.
    assert(m_queue.empty());
    // Find a valid assignment. We stop when we reach one or when an agent is enqueued 100 times
    m_queue.push(agent);
    m_enqueued.at(agent) += 1;
    const int old_time = m_time_to_move.at(agent);
    m_forward.at(agent) = true;
    m_time_to_move.at(agent) = steps;
    bool invalid = false; // We do not find a solution
    std::vector<int> collisions;
    int count = 0;
    // print_path(agent);
    do {
        const int a = m_queue.front(); m_queue.pop();
        // cout << "a" << a << " (count:" << m_enqueued.at(a) << ") <- queue" << endl; // @debug
        const int cur_loc = best_move(a, steps, collisions);
        if (cur_loc < 0) {
            invalid = true;
            break;
        }
        // cout << "a" << a << ": " << m_location.at(a).at(0) << " -> " << m_location.at(a).at(1) << endl; // @debug
        for (int a2 : collisions) {
            // cout << "queue <- a" << a2 << " (count:" << m_enqueued.at(a2)+1 << ")" << endl; // @debug
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
    if (!invalid) { // check that we do not delay a forward agent
        const std::vector<int> cur_time_to_move = make_time_to_move();
        for (int a = 0; a < env->num_of_agents && !invalid; a++) {
            if (m_forward.at(a) && cur_time_to_move.at(a) > m_time_to_move.at(a)) {
                invalid = true;
            }
        }
        if (!invalid) {
            m_time_to_move = cur_time_to_move;
        }
    }
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









/// @brief Compute, for each agent, the time to move to the next location
/// @details The time to move of each agent is the maximum of its time to move to its next location and 
/// the time to move of the agent that is currently at its next location.
/// @note I tried to make a faster algorithm that tries to compute it by locally updating m_time_to_move around the replanned agent, 
/// but it did not work because when we plan an agent we change many agents and I am not sure which ones.
std::vector<int> PlannerSAT::make_time_to_move() const {
    std::vector<int> time_to_move(env->num_of_agents, 0);
    std::vector<bool> visited(env->num_of_agents, false); // visited[a] = true if its true value has already been computed
    // Compute time_to_move for paths. 
    // We find agents moving to a free location and go up that path
    for (int a = 0; a < env->num_of_agents; a++) {
        const int next_location = m_location.at(a).at(1);
        if (get_agent(next_location, 0) < 0) {
            time_to_move.at(a) = steps_to_move(a, next_location);
            visited.at(a) = true;
            int prev_agent = a;
            int cur_agent = get_agent(m_location.at(prev_agent).at(0), 1);
            while (cur_agent >= 0 && time_to_move.at(cur_agent) == 0) {
                const int own_time = steps_to_move(cur_agent, m_location.at(cur_agent).at(1));
                const int wait_time = time_to_move.at(prev_agent);
                time_to_move.at(cur_agent) = std::max(own_time, wait_time);
                visited.at(cur_agent) = true;
                prev_agent = cur_agent;
                cur_agent = get_agent(m_location.at(cur_agent).at(0), 1);
            }
        }
    }
    // Now do the loops
    for (int a = 0; a < env->num_of_agents; a++) {
        if (!visited.at(a)) {
            std::list<int> loop;
            int loop_max = steps_to_move(a, m_location.at(a).at(1));
            visited.at(a) = true;
            loop.push_back(a);
            int prev_agent = a;
            int cur_agent = get_agent(m_location.at(prev_agent).at(0), 1);
            while (cur_agent >= 0 && !visited.at(cur_agent)) {
                const int own_time = steps_to_move(cur_agent, m_location.at(cur_agent).at(1));
                loop_max = std::max(loop_max, own_time);
                visited.at(cur_agent) = true;
                loop.push_back(cur_agent);
                prev_agent = cur_agent;
                cur_agent = get_agent(m_location.at(cur_agent).at(0), 1);
            }
            for (int cur_a : loop) {
                time_to_move.at(cur_a) = loop_max;
            }
        }
    }
    return time_to_move;
    // // This is a dumb algorithm: we compute the initial time to move and then we update until stability
    // for (int a = 0; a < env->num_of_agents; a++) {
    //     const int next_location = m_location.at(a).at(1);
    //     m_time_to_move.at(a) = steps_to_move(a, next_location);
    // }
    // bool stability = false;
    // while (!stability) {
    //     stability = true;
    //     for (int a = 0; a < env->num_of_agents; a++) {
    //         const int next_location = m_location.at(a).at(1);
    //         int prev_a = get_agent(next_location, 0);
    //         if (prev_a >= 0 && m_time_to_move.at(prev_a) > m_time_to_move.at(a)) {
    //             m_time_to_move.at(a) = m_time_to_move.at(prev_a);
    //             stability = false;
    //         }
    //     }
    // }
}


/// @brief Get the next location after an action
/// @param location Current location in the map
/// @param direction A direction to move: -1 (do not move), 0 (east), 1 (south)...
/// @return The next location. It returns -1 if it exists the map boundaries
int PlannerSAT::move_loc(int location, int direction) const {
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


/// @brief Get the adjacent location along a direction, without any validation
/// @param location Current location in the map
/// @param direction A direction to move: -1 (do not move), 0 (east), 1 (south)...
/// @return The next location. It returns -1 if it exists the map boundaries
int PlannerSAT::unsafe_move(int location, int direction) const {
    switch (direction) {
        case 0: return location + 1;
        case 1: return location + env->cols;
        case 2: return location - 1;
        case 3: return location - env->cols;
    }
    return location;
}

/// @brief Check if an agent can move from a location to a direction
/// @param location Location in the map
/// @param direction Direction of the move: -1 (do not move), 0 (east), 1 (south)...
/// @return True if the agent can make that move: it remains in the map, it does not go into an obstacle and it respects the barriers
bool PlannerSAT::good_move(int location, int direction) const {
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
    const int next_loc = unsafe_move(location, direction);
    return !obstacle(next_loc);
}


/// @brief Compute the next move for each agent given its next location.
/// @param plan 
/// @details First, we assign a rotation to the agents that will move to a location that they are 
/// not facing, and put the rest of the robots in a list. Then, we identify the robots that will not
/// be able to advance because their next locations is occupied by a rotation agent. We back propagate
/// from these agents to identify all the agents that cannot move. And thus we move the rest of the agents.
void PlannerSAT::set_moves(std::vector<Action> & plan) const {
    // validate_correspondance(); // @debug
    // Find agents that must rotate and agents that are ready to move
    std::vector<int> ready_agents; // agents that face their next location
    for (int a = 0; a < env->num_of_agents; a++) {
        const int cur_dir = direction(m_location.at(a).at(0), m_location.at(a).at(1));
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


/// @brief Get the next location of an agent. We compute the direction that makes this agent closer to its task, or just any direction,
/// but it has to satisfy a bound on the number of steps it takes to move to the location.
/// @param agent The (index of the) agent
/// @param steps The maximum number of steps it can take to move to the next locations
/// @return A list of directions: -1 (do not move), 0 (east), 1 (south)...
std::vector<int> PlannerSAT::next_locations(int agent, int steps) const {
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
            const int next_location = unsafe_move(cur_loc, direction);
            if (steps_to_move(agent, next_location) <= steps)
                locations.push_back(next_location);
        }
    } else {
        for (int direction = -1; direction <= 3; direction++) {
            const int next_location = unsafe_move(cur_loc, direction);
            if (good_move(cur_loc, direction) && steps_to_move(agent, next_location) <= steps) 
                locations.push_back(next_location);
        }
    }
    return locations;
}


void PlannerSAT::validate_correspondance() const {
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

int PlannerSAT::get_agent(int location, int time) const {
    const int i_loc = m_index_of_location.at(location);
    if (i_loc < 0)
        return -1;
    return m_agent.at(i_loc).at(time);
}

void PlannerSAT::set_agent(int location, int time, int agent) {
    const int i_loc = m_index_of_location.at(location);
    assert(i_loc >= 0);
    m_agent.at(i_loc).at(time) = agent;
}


/// @brief Rebuild the map m_agent 
void PlannerSAT::backup_agent() {
    std::fill(m_agent.begin(), m_agent.end(), std::vector<int>(2, -1));
    for (int a = 0; a < env->num_of_agents; a++) {
        for (int t = 0; t <= 1; t++) {
            set_agent(m_location.at(a).at(t), t, a);
        }
    }
}


/// @brief Count the number of steps to move an agent to an adjacent location
/// @param agent Current location
/// @param direction Current direction
/// @param next_location Next location 
/// @return The number of rotations plus 1
int PlannerSAT::steps_to_move(int agent, int next_location) const {
    // const int location = env->curr_states[agent].location;
    // const int direction = env->curr_states[agent].orientation;
    const Position & pos = m_position.at(agent);
    if (pos.location == next_location)
        return 0;
    // Get the direction needed to move
    int next_direction = 0;
    if (next_location == pos.location + env->cols)
        next_direction = 1;
    else if (next_location == pos.location - 1)
        next_direction = 2;
    else if (next_location == pos.location - env->cols)
        next_direction = 3;
    // Count the number of necessary rotations
    if (next_direction == pos.direction)
        return 1;
    if ((next_direction - pos.direction + 4) % 4 == 2)
        return 3;
    return 2;
}



void PlannerSAT::print_path(int agent) const {
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
        // if (cur_a >= 0 && m_forward.at(cur_a))
        //     cout << "*";
        if (m_location.at(cur_a).at(1) == m_location.at(cur_a).at(0))
            cout << "(-";
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


/// @brief Rapidly plan the agents that can move to a closer location without collisions
void PlannerSAT::fast_plan() {
    if (m_map != InstanceMap::Game)
        return;

    int count = 0;
    for (int a = 0; a < env->num_of_agents; a++) {
        const int cur_loc = m_location.at(a).at(0);
        const int end = task(a);
        if (cur_loc == end)
            continue;
        std::list<int> directions;
        if (good_move(cur_loc, 0) && closer_east(cur_loc, end))
            directions.push_back(0);
        if (good_move(cur_loc, 1) && closer_south(cur_loc, end))
            directions.push_back(1);
        if (good_move(cur_loc, 2) && !closer_east(cur_loc - 1, end))
            directions.push_back(2);
        if (good_move(cur_loc, 3) && !closer_south(cur_loc - env->cols, end))
            directions.push_back(3);
        for (int dir : directions) {
            const int next_loc = unsafe_move(cur_loc, dir);
            const int a1 = get_agent(next_loc, 0);
            const int a2 = get_agent(next_loc, 1);
            if ((a1 < 0 || a1 == a) && (a2 < 0 || a2 == a)) {
                int old_loc = m_location.at(a).at(1);
                m_location.at(a).at(1) = next_loc;
                if (old_loc >= 0) {
                    set_agent(old_loc, 1, -1);
                }
                set_agent(next_loc, 1, a);
                count++;
                break;
            }
        }      
    }
    std::cout << "-- fast_plan found directions for " << count << " agents" << std::endl;
}

/// @brief Try to plan a given agent to make one move towards its task in as few steps as possible
/// without delaying already planned agents and in as fe
/// @param agent Index of the agent to plan
/// @return True if we can push it towards its task
bool PlannerSAT::push_agent(int agent) {
    if (!m_greedy) {
        return conflict_solver(agent, 3);
    }
    const int start = min_moves(agent);
    for (int steps = start; steps <= 3; steps++) {
        if (conflict_solver(agent, steps)) {
            return true;
        }
    }
    return false;
}



/// @brief Find the move for an agent that minimizes the conflicts.
/// @param agent Index of the agent to plan
/// @param collisions Vector of (the indices of) the agents that have a collision with the planned move
/// @return Direction of the move, or -1 if there is none.
/// @note I tried to prioritize locations that are in front of the agent, but that did not work well
/// @details We find the best move (minimizing the conflicts) such that:
/// 1) the agent advances towards its task
/// 2) it will reach its next location in `steps` steps
/// 3) it does not delay previously planned agents
/// Note, however, that this move can create a collision, and solving that collision can delay a forward agent.
/// Hence, we must check that we do not delay forward agents in PlannerSAT::conflict_solver()
int PlannerSAT::best_move(int agent, int steps, std::vector<int> & collisions) {
    const int steps_bound = available_time(agent);
    int best_location = -1;
    double best_value = -1;
    collisions.clear();
    const std::vector<int> locations = next_locations(agent, steps_bound);
    if (locations.empty()) {
        // This can happen if we need three steps to move to the other position but we have
        // a path of robots coming from that same direction in three steps
        // assert(steps < 3); // this can happen if we need three steps to move to the
        return -1;
    }
    for (int loc : locations) {
        const std::vector<int> cur_collisions = get_collisions(agent, loc, steps_bound);
        double value = 0.1 * steps_to_move(agent, loc); // @note I think that I can simply set value to 0
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

/// @brief Count the number of rotations to put an agent in a given direction
/// @param agent 
/// @param direction 
/// @return 
int PlannerSAT::rotations(int agent, int direction) const {
    const int cur_direction = env->curr_states[agent].orientation;
    if (cur_direction == direction)
        return 0;
    if ((cur_direction - direction + 4) % 4 == 2)
        return 2;
    return 1;
}

/// @brief Get the collisions with other agents when the agent moves to a next_location within a number of steps
/// @param agent Agent that is moving
/// @param next_location Next location of the agent
/// @param steps Number of steps it needs to move
/// @return 
/// @note There are at most 2 collisions. Compared to get_collisions() this function sees a collision if we want to move
/// an agent to a location withing a number of steps and that there is an agent at that location that, even if it moves,
/// it is not fast enough
std::vector<int> PlannerSAT::get_collisions(int agent, int next_location, int steps) const {
    std::vector<int> collisions;
    const int cur_location = m_location.at(agent).at(0);
    // there is a collision if there is another agent at the next location in the next step
    const int a1 = get_agent(next_location, 1);
    if (a1 >= 0 && a1 != agent) {
        collisions.push_back(a1);
    }
    // edge collision
    const int a2 = get_agent(next_location, 0);
    if (a2 >= 0 && a2 != a1 && m_location.at(a2).at(1) >= 0) {
        if (m_location.at(a2).at(1) == cur_location) {
            collisions.push_back(a2);
        } else if (m_greedy && m_time_to_move.at(a2) > steps) {
            collisions.push_back(a2);
        }
    }
    return collisions;
}


/// @brief Tell if an agent moving towards a direction makes a forward agent after him increase its arrival time
/// @param agent 
/// @param direction 
/// @return 
/// @note This does not imply that this move will be good. Indeed, this move can produce a conflict that eventually
/// delays a forward agent.
bool PlannerSAT::punctual(int agent, int direction) const {
    const int next_location = unsafe_move(m_position.at(agent).location, direction);
    const int cur_time = steps_to_move(agent, next_location); // time to move to the next location
    if (m_greedy) {
        // In the greedy version, if this agent is *forward* and slower than the bound fixed in PlannerSAT::poush_agent(), 
        /// then we know that the agent is not punctual
        if (m_forward.at(agent) && cur_time > m_time_to_move.at(agent))
            return false;
    } else {
        // In the non-greedy version, if the first move is faster that its previous time, we know that
        // it does not delay other agents
        if (cur_time <= m_time_to_move.at(agent))
            return true;
    }
    int prev_agent = agent;
    int cur_agent = get_agent(m_location.at(prev_agent).at(0), 1);
    while (cur_agent >= 0) {
        // It is ok if this agent moves at least as fast as the robot coming to its position
        // We also return true if this is a loop
        if (cur_time <= m_time_to_move.at(cur_agent) || cur_agent == agent)
            return true;
        // It is not ok if the agent delays a forward agent
        if (m_forward.at(cur_agent))
            return false;
        prev_agent = cur_agent;
        cur_agent = get_agent(m_location.at(cur_agent).at(0), 1);
    }
    return true;
}


/// @brief Available time for an agent to move, given that it cannot delay the forward agents
/// @param agent 
/// @return Maximum number of steps it can take to move
int PlannerSAT::available_time(int agent) const {
    int cur_agent = agent;
    do {
        if (m_forward.at(cur_agent))
            return m_time_to_move.at(cur_agent);
        cur_agent = get_agent(m_location.at(cur_agent).at(0), 1);
    } while (cur_agent >= 0);
    return 3;
}


/// @brief Plan the second next locations
std::vector<std::vector<int>> PlannerSAT::second_plan() {
    std::vector<std::vector<int>> locations(env->num_of_agents, std::vector<int>(3, -1));  // vector with the locations now, then and later
    for (int a = 0; a < env->num_of_agents; a++) {
        locations.at(a).at(0) = m_location.at(a).at(0);
        locations.at(a).at(1) = m_location.at(a).at(1);
    }
    // Set the moves and apply them to the locations
    std::vector<Action> first_actions(env->num_of_agents, Action::W); // set the default action of waiting
    set_moves(first_actions);
    // Update the variables
    std::fill(m_agent.begin(), m_agent.end(), std::vector<int>(2, -1));
    for (int a = 0; a < env->num_of_agents; a++) {
        m_position.at(a) = move(m_position.at(a), first_actions.at(a));
        m_location.at(a).at(0) = m_position.at(a).location;
        for (int t = 0; t <= 1; t++) {
            const int loc = m_location.at(a).at(t);
            const int i_loc = m_index_of_location.at(loc);
            m_agent.at(i_loc).at(t) = a;
        }
    }
    m_time_to_move = make_time_to_move();
    std::fill(m_enqueued.begin(), m_enqueued.end(), 0); // I think that I should keep the same at each call of conflict_solver
    std::fill(m_forward.begin(), m_forward.end(), false);
    // Plan the next locations using the new data
    const std::vector<int> agents = sorted_agents(-1);
    for (int i = 0; i < agents.size(); i++) {
        conflict_solver(agents.at(i), 3);
        if (time_diff(m_start_time) >= m_time_bound)
            break;
    }
    for (int a = 0; a < env->num_of_agents; a++) {
        locations.at(a).at(2) = m_location.at(a).at(1);
    }
    return locations;
}


/// @brief Set the actions given the next two locations
/// @param locations 
/// @param plan 
void PlannerSAT::set_moves_extended(const std::vector<std::vector<int>> & location, std::vector<Action> & plan) const {
    std::vector<int> ready_agents; // agents that face their next location
    // The agents that need to turn to move to the next location, turn
    for (int a = 0; a < env->num_of_agents; a++) {
        bool move_now = true; // true if the robot should move now, that is if location[a][0] != location[a][1]
        int cur_dir = direction(location.at(a).at(0), location.at(a).at(1));
        if (cur_dir < 0) {
            cur_dir = direction(location.at(a).at(1), location.at(a).at(2));
            move_now = false;
        }
        if (cur_dir >= 0) {
            const int delta = (env->curr_states.at(a).orientation - cur_dir + 4) % 4; // @note use env instead of m_position
            if (delta == 0 && move_now) { // send to move if it faces the next location and it has to move now
                ready_agents.push_back(a);
            // } else if (delta == 2 && !move_now) {
            //     plan.at(a) = Action::W; // @todo I AM TESTING THIS
            } else if (delta == 1 || delta == 2) {
                plan.at(a) = Action::CCR;
            } else if (delta == 3) {
                plan.at(a) = Action::CR;
            }
        }
    }
    // Find robots that can move
    std::vector<int> prev_agent(env->num_of_agents, -1); // prev_agent[a] = agent that is planned to move to the location of a
    std::list<int> waiting_robots; // agents facing its next location that cannot advance (because there is an agent turning in front of them)
    for (int a : ready_agents) {
        const int next_location = location.at(a).at(1);
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


/// @brief Turn waiting robots that are facing an obstacle
void PlannerSAT::wide_view(std::vector<Action> & actions) {
    // return;
    const bool careful = true;
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
                const int dir_cr = (env->curr_states.at(a).orientation + 1) % 4;
                const int dir_ccr = (env->curr_states.at(a).orientation + 3) % 4;
                const bool turn_right = good_move(env->curr_states.at(a).location, dir_cr);
                const bool turn_left = good_move(env->curr_states.at(a).location, dir_ccr);
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


/// @brief How many moves a robot needs to advance towards its location
/// @param agent 
/// @return The minimum number of moves that an agent needs to move to a neighboring
/// location that is closer to its task
int PlannerSAT::min_moves(int agent) const {
    // return 1;

    int moves = 3;
    const int cur_loc = m_location.at(agent).at(0);
    const int end = task(agent);
    std::vector<int> locations;
    if (cur_loc == end) { // @note this can really happen
        return 1;
    }
    std::vector<int> directions;
    if (good_move(cur_loc, 0) && closer_east(cur_loc, end))                 directions.push_back(0);
    if (good_move(cur_loc, 1) && closer_south(cur_loc, end))                directions.push_back(1);
    if (good_move(cur_loc, 2) && !closer_east(cur_loc - 1,          end))   directions.push_back(2);
    if (good_move(cur_loc, 3) && !closer_south(cur_loc - env->cols, end))   directions.push_back(3);
    assert(!directions.empty());
    const Position & pos = m_position.at(agent);
    for (int direction : directions) {
        int cur_moves = 2;
        if (direction == pos.direction)                 cur_moves = 1;
        if ((direction - pos.direction + 4) % 4 == 2)   cur_moves = 3;
        moves = min(moves, cur_moves);
    }
    return moves;
}


int PlannerSAT::evaluate_locations() const {
    int distance_sum = 0;
    for (int a = 0; a < env->num_of_agents; a++) {
        const int source = m_location.at(a).at(1);
        const int target = task(a);
        distance_sum += distance(source, target);
    }
    return distance_sum;
}
