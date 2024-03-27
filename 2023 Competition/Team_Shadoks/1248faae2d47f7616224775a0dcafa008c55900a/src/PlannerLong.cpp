#include <PlannerLong.h>
#include "nlohmann/json.hpp"

#include <random>
#include <fstream>  // ofstream

PlannerLong::PlannerLong() : Planner() {}


void PlannerLong::init(SharedEnvironment* _env, const Parameters & parameters) {
    Planner::init(_env, parameters);
    m_congestion = parameters.values.at("congestion");
    m_exp = parameters.values.at("exp");
    m_sub = parameters.values.at("sub");
    m_conf1 = parameters.values.at("conf1");
    m_conf2 = parameters.values.at("conf2");
    m_confadd = parameters.values.at("confadd");
    m_confexp = parameters.values.at("confexp");
    m_probexp = parameters.values.at("probexp");
    m_slack = parameters.values.at("slack");
    m_confdistexp = parameters.values.at("confdistexp");

    for(auto &[k,v] : parameters.values)
        cout << k << " = " << v << endl;

    if(m_num_of_agents > 200) {
        cout << "MAX 200 agents in this version (check bitset in Planner.h)" << endl;
        exit(1);
    }

    cout << "PlannerLong initialized" << endl;
}

void print_queue(std::queue<int> q){
  while (!q.empty())
  {
    std::cout << q.front() << " ";
    q.pop();
  }
  std::cout << std::endl;
}

/**
 * @brief PlannerLong::plan We plan a short path for each robot using A* search and
 * take the first step for this timestep
 * @param time_limit
 * @param actions
 */
void PlannerLong::plan(vector<Action> & actions, double timeLeft) {
    std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();

    actions = std::vector<Action>(env->curr_states.size(), Action::W); // set the default action of waiting
    if (halt())
        return;

    int maxSteps = 500;

    if(m_timestep == maxSteps) {
        actions = std::vector<Action>(env->curr_states.size(), Action::FW); // make it crash
        // return;
    }
    
    std::vector<int> actualDistance(m_num_of_agents, 0);
    std::vector<bool> doable(m_num_of_agents, 0);
    int num_undoable = 0;
    for(int a = 0; a < m_num_of_agents; a++) {
        // cout << a << " " << std::flush;
        if (env->goal_locations.at(a).empty())
            continue;
        const Position start(env->curr_states.at(a), m_timestep);
        auto path = plan_path_alone(start, task(a));
        actualDistance.at(a) = path.size() - 1;
        doable.at(a) = (m_timestep + actualDistance.at(a) <= maxSteps);
        num_undoable += !doable.at(a);
        // cout << actualDistance.at(a) << " " << m_paths.at(a).size() << endl;
    }

    // cout << "Undoable: " << num_undoable << endl;
    update_paths();
    usage = std::vector<int>(env->cols*env->rows, 0);
    for(int a = 0; a < m_num_of_agents; a++) {
        for(const Position &pos : m_paths.at(a)) {
            usage.at(pos.location)++;
        }
    }

    // Put the unplanned agents into the queue
    std::queue<int> queue;
    std::vector<int> enqueued(m_num_of_agents, 1); // enqueued[a] counts the number of times a has been enqueued
    std::vector<int> is_queued(m_num_of_agents, 0); // 0 for not currently in queue, 1 for currently in queue
    std::vector<int> valid_path(m_num_of_agents, 0); // 0 for not valid or empty, 1 for valid reaching goal, 3 for valid but not reaching target
    std::vector<std::vector<Position>> best_paths = m_paths;
    // fill_queue(queue, enqueued, is_queued);

    int num_reach = 0, num_valid = 0;

    for(int a = 0; a < m_num_of_agents; a++) {
        if (env->goal_locations.at(a).empty()) {
            cout << "Warning: agent a" << a << " has no task" << endl;
            continue;
        }
        // if(!doable.at(a)) {
        //     valid_path.at(a) = 1;
        //     continue;
        // }

        if(m_paths.at(a).back().location == task(a)) {
            valid_path.at(a) = 1;
            num_reach++;
        }
        else if(m_paths.at(a).size() > 1) {
            valid_path.at(a) = 3;
            num_valid++;
        }

        if(m_paths.at(a).back().location != task(a)
           || m_paths.at(a).size() > actualDistance.at(a) + m_slack) {
            queue.push(a);
            is_queued.at(a) = true;
        }
    }
    sort_queue(queue);

    int phase = 0;
    if(queue.empty()) {
        phase++;
        std::cout << '|';
    }

    double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();
    cout << elapsed_time << ": undoable=" << num_undoable << " queue=" << queue.size() << " reached=" << num_reach << " valid=" << num_valid + num_reach << endl;

    int best_valid = 0, best_len = INT_MAX, best_reach = 0;
    std::unordered_map<int,std::vector<Position>> saved_paths;

    int max_enqueued = 1;
    // Process the queue
    while (elapsed_time < timeLeft - .025) {
        // cout << queue.size() << " " << std::flush;
        int a;
        if(queue.size()) {
            a = queue.front(); queue.pop();
            is_queued.at(a) = false;
        }
        else {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::vector<double> prob;
            for(int a = 0; a < m_num_of_agents; a++) {
                if(doable.at(a) && !(m_paths.at(a).back().location == task(a) && m_paths.at(a).size() <= actualDistance.at(a) + 1))
                    prob.push_back(pow(1.0/m_paths.at(a).size(),m_probexp));
                else // If already optimal or not doable, make prob=0
                    prob.push_back(0);
            }
            std::discrete_distribution<int> d(prob.begin(), prob.end());
            a = d(gen);
        }
        // enqueued.at(a)++; // Try to update enqueued on dequeuing instead
        if(phase >= 1 && !saved_paths.contains(a))
            saved_paths[a] = m_paths.at(a);

        if(enqueued.at(a) > max_enqueued) {
            // cout << max_enqueued << " ";
            max_enqueued = enqueued.at(a);
        }

        const Position cur_pos(env->curr_states.at(a), m_timestep);

        double exp = m_exp, sub = m_sub;
        int min_save = 2;
        if(m_num_of_agents < 150 && m_timestep == 0) exp += .2;
        if(m_num_of_agents > 150 && m_timestep <= 3) exp += .4;

        int path_bound = pow(std::max(enqueued.at(a)-sub, 1.0), exp) - 1 + actualDistance.at(a);
        if(!doable.at(a)) {
            path_bound = 16;
        }
        if(!doable.at(a) || enqueued.at(a) < 8) {
            if(path_bound > maxSteps - m_timestep)
                path_bound = maxSteps - m_timestep;
        }

        std::vector<int> conflicts;
        unregister_path(a);
        if(doable.at(a)) {
            conflicts = plan_path_with_conflicts(a, cur_pos, task(a),path_bound, enqueued, true, actualDistance);
            // cout << "doable a" << a << " has " << conflicts.size() << " conflicts and length " << m_paths.at(a).size() << endl;
        }
        else {
            conflicts = plan_path_with_conflicts(a, cur_pos, task(a),path_bound, enqueued, false, actualDistance);
            // cout << "undoable a" << a << " has " << conflicts.size() << " conflicts and length " << m_paths.at(a).size() << endl;
        }

        register_path(a);
        if(valid_path.at(a) == 3) num_valid--;
        if(valid_path.at(a) == 1) num_reach--;

        if(m_paths.at(a).size() > maxSteps + 1 - m_timestep)
            m_paths.at(a).resize(maxSteps + 1 - m_timestep);

        if(m_paths.at(a).back().location != task(a)) {
            valid_path.at(a) = 3;
            num_valid++;
        }
        else {
            valid_path.at(a) = 1;
            num_reach++;
        }

        if(m_paths.at(a).back().location != task(a) && doable.at(a)) {
            if(!is_queued.at(a)) {
                queue.push(a);
                enqueued.at(a)++;
                is_queued.at(a) = true;
            }
            // cout << "Q"  << conflicts.size();
        }
        else {
            // cout << "G" << conflicts.size();
        }

        for(int ca : conflicts) {
            // cout << " -- a" << a << " unplans a" << ca << "(" << queue.size()+1 << " in queue)" << endl; // @debug
            const Position first_pos(env->curr_states.at(ca), m_timestep);
            // m_paths.at(ca).clear();
            // m_paths.at(ca).push_back(first_pos);
            // register_path(ca);
            if(valid_path.at(ca) == 3) num_valid--;
            if(valid_path.at(ca) == 1) num_reach--;

            valid_path.at(ca) = 0;
            // append it to the queue
            if(!is_queued.at(ca)) {
                queue.push(ca);
                enqueued.at(ca)++;
                is_queued.at(ca) = true;
            }
        }

        if(phase == 0 && std::make_pair(num_reach, num_valid) >= std::make_pair(best_reach, best_valid)) {
            int len = 0;

            for(int a = 0; a < m_num_of_agents; a++) {
                if(valid_path.at(a) == 1 && doable.at(a)) { // Only count paths reaching goal
                    len += m_paths.at(a).size();
                }
            }

            // cout << "num_reach=" << num_reach << " num_valid=" << num_valid
            //      << "len=" << len << endl;

            if (phase == 0 &&
                std::make_tuple(num_reach,num_valid,-len) >
                std::make_tuple(best_reach,best_valid,-best_len)) {
                best_len = len;
                best_valid = num_valid;
                best_reach = num_reach;
                for(int a = 0; a < m_num_of_agents; a++) {
                    best_paths.at(a) = m_paths.at(a);
                    if(!valid_path.at(a)) {
                        if(best_paths.at(a).size() > 1) {
                            m_paths.at(a).resize(2);
                        }
                        else {
                            Position cur_pos(env->curr_states.at(a), m_timestep);
                            best_paths.at(a) = {cur_pos};
                        }
                    }
                }
                cout << '+';
                // cout << num_valid << "," << len;
            }
            // else
                // cout << '-';
        }

         // Start a new phase
        if(queue.empty()) {
            if(phase==0)
                cout << '|';
            else { // Not the first new phase, so saved exists
                int sum_saved = 0, sum_now = 0;
                for(auto &[a, path] : saved_paths) {
                    enqueued.at(a) = 1;
                    if(doable.at(a)) {
                        sum_saved += path.size();
                        sum_now += m_paths.at(a).size();
                    }
                }
                if(sum_now > sum_saved) { // Solution got worse, undo
                    for(auto &[a, path] : saved_paths) {
                        m_paths.at(a) = path;
                    }
                    cout << '-';
                }
                else if(sum_now < sum_saved) { // Solution got better, update best
                    best_paths = m_paths;
                    cout << '+';
                }
                else cout << '.'; // Solution did not change quality, ignore
            }
            saved_paths.clear();
            phase++;
        }

        elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() -
        start_time).count();
        // cout << elapsed_time << endl;
    } // End while
    cout << endl << "max_enqueued: " << max_enqueued << " queue: " << queue.size() << endl;
    // print_queue(queue);

    int num_opt = 0;
    for(int a = 0; a < m_num_of_agents; a++) {
        if(m_paths.at(a).back().location == task(a) && m_paths.at(a).size() <= actualDistance.at(a) + 1)
            num_opt++;
    }

    cout << "reached=" << best_reach << " valid=" << best_reach+best_valid << " optimal=" << num_opt << endl;

    while(queue.size()) {
        int a = queue.front(); queue.pop();
        if(!valid_path.at(a)) {
            const Position cur_pos(env->curr_states.at(a), m_timestep);
            if(m_paths.at(a).size() > 1) {
                m_paths.at(a).resize(2);
            }
            else {
                m_paths.at(a).clear();
                m_paths.at(a).push_back(cur_pos);
            }
            // register_path(a);
        }
    }

    if(best_paths.at(0).size()) { // There is a better solution
        m_paths = best_paths;
    }

    get_actions(actions);
    avoid_collisions(actions); // Does not need m_passing* m_ending*
    update_tasks_done(actions);
    print_elapsed_time();
    m_timestep++;

    cout << "A* calls: " << ppwc_calls << endl << endl;

    assert(check_moves(actions));
    return;
}

// std::vector<int> PlannerLong::plan_undoable_path(int agent, const Position &start) {
//     if (env->map.at(start.location) == 1) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
//     // assert(m_paths.at(agent).front() == start);
//     if (!(m_paths.at(agent).front() == start)) {
//         cout << "a" << agent << " t:" << m_timestep << endl;
//         m_paths.at(agent).front().print();
//         start.print();
//     }
//     auto neighborhood = neighbors_with_conflicts(agent, Position(start.location, start.direction, m_timestep));
//     std::vector<int> bestConf = neighborhood.at(0).second;
//     Position bestPos;
//     for (const auto &[neighbor,conflicts] : neighborhood) {
//         if(conflicts.size() <= bestConf.size()) {
//             bestPos = neighbor;
//             bestConf = conflicts;
//         }
//     }
//     m_paths.at(agent) = std::vector<Position>{start,bestPos};
//     return bestConf;
// }

/// @brief Update the planned paths.
/// @details At timestep 0, we set the initial position instead. For the next timesteps,
/// we remove the first position if it was used
void PlannerLong::update_paths() {
    // int reset_count = 0;
    // for (int a = 0; a < m_num_of_agents; a++) {
    //     const Position curr_pos(env->curr_states.at(a), env->curr_timestep);
    //     if(m_paths.at(a).size() <= 1) {
    //         m_paths.at(a).clear();
    //         m_paths.at(a).push_back(curr_pos);
    //         reset_count++;
    //     }
    //     else if(curr_pos == m_paths.at(a).at(1)) { // Moved fine
    //         m_paths.at(a).erase(m_paths.at(a).begin());
    //     }
    //     else {
    //         m_paths.at(a).clear();
    //         m_paths.at(a).push_back(curr_pos);
    //         reset_count++;
    //     }
    // }
    //
    // if(reset_count)
    //     cout << "Reset " << reset_count << " agents" << endl;

    bool has_moved = true;
    if (m_timestep == 0) {
        // Add the initial position for all
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
    m_passing.clear();
    m_passing.resize(env->cols*env->rows);
    m_passing_hash.clear();
    m_ending_hash.clear();

    for (int a = 0; a < m_num_of_agents; a++)
        register_path(a);
}


void PlannerLong::fill_queue(std::queue<int> & queue, std::vector<int> &enqueued, std::vector<int> &is_queued) const {
    for (int a = 0; a < m_num_of_agents; a++) {
        queue.push(a);
        enqueued.at(a) = 1;
        is_queued.at(a) = 1;
    }
    sort_queue(queue);
}

/**
 * @brief PlannerLong::plan_path Get the shortest path for agent without collisions with the already planned agents
 * @param agent index of the current agent
 * @param start start position
 * @param end end location
 * @return A (possibly empty) list of conflicting agents that prevent to find a path
 */
std::vector<int> PlannerLong::plan_path_with_conflicts(int agent, const Position &start, int end, int path_bound, std::vector<int> &enqueued, bool doable, std::vector<int> &actualDistance) {
    ppwc_calls++;
    if (env->map.at(start.location) == 1) cout << "Error: agent a" << agent << " is at location " << start.location << " which is an obstacle" << endl;
    if (env->map.at(end) == 1) cout << "Error: agent a" << agent << " is assigned to location " << end << " which is an obstacle" << endl;
    // assert(m_paths.at(agent).front() == start);
    if (!(m_paths.at(agent).front() == start)) {
        cout << "a" << agent << " t:" << m_timestep << endl;
        m_paths.at(agent).front().print();
        start.print();
    }

    std::vector<Position> path;
    std::priority_queue<std::tuple<float,float,float,AstarNC*>, vector<std::tuple<float,float,float,AstarNC*>>> open_list;
    std::unordered_map<long long int, AstarNC*> all_nodes; // nodes indexed by location
    std::unordered_set<long long int> close_list; // set of done nodes
    AstarNC* bestFound = nullptr;
    AstarNC* bestAchievingFound = nullptr;
    AstarNC* s = new AstarNC(start.location, start.direction, start.time, 0, 0, nullptr); // start node
    open_list.push(std::make_tuple(0,-s->f, -s->g, s));
    all_nodes[encoding(start)] = s;

    while (!open_list.empty()) {
        auto c_f_g_node = open_list.top(); open_list.pop(); // get the next node
        AstarNC* curr = std::get<3>(c_f_g_node);
        if(curr->closed)
            continue; // This is an outdated version that should be ignored
        curr->closed = true;
        const Position end_pos(curr->location, curr->direction, curr->time);
        close_list.emplace(encoding(end_pos)); // mark it as done

        if(!doable) {
            if(bestFound == nullptr ||
                  std::make_pair(-curr->g2,curr->conflicts.count())
                < std::make_pair(-bestFound->g2, bestFound->conflicts.count()))
                bestFound = curr;
        } else if(bestFound == nullptr || curr->g2 > bestFound->g2) {
            bestFound = curr;
        }
        if (curr->location == end) { // Goal achieved
            bestAchievingFound = curr;
            break;
        }
        if(curr->g >= path_bound)
            break;

        // otherwise, try every next step possible and add the node to the open list
        if(curr->g2 < path_bound) {
            auto neighborhood = neighbors_with_conflicts(agent, Position(curr->location, curr->direction, curr->g2 + m_timestep));
            for (const auto &[neighbor,conflicts] : neighborhood) {
                if (close_list.contains(encoding(neighbor))) // the node is already done
                    continue;
                AstarNC* next_node = nullptr;

                if (all_nodes.contains(encoding(neighbor))) { // the node is already in the list: update its value
                    AstarNC* old = all_nodes.at(encoding(neighbor));
                    // if (curr->conflict_cost < old->conflict_cost ||
                        // (curr->conflict_cost == old->conflict_cost && curr->g  < old->g)) {

                    if (  std::make_tuple(curr->conflict_cost,curr->f,curr->g)
                        < std::make_tuple( old->conflict_cost, old->f, old->g)) {
                        next_node = all_nodes[encoding(neighbor)];
                    }
                }
                else {
                    next_node = new AstarNC;
                    all_nodes[encoding(neighbor)] = next_node;
                }
                if(next_node != nullptr) {
                    next_node->location = neighbor.location;
                    next_node->direction = neighbor.direction;
                    next_node->time = curr->time + 1;
                    next_node->g = curr->g + 1.0 + congestion(neighbor.location);
                    next_node->g2 = curr->g2 + 1.0;
                    next_node->h = curr->h + distance_diff(curr->location, neighbor.location, end);
                    next_node->f = next_node->g + next_node->h;
                    next_node->parent = curr;
                    next_node->conflicts = curr->conflicts;
                    next_node->conflict_cost = 0.0;
                    for(int ca : conflicts) {
                        double delta;
                        if(!next_node->conflicts[ca]) { // Was not in conflict
                            // next_node->conflict_cost += 1 + enqueued.at(ca) * enqueued.at(ca) / m_conf1;
                            next_node->conflicts.set(ca);
                            delta = (m_confadd + pow(enqueued.at(ca),m_confexp) / m_conf1);// / actualDistance.at(ca);
                            // if(actualDistance.at(agent) > actualDistance.at(ca)) next_node->conflict_cost += m_confadd;

                        }
                        else {// Was already in conflict, but we hit it again
                            // next_node->conflict_cost += 1 + enqueued.at(ca) * enqueued.at(ca) / m_conf2;
                            delta = (m_confadd + pow(enqueued.at(ca),m_confexp) / m_conf2);// / actualDistance.at(ca);
                            // if(actualDistance.at(agent) > actualDistance.at(ca)) next_node->conflict_cost += m_confadd;
                        }
                        delta *= pow((double) actualDistance[agent] / actualDistance[ca], m_confdistexp);
                        next_node->conflict_cost += delta;
                    }
                    open_list.push(std::make_tuple(-next_node->conflict_cost,
                                                   -next_node->f,
                                                   -next_node->g,
                                                   next_node));
                }
            }
        }
    }

    std::vector<int> vector_conflicts;

    // if we find the goal, trace back the path
    AstarNC* curr = bestAchievingFound;
    if(curr == nullptr)
        curr = bestFound;
    if(curr != nullptr) {
        // for(int i : curr->conflicts)
        for(int i = 0; i < m_num_of_agents; i++)
            if(curr->conflicts[i])
                vector_conflicts.push_back(i);
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
        cout << "ERROR: could not find a path for a" << agent << " at t:" << m_timestep << endl;
        _exit(1); // This should never hapen
    }

    for (auto n : all_nodes) {
        delete n.second;
    }

    return vector_conflicts;
}

/// @brief Compute all the valid positions after a given position of an agent
/// @param agent Index of the current agent
/// @param position Its current position
/// @return Vector of pairs of valid positions with a conflicting agent or -1 for no conflict
std::vector<std::pair<Planner::Position,std::vector<int>>> PlannerLong::neighbors_with_conflicts(int agent, const Planner::Position &position) const {
    std::vector<std::pair<Planner::Position,std::vector<int>>> neighbors;
    std::vector<Planner::Position> positions = {
        move(position, Action::FW),
        move(position, Action::CR),
        move(position, Action::CCR),
        move(position, Action::W)
    };

    const int t = position.time;
    for (const Planner::Position & next : positions) {
        if (next.location < 0 || obstacle(next.location))
            continue;

        std::unordered_set<int> relevant_agents;

        for(int tt = t; tt <= t+1; tt++)
            if(m_passing_hash.contains(std::make_pair(next.location,tt)))
                for(int i : m_passing_hash.at(std::make_pair(next.location,tt)))
                    if (i != agent)
                        relevant_agents.insert(i);

        if(m_ending_hash.contains(next.location))
            for(int i : m_ending_hash.at(next.location))
                if (i != agent && m_paths[i].back().time >= t - m_must_have_moved)
                    relevant_agents.insert(i);

        std::unordered_set<int> conflictsHere;
        for (int i : relevant_agents) {
            if (m_paths[i].size() > t+1-m_timestep) {
                if (m_paths[i][t+1-m_timestep].location == next.location) {
                    // cout << "hits a" << i << endl;
                    conflictsHere.insert(i);
                } else if (m_paths[i][t-m_timestep].location == next.location && m_paths[i][t+1-m_timestep].location == position.location) {
                    // cout << "crosses a" << i << endl;
                    conflictsHere.insert(i);
                }
            }
            else if (m_paths[i].size() > t-m_timestep) {
                const Position & other = m_paths[i][t-m_timestep];
                if (next.location == other.location && (next.direction - other.direction + 4) % 4 == 2) {
                    // cout << "invade facing a" << i << endl;
                    conflictsHere.insert(i);
                }
            }
            // if (m_paths.at(i).size() <= t+1-m_timestep && m_paths.at(i).back().location == next.location) {
            //     // cout << "hits a" << i << endl;
            //     valid = false;
            //     neighbors.push_back(std::make_pair(next,i));
            //     break;
            // }
        }
        std::vector<int> conflictsHereVec(conflictsHere.begin(), conflictsHere.end());
        neighbors.push_back(std::make_pair(next,conflictsHereVec));
    }

    return neighbors;
}


/**
 * @brief Planner::neighbors Compute all the valid positions after a given position of an agent
 * @param agent Index of the current agent
 * @param position Its current position
 * @return A list of valid positions
 */
vector<Planner::Position> PlannerLong::neighbors_alone(const Position &position) const {
    std::vector<Position> neighbors;
    std::vector<Position> positions = {
        move(position, Action::FW),
        move(position, Action::CR),
        move(position, Action::CCR),
        move(position, Action::W)
    };

    for (const Position & next : positions) {
        if (next.location >= 0 && !obstacle(next.location))
            neighbors.push_back(next);
    }

    return neighbors;
}

// Avoid collisions by stopping agents (and sending them to replan)
/**
 * @brief PlannerLong::avoid_collisions
 * We check if there are agents that are going to collide and stop them.
 * This happens when an agent reaches its task and a previously planned agent passes by the same location.
 * See @brief PlannerLong.
 * @param actions Moves to do in this timestep
 */

void PlannerLong::avoid_wait_collisions(vector<Action> & actions) {
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
    }
}

void PlannerLong::avoid_collisions(vector<Action> & actions) {

    // Handle collisions between two moving agents (vertex or edge)
    {
        std::unordered_set<int> invadingAgents; // Agents that we force to wait
        std::unordered_map<int,std::unordered_set<int>> nextLocations; // key: nextLocation, value: agents
        for(int a = 0; a < m_num_of_agents; a++) {
            const Action &act = actions.at(a);
            int location;
            if(act == Action::FW) {
                location = m_paths.at(a).at(1).location;
            }
            else {
                location = m_paths.at(a).at(0).location;
            }
            nextLocations[location].insert(a);
        }

        for(int a = 0; a < m_num_of_agents; a++) {
            const Action &act = actions.at(a);
            if(act == Action::FW
                    && nextLocations.contains(m_paths.at(a).at(0).location)) {
                for(int a2 : nextLocations.at(m_paths.at(a).at(0).location)) {
                    if(m_paths.at(a2).at(0).location == m_paths.at(a).at(1).location) {// Edge collision
                        invadingAgents.insert(a);
                        invadingAgents.insert(a2);
                    }
                }
            }

            if(act == Action::FW
               && nextLocations.at(m_paths.at(a).at(1).location).size() > 1) {
                for(int a2 : nextLocations.at(m_paths.at(a).at(1).location)) {
                    if(a2 != a) {// Vertex collision
                        invadingAgents.insert(a2);
                    }
                }
            }
        }

        for(int invading : invadingAgents) {
            cout << "-- moving collision unplanning a" << invading << endl;
            m_paths.at(invading).erase(m_paths.at(invading).begin() + 1, m_paths.at(invading).end());
            m_paths.at(invading).push_back(move(m_paths.at(invading).front(), Action::W));
            actions.at(invading) = Action::W;
        }
    }

    // Handle collisions between a moving agent and a non-moving agent
    {
        std::unordered_set<int> invadingAgents; // Agents that we force to wait
        std::unordered_set<int> waitingLocations;
        std::unordered_map<int,int> movingAgents; // key: nextLocation, value: agent
        for(int a = 0; a < m_num_of_agents; a++) {
            const Action &act = actions.at(a);
            if(act == Action::FW) {
                movingAgents[m_paths.at(a).at(1).location] = a;
            }
            else {
                int location = m_paths.at(a).at(0).location;
                waitingLocations.insert(location);
            }
        }

        for(std::pair<int,int> pos_a : movingAgents) {
            int nextLocation = pos_a.first;
            int a = pos_a.second;
            while(a != -1 && waitingLocations.contains(nextLocation)) {
                // Agent a can no longer move, it will wait at previousLocation instead
                invadingAgents.insert(a);
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
            cout << "-- static collision unplanning a" << invading << endl;
            m_paths.at(invading).erase(m_paths.at(invading).begin() + 1, m_paths.at(invading).end());
            m_paths.at(invading).push_back(move(m_paths.at(invading).front(), Action::W));
            actions.at(invading) = Action::W;
        }
    }
}

void PlannerLong::sort_queue(std::queue<int> & queue) const {
    std::vector<int> temp;
    while (!queue.empty()) {
        temp.push_back(queue.front());
        queue.pop();
    }
    // Compute approximate distances
     std::vector<int> dist(m_num_of_agents);
     for (int a = 0; a < m_num_of_agents; a++) {
         const int source = env->curr_states[a].location;
         const int target = task(a);
         dist.at(a) = manhattan_distance(source, target);
        //  dist.at(a) = distance(source, target);
     }
    // Sort by distance
    std::sort(temp.begin(), temp.end(), [&dist, this](int a, int b) {
        return std::make_tuple(m_paths.at(a).size(), dist.at(a))
             < std::make_tuple(m_paths.at(b).size(), dist.at(b));
    });
    for (int x : temp) {
        queue.push(x);
    }
}

// void PlannerLong::sort_by_length(std::queue<int> & queue) const {
//     std::vector<int> temp;
//     while (!queue.empty()) {
//         temp.push_back(queue.front());
//         queue.pop();
//     }
//     std::sort(temp.begin(), temp.end(), [this](int a, int b) {
//         return m_paths.at(a).size() < m_paths.at(b).size();
//     });
//     for (int x : temp) {
//         queue.push(x);
//     }
// }

std::vector<Planner::Position> PlannerLong::plan_path_alone(const Position &start, int end) {
//    cout << "Planning the path of a" << agent << " from " << start.location << " to " << end << endl;
    if (env->map.at(start.location) == 1) cout << "Error: is at location " << start.location << " which is an obstacle" << endl;
    if (env->map.at(end) == 1) cout << "Error: is assigned to location " << end << " which is an obstacle" << endl;

    // read_direction_map(end);

    std::priority_queue<std::tuple<float,float,AstarNode*>, vector<std::tuple<float,float,AstarNode*>>> open_list;
    std::unordered_map<long long int, AstarNode*> all_nodes; // nodes indexed by location
    std::unordered_set<long long int> close_list; // set of done nodes
    AstarNode* bestFound = nullptr;
    AstarNode* s = new AstarNode(start.location, start.direction, start.time, 0, 0, nullptr); // start node
    open_list.push(std::make_tuple(-s->f, -s->g, s));
    all_nodes[encoding(start)] = s;

    while (!open_list.empty()) {
        auto f_g_node = open_list.top(); open_list.pop(); // get the next node
        AstarNode* curr = std::get<2>(f_g_node);
        if(curr->closed)
            continue; // This is an outdated version that should be ignored
        curr->closed = true;
        const Position end_pos(curr->location, curr->direction, curr->time);
        close_list.emplace(encoding(end_pos)); // mark it as done

        if (curr->location == end) { // Goal achieved
            bestFound = curr;
            break;
        }

        // otherwise, try every next step possible and add the node to the open list
        auto neighborhood = neighbors_alone(Position(curr->location, curr->direction));
        for (const Position &neighbor : neighborhood) {
            // neighbor.print();
            if (close_list.contains(encoding(neighbor))) // the node is already done
                continue;
            if (all_nodes.contains(encoding(neighbor))) { // the node is already in the list: update its value
                AstarNode* old = all_nodes.at(encoding(neighbor));
                if (curr->g + 1.0 < old->g) { // + congestion(neighbor.location)
                    old->g = curr->g + 1.0; // + congestion(neighbor.location);
                    old->f = old->h + old->g;
                    old->parent = curr;
                    old->g2 = curr->g2 + 1;
                    open_list.push(std::make_tuple(old->f, old->g, old)); // Added duplicate
                }
            } else { // create the node
                AstarNode* next_node = new AstarNode(
                    neighbor.location,
                    neighbor.direction,
                    curr->time + 1,
                    curr->g + 1.0, //+ congestion(neighbor.location),
                    curr->h + distance_diff(curr->location, neighbor.location, end),
                    curr,
                    curr->g2 + 1
                );
                open_list.push(std::make_tuple(-next_node->f, -next_node->g, next_node));
                all_nodes[encoding(neighbor)] = next_node;
            }
        }
    }

    // cout << "Found path" << endl;

    std::vector<Position> path;
    // if we find the goal, trace back the path
    if(bestFound!= nullptr) {
        AstarNode* curr = bestFound;
        while (curr->parent != NULL) {
            const Position pos(curr->location, curr->direction, curr->time);
            path.push_back(pos);
            curr = curr->parent;
        }
        // append the new path
        path.push_back(start);
        // TODO: NEED TO REVERSE PATH
    }

    for (auto n : all_nodes) {
        delete n.second;
    }

    return path;
}

float PlannerLong::congestion(int location) const {
    if(m_congestion == 0.0)
        return 0.0;
    return usage.at(location) / m_congestion;
}

long long int PlannerLong::encoding(const Position & pos) const {
    const long long int npixels = env->cols*env->rows;
    return 4 * (npixels * pos.time + pos.location) + pos.direction;
}
