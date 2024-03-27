#pragma once

#include <algorithm>
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <regex>
#include <random>
#include <vector>
#include <string>
#include "tsl/hopscotch_set.h"
#include "tsl/hopscotch_map.h"

#include "Planner.h"

/// @brief A valid path for an agent
struct MovePath {
    int id;                                 // Unique identifier of this MovePath
    int agent;
    std::vector<Planner::Position> path;    // Sequence of positions
    std::string str;                        // Sequence of actions
    double value;                              // A value encoding the "quality" of this path... see PlannerComb::buildMovePaths()
    int achieved;
};


class PlannerComb : public Planner {
protected:
    int m_path_bound; // Size of the paths
    int m_maxqueue;
    double m_confexp;
    double m_valuemul;
    double m_fwbonus;       // Parameter for the value function of a MovePath: cost of moving forward in the next step
    double m_achievebonus;  // Parameter for the value function of a MovePath: cost of finishing a task
    double m_elite_value;
    double m_nonelite;
    bool m_greedy;
    std::vector<std::string> m_moves; // All the possible sequences of actions
    std::vector<MovePath> m_movepaths;
    std::vector<std::vector<int>> m_possibilities;
    std::vector<int> m_solution;
    std::vector<tsl::hopscotch_set<int>> m_conflicts; // m_conflicts[mp] = paths having a vertex or edge conflict with the MovePath mp
    std::vector<int> m_elite;
    std::vector<int> m_is_elite;

public:
    PlannerComb() {}

    void init(SharedEnvironment* _env, const Parameters & parameters) {
        Planner::init(_env, parameters);
        m_path_bound = parameters.values.at("path_bound");
        m_confexp = parameters.values.at("confexp");
        m_valuemul = parameters.values.at("valuemul");
        m_maxqueue = parameters.values.at("maxqueue");
        m_fwbonus =  parameters.values.at("fwbonus");
        m_achievebonus =  parameters.values.at("achievebonus");
        m_elite_value =  parameters.values.at("elite");
        m_greedy =  (parameters.values.at("greedy") != 0.0);
        m_nonelite=  parameters.values.at("nonelite");


        for(auto &[k,v] : parameters.values)
            cout << k << " = " << v << endl;

        m_moves = simplified(allStrings(m_path_bound));

        m_paths = std::vector<std::vector<Position>>(m_num_of_agents);

        cout << "PlannerComb initialized with " << m_moves.size() << " moves" << endl;
    }

    void plan(std::vector<Action> &actions, double timeLeft = 1.0) {
        std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();

        actions = std::vector<Action>(env->curr_states.size(), Action::W); // default action of waiting
        if (halt())
            return;

        buildElite();

        m_possibilities = std::vector<std::vector<int>>(m_num_of_agents);
        m_solution = std::vector<int>(m_num_of_agents, -1);
        // Build all the paths
        m_movepaths.clear();
        for(int a = 0; a < m_num_of_agents; a++) {
            const Position start(env->curr_states.at(a), m_timestep);
            buildMovePaths(a, start);
        }

        cout << endl << "Time to build "<< m_movepaths.size() << " paths: " << timeSince(start_time) << endl;

        reduceMovePaths();

        cout << "Time to reduce to "<< m_movepaths.size() << " paths: " << timeSince(start_time) << endl;

        buildConflicts();
        size_t nconf = 0; // Total number of conflicts (each counted twice?)
        for(auto &conflicts : m_conflicts) {
            nconf += conflicts.size();
        }

        cout << "Time to build "<< nconf << " conflicts on " << m_conflicts.size() <<" ids: " << timeSince(start_time) << endl;

        trivialSolution();
        cout << "Initial solution time: " << timeSince(start_time) << endl;

        improveSolution(timeLeft - timeSince(start_time) - .03);
        cout << "Improving time: " << timeSince(start_time) << endl;

        if(m_greedy) {
            greedyImprove();
            cout << "Greedy improved to " << solValue() << " in time: " << timeSince(start_time) << endl;
        }

        m_recent = std::vector<int>(env->cols*env->rows, 0);
        for(int a :m_elite)
            for(Position &pos : m_movepaths.at(m_solution.at(a)).path)
                m_recent.at(pos.location)++;

        get_actions(actions);
        update_tasks_done(actions);
        print_elapsed_time();
        m_timestep++;
    }

protected:
    /// @brief Remove paths that are impossible and save the rest in m_possibilites
    void reduceMovePaths() {
        std::unordered_set<int> cantMove;
        // Put locations of agents that cannot move in the first step
        for(int a = 0; a < m_num_of_agents; a++) {
            Position start(env->curr_states.at(a), m_timestep);
            if(!tryAction(start, 'F'))
                cantMove.insert(start.location);
        }

        size_t previousSize;
        do {
            previousSize = cantMove.size();
            for(int a = 0; a < m_num_of_agents; a++) {
                Position start(env->curr_states.at(a), m_timestep);
                Position pos = start;

                if(tryAction(pos, 'F') && cantMove.contains(pos.location))
                    cantMove.insert(start.location);
            }

        } while(cantMove.size() != previousSize);


        auto oldMovePaths = m_movepaths;
        m_movepaths.clear();
        for(auto &mp : oldMovePaths) {
            if(mp.path.at(0).location  == mp.path.at(1).location
               || !cantMove.contains(mp.path.at(1).location)) {
                mp.id = m_movepaths.size();
                m_movepaths.push_back(mp);
            }
        }

        for(auto &mp : m_movepaths)
            m_possibilities[mp.agent].push_back(mp.id);

    }


    /// @brief Build all the possible paths for an agent with their value and save them
    /// @param a The agent
    /// @param start Its start position
    /// @details The value of a path is the number of locations it advances towards its task. However, 
    /// if the path finishes the task, it is 
    void buildMovePaths(int a, const Position &start) {
        const int startDist = distance(start.location, task(a));
        for(std::string move : m_moves) {
            Position cur = start;
            std::vector<Position> path;
            for(int i = -1; i < (int) move.size(); i++) {
                if(i >= 0 && !tryAction(cur, move[i]))
                    break;
                path.push_back(cur);
            }
            if(path.size() == move.size() + 1) {
                MovePath mp;
                mp.id = m_movepaths.size();
                mp.agent = a;
                mp.path = path;
                mp.str = move;
                mp.achieved = 0;
                if(m_is_elite.at(a)) {
                    mp.value = startDist - distance(path.back().location, task(a)); // @todo Should we use the real distance, counting turns?
                    for(int i = 0; i < path.size(); i++) {
                        if(path[i].location == task(a)) {
                            mp.achieved = path.size() - i;
                            break;
                        }
                    }
                    if(mp.achieved) mp.value = mp.str.size() + mp.achieved * m_achievebonus; // Great value for achieving

                    if(mp.str[0] == 'F') mp.value += m_fwbonus; // Bonus for moving forward in the first step
                }
                else { // Avoid crowded areas for non-elite robots
                    mp.value = 0.0;
                    if(m_recent.size())
                        for(Position &pos : path)
                            mp.value += 1.0 / (1.0 + m_recent.at(pos.location));
                    mp.value /= m_nonelite;
                }
                mp.value *= m_valuemul;

                m_movepaths.push_back(mp);
            }
        }
    }

    /// @brief Get the next string
    /// @param s 
    /// @return True if this is not the last string
    bool nextString(std::string &s) {
        int n = s.size();
        bool carry = true;
        for(int i = n-1; i >= 0 && carry; i--) {
            s[i] ++;
            carry = false;
            if(s[i] == 'e') {
                s[i] = 'a';
                carry = true;
            }
        }
        return !carry;
    }

    /// @brief Translate a string to make a string encoding the path of an agent
    /// @param s A string containing the characters ['a', 'b', 'c', 'd']
    /// @return A string
    std::string translate(const std::string &s) {
        std::string t;
        for(char c : s) {
            if(c == 'a') t.push_back('F');
            else if(c == 'b') t.push_back('C');
            else if(c == 'c') t.push_back('R');
            else if(c == 'd') t.push_back('W');
        }

        return t;
    }

    /// @brief Get all the possible paths, made of combinations of FX, CR, CCR and W
    /// @param n Size of the string
    /// @return A vector with all the possible strings
    std::vector<std::string> allStrings(int n) {
        std::vector<std::string> ret;
        std::string s(n,'a');
        bool b;
        do {
            ret.push_back(translate(s));
            b = nextString(s);
        } while(b);
        return ret;
    }

    /// @brief Filter out the unnecessary paths to reduce the number of paths to explore
    /// @param v A vector of strings
    /// @return A subset of v
    std::vector<std::string> simplified(std::vector<std::string> v) {
        std::vector<std::string> ret;

        for(std::string s : v) {
            if(std::regex_match(s, std::regex(".*RW*C.*"))) continue;     // No turn and unturn
            if(std::regex_match(s, std::regex(".*CW*R.*"))) continue;     // No turn and unturn
            if(std::regex_match(s, std::regex(".*W[CR].*"))) continue;    // No wait and rotate, better to rotate first
            if(std::regex_match(s, std::regex(".*(CW*){4}.*"))) continue; // No turn 360 left
            if(std::regex_match(s, std::regex(".*(RW*){4}.*"))) continue; // No turn 360 right
            if(std::regex_match(s, std::regex(".*(CW*){3}.*"))) continue; // No turn 3 times in the same direction
            if(std::regex_match(s, std::regex(".*(RW*){3}.*"))) continue; // No turn 3 times in the same direction
            if(std::regex_match(s, std::regex(".*CW*C.*"))) continue;     // No turn around through the left
            if(std::regex_match(s, std::regex(".*[CR]W*"))) continue;     // No rotate in the end since only position is used
            // if(std::regex_match(s, std::regex("W*"))) s[0] = 'R';         // No waiting only, turn instead

            // cout << s << endl;
            ret.push_back(s);
        }
        return ret;
    }

    /// @brief Apply an action to a position
    /// @param pos A position
    /// @param action An action encoded as a character [FCRW]
    /// @return True if this action can be made
    bool tryAction(Position &pos, char action) {
        Position next(pos.location, pos.direction, pos.time + 1);
        if (action == 'F') {
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
            if(next.location < 0 || obstacle(next.location))
                return false;
        } else if (action == 'R') {
            next.direction = (next.direction+1) % 4;
        } else if (action == 'C') {
            next.direction = (next.direction+3) % 4;
        }
        pos = next;
        return true;
    }

    /// @brief Build the graph of collisions between the MovePath
    /// @details It is a graph where two MovePath are adjacent if they have a conflict (vertex or edge)
    /// This graph is encoded as an adjacency list in m_conflicts
    void buildConflicts() {
        m_conflicts.clear();
        m_conflicts.resize(m_movepaths.size());

        // Vertex collisions
        std::unordered_map<std::pair<int,int>, std::unordered_map<int, std::vector<int>>> passing;
        for(const MovePath &mp : m_movepaths) {
            for(const Position &pos : mp.path) {
                passing[std::make_pair(pos.location,pos.time)][mp.agent].push_back(mp.id);
            }
        }

        for(const auto &[loctime, pmap] : passing) {
            for(const auto &[a1, ids1] : pmap) {
                for(const auto &[a2, ids2] : pmap) {
                    if(a1 != a2) {
                        for(int id1 : ids1) {
                            m_conflicts[id1].insert(ids2.begin(),ids2.end());
                        }
                    }
                }
            }
        }

        // Edge collisions
        passing.clear();
        for(const MovePath &mp : m_movepaths) {
            for(int i = 0; i < mp.str.size(); i++) {
                if(mp.str[i] == 'F') {
                    int edgeid = (std::min(mp.path[i].location, mp.path[i+1].location) << 2) + mp.path[i].direction%2;
                    passing[std::make_pair(edgeid,mp.path[i].time)][mp.agent].push_back(mp.id);
                }
            }
        }

        for(const auto &[loc, pmap] : passing) {
            for(const auto &[a1, ids1] : pmap) {
                for(const auto &[a2, ids2] : pmap) {
                    if(a1 != a2) {
                        for(int id1 : ids1) {
                            m_conflicts[id1].insert(ids2.begin(),ids2.end());
                        }
                    }
                }
            }
        }
    }

    /// @brief Set an initial solution where the agents do not move
    void trivialSolution() {
        for(int a = 0; a < m_num_of_agents; a++)
            m_solution[a] = m_possibilities[a].back(); // Waiting path is at the back
    }


    /// @brief Compute a solution to the optimization problem starting from a trivial solution
    /// @param timeLeft 
    void improveSolution(double timeLeft) {
        std::chrono::high_resolution_clock::time_point start_time  = std::chrono::high_resolution_clock::now();
        static std::random_device rd;
        static std::mt19937 gen(rd());

        for(std::vector<int> &ids : m_possibilities) {
            // Sort moves of each agent by decreasing value to speed things up a little
            //
            std::stable_sort(ids.begin(), ids.end(), [this](int x, int y) {
                return m_movepaths.at(x).value > m_movepaths.at(y).value;
            });
        }

        cout << "Before: " << solValue();
        int count = 0;
        std::vector<int> touched(m_num_of_agents,0);
        while(timeSince(start_time) < timeLeft) {
            int firstid, firsta;
            firsta = m_elite[count % m_elite.size()];

            std::vector<int> possibilities_a = m_possibilities.at(firsta);
            std::shuffle(possibilities_a.begin(), possibilities_a.end(), gen);
            firstid = *std::max_element(possibilities_a.begin(),
                                        possibilities_a.end(),
                                        [this](int x, int y) {
                                            return m_movepaths.at(x).value > m_movepaths.at(y).value;
                                        });

            std::vector<int> dequeued(m_num_of_agents);
            std::queue<int> queue;

            std::vector<int> is_queued(m_num_of_agents);
            std::vector<int> previous = m_solution;
            queue.push(firsta);
            is_queued.at(firsta) = true;
            m_solution.at(firsta) = -1;

            while(!queue.empty()) {
                int a = queue.front(); // front() for queue, top() for priority_queue
                touched[a]++;
                is_queued.at(a) = false;
                dequeued.at(a)++;
                if(dequeued.at(a) > m_maxqueue)
                    break;
                queue.pop();

                double bestcost = 9999;
                int bestid = -1;
                if(a == firsta && dequeued.at(a) == 1)
                    bestid = firstid;
                else {
                    for(int i = 0; i < m_possibilities.at(a).size(); i++) {
                        MovePath &mp = m_movepaths.at(m_possibilities.at(a).at(i));
                        int newid = mp.id;
                        // std::normal_distribution normal{0.0, .1};
                        double cost = - mp.value;

                        for(int cid : m_conflicts.at(newid)) {
                            int ca = m_movepaths.at(cid).agent;
                            if(m_solution.at(ca) == cid) {
                                cost += pow(1.0 + dequeued.at(ca), m_confexp);
                                if(cost > bestcost)
                                    break;
                            }
                        }
                        if(cost < bestcost) {
                            bestcost = cost;
                            bestid = newid;
                        }
                    }
                }
                m_solution.at(a) = bestid;

                for(int cid : m_conflicts.at(bestid)) {
                    int ca = m_movepaths.at(cid).agent;
                    if(m_solution.at(ca) == cid) {
                        queue.push(ca);
                        is_queued.at(ca) = true;
                        m_solution.at(ca) = -1;
                    }
                }
            }
            if(!queue.empty()) {
                m_solution = previous;
                cout << '.';
            }
            else {
                double sv = solValue(m_solution, m_elite);
                double pv = solValue(previous, m_elite);
                if(sv < pv) {
                    cout << '-';
                    m_solution = previous;
                } else if(sv == pv) {
                    cout << '=';
                } else {
                    cout << '+';
                }
            }
            count++;
        }
        cout << "After " << count << " steps: " << solValue() << endl;
    }

    double solValue() {
        return solValue(m_solution);
    }

    double solValue(const std::vector<int> &sol) {
        double ret = 0;
        for(int id : sol) {
            if(id != -1) {
                MovePath &mp = m_movepaths.at(id);
                ret += mp.value;
            }
            else {
                cout << "INCOMPLETE SOLUTION!!!" << endl;
                return -999999; // Should not happen
            }
        }
        return ret;
    }

    double solValue(const std::vector<int> &sol, const std::vector<int> &agents) {
        double ret = 0;
        for(int a : agents) {
            int id = sol.at(a);
            if(id != -1) {
                MovePath &mp = m_movepaths.at(id);
                ret += mp.value;
            }
            else {
                cout << "INCOMPLETE SOLUTION!!!" << endl;
                return -999999; // Should not happen
            }
        }
        return ret;
    }

    double timeSince(std::chrono::high_resolution_clock::time_point &start_time) {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();
    }

    void get_actions(std::vector<Action> &actions) { //FW, CR, CCR, W
        for(int a = 0; a < m_num_of_agents; a++) {
            int id = m_solution.at(a);
            MovePath &mp = m_movepaths.at(id);

            if(mp.str[0] == 'W') actions[a] = Action::W;
            else if(mp.str[0] == 'F') actions[a] = Action::FW;
            else if(mp.str[0] == 'R') actions[a] = Action::CR;
            else if(mp.str[0] == 'C') actions[a] = Action::CCR;

            // Creat rotation for waiting forever agents
            if(actions[a] == Action::W && std::regex_match(mp.str, std::regex("W*"))) {
                const Position start(env->curr_states.at(a), m_timestep);
                if(start.direction == m_timestep % 4)
                    actions.at(a) = Action::CR;
                else if(start.direction != (m_timestep+1) % 4)
                    actions.at(a) = Action::CCR;
            }
        }

    }

    void greedyImprove() {
        auto sortedPaths = m_movepaths;
        std::stable_sort(sortedPaths.begin(), sortedPaths.end(), [](const MovePath &a, const MovePath &b) {return a.value > b.value;});

        for(MovePath &mp : sortedPaths)  {
            int a = mp.agent;
            if(m_solution.at(a) == -1) {
                cout << "NO SOLUTION FOR " << a << endl;
                break;
            }
            MovePath &solmp = m_movepaths.at(m_solution.at(a));
            if(solmp.value >= mp.value) continue;
            bool failed = false;
            for(int cid : m_conflicts.at(mp.id)) {
                int ca = m_movepaths.at(cid).agent;
                if(m_solution.at(ca) == cid) {
                    failed = true;
                    break;
                }
            }
            if(!failed)
                m_solution.at(a) = mp.id;
        }
    }

    void buildElite() {
        std::vector<int> agents;
        for(int a = 0; a <m_num_of_agents; a++)
            agents.push_back(a);

        std::stable_sort(agents.begin(), agents.end(), [this](int x, int y) {
            const Position startx(env->curr_states.at(x), m_timestep);
            const Position starty(env->curr_states.at(y), m_timestep);
            return distance(startx.location, task(x)) < distance(starty.location, task(y));
        });

        m_elite = std::vector<int>(agents.begin(), agents.begin() + agents.size() * m_elite_value);
        m_is_elite = std::vector<int>(m_num_of_agents, 0);
        for(int a : m_elite)
            m_is_elite.at(a) = true;
    }
};
