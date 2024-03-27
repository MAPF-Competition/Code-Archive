#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Planner.h"

/// @brief We plan five next steps to try to reach a closer location.
/// @details We put agents in a queue in order of distance to their task. For each agent, we try to find a path of length 5 
/// so that it gets closer to its path. We use conflict optimization
/// The conflict optimization has a queue of unplanned agents. We take the first and find the path that minimizes conflicts 
/// with other planned agents. Moreover, if the agents is labelled as *forward*, we must find a path that gets two locations 
/// closer to its task. To do this, we make a priority queue where we have nodes = location+time (we omit the direction). We stop when we have a path of length 5 and, if it is
/// a *forward* agent, it must finish in the good zone (which we precompute).
///
/// It seems better than PlannerSAT only for random_600, without barriers. So I guess I could call it PlannerRandom600 ;)

class PlannerTest : public Planner
{
public:
    PlannerTest();

public:
    /// @brief Node for the search of the best path for an agent
    struct SearchNode {
        int location;
        int time;
        std::set<int> collisions;
        double cost;
        SearchNode* parent;
        // bool closed = false;
        SearchNode(int _location, int _time, std::set<int> _collisions, double _cost, SearchNode* _parent):
            location(_location), time(_time), collisions(_collisions), cost(_cost), parent(_parent) {}
        void print() const {
            cout << "(location:" << location << ", t:" << time << ", cost:" << cost << ")" << endl;
        }
    };

    /// @brief Comparison between SearchNodes for the prioroity queue
    /// @return True if 
    struct cmp {
        bool operator()(SearchNode* a, SearchNode* b) {
            return (b->cost < a->cost);
        }
    };

    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan);

private:
    bool conflict_solver(int agent);
    void reset();
    // std::vector<int> get_collisions(int agent, int direction) const;
    int best_move(int agent, std::set<int> & collisions);
    void make_closer_locations(int agent);
    bool wrong_path(int agent, const SearchNode* node) const;
    int move(int location, int direction) const;
    bool good_move(int location, int direction) const;
    void set_moves(std::vector<Action> & plan) const;
    std::vector<int> sorted_agents() const;
    inline int encoding(int location, int time) const { return (env->cols*env->rows) * time + location; }
    bool finished(int agent, const SearchNode* node) const;
    std::vector<int> neighbors(int agent, const SearchNode* node) const;
    double get_collisions(int agent, const SearchNode * node, int location, std::set<int> & collisions) const;
    bool valid_move(int loc1, int loc2) const;

private:
    int m_path_bound = 0;
    double m_time_bound = 0.9;
    std::vector<std::vector<int>> m_location; // m_paths[a][t] = location of agent a at time t
    std::vector<std::vector<int>> m_agent; // m_agent[loc][t] = agent at location loc at time t (-1 if none)
    std::queue<int> m_queue;
    std::vector<int> m_enqueued;
    std::vector<bool> m_forward; // m_forward[a] = true if agent a moves towards its task
    std::vector<std::set<int>> m_next_locations; // m_next_locations[a] = neighbors of the location that are closer to its task
};
