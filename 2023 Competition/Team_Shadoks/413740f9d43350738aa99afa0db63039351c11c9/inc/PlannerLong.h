#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Planner.h"

/// @brief Algoritm that computes a path for each agent until its task
/// We use conflict optimization to obtain the best path for each agent
class PlannerLong : public Planner
{
public:
    PlannerLong();

public:

    /// @brief Node struct for the A* search algorithm
    struct Node {
        int location; // Location of the node
        int direction;  // Its direction
        int time;       // Its (absolute) time, ranging from 0 to 5000 at most
        int conflict_cost = 0;
        float g = 0;    // cost of the current path: length and conflicts
        int h = 0;      // estimate of the cost of the remaining path
        float f = 0;    // sum of g and h
        Node* parent;
        std::bitset<200> conflicts; // Bitset gives 10% more A* searches in 1 second
        bool closed = false;
        Node(){};
        Node(const Position & pos, Node* _parent)
            : location(pos.location), direction(pos.direction), time(pos.time), parent(_parent) {}
        void print(int ncols) const {
            const int x = location % ncols;
            const int y = location / ncols;
            cout << " (x:" << x << ", y:" << y << ", dir:" << direction << ", t:" << time << ")";
        }
    };
    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan, double time_limit = 1.0);

private:
    // Paths
    bool try_planning(std::queue<int> & queue);
    std::vector<int> plan_path(int agent, const Position &start);
    std::vector<int> plan_dummy_path(int agent, const Position &start);
    bool update_paths();
    void fill_queue(std::queue<int> & queue);
    std::vector<std::pair<Planner::Position, std::vector<int>>> neighbors(int agent, const Planner::Position &position) const;
    std::vector<int> conflicts_fw(int agent, const Position & cur_pos, const Position & next_pos) const;
    std::vector<int> conflicts_cr(int agent, const Position & cur_pos, const Position & next_pos) const;
    bool valid_move(int agent, const Position & cur_pos, const Position & next_pos) const;
    std::pair<int, std::bitset<200>> sum_conflicts(Node * curr, const std::vector<int> & conflicts) const;
    void forbid_position(int agent, const std::vector<int> & conflicts);
    void update_dummy();
    double evaluate_paths(bool print = false) const;

    // Distances
    void sort_by_distance(std::queue<int> & queue);

private:
    // int m_block_span; // once a robot is unplanned after a conflict, for how long we fix its position
    float m_alpha = 0.1; // objective function = length + alpha * sum of costs**beta
    float m_beta  = 2.0; // 
    int m_enqueue_bound = 20;
    int m_nb_steps = 500; // number of steps
    std::chrono::high_resolution_clock::time_point m_start_time;
    double m_time_limit = 1; // Allowed time for planning
    std::vector<int> m_enqueued;
    std::set<std::tuple<int, int, int>> m_forbidden_position; // m_forbidden_position has (agent, location i) if...
    std::vector<bool> m_dummy; // m_dummy[agent] == true if agent cannot finish its task within 500 steps. Then we minimize conflicts, not the length
    int m_round; // We plan the agents several times. m_round is the number of trial
};
