#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Planner.h"

/// @brief We compute only one move.
/// @details We sort the agents by distance to their tasks. For each agent, we try to
/// move it towards its tasks, possibly pushing other robots, but without replanning previously
/// (with higher priority) agents. We do this with a conflict solver.
///
/// I use this solver for very dense instances, like random_600, random_800 and game_6500. It cannot avoid congestion
/// because it does not look into the future to see which is the best path (it only looks at the next timestep).
/// It is very fast for random_600 and random_800, it can plan every agent even if we set a high bound (path_bound=600) for
/// the conflict solver
/// The instance game_6500 is much bigger and it cannot plan all agents at every step, so the farthest agents do not move
/// (unless another agent pushes it). I tried planning two agents at the same time for this instance so that every agent
/// is planned, but it gave worse results.
class PlannerSAT : public Planner
{
public:
    PlannerSAT();

public:
    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan);

private:
    void reset();
    std::vector<int> sorted_agents(int shot) const;
    bool push_agent(int agent);
    void set_moves(std::vector<Action> & plan) const;
    void set_moves_extended(const std::vector<std::vector<int>> & locations, std::vector<Action> & plan) const;
    std::vector<int> make_time_to_move() const;
    bool conflict_solver(int agent, int steps);
    int best_move(int agent, int steps, std::vector<int> & collisions);
    std::vector<int> next_locations(int agent, int steps) const;
    std::vector<int> get_collisions(int agent, int next_location, int steps) const;
    std::vector<std::vector<int>> second_plan();
    void wide_view(std::vector<Action> & actions);
    int min_moves(int agent) const;
    int evaluate_locations() const;

    int move_loc(int location, int direction) const;
    int unsafe_move(int location, int direction) const;
    bool good_move(int location, int direction) const;
    void validate_correspondance() const;
    int get_agent(int location, int time) const;
    void set_agent(int location, int time, int agent);
    void backup_agent();
    int steps_to_move(int agent, int next_location) const;
    bool punctual(int agent, int direction) const;
    int available_time(int agent) const;
    void print_path(int agent) const;
    void fast_plan();
    int rotations(int agent, int direction) const;
    // void backup_data(const std::vector<std::vector<int>> & agents);

    void test_sat() const;

private:
    int m_enqueue_bound = 0;
    double m_time_bound = 0.95;
    int m_nb_steps = 2; // number of steps that we compute (1 by default)
    bool m_greedy = true; // True if we force the agents to move in as few steps as possible 
    std::vector<std::vector<int>> m_location;   // m_location[a][t] = location of agent a at time t
    std::vector<std::vector<int>> m_agent;      // m_agent[loc][t] = agent at location loc at time t (-1 if none)
    std::queue<int> m_queue;
    std::vector<int> m_enqueued;
    std::vector<bool> m_forward; // m_forward[a] = true if agent a moves towards its task
    std::vector<Position> m_position; // m_position[a] = position of agent a
    std::vector<int> m_time_to_move; // m_time_to_move[a] = number of steps to move to its next location
    std::chrono::high_resolution_clock::time_point m_start_time; // start of a plan

    std::vector<std::vector<double>> m_agent_weight; // list random weights for each agent to randomize the sorting by distances
    int m_shots = 1; // number of times we try to compute the next location
    double m_agents_bound = 1.0;
};
