#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Planner.h"

/// @brief We compute only one move. 
/// @details This is a simplified version of PlannerSAT. This works better for the game instance (I ignore why)
/// @note If this Planner had more time, it would be worse. Indeed, planning too many agents makes a lot of congestion in
/// the bottlenecks. Hence, we must find the right balance between finished tasks and planned agents.
class PlannerGame : public Planner
{
public:
    PlannerGame();

public:
    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan);

private:
    bool conflict_solver(int agent);
    void reset();
    std::vector<int> get_collisions(int agent, int next_location) const;
    int best_move(int agent, std::vector<int> & collisions);
    int move(int location, int direction) const;
    bool good_move(int location, int direction) const;
    void set_moves(std::vector<Action> & plan) const;
    std::vector<int> next_locations(int agent) const;
    std::vector<int> sorted_agents() const;
    void validate_correspondance() const;
    int get_agent(int location, int time) const;
    void set_agent(int location, int time, int agent);
    void backup_agent();
    int steps_to_move(int agent, int next_location) const;
    void update_time_to_move();
    bool punctual(int agent, int direction) const;
    void print_path(int agent) const;

private:
    int m_enqueue_bound = 0;
    double m_time_bound = 0.95;
    std::vector<std::vector<int>> m_location;   // m_location[a][t] = location of agent a at time t
    std::vector<std::vector<int>> m_agent;      // m_agent[loc][t] = agent at location loc at time t (-1 if none)
    std::queue<int> m_queue;
    std::vector<int> m_enqueued;
    std::vector<bool> m_forward; // m_forward[a] = true if agent a moves towards its task
    std::vector<Position> m_position; // m_position[a] = position of agent a
    std::vector<int> m_time_to_move; // m_time_to_move[a] = number of steps to move to its next location
    double m_agents_bound = 1.0;
};
