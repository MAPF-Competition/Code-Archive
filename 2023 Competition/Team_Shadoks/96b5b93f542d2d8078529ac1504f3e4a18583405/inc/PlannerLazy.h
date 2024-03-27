#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Planner.h"

/**
 * @brief The PlannerLazy class
 * This planner puts the agents in a queue and process them in order.
 * Each agent has a computed path and stays at its last position forever. Note that this
 * last position can be its task or just an intermediate location (if we could not reach the task
 * in the given limit).
 *
 * Note that if I plan a0 (passing by a location L) and a1 (whose task is in L), then a1 stops at L 
 * (we do not compute a full path if it reaches its task) and interrupts the path of a0. I decide not to
 * look for collisions with a1 after she reaches its task nor computing a full path for her that avoids
 * other robots (since that could prevent her from reaching its task). Instead, I prefer to catch any 
 * collision created like this when it arrives and stop the moving agent.
 * Note that I may need to unplan neighboring agents. This is done in PlannerLazy::avoid_collisions
 * 
 * At some point this algorithm got a lot worse than before. I think that the problem was in PlannerLazy::neighbors().
 * I keep the old function in the comments because it was a lot smarter.
 */
class PlannerLazy : public Planner
{
public:
    PlannerLazy();

public:
    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan, double timeLeft = 1.0);

private:
    // Paths
    void plan_path(int agent, const Position &start, int end, int path_bound, double timeLeft);
    bool update_paths();
    std::vector<Position> neighbors(int agent, const Position &position) const;
    std::vector<Position> neighbors_v2(int agent, const Position &position) const;
    std::vector<Position> neighbors_v3(int agent, const Position &position) const;
    bool valid_fw(int agent, const Position & cur_pos, const Position & next_pos) const;
    bool valid_cr(int agent, const Position & cur_pos, const Position & next_pos) const;
    void get_actions(std::vector<Action> & actions);
    void avoid_wait_collisions(std::vector<Action> & actions);

    void move_to_front(int a);
    float congestion(int location);
private:
    int m_path_bound;
    float m_congestion;
    bool m_safety_gap = false; // m_safety_gap == true if agents are not allowed to follow each other at distance 0
    std::deque<int> m_queue;
    bool m_halt;
    int scount = 0;
};
