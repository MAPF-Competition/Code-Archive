#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


#include "Planner.h"

/**/
class PlannerLong : public Planner
{
public:
    PlannerLong();

public:
    void init(SharedEnvironment* _env, const Parameters & parameters);
    void plan(std::vector<Action> & plan, double timeLeft = 1.0);

private:
    // Paths
    void update_paths();
    void fill_queue(std::queue<int> & queue, std::vector<int> &enqueued, std::vector<int> &is_queued) const;
    // std::list<Position> neighbors_long(int agent, const Position &position) const;

    std::vector<Position> plan_path_alone(const Position &start, int end);
    std::vector<Position> neighbors_alone(const Position &position) const;
    std::vector<int> conflicts_fw(int agent, const Position & cur_pos, const Position & next_pos) const;
    std::vector<int> conflicts_cr(int agent, const Position & cur_pos, const Position & next_pos) const;

    std::vector<int> plan_path_with_conflicts(int agent, const Position &start, int end, int path_bound, std::vector<int> &enqueued, bool doable, std::vector<int> &actualDistance);
    // std::vector<int> plan_undoable_path(int agent, const Position &start);
    std::vector<std::pair<Planner::Position,std::vector<int>>> neighbors_with_conflicts(int agent, const Planner::Position &position) const;
    std::vector<std::pair<Planner::Position,std::vector<int>>> neighbors_with_conflicts_v2(int agent, const Planner::Position &position) const;
    std::vector<std::pair<Planner::Position,std::vector<int>>> neighbors_with_conflicts_v3(int agent, const Planner::Position &position) const;

    void avoid_collisions(std::vector<Action> & actions);
    void avoid_wait_collisions(std::vector<Action> & actions);
    long long int encoding(const Position & pos) const;

    // Distances
    void sort_queue(std::queue<int> & queue) const;
    float congestion(int location) const;

private:
    float m_congestion;
    double m_sub=0, m_exp=1.3, m_conf1 = 2.0, m_conf2 = 4.0, m_confexp, m_confadd, m_probexp, m_confdistexp;
    int m_slack;
    std::vector<int> usage; // usage[location] = number of steps where this location is occupied by an agent
};
