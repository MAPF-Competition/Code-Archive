#pragma once

#include "SharedEnv.h"

#include "ash/preprocess.hpp"
#include "ash/astar.hpp"
#include "ash/utils.hpp"

namespace ash
{

class AstarPlanner
{
public:

    AstarPlanner(int win_length = 50);

    void initialize(const SharedEnvironment& env, int preprocess_time_limit);

    void plan(int time_limit, std::vector<Action>& planned_actions);


private:

    TimedAgentPosition get_agent_state(int agid) const;

    std::pair<int,int> get_agent_goal(int agid) const;

    void reset_plan(int agid, const TimedAgentPosition& start);

    void build_initial_plans();

    void step_plans();

    void update_priorities();

    bool update_agent_schedule(int agid);

    // attributes
    
    const SharedEnvironment* env;
    Preprocessor preprocessor;
    Astar<TimedAgentPosition> astar;
    int win_length;

    ConstraintSet constraint_set;
    std::vector<Plan<TimedAgentPosition>> plans;
    std::vector<int> agent_queue;
    int queue_pointer;
        
};

} // namespace ash

