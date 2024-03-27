#pragma once

#include "SharedEnv.h"

#include "ash/preprocess.hpp"
#include "ash/push_search.hpp"
#include "ash/utils.hpp"

namespace ash
{

class PushPlanner
{
public:

    PushPlanner();

    void initialize(const SharedEnvironment& env, int preprocess_time_limit);

    void plan(int time_limit, std::vector<Action>& planned_actions);


private:

    TimedAgentPosition get_agent_state(int agent_id) const;

    std::pair<int,int> get_agent_goal(int agent_id) const;

    //void build_initial_plans();

    //void step_plans();

    void update_priorities();

    // attributes
    
    const SharedEnvironment* env;
    Preprocessor preprocessor;
    PushSearch search;

    std::vector<int> agent_queue;
    //std::vector<bool> blocked;
        
};

} // namespace ash

