#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "Shadocks/PlannerLong.h"
#include "Shadocks/PlannerR100.h"
// #include "PlannerBully.h"
#include "Shadocks/PlannerLazy.h"
// #include "PlannerBobo.h"
#include "Shadocks/PlannerSat.h"
#include "Shadocks/Secretary.h"
#include "Shadocks/PlannerComb.h"
#include "Shadocks/PlannerGame.h"

namespace Shadocks {

class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){
        // delete env;
    };


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, ::std::vector<Action> & plan);

private:
    void import_parameters(Parameters & parameters) const;

    Parameters param;

    PlannerLong m_algo_long;
    Secretary<PlannerLong> s_algo_long = Secretary<PlannerLong>(m_algo_long);

    PlannerR100 m_algo_r100;

    PlannerComb m_algo_comb;

    PlannerLazy m_algo_lazy;
    // Secretary<PlannerLazy> s_algo_lazy = Secretary<PlannerLazy>(m_algo_lazy);
    
    PlannerSAT m_algo_sat;
    PlannerGame m_algo_game;
};

}; // namespace Shadocks