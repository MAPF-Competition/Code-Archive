#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "PlannerLong.h"
#include "PlannerLazy.h"
#include "PlannerSat.h"
#include "PlannerGame.h"
#include "PlannerTest.h"

class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

private:
    void import_parameters(Parameters & parameters) const;

    Parameters param;
    PlannerLong m_algo_long;
    PlannerLazy m_algo_lazy;
    PlannerSAT m_algo_sat;
    PlannerGame m_algo_game;
    PlannerTest m_algo_test;
};
