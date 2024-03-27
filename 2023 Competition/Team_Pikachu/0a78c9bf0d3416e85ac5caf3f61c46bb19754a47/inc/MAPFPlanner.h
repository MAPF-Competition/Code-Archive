#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "RHCR/interface/RHCRSolver.h"
#include "nlohmann/json.hpp"
#include "RHCR/main/SingleAgentSolver.h"
#include "RHCR/main/MAPFSolver.h"
#include "RHCR/main/WHCAStar.h"
#include "RHCR/main/ECBS.h"
#include "RHCR/main/LRAStar.h"
#include "RHCR/main/PBS.h"
#include "RHCR/main/ID.h"
#include "RHCR/interface/RHCRSolver.h"
#include "RHCR/interface/CompetitionGraph.h"
#include "common.h"
#include <memory>
#include "PIBT/PIBTSolver.h"

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

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    bool consider_rotation=true;
    string lifelong_solver_name;
    std::shared_ptr<RHCR::RHCRSolver> solver; 
    std::shared_ptr<PIBT::PIBTSolver> pibt_solver;
    nlohmann::json config;
    void load_configs();
    RHCR::MAPFSolver* build_mapf_solver(RHCR::CompetitionGraph & graph);
    void config_solver();
};
