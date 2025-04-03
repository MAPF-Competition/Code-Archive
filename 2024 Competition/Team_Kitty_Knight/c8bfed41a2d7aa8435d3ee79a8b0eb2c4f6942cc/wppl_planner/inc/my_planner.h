#pragma once

#include <ctime>
#include "SharedEnv.h"
#include "util/CompetitionActionModel.h"
#include "nlohmann/json.hpp"
#include "common.h"
#include <memory>
#include "LaCAM2/LaCAM2Solver.hpp"
#include "LNS/LNSSolver.h"
#include "util/HeuristicTable.h"
#include "Shadocks/MAPFPlanner.h"

namespace MyPlanner {

class WPPLPlanner
{
public:

    SharedEnvironment* env;

	WPPLPlanner(SharedEnvironment* env): env(env), action_model(env){};
    // WPPLPlanner(){env = new SharedEnvironment();};
	virtual ~WPPLPlanner(){
        // delete env; // it is maintained outside
    };


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    bool consider_rotation=true;
    string lifelong_solver_name;
    std::shared_ptr<LaCAM2::LaCAM2Solver> lacam2_solver;
    std::shared_ptr<LNS::LNSSolver> lns_solver;
    std::shared_ptr<Shadocks::MAPFPlanner> shadocks_planner;
    std::shared_ptr<HeuristicTable> heuristics;

    double max_step_time=0;

    std::shared_ptr<std::vector<float> > map_weights;
    nlohmann::json config;
    void load_configs();
    std::string load_map_weights(string weights_path);

    int max_execution_steps;

    CompetitionActionModelWithRotate action_model;    
    int max_task_completed;
    int num_task_completed=0;

};

}; // namespace WPPL