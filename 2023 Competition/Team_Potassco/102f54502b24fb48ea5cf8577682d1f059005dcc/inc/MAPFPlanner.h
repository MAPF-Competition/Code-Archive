#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Map.h"
#include <clingo.hh>


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

    void add_instance(Clingo::Backend &bck, unsigned int delta);

    auto get_start(int agent_id) -> directional_pos;

    auto get_goal(int agent_id) -> directional_pos;

    int get_horizon(int agent_id);

    void get_next_step(std::vector<Action> & actions);

    void solve();

    Action convert_action(directional_pos start, directional_pos goal);

private:
    MAP map;
    std::vector< std::vector<Action> > computed_plans_;
    unsigned int last_plan_time_ = 0;
    bool first_solve = true;

    float preprocess_time_ = 0;
    unsigned int total_calls_ = 0;
    float total_time_ = 0;
    float total_solve_time_ = 0;
    unsigned int total_choices_ = 0;
    unsigned int total_conflicts_ = 0;
};
