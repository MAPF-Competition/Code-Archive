#pragma once
#include <ctime>
#include <iomanip>
#include <random>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "planner.h"
#include "rl_module.h"
#include "BS_thread_pool.hpp"

class MAPFPlanner
{
public:
    SharedEnvironment* env;
    std::vector<planner> planners;
    RL_module actor;
    std::vector<int> agents_pos;
    std::vector<std::mt19937> generators;
    BS::thread_pool pool;
    std::vector<int> penalties;

    MAPFPlanner(SharedEnvironment* env): env(env) { };
    MAPFPlanner() { env = new SharedEnvironment(); };
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);
    Action get_action(int agent_idx);
    std::vector<float> generate_input(size_t a_id, int radius, const std::list<int>& path);
    std::vector<int> precompute_penalty_matrix();
    double get_avg_distance(int loc);
    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);
    void cooperate_actions(vector<Action> & actions);
    void revert_action(int agent_idx, int next_loc, std::unordered_map<int, std::set<int>>& used_cells, vector<Action>& actions);
};
