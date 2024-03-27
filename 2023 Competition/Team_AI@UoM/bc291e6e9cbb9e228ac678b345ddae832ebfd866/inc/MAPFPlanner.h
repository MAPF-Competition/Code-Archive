#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // Main planner.
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // A* Algorithm.
    std::vector<pair<int,int>>single_agent_plan(int start,int start_direct, int end, auto start_time);
    std::vector<pair<int,int>> getNeighbors(int location, int direction, int timestamp, int end);
    bool validateMove(int next_loc, int curr_loc, int direction, int timestamp, int end);
    int getManhattanDistance(int loc1, int loc2);

    // Helper functions.
    void validate_path(int index, vector<int>& to_recalculate);
    void set_actions(vector<Action>& actions);
    void recalculate_path(int index, auto start_time);
    void print_map(int ts);
};
