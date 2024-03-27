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


    // list of ints with the current location of the agents
    //std::vector<int> agent_current_locations;

    // vector of ints with the stuck state of the agents
    std::vector<int> agent_stuck;
    std::vector<int> random_goal_vector;
    std::vector<int> random_goal_time;
    
    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end, vector<int> occupancy_grid);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction, vector<int> occupancy_grid, int start, int start_direct);
    bool validateMove(int loc,int loc2, vector<int> occupancy_grid, int start, int start_direct);
};
