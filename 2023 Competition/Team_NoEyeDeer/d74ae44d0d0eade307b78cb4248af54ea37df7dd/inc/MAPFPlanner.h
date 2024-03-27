#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

#include "ecbs.h"



class MAPFPlanner
{
public:
    SharedEnvironment* env;

    MAPFPlanner(SharedEnvironment* env) : env(env) {};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<Action> & plan);
    virtual void plan_sample(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    void plan_all();
    void printAllPaths();
    // new added
    
    std::unique_ptr<cbs::ECBSSolver> solver;
    int default_ecbs_horizon = INT_MAX;
    int ecbs_horizon = 5;
    long failed_count = 0;
    int REPLAN_THR = 0;
    int replan_timer = REPLAN_THR;
    
    std::vector<std::list<State> > sol;
    // std::vector<int> current_goal_locations;
    // std::vector<int> next_goal_locations;
    bool plan_for_all;
    void getCostMap();
    // 以某个点为起点，到地图其他点的最短距离
    vector<vector<int>> costmaps; // costmap of every map loc
    vector<vector<int>>  predecessors;

    long plan_count = 0;
};
