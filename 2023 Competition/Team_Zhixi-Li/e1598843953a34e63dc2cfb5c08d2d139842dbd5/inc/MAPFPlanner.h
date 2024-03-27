#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "KivaGraph.h"
#include "PBS.h"
#include "MAPFSolver.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

    KivaGrid* G;
    PBS* pbs;

    vector<State> starts;
    vector< vector<pair<int, int> > > goal_locations;
    bool has_plan;
    int timestep;
    int plan_timestep;
    int simulation_window;
    int planning_window;
    int skip_time = 0;
    std::vector<Path> paths;



	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);
    virtual void plan2(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    // 相关函数
    void update_start_locations();
    void update_goal_locations();
    void solve();
    void updatePaths(vector<Path> solver_paths, int max_timestep);

    bool needReplan();

};
