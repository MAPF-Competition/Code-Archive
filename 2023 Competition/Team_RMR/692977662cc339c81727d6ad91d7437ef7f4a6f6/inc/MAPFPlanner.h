#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "elrandom.hh"
#include "utils.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env) {epsilon=0.5;};
    MAPFPlanner() {env = new SharedEnvironment(); epsilon=0.5; };
	virtual ~MAPFPlanner(){delete env;};


    void setEpsilon(double e){epsilon=e;}

    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>> single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);
    pair<int,int> simulateAction(pair<int,int> loc_or, Action action);
    //Added by me
    void printActions(const vector<Action>& actions) const;
    bool feasibleAction(int i, const vector<Action>& actions) const;

  private:
    double epsilon;  
    vector<double> eps_vec;
    vector<list<pair<int,int>> > path;  //The current path for each agent
    vector<bool> on_path;  //It tells whether the agent is on path (true), or we should recompute ti (false)
};
