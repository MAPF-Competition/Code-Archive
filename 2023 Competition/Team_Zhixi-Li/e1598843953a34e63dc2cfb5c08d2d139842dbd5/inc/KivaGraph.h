#pragma once
#include "BasicGraph.h"
#include "SharedEnv.h"


class KivaGrid :
	public BasicGraph
{
public:
	vector<int> endpoints;
	vector<int> agent_home_locations;

    bool load_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
    bool load_env_map(SharedEnvironment* env);
    void computeGoalHeuristics(int goal);
private:
    bool load_weighted_map(string fname);
    bool load_unweighted_map(string fname);
};
