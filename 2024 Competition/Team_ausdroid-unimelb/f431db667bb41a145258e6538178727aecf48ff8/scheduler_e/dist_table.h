
#ifndef dt_heuristics_hpp
#define dt_heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"
#include <iostream>
#include <fstream>

namespace SchedulerE{

void dt_init_heuristics(SharedEnvironment* env);

void dt_init_neighbor(SharedEnvironment* env);

void dt_init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

int dt_get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

int dt_get_h(SharedEnvironment* env, int source, int target);
int dt_get_hh(SharedEnvironment* env, int source, int target);


void dt_init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> dt_get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int dt_get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

}
#endif