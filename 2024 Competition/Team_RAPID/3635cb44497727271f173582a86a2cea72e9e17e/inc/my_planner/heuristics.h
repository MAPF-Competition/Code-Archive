
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace MyPlanner{

void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

int get_h(SharedEnvironment* env, int source, int target);

void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

void init_heuristic_rot(HeuristicTableRot& ht, SharedEnvironment* env,
                        int goal_loc, int goal_dir);

int get_heuristic_rot(HeuristicTableRot& ht,
                      SharedEnvironment* env,
                      int source_loc, int source_dir,
                      Neighbors* ns);

bool isForward(int curr_loc, int next_loc, int direction, int cols);

int get_h_rot(SharedEnvironment* env,
              int source_loc, int source_dir,
              int target_loc, int target_dir);

}
#endif