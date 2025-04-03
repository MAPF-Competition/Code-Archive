//
// Created by take_ on 25-1-9.
//

#ifndef ASTAR_H
#define ASTAR_H

// #include "Types.h"
#include "basic_type.h"
// #include "utils.h"
#include "usage.h"
#include "node_pool.h"
#include "pqueue.h"
#include "astar_node.h"
// #include "heuristics.h"
#include "distance.h"


// a astar minimized the opposide traffic flow with existing traffic flow
s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
                             HeuristicTable& ht, Traj& traj,
                             const vector<vector<int>>& _extra_weights,
                             MemoryPool& mem, int start, int goal, Neighbors* ns);

// 无解的时候不要让整个代码停止运行, 留下改正的余地
bool astar_highway(SharedEnvironment* env, std::vector<Int4>& flow,
             HeuristicTable& ht, Traj& traj,
             const vector<vector<int>>& _extra_weights,
             MemoryPool& mem, int start, int goal, Neighbors* ns);


#endif // ASTAR_H
