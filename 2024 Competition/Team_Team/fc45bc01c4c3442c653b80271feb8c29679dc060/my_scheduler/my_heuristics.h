//
// Created by Chuanlong Zang on 02.02.25.
//
#include "my_types.h"
#include <queue>


#ifndef LIFELONG_MY_HEURISTICS_H
#define LIFELONG_MY_HEURISTICS_H

namespace MyPlanner{

    void init_heuristics(SharedEnvironment* env);

    void init_neighbor(SharedEnvironment* env);

    void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

    Neighbor getNeighbor(std::vector<Neighbor>* ns, int location);

    int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, std::vector<Neighbor>* ns);

    int get_h(SharedEnvironment* env, int source, int target);

    void export_heuristic_table(const std::string &filename);
}


#endif //LIFELONG_MY_HEURISTICS_H
