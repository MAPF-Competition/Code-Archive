#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include "heuristics.h"
#include <iostream>

#include <set>

namespace DefaultPlanner{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

extern std::vector<HeuristicTable> global_heuristictable;
extern Neighbors global_neighbors;

struct FW_Metric{
    int id;                     // agent id
    int deviation;              // deviation from the guided path
    int last_replan_t;          // last time the agent replanned
    int rand;                   // random number for tie breaking

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

struct FlowHeuristic{
    HeuristicTable* h; 
    int target;
    int origin;
    pqueue_min_of open;
    MemoryPool mem;


    bool empty(){
        return mem.generated() == 0;
    }
    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;                             // store the goal location of each agent

    TimePoint start_time;
    int t_ms=0;

    std::vector<Traj> trajs;                            // store the guided path of each agent

    std::vector<std::pair<int,int>> deviation_agents;   // store the deviation of each agent from the guided path

    std::vector<Int4> flow;                             // store the flow of each location, how many higher priority agents are moving to each direction
    std::vector<HeuristicTable>& heuristics;            // store the heuristic table for each goal location
    std::vector<Dist2Path> traj_dists;                  // store the distance to the guided path for each agent
    std::vector<s_node> goal_nodes;                     // store the nodes along the guided path towards goal for each agent, along with the cost information

    std::vector<FW_Metric> fw_metrics;                  // store the flow metric for each agent
    Neighbors& neighbors;                               // store the neighbor information of each location


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;                                        // store the sum of cost of each agent's guided path

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    TrajLNS(SharedEnvironment* env, std::vector<HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(heuristics),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors){
        };


    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};

    

};
}
#endif