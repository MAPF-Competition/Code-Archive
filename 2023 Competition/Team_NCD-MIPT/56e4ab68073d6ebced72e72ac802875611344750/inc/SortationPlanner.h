#pragma once
#include <thread>
#include <ctime>
#include "SharedEnv.h"  
#include "skeleton.h"
#include "ActionModel.h"

class SortationPlanner
{
public:
    SharedEnvironment *env;

    bool *closed_list[111];
    SortationPlanner(SharedEnvironment *env) : env(env){};
    virtual ~SortationPlanner() {
        delete [] mapFree; 
        delete [] freeCells;
        delete [] nebors;
        delete [] emergency_nebors;
        delete [] curr_states;
        delete [] goal_locations;
        for(int i=0; i<111; ++i)
            delete [] closed_list[i];
        delete []safePlace;
        delete []safePlaceFree;
        delete []dangerousPlace;
        delete []dangerousPlaceFree;
        for(int i=0; i<4; ++i){
            delete [] blocked_edges[i];
        }
        delete []adj;
        delete []occupied;
        delete []nxt_occupied;
        delete []vis;
        delete []vis_local;
        delete []allowedToMove;
        delete []sz;
        delete []row;
        delete []col;
        delete []isGoal;
        delete []old_paths;
        delete []ignore;
        delete []solved;
        delete []out_nebors_cnt;
    };
    virtual void initialize(int preprocess_time_limit);
    void findSafePlaces();
    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);
    void partial_plan(int thread_id, int start_id, int end_id);
    // Start kit dummy implementation
    void single_agent_plan(int thread_id, int start, int start_direct, int end, std::list<pair<int, int>> &path);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int, int>> getNeighbors(int location, int direction, bool is_start);
    list<pair<int, int>> * old_paths;
    std::set<int> * blocked_edges[4];
    int shift_id[111], max_time_limit; 
    const int processor_count = std::thread::hardware_concurrency();
    void compressFreeCells();
    int timeStep;
    bool * mapFree;
    bool * safePlace, * safePlaceFree;
    bool * dangerousPlace, * dangerousPlaceFree;
    int * out_nebors_cnt;
    void genAllNeighors();
    int * freeCells, * row, * col, * nebors, *emergency_nebors;
    bool stopFlag, flagReplan;
    pair<int, int> * curr_states;
    int * goal_locations;
    bool * isGoal;
    vector<int> safeValues;
    int numOfFreeCells;
    int mx_edge_x, mx_edge_y;
    int safeExtraLine;
    vector<Action> lastActions;
    vector<int> *adj;
    int * occupied, * sz;
    vector<int> * nxt_occupied;
    bool * vis, * vis_local, *allowedToMove, *ignore;
    void dfs_fill_sizes(int x);
    void dfs_mark_valids(int x);
    bool dfs_fill_cycles(int x, int pa);
    void markWhoAllowedToMove();
    int dir_h, dir_v, dr=0, drh[4]={0,0,1,1}, drv[4]={0,1,1,0};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_plan_time;
    bool *solved;
    void move_away(int id);
    int W1, W2; //weights of A*
};