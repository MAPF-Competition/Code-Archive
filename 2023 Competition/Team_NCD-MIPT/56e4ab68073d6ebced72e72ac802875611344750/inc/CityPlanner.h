#pragma once
#include <thread>
#include <ctime>
#include "SharedEnv.h"
#include "skeleton.h"
#include "ActionModel.h"
using namespace std;

class CityPlanner
{
public:
    SharedEnvironment *env;

    bool *closed_list[111];
    CityPlanner(SharedEnvironment *env) : env(env){};
    virtual ~CityPlanner() {
        delete [] mapFree; 
        delete [] freeCells;
        delete [] nebors;
        delete [] available;
        delete [] bridge;
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
        delete []tmp_paths;
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
    void single_agent_plan(int thread_id, int start, int start_direct, int end, std::deque<pair<int, int>> &path, bool wasCleared);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int, int>> getNeighbors(int location, int direction, bool is_start, bool wasCleared);
    deque<pair<int, int>> * old_paths, *tmp_paths;
    std::set<int> * blocked_edges[4];
    int shift_id[111], max_time_limit;   
    const int processor_count = std::thread::hardware_concurrency();
    bool flagMapisWarehouse, flagIsMapSortation, flagMapIsRandom, flagMapIsGame, flagMapIsCity;
    void compressFreeCells();
    int timeStep;
    bool * mapFree;
    bool * safePlace, * safePlaceFree;
    bool * dangerousPlace, * dangerousPlaceFree, *available, *bridge;
    int * out_nebors_cnt;
    void genAllNeighors();
    int * freeCells, * row, * col, * nebors, *emergency_nebors, *costEdge, *priority, *dangerous_branch, *wasCleared, *cellCost;
    vector<int> costEdgeVector;
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
    void bfs();
    void bfs_outer_cycle();
    bool connect(int i, int k, int ii);
    void dfs_similar(int x, int y, int id, vector<int>&vis, vector<int>&mp);
    void dfs_full(int x, int y, vector<int>&vis);
    void dfs_count_reach(int x, int y, vector<int>&vis, int& cnt);
    void move_away(int id);
    void dfs_one_cycle(int i, int j, int dir, bool ccw, vector<int> &mp, int id, int cnt, int i_st, int j_st, vector<int>&vis);
    bool onTheEdge(int i, int j, int dir, int ccw, vector<int> &mp, int id);
    int W1, W2; //weights of A*
    void writeMap();
    void writeGoals();
    void writePaths();
    void writePathsSteps();
    vector<pair<int, int>> bridges;
    void checkBridges();
    void assertBridges();
    // bool canGetToGoal(int i);
    bool dfs_fix_in(int i, int j, int pa);
    bool dfs_fix_out(int i, int j, int pa);
    bool dfsOccInBigDpth(int comp);
    int files_counter;
    void print_location(int loc);
    void print_agent_information(int ag);
    bool force_connect(int i, int k);
    vector<int> safe_places_vector;
    void dfs_safe_places(int x, bool *vis);
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    mutex mutex_cellCost;
};
