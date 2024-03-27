#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "skeleton.h"
#include "ActionModel.h"
using namespace std;

class RandomPlanner2
{
public:
    SharedEnvironment *env;

    bool *closed_list;
    RandomPlanner2(SharedEnvironment *env) : env(env){};
    virtual ~RandomPlanner2() {
        delete [] mapFree; 
        delete [] freeCells;
        delete [] nebors;
        delete [] available;
        delete [] emergency_nebors;
        delete [] curr_states;
        delete [] goal_locations;
        delete [] closed_list;
        delete []is_escape_place;
        delete []dangerousPlaceFree;
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
        delete []escape_place;
        delete []passes;
        // for(int i=0; i<1000; ++i)
            delete cellCost;
    };

    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<Action> &plan);
    // Start kit dummy implementation
    deque<pair<int, int>>  single_agent_plan(int start, int start_direct, int end);
    deque<pair<int, int>>  single_agent_plan2(int start, int start_direct, int end);
    deque<pair<int, int>>  single_agent_plan_nonempty(int start, int start_direct);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int, int>> getNeighbors(int location, int direction, bool is_start);
    deque<pair<int, int>> * old_paths;
    int shift_id, max_time_limit; 
    void compressFreeCells();
    int timeStep;
    bool * mapFree;
    bool * is_escape_place, * dangerousPlaceFree, *available;
    int * escape_place;
    void genAllNeighors();
    int * freeCells, * row, * col, * nebors, *fixed_nebors, *emergency_nebors, *passes, *cellCost; //*cellCost[1000];
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
    int * occupied, * sz, *future_occupied;
    vector<int> * nxt_occupied;
    bool * vis, * vis_local, *allowedToMove, *ignore;
    void dfs_fill_sizes(int x);
    void dfs_mark_valids(int x);
    bool dfs_fill_cycles(int x, int pa);
    void markWhoAllowedToMove();
    std::chrono::time_point<std::chrono::high_resolution_clock> start_plan_time;
    bool *solved;
    int W1, W2; //weights of A*
    void writeMap();
    void writePaths();
    void writePathsSteps();
    int files_counter;
    void print_location(int loc, int dr);
    void print_agent_information(int ag);
    void my_assert(bool condition);
    void checkPasses();
    bool connect(int i, int k, int ii);
    void process_path(int start, deque<pair<int, int>> &path);
    vector<pair<pair<int, int>, int>> agents_vector;
    void sortAgents();
    void reConnectMap();
    deque<pair<int, int>>  single_agent_plan_passes(int start, int start_direct, int end);
    deque<pair<int, int>>  single_agent_plan_nonempty_passes(int start, int start_direct);
    list<pair<int, int>> getNeighborsPasses(int location, int direction, bool is_start);
    void dfsConnect(int x, vector<bool>&vis);
    bool isForbiddenToMove(int agent);
    pair<int, int> path_cost(int agent, deque<pair<int, int>> &path);
    void writeGaolPositions();
};
