#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "skeleton.h"
#include "ActionModel.h"
// #include "map.h"
// #include "task.h"
// #include "cbs.h"

class CityPlanner2
{
public:
    SharedEnvironment *env;

    std::set<pair<int, int>> * map;
    // std::set<int> *closed_list;
    CityPlanner2(SharedEnvironment *env) : env(env){};
    virtual ~CityPlanner2() {
        delete [] map;
        delete [] mapFree; 
        delete [] freeCells;
        delete [] nebors;
        delete [] curr_states;
        delete [] goal_locations;
        // delete [] closed_list;
        delete []safePlace;
        delete []safePlaceFree;
        delete []dangerousPlace;
        delete []dangerousPlaceFree;
        for(int i=0; i<4; ++i){
            delete [] blocked_edges[i];
        }
        delete []wasSearched;
        delete []bestCost;
        delete []sortCost;
    };

    virtual void initialize(int preprocess_time_limit);
    void findSafePlaces();
    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);

    // Start kit dummy implementation
    void single_agent_plan(int start, int start_direct, int end, std::list<pair<pair<int, int>,pair<int, int>>> &path);
    void reserve(const std::list<pair<pair<int, int>, pair<int, int>>> &path);
    void del_reservations(const std::list<pair<pair<int, int>, pair<int, int>>> &path);
    void del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int, pair<int, int>>> getNeighbors(int location, int direction, int t);
    list<pair<pair<int, int>,pair<int,int>>> * old_paths;
    bool check(list<pair<int, int>> &path);
    std::set<int> * blocked_edges[4];
    bool is_safe_place(int loc);
    int shift_id;
    int max_time_limit;   
    void checkReservations();
    void compressFreeCells();
    int timeStep;
    int * mapFree;
    bool * safePlace, * safePlaceFree, *wasSearched;
    bool * dangerousPlace, * dangerousPlaceFree;
    void genAllNeighbors();
    int * freeCells, * row, * col, * nebors, *bestCost, *sortCost;
    bool stopFlag;
    pair<int, int> * curr_states;
    int * goal_locations;
    int * isGoal;
    vector<int> safeValues;
    int numOfFreeCells;
    int mx_edge_x, mx_edge_y;
    int safeExtraLine;
    vector<Action> lastActions;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_plan_time;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    void writeSafePlaces();
    void print_location(int loc);
    void print_agent_information(int ag);
    void single_agent_plan_dummy(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path);
    // Map map;
    // Task task;
    // CBS cbs;
    // void fillMap();
    // void fillTask();
    // void getSolutions();
};