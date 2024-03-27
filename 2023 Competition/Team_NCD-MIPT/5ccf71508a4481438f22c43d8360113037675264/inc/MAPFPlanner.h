#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "skeleton.h"
#include "ActionModel.h"
#include <fstream>

class MAPFPlanner
{
public:
    SharedEnvironment *env;

    std::set<pair<int, int>> * map;
    std::set<int> *closed_list;
    MAPFPlanner(SharedEnvironment *env) : env(env){};
    MAPFPlanner() { env = new SharedEnvironment(); };
    virtual ~MAPFPlanner() {
        delete [] map;
        delete [] mapFree; 
        delete [] freeCells;
        delete [] nebors;
        delete [] curr_states;
        delete [] goal_locations;
        delete env;
        delete [] closed_list;
        delete []safePlace;
        delete []safePlaceFree;
        delete []dangerousPlace;
        delete []dangerousPlaceFree;
        for(int i=0; i<4; ++i){
            delete [] blocked_edges[i];
        }
    };
    std::chrono::time_point<std::chrono::high_resolution_clock> start_plan_time;

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
    int shift_id, max_time_limit, shift_id1;
    bool isMapWarehouse();
    bool isMapSortation();
    bool isMapCity();
    bool isMapRandom();
    bool isMapGame();
    bool flagMapisWarehouse;
    bool flagIsMapSortation;
    void checkReservations();
    void compressFreeCells();
    int timeStep;
    bool * mapFree;
    bool * safePlace, * safePlaceFree;
    bool * dangerousPlace, * dangerousPlaceFree;
    void genAllNeighors();
    int * freeCells, * row, * col, * nebors;
    bool stopFlag;
    pair<int, int> * curr_states;
    int * goal_locations;
    bool * isGoal;
    vector<int> safeValues;
    int numOfFreeCells;
    int mx_edge_x, mx_edge_y;
    int safeExtraLine;
};
