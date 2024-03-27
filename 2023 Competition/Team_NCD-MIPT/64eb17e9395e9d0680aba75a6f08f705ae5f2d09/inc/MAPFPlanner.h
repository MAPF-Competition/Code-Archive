#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

class MAPFPlanner
{
public:
    SharedEnvironment *env;

    std::vector<std::set<pair<int, int>>> map;
    std::vector<std::set<int>> closed_list;
    MAPFPlanner(SharedEnvironment *env) : env(env){};
    MAPFPlanner() { env = new SharedEnvironment(); };
    virtual ~MAPFPlanner() { delete env; };
    std::chrono::time_point<std::chrono::high_resolution_clock> start_plan_time;

    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);

    // Start kit dummy implementation
    void single_agent_plan(int start, int start_direct, int end, std::list<pair<pair<int, int>,pair<int, int>>> &path);
    void reserve(std::list<pair<pair<int, int>, pair<int, int>>> &path);
    void del_reservations(std::list<pair<pair<int, int>, pair<int, int>>> &path);
    void del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int, pair<int, int>>> getNeighbors(int location, int direction, int t);
    bool validateMove(int loc, int loc2, int t);
    vector<list<pair<pair<int, int>,pair<int,int>>>> old_paths;
    bool check(list<pair<int, int>> &path);
    vector<std::set<int>> blocked_edges[4];
    bool is_edge(int loc);
    int shift_id, max_time_limit;
};
