#ifndef MYSCHEDULER
#define MYSCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>

namespace MyScheduler{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_from_t(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_from_twm(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_from_a(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_from_awm(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_pibt(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_pibt_starter(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

void addToHeatMap(SharedEnvironment* env, vector<int> locations);     
void addAgentsToHeatMap(SharedEnvironment* env, vector<State> states);     
void removeFromHeatMap(SharedEnvironment* env, vector<int> locations);  
double getFromHeatMap(SharedEnvironment* env, vector<int> locations);     
void addToHeatMapWP(SharedEnvironment* env, vector<int> locations);   
void removeFromHeatMapWP(SharedEnvironment* env, vector<int> locations);  
double getFromHeatMapWP(SharedEnvironment* env, vector<int> locations);   
void addToHeatFlowMap(SharedEnvironment* env, vector<int> locations);   
void removeFromHeatFlowMap(SharedEnvironment* env, vector<int> locations);  
double getFromHeatFlowMap(SharedEnvironment* env, vector<int> locations);   
void createHeatMap(SharedEnvironment* env, int gridsize);
bool bottleneck(SharedEnvironment* env, vector<int> locations);
void addToHeatTimeMap(SharedEnvironment* env, vector<int> locations, std::vector<int> stays);
void removeFromHeatTimeMap(SharedEnvironment* env, int added_time, vector<int> locations);
bool timedBottleneck(SharedEnvironment* env, vector<int> locations, std::vector<int> stays);
double getFromHeatTimeMap(SharedEnvironment* env, vector<int> locations, std::vector<int> stays);

void task_finished(SharedEnvironment* env, int agent_id);
void remove_task_from_reducedmap(int task_id);
void process_new_task(SharedEnvironment* env, Task task);
void add_task_to_reducedmap(int task_id);
void hierarchyMapStart(SharedEnvironment* env, int grid);
void hierarchyMapWaypointsStart(SharedEnvironment* env, int grid);
int hierarchy_single_agent_plan(SharedEnvironment* env, int start,int start_direct,int end, int top_left_x, int top_left_y, int grid);  
list<pair<int,int>> getNeighbors(SharedEnvironment* env, int location,int direction);
list<pair<int,int>> getNeighborsWithTurn(SharedEnvironment* env, int location,int direction);
void reduceMapStart(SharedEnvironment* env);
bool reduceMap(SharedEnvironment* env, bool keepSalient);
void reduceMapUpdate(SharedEnvironment* env);
bool areNeighboursTraversable(SharedEnvironment* env, int location);
bool isNorthEastEmpty(SharedEnvironment* env, int location, int x, int y);
bool isSouthEastEmpty(SharedEnvironment* env, int location, int x, int y);
bool isNorthWestEmpty(SharedEnvironment* env, int location, int x, int y);
bool isSouthWestEmpty(SharedEnvironment* env, int location, int x, int y);
int countReducedVerticalNeighbours(SharedEnvironment* env, int location);
int countReducedHorizontalNeighbours(SharedEnvironment* env, int location);
void reduceMapWaypointsStart(SharedEnvironment* env);
bool reduceReduceMapWaypoints(SharedEnvironment* env, int distance);
void printReducedMap(SharedEnvironment* env);
int getManhattanDistance(SharedEnvironment* env, int loc1, int loc2);
int reducedMapWaypointsAStar(SharedEnvironment* env, int start, int end, vector<int> & waypoints); 
int singleAgentAStar(SharedEnvironment* env, int start, int end, std::vector<int> & waypoints, std::vector<int> & stays);
int single_agent_plan(SharedEnvironment* env, int start, int end, vector<int> & waypoints);  
void update_waiting_tasks();
int BFStoA(SharedEnvironment* env,TimePoint endtime, int start); 
int BFStoWorkingA(SharedEnvironment* env,TimePoint endtime, int start); 
void BFStoT(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<pair<int,int>> & tasks); 
void BFStoTWithTurn(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<pair<int,int>> & tasks); 
void single_agent_BFStoT(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<pair<int,int>> & tasks); 
pair<int,int> BFStoReducedMap(SharedEnvironment* env, int start); 
list<int> getPlainNeighbors(SharedEnvironment* env, int location);
bool validateMove(SharedEnvironment* env, int loc, int loc2);
void updateAgentMap(SharedEnvironment* env);

}

#endif