#ifndef Z_SCHEDULER
#define Z_SCHEDULER

#include <limits.h>
#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>
#include <queue>
#include "SharedEnv.h"
#include <random>

namespace ZainPlanner{
extern long long number_of_free_cells;
extern int *free_cells;
extern int *inverse_free_cells;
extern bool map_is_random, map_is_warehouse;
extern vector<int> *neighbors;
extern int16_t *distancesBetweenCells;


void readMap(SharedEnvironment* env);
void init1();
void init2();
void dynInit(int num_of_goals);
void dynDel();
void del();
void compressFreeCells();
void genAllNeighbors();
void dynGenAllNeighbors(int num_of_goals);
int solve(int mx);
void readMap(SharedEnvironment* env);
void fillInputs(vector<int> &agents_locations, vector<pair<int, int>> &task_locations_and_costs);
void getAssignments(vector<int>&assignments);
void dynReset();
int manhattanCost(int cell1, int cell2, const SharedEnvironment *env);
int manhattanCost(Task &task, const SharedEnvironment *env);
int realCost(long long cell1, long long cell2);
int taskCost(Task &task);
// void calculateAndSaveRealDistances(SharedEnvironment *env);
// void readDistances(SharedEnvironment* env);

void getNewInputs(vector<pair<int, int>> &agents_locations, vector<pair<int, int>> &tasks_locations,  SharedEnvironment* env);


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
void refineTaskPool(SharedEnvironment &env);
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif