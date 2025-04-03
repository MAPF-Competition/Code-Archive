#ifndef AMAPFD_SOC_PLANNER
#define AMAPFD_SOC_PLANNER

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

namespace AMAPFDSOCPlanner{
extern long long number_of_free_cells;
extern int *free_cells;
extern int *inverse_free_cells;
extern vector<int> *neighbors;

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
}

#endif