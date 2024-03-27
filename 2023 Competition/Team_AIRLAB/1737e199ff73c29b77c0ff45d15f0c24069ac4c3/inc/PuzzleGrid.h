#pragma once

#include "SharedEnv.h"
#include "SingleAgentSolver.h"
#include "StateTimeAStarNode.h"
#include "customcommon.h"

#define WEIGHT_MAX INT_MAX / 2

class PuzzleGrid {
 public:
  string fname;  // heuristic file name
  int rows, cols;
  int N;                                // whole grid size
  int num_of_ids;                       // number of pieces
  int depth;                            // depth of piece
  vector<tuple<int, int>> puzzle_grid;  // <id, location>
  vector<int> center_locations;         // location of ith id
  unordered_map<int, vector<double>> heuristics;
  vector<string> types;
  vector<vector<double>> weights;

  SharedEnvironment* env;
  int cells;
  vector<int> directions;
  vector<int> num_of_cells_in_id;

  PuzzleGrid(SharedEnvironment* env, std::string fname, int rows, int cols, int depth)
      : env(env), fname(fname), rows(rows), cols(cols), depth(depth) {
    N = rows * cols;
    directions = {1, env->cols, -1, -env->cols};
    cells = 9;
  }
  ~PuzzleGrid() {};

  void puzzleGridRead(std::string fname);

  void heuristicsAgent(SingleAgentSolver* path_planner);

  void puzzleGridPrint();

  void generatePuzzleGrid(std::string fname);

  void generatePuzzleByCell(int startLocation, int id);

  void generatePuzzleByDepth(int startLocation, int id);

  void postProcessing();

  bool load_unweighted_map(SharedEnvironment* env);

  void preprocessing(SharedEnvironment* env, PuzzleGrid* puzzleGrid);

  int get_Manhattan_distance(int loc1, int loc2) const {
    return abs(loc1 / cols - loc2 / cols) + abs(loc1 % cols - loc2 % cols);
  }

  bool load_heuristics_table(std::ifstream& myfile);
  vector<double> compute_heuristics(int root_location, std::vector<int>& puzzleGridCenterPoint);
  list<State> get_reverse_neighbors(const State& s) const;

  double get_weight(int from, int to) const;

  int get_direction(int from, int to) const;

  void save_heuristics_table(std::string fname, int step, int first_step,
                             std::vector<int>& puzzleGridCenterPoint);
  vector<int> getFrag(Instance& instance, SharedEnvironment* env, float frag_ratio);
};
