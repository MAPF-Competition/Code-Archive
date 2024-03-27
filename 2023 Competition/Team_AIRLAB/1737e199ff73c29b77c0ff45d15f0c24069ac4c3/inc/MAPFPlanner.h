#pragma once
#include <ctime>

#include "ActionModel.h"
#include "Instance.h"
#include "LNS.h"
#include "PuzzleGrid.h"
#include "SharedEnv.h"
#include "customcommon.h"

class MAPFPlanner {
 public:
  SharedEnvironment* env;
  Instance instance;
  LNS* lns;
  PuzzleGrid* puzzleGrid;
  float ratio_frag = 1.0;
  vector<int> frag;

  int time_interval = 15;
  int time_step = 0;
  double running_time = 0;
  double longest_time = 0;
  float cnt_overtime = 0;


  bool succ;
  int time_limit = 7200;
  string initAlgo = "PP";
  string replanAlgo = "PP";
  string destoryStrategy = "Adaptive";
  int neighborSize = 8;
  int maxIterations = 0;
  bool initLNS = true;
  string initDestoryStrategy = "Adaptive";
  bool sipp = true;
  int screen = 0;

  MAPFPlanner(SharedEnvironment* env) : env(env), instance(Instance(env, 5)){};
  MAPFPlanner() {
    env = new SharedEnvironment();
    instance = Instance(env, 5);
  };
  string outputPaths = "path.txt";
  string output = "test";

  virtual ~MAPFPlanner() {
    delete puzzleGrid;
    delete env;
  };

  virtual void initialize(int preprocess_time_limit);

  // return next states for all agents
  virtual void plan(int time_limit, std::vector<Action>& plan);

  // Start kit dummy implementation
  std::list<pair<int, int>> single_agent_plan(int start, int start_direct, int end);
  int getManhattanDistance(int loc1, int loc2);
  std::list<pair<int, int>> getNeighbors(int location, int direction);
  bool validateMove(int loc, int loc2);
  void getConst(string map_name, int num_of_agents);
};
