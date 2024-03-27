#pragma once
#include <random>

#include "BasicLNS.h"
#include "PuzzleGrid.h"
class Priority {
 public:
  std::vector<int> agent_existed;  // agent가 존재하는지 여부 (1: 존재, 2: 장애물, 0: 존재X)
  int size;                        // rows * cols

  vector<Agent>& agents;
  PuzzleGrid& puzzleGrid;
  vector<int> shuffled_agents;

  vector<float> around_num;  // 주변에 agent or 장애물이 몇개 있는지

  Priority(vector<Agent>& agents, PuzzleGrid& puzzleGrid, vector<int> shuffled_agents)
      : agents(agents),
        puzzleGrid(puzzleGrid),
        shuffled_agents(shuffled_agents),
        size(puzzleGrid.N) {
    agent_existed.resize(size, 0);
    around_num.resize(shuffled_agents.size(), 0);
    makeExisted();
  };

  void makeExisted();
  vector<int> getPriority(int ascending);
  // 소멸자
  ~Priority(){};
};
