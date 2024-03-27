#include "Priority.h"
vector<int> Priority::getPriority(int ascending) {
  vector<int> priority(shuffled_agents.size(), 0);
  vector<int> direction = {1,
                           puzzleGrid.cols,
                           -1,
                           -puzzleGrid.cols,
                           1 + puzzleGrid.cols,
                           1 - puzzleGrid.cols,
                           -1 + puzzleGrid.cols,
                           -1 - puzzleGrid.cols};
  // 각 에이전트 주변의 에이전트/장애물 수 계산
  for (int i = 0; i < shuffled_agents.size(); i++) {
    int agent_idx = shuffled_agents[i];
    int agent_loc = agents[agent_idx].path_planner->start_state.location;

    for (int dir : direction) {
      int adjacent_loc = agent_loc + dir;
      if (adjacent_loc >= 0 && adjacent_loc < size) {
        if (agent_existed[adjacent_loc] != 0) {
          if (agent_existed[adjacent_loc] == 1)  // agent
            around_num[i] += 1;
          else if (agent_existed[adjacent_loc] == 2)  // 장애물
            around_num[i] += 1;
        }
      }
    }
  }

  // 장애물 수에 따라 정렬 (동일한 경우 원래 순서 유지) -> 주변 장애물 수가 많은 에이전트가 먼저 (>)
  if (ascending == 1)
    std::stable_sort(shuffled_agents.begin(), shuffled_agents.end(),
                     [&](const int& a, const int& b) { return around_num[a] < around_num[b]; });
  else if (ascending == -1)
    std::stable_sort(shuffled_agents.begin(), shuffled_agents.end(),
                    [&](const int& a, const int& b) { return around_num[a] > around_num[b]; });

  // 동일한 주변 장애물 수를 가진 에이전트들을 랜덤으로 섞기
  auto rng = std::default_random_engine{};
  for (int i = 0; i < shuffled_agents.size();) {
    int start = i;
    int count = 1;
    while (i + 1 < shuffled_agents.size() &&
           around_num[shuffled_agents[i]] == around_num[shuffled_agents[i + 1]]) {
      i++;
      count++;
    }
    std::shuffle(shuffled_agents.begin() + start, shuffled_agents.begin() + start + count, rng);
    i++;
  }

  //   for (int i = 0; i < priority.size(); i++) {
  //     cout << shuffled_agents[i] << " "<< around_num[shuffled_agents[i]] << " // ";
  //     //cout << priority[i] << " "<< around_num[priority[i]] << " // ";
  //   }
  return shuffled_agents;
}

void Priority::makeExisted() {
  for (int i = 0; i < agents.size(); i++) {
    agent_existed[agents[i].path_planner->start_state.location] = 1;
  }
  for (int i = 0; i < puzzleGrid.puzzle_grid.size(); i++) {
    if (get<0>(puzzleGrid.puzzle_grid[i]) == -1) {
      agent_existed[get<1>(puzzleGrid.puzzle_grid[i])] = 2;
    }
  }
}