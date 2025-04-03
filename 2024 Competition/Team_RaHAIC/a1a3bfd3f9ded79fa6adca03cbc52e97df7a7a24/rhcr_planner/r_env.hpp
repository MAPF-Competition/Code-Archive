
//添加缺失的头文件
#include <vector>
#include <unordered_set>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <boost/functional/hash.hpp>

#include "r_planner.hpp"
#include "r_neighbor.hpp"
#include "r_planresult.hpp"
#include "r_ecbs.hpp"
#include "r_timer.hpp"

using namespace RHCR_Planner;

struct r_Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const r_Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Agent : " << c.agent1 << "-" << c.agent2 << " at Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Agent : " << c.agent1 << "-" << c.agent2 <<  "at Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};


struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    //std::cout << "进入overlap函数" << std::endl;
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        std::cout << "VC overlap" << std::endl;
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        std::cout << "EC overlap" << std::endl;
        return true;
      }
    }
    return false;
  }

  bool operator==(const Constraints& other) const {
    return vertexConstraints == other.vertexConstraints &&
           edgeConstraints == other.edgeConstraints ;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  Location() : x(-1), y(-1) {}  // 默认构造函数
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std


///
class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = false)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal),
        m_heat_map()
        
  {
    if (m_goals.empty()) {
      std::cerr << "Error: m_goals is empty." << std::endl;
      throw std::runtime_error("m_goals is empty");
    }
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  float getHeatValue(const State2& s) {
    
    return m_heat_map[s.loc.first][s.loc.second];
  }

  // 添加新的函数来设置临时障碍物 -hzj
    void setTemporaryObstacles(const std::vector<State2>& initialStates,
                              const std::unordered_set<size_t>& activeAgents) {
        // 每次设置前都先清空
        m_temp_obstacles.clear();

        // 添加新的临时障碍物
        for (size_t i = 0; i < initialStates.size(); ++i) {
            if (activeAgents.count(i) == 0) {
                m_temp_obstacles.insert(Location(
                    initialStates[i].loc.first, 
                    initialStates[i].loc.second));
            }
        }
    }
  
  // 辅助函数 -hzj
  int countNearbyInactiveAgents(const State2& state, int radius = 2) {
    int count = 0;
    int x = state.loc.first;
    int y = state.loc.second;
    
    // 检查周围区域内的非活跃智能体
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            int newX = x + dx;
            int newY = y + dy;
            
            // 检查边界
            if (newX >= 0 && newX < m_dimx && newY >= 0 && newY < m_dimy) {
                // 检查该位置是否有非活跃智能体（临时障碍物）
                if (m_temp_obstacles.find(Location(newX, newY)) != m_temp_obstacles.end()) {
                    count++;
                }
            }
        }
    }
    return count;
}

  void calculate_heat_map(const std::vector<State2>& startStates) { 
    int rows = m_dimy; 
    int cols = m_dimx; 
    // std::cout << "rows = " << rows << ", cols = " << cols << std::endl; 
    m_heat_map.resize(cols,std::vector<float>(rows, 0.0)); 
    // std::cout << "startStates.size() = " << startStates.size() << std::endl; 
    for (const auto& startState : startStates) { 
      // agent_map[startState.loc.first][startState.loc.second] = 1; 
      // 1 for agent, 0 for empty 
      int directions[9][2] = {{0, 0}, {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}; 
      for (auto& dir : directions) { 
        int ni = startState.loc.second + dir[0]; 
        int nj = startState.loc.first + dir[1]; 
        if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) { 
          m_heat_map[nj][ni] += 0.1; 
        } 
      } 
    } 
    // //遍历每一行
    // for (int i = 0; i < m_heat_map[0].size(); ++i) {
    //     // 遍历每一列
    //     for (int j = 0; j < m_heat_map.size(); ++j) {
    //         std::cout << m_heat_map[j][i] << " "; // 打印热量值
    //     }
    //     std::cout << std::endl; // 换行
    // }
  }

  int admissibleHeuristic(const State2& s) {
    if (m_goals.empty()) {
      std::cerr << "Error: m_goals is empty." << std::endl;
      throw std::runtime_error("m_goals is empty");
    }

    // 检查 m_agentIdx 是否在有效范围内
    if (m_agentIdx >= m_goals.size()) {
      std::cerr << "Error: m_agentIdx out of range." << std::endl;
      throw std::runtime_error("m_agentIdx out of range");
    }
    return std::abs(s.loc.first - m_goals[m_agentIdx].x) +
           std::abs(s.loc.second - m_goals[m_agentIdx].y) + getHeatValue(s);
  }

  // low-level
  int focalStateHeuristic(
      const State2& s, int /*gScore*/,
      const std::vector<PlanResult<State2, Action, float> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State2 state2 = getState(i, solution, s.timestep);
        if (s.equalExceptTime(state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State2& s1a, const State2& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State2, Action, float> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State2 s2a = getState(i, solution, s1a.timestep);
        State2 s2b = getState(i, solution, s1b.timestep);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State2, Action, float> >& solution) {
    int numConflicts = 0;
    
    // 注释掉，没用max_t了 -hzj
    // int max_t = 0;
    // for (const auto& sol : solution) {
    //   max_t = std::max<int>(max_t, sol.states.size() - 1);
    // }
    
    // 修改t < mat_t 为 t < 5 -hzj
    for (int t = 0; t < 5; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State2 state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State2 state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State2 state1a = getState(i, solution, t);
        State2 state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State2 state2a = getState(j, solution, t);
          State2 state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State2& s) {
    return s.loc.first == m_goals[m_agentIdx].x && s.loc.second == m_goals[m_agentIdx].y &&
           s.timestep > m_lastGoalConstraint;
  }

  State2 result_state2(const State2& s, Action action) {
    std::pair<int,int> loc_new = s.loc;
    int ori_new = s.orientation;
    if (action == Action::FW)
        {
            if (s.orientation == 0)
                loc_new = {s.loc.first + 1, s.loc.second};
            else if (s.orientation == 1)
                loc_new = {s.loc.first, s.loc.second + 1};
            else if (s.orientation == 2)
                loc_new = {s.loc.first - 1, s.loc.second};
            else if (s.orientation == 3)
                loc_new = {s.loc.first , s.loc.second - 1};
        }
        else if (action == Action::CR)
        {
            ori_new = (s.orientation + 1) % 4;
      
        }
        else if (action == Action::CCR)
        {
            ori_new = (s.orientation - 1) % 4;
            if (ori_new == -1)
                ori_new = 3;
        }

        return State2(loc_new,s.timestep + 1,ori_new); 
  }


  // 获取m_heat_map表
  const std::vector<std::vector<float>>& getHeatMap() const {
      return m_heat_map;
  }

  void getNeighbors(const State2& s,
                    std::vector<Neighbor<State2, Action, int> >& neighbors) {
    neighbors.clear();
        {
            State2 n = result_state2(s, Action::FW);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    Neighbor<State2, Action, int>(n, Action::FW, 1));
            }
        }
        {
            State2 n = result_state2(s, Action::CR);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    Neighbor<State2, Action, int>(n, Action::CR, 1));
            }
        }
        {
            State2 n = result_state2(s, Action::CCR);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    Neighbor<State2, Action, int>(n, Action::CCR, 1));
            }
        }
        {
            State2 n = result_state2(s, Action::W);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    Neighbor<State2, Action, int>(n, Action::W, 1));
            }
        }
  }

  std::vector<std::vector<int>> makeAgentAreas(const std::vector<State2>& initialStates, int width , int overlap)
  {
    int x_area_num = std::ceil((m_dimx - overlap) / (double)(width - overlap));
    int y_area_num = std::ceil((m_dimy - overlap) / (double)(width - overlap));
    std::vector<std::vector<int>> area_agent(x_area_num * y_area_num);
    for (int i = 0; i < initialStates.size(); ++i)
    {
      int x = initialStates[i].loc.first;
      int y = initialStates[i].loc.second;
      std::vector<int> regions;

        // 遍历所有可能的区域
        for (int col = 0; col < x_area_num; ++col) {
            for (int row = 0; row < y_area_num; ++row) {
                // 计算当前区域的范围
                int startX = col * (width - overlap);
                int endX = std::min(startX + width, m_dimx);
                int startY = row * (width - overlap);
                int endY = std::min(startY + width, m_dimy);

                // 检查坐标是否在当前区域范围内
                if (x >= startX && x < endX && y >= startY && y < endY) {
                    // 计算区域编号
                    int regionIndex = row * (x_area_num) + col ;
                    area_agent[regionIndex].push_back(i);
                }
            }
        }
    }
    return area_agent;

  }

  bool getFirstConflict_areas(
    const std::vector<PlanResult<State2, Action, float> >& solution,
      r_Conflict& result, std::vector<std::vector<int>>& Area_Agent)
  {
    //注释掉 没用max_t了 -hzj
    // int max_t = 0;
    // for (const auto& sol : solution) {
    //   max_t = std::max<int>(max_t, sol.states.size() - 1);
    // }

    for (int t = 0; t <= 3; ++t) {
      std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> has_checked;
      for (int k = 0; k < Area_Agent.size(); ++k)
      {
        for (int m = 0; m < Area_Agent[k].size(); ++m)
        {
          State2 state1a = getState(Area_Agent[k][m], solution, t);
          State2 state1b = getState(Area_Agent[k][m], solution, t + 1);

          for (int n = m + 1; n < Area_Agent[k].size(); ++n)
          {            
            State2 state2a = getState(Area_Agent[k][n], solution, t);
            State2 state2b = getState(Area_Agent[k][n], solution, t + 1);
            if (has_checked.count(std::make_pair(Area_Agent[k][m],Area_Agent[k][n])) == 0 && 
            has_checked.count(std::make_pair(Area_Agent[k][n],Area_Agent[k][m])) == 0 )
            {
              
              if (state1a.equalExceptTime(state2a))
              {
                result.time = t;
                result.agent1 = Area_Agent[k][m];
                result.agent2 = Area_Agent[k][n];
                result.type = r_Conflict::Vertex;
                result.x1 = state1a.loc.first;
                result.y1 = state1a.loc.second;
                return true;
              }
              if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
                result.time = t;
                result.agent1 = Area_Agent[k][m];
                result.agent2 = Area_Agent[k][n];
                result.type = r_Conflict::Edge;
                result.x1 = state1a.loc.first;
                result.y1 = state1a.loc.second;
                result.x2 = state1b.loc.first;
                result.y2 = state1b.loc.second;
                return true;
              }
              has_checked.insert(Area_Agent[k][m] < Area_Agent[k][n] ? 
              std::make_pair(Area_Agent[k][m],Area_Agent[k][n]) : std::make_pair(Area_Agent[k][n],Area_Agent[k][m]));  
            }
            
          }
        }

      }
    }
    return false;
  }

bool getFirstConflict(
    const std::vector<PlanResult<State2, Action, float>>& solution,
    r_Conflict& result) {
    
    // 更新位置缓存 -hzj
    updatePositionCache(solution);
    
    // 检查t<=3的时间步
    for (int t = 0; t <= 3; ++t) {
        // 只遍历活跃智能体
        for (size_t i : activeAgents) {
            if (t >= m_agent_positions[i].size()) continue;
            const auto& pos1 = m_agent_positions[i][t];
            
            // 边界检查
            if (pos1.first < 0 || pos1.first >= m_dimx || 
                pos1.second < 0 || pos1.second >= m_dimy) {
                continue;
            }
            
            // 检查在当前位置的其他智能体（顶点冲突）
            const auto& agents_here = m_agent_grid[t][pos1.first][pos1.second];
            for (size_t j : agents_here) {
                // 跳过自己和非活跃智能体
                if (j <= i || !activeAgents.count(j)) continue;
                
                // 顶点冲突已确认
                result.time = t;
                result.agent1 = i;
                result.agent2 = j;
                result.type = r_Conflict::Vertex;
                result.x1 = pos1.first;
                result.y1 = pos1.second;
                return true;
            }
            
            // 检查边缘冲突
            if (t < 3) {
                const auto& pos1_next = m_agent_positions[i][t + 1];
                
                // 边界检查
                if (pos1_next.first < 0 || pos1_next.first >= m_dimx || 
                    pos1_next.second < 0 || pos1_next.second >= m_dimy) {
                    continue;
                }
                
                // 只检查活跃智能体
                for (size_t j : activeAgents) {
                    if (j <= i) continue;
                    
                    const auto& pos2 = m_agent_positions[j][t];
                    const auto& pos2_next = m_agent_positions[j][t + 1];
                    
                    // 使用曼哈顿距离剪枝
                    int manhattan_dist = std::abs(pos1.first - pos2.first) + 
                                      std::abs(pos1.second - pos2.second);
                    if (manhattan_dist > 2) continue;
                    
                    // 检查边缘冲突
                    if (pos1 == pos2_next && pos1_next == pos2) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = r_Conflict::Edge;
                        result.x1 = pos1.first;
                        result.y1 = pos1.second;
                        result.x2 = pos1_next.first;
                        result.y2 = pos1_next.second;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}
// // -hzj
// bool getFirstConflict(
//     const std::vector<PlanResult<State2, Action, float>>& solution,
//     r_Conflict& result) {
    

//     // 只检查前3个时间步
//     for (int t = 0; t <= 3; ++t) {
//         // 合并顶点冲突和边缘冲突的检查
//         for (size_t i : activeAgents) {
//             State2 state1a = getState(i, solution, t);
//             State2 state1b = getState(i, solution, t + 1);
            
//             for (size_t j : activeAgents) {
//                 if (j <= i) continue;
                
//                 State2 state2a = getState(j, solution, t);
                
//                 // 计算两个智能体在当前时间步的曼哈顿距离
//                 int manhattan_dist = std::abs(state1a.loc.first - state2a.loc.first) + 
//                                    std::abs(state1a.loc.second - state2a.loc.second);
                
//                 // 如果距离大于2，这个时间步不可能发生冲突
//                 if (manhattan_dist > 2) continue;
                
//                 // 检查顶点冲突
//                 if (state1a.equalExceptTime(state2a)) {
//                     result.time = t;
//                     result.agent1 = i;
//                     result.agent2 = j;
//                     result.type = r_Conflict::Vertex;
//                     result.x1 = state1a.loc.first;
//                     result.y1 = state1a.loc.second;
//                     return true;
//                 }
                
//                 // 检查边缘冲突
//                 State2 state2b = getState(j, solution, t + 1);
//                 if (state1a.equalExceptTime(state2b) && 
//                     state1b.equalExceptTime(state2a)) {
//                     result.time = t;
//                     result.agent1 = i;
//                     result.agent2 = j;
//                     result.type = r_Conflict::Edge;
//                     result.x1 = state1a.loc.first;
//                     result.y1 = state1a.loc.second;
//                     result.x2 = state1b.loc.first;
//                     result.y2 = state1b.loc.second;
//                     return true;
//                 }
//             }
//         }
//     }
//     return false;
// }

  // bool getFirstConflict(
  //     const std::vector<PlanResult<State2, Action, float> >& solution,
  //     r_Conflict& result) {
    
  //   // 直接使用全局的 activeAgents -hzj
  //   for (int t = 0; t <= 3; ++t) {
  //     // check drive-drive vertex collisions
  //     for (size_t i : activeAgents) {
  //       State2 state1 = getState(i, solution, t);

  //       for (size_t j : activeAgents) {
  //         // 只检查不同的智能体对，且保证每对只检查一次
  //         if (j <= i) continue;

  //         State2 state2 = getState(j, solution, t);
  //         if (state1.equalExceptTime(state2)) {
  //           result.time = t;
  //           result.agent1 = i;
  //           result.agent2 = j;
  //           result.type = r_Conflict::Vertex;
  //           result.x1 = state1.loc.first;
  //           result.y1 = state1.loc.second;
  //           return true;
  //         }
  //       }
  //     }
  //     // drive-drive edge (swap)
  //     for (size_t i : activeAgents) {
  //       State2 state1a = getState(i, solution, t);
  //       State2 state1b = getState(i, solution, t + 1);
  //       for (size_t j : activeAgents) {
  //         // 只检查不同的智能体对，且保证每对只检查一次
  //         if (j <= i) continue;  
          
  //         State2 state2a = getState(j, solution, t);
  //         State2 state2b = getState(j, solution, t + 1);
  //         if (state1a.equalExceptTime(state2b) &&
  //             state1b.equalExceptTime(state2a)) {
  //           result.time = t;
  //           result.agent1 = i;
  //           result.agent2 = j;
  //           result.type = r_Conflict::Edge;
  //           result.x1 = state1a.loc.first;
  //           result.y1 = state1a.loc.second;
  //           result.x2 = state1b.loc.first;
  //           result.y2 = state1b.loc.second;
  //           return true;
  //         }
  //       }
  //     }
  //   }

  //   return false;
  // }

  void createConstraintsFromConflict(
      const r_Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == r_Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == r_Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  bool isGoalAsConstraint(int idx, const Constraints& cons) {
    for (const auto& vc : cons.vertexConstraints) {
      if (vc.x == m_goals[idx].x && vc.y == m_goals[idx].y) {
        return true;
      }
    }
    for (const auto& ec : cons.edgeConstraints) {
      if (ec.x2 == m_goals[idx].x && ec.y2 == m_goals[idx].y) {
        return true;
      }
    }
    return false;
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State2& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  // 在 r_env.hpp 的 Environment 类中添加 -hzj
std::pair<int, int> get_goal(size_t agentIdx) const {
    return std::make_pair(m_goals[agentIdx].x, m_goals[agentIdx].y);
}
 
 private:
  State2 getState(size_t agentIdx,
                 const std::vector<PlanResult<State2, Action, float> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
    //   return State2(-1, -1 * (agentIdx+1), -1);
      return State2({-10 * (agentIdx + 1), -20 * (agentIdx + 1)}, -1, 0);
    }
    return solution[agentIdx].states.back().first;
  }
  
  // -hzj
  void updatePositionCache(const std::vector<PlanResult<State2, Action, float>>& solution) {
    const int MAX_TIMESTEP1 = 4;  // t <= 3 需要 t+1 的数据
    
    // 初始化 agent positions 缓存
    m_agent_positions.clear();
    m_agent_positions.resize(solution.size());
    for (size_t i : activeAgents) {
        m_agent_positions[i].resize(MAX_TIMESTEP1);
    }
    
    // 初始化网格缓存
    m_agent_grid.clear();
    m_agent_grid.resize(MAX_TIMESTEP1);
    for (int t = 0; t < MAX_TIMESTEP1; ++t) {
        m_agent_grid[t].resize(m_dimx);
        for (int x = 0; x < m_dimx; ++x) {
            m_agent_grid[t][x].resize(m_dimy);
            // 不需要预先分配 vector<size_t>，让它根据需要增长
        }
    }
    
    // 填充缓存
    for (size_t i : activeAgents) {        
        for (int t = 0; t < MAX_TIMESTEP1; ++t) {
            State2 state = getState(i, solution, t);
            int x = state.loc.first;
            int y = state.loc.second;
            
            // 边界检查
            if (x >= 0 && x < m_dimx && y >= 0 && y < m_dimy) {
                // 更新位置缓存
                m_agent_positions[i][t] = state.loc;
                
                // 更新网格缓存
                m_agent_grid[t][x][y].push_back(i);
            }
        }
    }
}

  // 新加临时障碍逻辑 -hzj
  bool stateValid(const State2& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.loc.first >= 0 && s.loc.first < m_dimx && s.loc.second >= 0 && s.loc.second < m_dimy &&
           m_obstacles.find(Location(s.loc.first, s.loc.second)) == m_obstacles.end() &&
           m_temp_obstacles.find(Location(s.loc.first, s.loc.second)) == m_temp_obstacles.end() &&
           con.find(VertexConstraint(s.timestep, s.loc.first, s.loc.second)) == con.end();
  }

  bool transitionValid(const State2& s1, const State2& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.timestep, s1.loc.first, s1.loc.second, s2.loc.first, s2.loc.second)) ==
           con.end();
  }

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::unordered_set<Location> m_temp_obstacles;  // 存储临时障碍物（非活跃智能体位置）-hzj
  // 网格缓存，记录每个位置的智能体 -hzj
  // 修改成员变量声明
    using AgentList = std::vector<size_t>;
    using GridCell = std::vector<AgentList>;
    using GridRow = std::vector<GridCell>;
    using GridTimestep = std::vector<GridRow>;
  GridTimestep m_agent_grid;  // [timestep][x][y]
  // 缓存每个智能体在每个时间步的位置 -hzj
  std::vector<std::vector<std::pair<int,int>>> m_agent_positions;  // [agent_id][timestep]

  std::vector<Location> m_goals;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
  std::vector<std::vector<float>> m_heat_map;
};