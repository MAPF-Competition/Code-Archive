#pragma once

#include <map>
#include <future>

#include "r_a_star_epsilon.hpp"
#include "r_timer.hpp"
#include "r_thread_pool.hpp"

namespace RHCR_Planner{


template <typename State2, typename Action, typename Cost, typename r_Conflict,
          typename Constraints, typename Environment>
class ECBS {
 public:
  ECBS(Environment& environment, float w) : m_env(environment), m_w(w) {}
      bool search1(const std::vector<State2>& initialStates,
              std::vector<PlanResult<State2, Action, Cost> >& solution) {
    
    Timer t1;
    
    //打印时间 -hzj
    Timer totalTimer;
    Timer initPathTimer;

    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.LB = 0;
    start.id = 0;


    m_env.calculate_heat_map(initialStates);
    
    // // 定义智能体优先级结构 -hzj
    // struct AgentPriority {
    //     size_t index;
    //     int distance;
        
    //     bool operator<(const AgentPriority& other) const {
    //         if (distance != other.distance)
    //             return distance > other.distance;  // 使用大顶堆，距离大的更容易被弹出
    //         return index > other.index;
    //     }
    // };

    // // 创建固定大小的优先队列 -hzj
    // const size_t MAX_ACTIVE_AGENTS = 1500;
    // std::priority_queue<AgentPriority> pq;
    // std::unordered_set<size_t> activeAgents;

    // // 计算所有智能体的距离并尝试加入优先队列 -hzj
    // for (size_t i = 0; i < initialStates.size(); ++i) {
    //   // 通过Environment获取目标位置
    //   // 直接使用目标位置计算距离
    // int distance =  std::abs(initialStates[i].loc.first - m_env.get_goal(i).first) +
    //               std::abs(initialStates[i].loc.second - m_env.get_goal(i).second);
        
    //     AgentPriority ap{i, distance};
        
    //     if (pq.size() < MAX_ACTIVE_AGENTS) {
    //         pq.push(ap);
    //         activeAgents.insert(i);
    //     } else if (ap < pq.top()) {  // 如果当前智能体比队列中最远的更近
    //         activeAgents.erase(pq.top().index);
    //         pq.pop();
    //         pq.push(ap);
    //         activeAgents.insert(i);
    //     }
    // }

    // // 将优先队列转换为vector以便遍历 -hzj
    // std::vector<size_t> prioritizedAgents;
    // while (!pq.empty()) {
    //     prioritizedAgents.push_back(pq.top().index);
    //     pq.pop();
    // }
    // std::reverse(prioritizedAgents.begin(), prioritizedAgents.end());  // 反转以保持距离升序

    // 添加一个智能体数量阈值和最大路径长度限制 -hzj
    const int AGENT_THRESHOLD = 200;  // 当智能体数量超过此值时启用路径长度限制
    const int MAX_PATH_LENGTH = 20;   // 最大路径长度限制

    // 根据智能体数量确定是否使用路径长度限制  -hzj
    bool useLengthLimit = initialStates.size() > AGENT_THRESHOLD;

    // 区域划分参数
    int area_width = 20;
    int area_overlap = 5;
    // 构建分区所包含的智能体id
    std::vector<std::vector<int>> Area_Agent; // n个区域，每个区域包含了当前区域的智能体id
    Area_Agent = m_env.makeAgentAreas(initialStates, area_width, area_overlap);

    // 初始化所有智能体的路径 -hzj
    for (size_t i = 0; i < initialStates.size(); ++i) {
        if (activeAgents.count(i) == 0) {
            // 非活跃智能体保持在原地
            start.solution[i].states.clear();
            start.solution[i].states.push_back(std::make_pair(initialStates[i], 0));
            start.solution[i].actions.clear();
            start.solution[i].cost = 0;
            start.solution[i].fmin = 0;
            continue;
        }

        if (i < solution.size() && solution[i].states.size() > 1) {
            assert(initialStates[i] == solution[i].states.front().first);
            start.solution[i] = solution[i];
        } else {
            LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                    start.solution);
            LowLevelSearch_t lowLevel(llenv, m_w);
            
            // 暂时注释掉这个时间，先不打印 -hzj
            //Timer lowLevelTimer;
            bool success = lowLevel.search(initialStates[i], start.solution[i], 3);
            
            // lowLevelTimer.stop();
            // std::cout << "Agent " << i << " lowLevel.search time: " 
            //           << lowLevelTimer.elapsedSeconds() << " size: "<< start.solution[i].states.size() << std::endl;

            if (!success) {
              // 如果3步内找不到路径，让智能体等待 -hzj
              start.solution[i].states.clear();
              start.solution[i].states.push_back(std::make_pair(initialStates[i], 0));
              continue;
                //return false;
            }
        }
        start.cost += start.solution[i].cost;
        start.LB += start.solution[i].fmin;
    }

    initPathTimer.stop();
    std::cout << "initPath.search time: "  << initPathTimer.elapsedSeconds()  << std::endl;



    // // 初始化每个智能体的初始路径
    // for (size_t i = 0; i < initialStates.size(); ++i) {
    //   if (i < solution.size() && solution[i].states.size() > 1) {
    //     assert(initialStates[i] == solution[i].states.front().first);
    //     start.solution[i] = solution[i];
    //   } else {
    //     LowLevelEnvironment llenv(m_env, i, start.constraints[i],
    //                               start.solution); 
    //     LowLevelSearch_t lowLevel(llenv, m_w);

    //     // 记录开始时间 -hzj
    //     Timer lowLevelTimer;
        
    //     bool success = lowLevel.search(initialStates[i], start.solution[i], 1);
        
    //     // 记录结束时间并计算耗时-hzj
    //         lowLevelTimer.stop();
    //         std::cout << "Agent " << i << " lowLevel.search time: " 
    //                   << lowLevelTimer.elapsedSeconds() << " size: "<< start.solution[i].states.size()<<std::endl;

    //     if (!success) {
    //       return false;
    //     }
    //   }
    //   start.cost += start.solution[i].cost;
    //   start.LB += start.solution[i].fmin;
    // }


    // 使用多线程进行初始路径
    // 创建固定大小的线程池
    // size_t numThreads = std::thread::hardware_concurrency();  // 根据 CPU 核心数设置线程池大小
    // std::cout << "线程池大小为: " << numThreads << std::endl;
    // ThreadPool pool(numThreads);

    // // 使用线程池并行处理每个智能体的路径规划
    // std::vector<std::future<std::pair<size_t, PlanResult<State2, Action, Cost>>>> futures;
    // for (size_t i = 0; i < initialStates.size(); ++i) {
    //     if (i < solution.size() && solution[i].states.size() > 1) {
    //         assert(initialStates[i] == solution[i].states.front().first);
    //         start.solution[i] = solution[i];
    //     } else {
    //       // 提交任务到线程池
    //       std::cout << "提交任务 " << i << " 到线程池" << std::endl;
    //         futures.push_back(pool.enqueue([this, i, &initialStates, &start] {
    //             LowLevelEnvironment llenv(m_env, i, start.constraints[i], start.solution);
    //             LowLevelSearch_t lowLevel(llenv, m_w);
    //             PlanResult<State2, Action, Cost> result;
    //             bool success = lowLevel.search(initialStates[i], start.solution[i], i);
    //             if (!success) {
    //                 throw std::runtime_error("Path planning failed for agent " + std::to_string(i));
    //             }
    //             return std::make_pair(i, result);
    //         }));
    //     }
    // }

    // // 等待所有任务完成
    // for (int m = 0; m < futures.size(); ++m) {
    //     start.cost += start.solution[m].cost;
    //     start.LB += start.solution[m].fmin;
    // }

    start.focalHeuristic = m_env.focalHeuristic1(start.solution);


    // std::priority_queue<HighLevelNode> open;
    openSet_t open;
    focalSet_t focal;

    auto handle = open.push(start);
    (*handle).handle = handle;
    focal.push(handle);

    Cost bestCost = (*handle).cost;

    solution.clear();
    int id = 1;
    t1.stop();

    // std::cout << "Agent 97 solution: " << std::endl;

    // std::cout << "start.solution[97].states.size(): " << start.solution[97].states.size() <<std::endl;
    // for (size_t n = 0; n < start.solution[97].states.size(); ++n) {
    //   std::cout <<"("<< start.solution[97].states[n].first.loc.first << ","
    //   << start.solution[97].states[n].first.loc.second << ","<<start.solution[97].states[n].first.orientation << ")" << std::endl;
    // }
    
    while (!open.empty()) {
      Timer t2;

      // std::cout << "000000000000000000 " << std::endl; 
      // m_env.calculate_heat_map(initialStates);
      // std::cout << "11111111111111111 " << std::endl; 
        
      Cost oldBestCost = bestCost;
      bestCost = open.top().cost;
      if (bestCost > oldBestCost) {
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter) {
          Cost val = iter->cost;
          if (val > oldBestCost * m_w && val <= bestCost * m_w) {
            const HighLevelNode& n = *iter;
            focal.push(n.handle);
          }
          if (val > bestCost * m_w) {
            break;
          }
        }
      }

      auto h = focal.top();
      HighLevelNode P = *h;
      m_env.onExpandHighLevelNode(P.cost);

      focal.pop();
      open.erase(h);
      
      r_Conflict conflict;

      
      // 检查冲突（只考虑活跃智能体） LORR24_spikenevergiveup_250216_晚_hzj
      //if (!m_env.getFirstConflict_areas(P.solution, conflict, Area_Agent)) {
      if (!m_env.getFirstConflict1(P.solution, conflict)) {
        solution = P.solution;
        
        //打印时间 -hzj
        totalTimer.stop();
        std::cout << "while(!open.empty()) cost time: "  << totalTimer.elapsedSeconds()  << std::endl;
        return true;
      }

      // 只处理活跃智能体的冲突 -hzj
      if (activeAgents.count(conflict.agent1) == 0 || 
          activeAgents.count(conflict.agent2) == 0) {
          continue;  // 跳过非活跃智能体的冲突
        }

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);

      // 处理每个涉及冲突的智能体 确保是活跃智能体
      for (const auto& c : constraints) {
        size_t i = c.first;

        // 确保是活跃智能体 -hzj
      if (activeAgents.count(i) == 0) {
          continue;
      }

        HighLevelNode newNode = P;
        newNode.id = id;

        // // -hzj
        // if (!newNode.constraints[i].overlap(c.second)) {
        //     newNode.constraints[i].add(c.second);

        //     // 在开始搜索前检查该智能体的约束数量
        //     size_t constraintCount = newNode.constraints[i].vertexConstraints.size() + 
        //                            newNode.constraints[i].edgeConstraints.size();
            
        //     // 如果约束数量过多，直接让智能体静止
        //     if (constraintCount > 5) {  // 设置合适的阈值
        //         // 让智能体保持在原地
        //         newNode.cost -= newNode.solution[i].cost;
        //         newNode.LB -= newNode.solution[i].fmin;
                
        //         newNode.solution[i].states.clear();
        //         newNode.solution[i].states.push_back(std::make_pair(initialStates[i], 0));
        //         newNode.solution[i].actions.clear();
        //         newNode.solution[i].cost = 0;
        //         newNode.solution[i].fmin = 0;

        //         newNode.cost += newNode.solution[i].cost;
        //         newNode.LB += newNode.solution[i].fmin;
        //         newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution);

        //         auto handle = open.push(newNode);
        //         (*handle).handle = handle;
        //         if (newNode.cost <= bestCost * m_w) {
        //             focal.push(handle);
        //         }
        //     } else {
        //         // 约束数量在可接受范围内，进行正常的路径搜索
        //         newNode.cost -= newNode.solution[i].cost;
        //         newNode.LB -= newNode.solution[i].fmin;

        //         LowLevelEnvironment llenv(m_env, i, newNode.constraints[i], newNode.solution);
        //         LowLevelSearch_t lowLevel(llenv, m_w);
        //         bool success = lowLevel.search(initialStates[i], newNode.solution[i], 3);

        //         if (success) {
        //             newNode.cost += newNode.solution[i].cost;
        //             newNode.LB += newNode.solution[i].fmin;
        //             newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution);

        //             auto handle = open.push(newNode);
        //             (*handle).handle = handle;
        //             if (newNode.cost <= bestCost * m_w) {
        //                 focal.push(handle);
        //             }
        //         }
        //     }
        // }
        // 先检查约束数量
        size_t constraintCount = newNode.constraints[i].vertexConstraints.size() + 
                               newNode.constraints[i].edgeConstraints.size() + m_env.countNearbyInactiveAgents(initialStates[i]);
            
        // 如果约束数量过多，直接让智能体静止
        if (constraintCount > 8) {  // 阈值可以调整
            newNode.cost -= newNode.solution[i].cost;
            newNode.LB -= newNode.solution[i].fmin;
            
            // 让智能体保持在原地
            newNode.solution[i].states.clear();
            newNode.solution[i].states.push_back(std::make_pair(initialStates[i], 0));
            newNode.solution[i].actions.clear();
            newNode.solution[i].cost = 0;
            newNode.solution[i].fmin = 0;

            newNode.cost += newNode.solution[i].cost;
            newNode.LB += newNode.solution[i].fmin;
            newNode.focalHeuristic = m_env.focalHeuristic1(newNode.solution);

            auto handle = open.push(newNode);
            (*handle).handle = handle;
            if (newNode.cost <= bestCost * m_w) {
                focal.push(handle);
            }
            ++id;  // 移到这里确保执行
            continue;
        }
        
        // 约束数量在可接受范围内，才进行overlap检查
        if (!newNode.constraints[i].overlap(c.second)) {
            // 添加新约束
            newNode.constraints[i].add(c.second);
            newNode.cost -= newNode.solution[i].cost;
            newNode.LB -= newNode.solution[i].fmin;

            LowLevelEnvironment llenv(m_env, i, newNode.constraints[i], newNode.solution);
            LowLevelSearch_t lowLevel(llenv, m_w);
            bool success = lowLevel.search(initialStates[i], newNode.solution[i], 3);

            if (success) {
                newNode.cost += newNode.solution[i].cost;
                newNode.LB += newNode.solution[i].fmin;
                newNode.focalHeuristic = m_env.focalHeuristic1(newNode.solution);

                auto handle = open.push(newNode);
                (*handle).handle = handle;
                if (newNode.cost <= bestCost * m_w) {
                    focal.push(handle);
                }
            }
        }

        ++id;
      }
      t2.stop();
    }

    return false;
  }
  
  bool search2(const std::vector<State2>& initialStates,
              std::vector<PlanResult<State2, Action, Cost> >& solution, std::vector<int> & only_task_ids) {
    
    Timer t1;
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.LB = 0;
    start.id = 0;


    m_env.calculate_heat_map(initialStates);

    // 区域划分参数
    int area_width = 10;
    int area_overlap = 5;
    
    // 构建分区所包含的智能体id
    // std::vector<std::vector<int>> Area_Agent; // n个区域，每个区域包含了当前区域的智能体id
    // Area_Agent = m_env.makeAgentAreas(initialStates, area_width, area_overlap);


    /*
    
    Timer mt_time;
    // 使用多线程进行初始路径
       // 多线程求初始路径
    // 初始化每个智能体的初始路径（多线程并行计算）
    // RHCR_Planner::ThreadPool pool(std::thread::hardware_concurrency()); // 增加线程池线程数
    RHCR_Planner::ThreadPool pool(16); // 增加线程池线程数

    std::atomic<bool> all_success(true);
    std::mutex cost_mutex, cout_mutex; // 确保 cout_mutex 被定义
    std::vector<std::future<void>> futures;
    solution.resize(initialStates.size());

    std::cout << "Multi Thread Initial Path Start " << std::endl;

    // 检查初始状态、解决方案和基础解的大小是否一致
    if (initialStates.size() != solution.size() || initialStates.size() != start.solution.size()) {
        std::cerr << "初始状态、解决方案和基础解的大小不匹配！" << std::endl;
        return false;
    }

    // // 确保 start.solution 足够大
     start.solution.resize(initialStates.size());

    for (size_t i = 0; i < initialStates.size(); ++i) {
        size_t idx = i; // 新的临时变量
        auto initialStates_size = initialStates.size();
        futures.emplace_back(pool.enqueue([this, idx, initialStates_size, &initialStates, &solution, &start, &all_success, &cost_mutex, &cout_mutex]() {
        //  return;
            if (!all_success.load()) return;

            // 日志输出加锁
            std::lock_guard<std::mutex> c_lock(cout_mutex);

            // 用 idx 替代 i
            size_t i = idx; // 将 idx 转换为 i

            // 线程安全的索引检查
            if (i >= initialStates_size || i >= solution.size() || i >= start.solution.size()) {
                std::cerr << "Index out of range: " << i << std::endl;
                all_success.store(false);
                return;
            }

            if (solution[i].states.size() > 1) {
                // 使用已有的解决方案
                assert(initialStates[i] == solution[i].states.front().first);
                std::lock_guard<std::mutex> lock(cost_mutex);
                start.solution[i] = solution[i];
            } else {
                // 并行搜索新路径
                // PlanResult<State2, Action, Cost> agent_solution;
                {
                    // std::cout << "Agent " << i << " thread Planning" << std::endl;

                    LowLevelEnvironment llenv(m_env, i, start.constraints[i], start.solution);
                    LowLevelSearch_t lowLevel(llenv, m_w);
                    bool success = lowLevel.search(initialStates[i], start.solution[i]);
                    // std::cout << "Agent " << i << " thread after search" << std::endl;
                    // 检查超时
                    if (!success) {
                        std::cerr << "Failed to find path for agent: " << i << std::endl;
                        all_success.store(false);
                        return;
                    }
                }

                std::lock_guard<std::mutex> lock(cost_mutex);
                // std::cout << "Agent " << i << " thread after lock" << std::endl;
                // start.solution[i] = agent_solution;
            }

            // 日志输出结束信息
            // std::cout << "Agent " << i << " thread before finished" << std::endl;
            //std::lock_guard<std::mutex> c_lock_finish(cout_mutex);
            // std::cout << "Agent " << i << " thread finished" << std::endl;
            std::cout << "" ;
        }));
    }
    std::cout <<" " << std::endl;
    mt_time.stop();
    std::cout << "Multi Thread Initial Path USE Time : " << mt_time.elapsedSeconds() << std::endl;

    // 等待所有任务完成
    for (auto& future : futures) {
        future.get();
    }


    if (!all_success.load()) {
        return false;
    }
    for (size_t i = 0; i < initialStates.size(); ++i) {
      start.cost += start.solution[i].cost;
      start.LB += start.solution[i].fmin;
    }

    */

                  

    
    Timer t8;
    

    std::vector<size_t> no_path_agents; // 用于记录没有路径的智能体id
    // std::unordered_set<size_t> modifiedAgents; // 记录被修改的智能体 LHY

    // 遍历所有智能体
    for (size_t i = 0; i < initialStates.size(); ++i) {
      // 如果智能体已经有路径，直接复用路径
      if (solution[i].states.size() > 1) {
          assert(initialStates[i] == solution[i].states.front().first);
          start.solution[i] = solution[i];

          start.cost += start.solution[i].cost;
          start.LB += start.solution[i].fmin;

      } else {
          // 如果没有初始路径，记录该智能体的id
          // std::cout << "Agent " << i << " has no initial path, will search." << std::endl;
          no_path_agents.push_back(i);
      }
    }

    // 开始计时
    auto search_start_time = std::chrono::steady_clock::now();
    int max_time = 150; // 限制时间为100ms


    size_t ncount = 0;
    // 对没有路径的智能体依次进行路径搜索
    for (size_t i = 0; i < no_path_agents.size(); ++i) {
      size_t agent_id = no_path_agents[i];

      LowLevelEnvironment llenv(m_env, agent_id, start.constraints[agent_id], start.solution);
      LowLevelSearch_t lowLevel(llenv, m_w);
      bool success = lowLevel.search(initialStates[agent_id], start.solution[agent_id],12);

      // 如果路径搜索失败，返回 false
      if (!success) {
          return false;
      }
      ncount++;

      // modifiedAgents.insert(agent_id); // 记录被修改的智能体 LHY

      start.cost += start.solution[agent_id].cost;
      start.LB += start.solution[agent_id].fmin;

      // 检查时间是否超过100ms，若超时则跳出循环
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - search_start_time
      ).count();

      if (elapsed_time >= max_time) {
          break; // 如果已经超时，跳出循环
      }
    }

    // 如果有未完成路径搜索的智能体，直接赋值当前位置
    for (size_t i = ncount; i < no_path_agents.size(); ++i) {
      size_t agent_id = no_path_agents[i];


      if (start.solution[agent_id].states.empty()) {
            start.solution[agent_id].states.clear();
            start.solution[agent_id].states.push_back(std::make_pair(initialStates[agent_id], 0));
            start.solution[agent_id].actions.clear();
            start.solution[agent_id].cost = 0;
            start.solution[agent_id].fmin = 0;
      }
    }


    std::cout << "已复用路径的智能体数: " << initialStates.size() - no_path_agents.size() << std::endl;
    std::cout << "已进行路径搜索的智能体数: " << ncount << std::endl;

    t8.stop();
    std::cout << "****** 1、求初始解用时: " << t8.elapsedSeconds() <<std::endl;
    





    Timer t4;

    start.focalHeuristic = m_env.focalHeuristic(start.solution, only_task_ids);


    t4.stop();
    std::cout << "****** ECBS search里第一次调用focalHeuristic耗时 : " << t4.elapsedSeconds() <<std::endl;


    // std::priority_queue<HighLevelNode> open;
    openSet_t open;
    focalSet_t focal;

    auto handle = open.push(start);
    (*handle).handle = handle;
    focal.push(handle);

    Cost bestCost = (*handle).cost;

    solution.clear();
    int id = 1;
    t1.stop();


    while (!open.empty()) {
      Timer t2;

        
      Cost oldBestCost = bestCost;
      bestCost = open.top().cost;
      if (bestCost > oldBestCost) {
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter) {
          Cost val = iter->cost;
          if (val > oldBestCost * m_w && val <= bestCost * m_w) {
            const HighLevelNode& n = *iter;
            focal.push(n.handle);
          }
          if (val > bestCost * m_w) {
            break;
          }
        }
      }

      auto h = focal.top();
      HighLevelNode P = *h;
      m_env.onExpandHighLevelNode(P.cost);
      focal.pop();
      open.erase(h);
      
      r_Conflict conflict;

      Timer t6;

      // if (!m_env.getFirstConflict_modified(P.solution, conflict, modifiedAgents)) {
      if (!m_env.getFirstConflict(P.solution, conflict,only_task_ids)) {//LHY
        solution = P.solution;
        return true;
      }

      t6.stop();
      std::cout << "while  里面一次getFirstConflict耗时 : " << t6.elapsedSeconds() <<std::endl;


      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);


      Timer t8;

      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);
        newNode.cost -= newNode.solution[i].cost;
        newNode.LB -= newNode.solution[i].fmin;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                  newNode.solution);
        LowLevelSearch_t lowLevel(llenv, m_w);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i],12);

        Timer t7;

        newNode.cost += newNode.solution[i].cost; 
        newNode.LB += newNode.solution[i].fmin;
        newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution,only_task_ids);

        t7.stop();
        std::cout << "while里面一次focalHeuristic耗时 : " << t7.elapsedSeconds() <<std::endl;  

        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          if (newNode.cost <= bestCost * m_w) {
            focal.push(handle);
          }
        }

        ++id;
      }
      t8.stop();
      std::cout << "while里面for循环总共focalHeuristic耗时  : " << t8.elapsedSeconds() <<std::endl;

      t2.stop();
      std::cout << "****** RHCR Time For while解决冲突循环时间 : " << t2.elapsedSeconds() <<std::endl;
    }

    return false;
  }

 private:
  struct HighLevelNode;

  typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type handle_t;

  struct HighLevelNode {
    std::vector<PlanResult<State2, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;
    Cost LB;  // sum of fmin of solution

    Cost focalHeuristic;

    int id;

    handle_t handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }

    // bool operator=(const HighLevelNode& n) const {
    //  // 每个成员赋值相等
    //   solution = n.solution;
    //   constraints = n.constraints;
    //   cost = n.cost;
    //   LB = n.LB;
    //   focalHeuristic = n.focalHeuristic;
    //   id = n.id;
    //   handle = n.handle;
    //   return true;
    // }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << " LB: " << c.LB
         << " focal: " << c.focalHeuristic << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct compareFocalHeuristic {
    bool operator()(const handle_t& h1, const handle_t& h2) const {
      // Our heap is a maximum heap, so we invert the comperator function here
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
      }
      return (*h1).cost > (*h2).cost;
    }
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<
      openSet_t, boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#else
  typedef typename boost::heap::d_ary_heap<
      handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#endif

  struct LowLevelEnvironment {
    LowLevelEnvironment(
        Environment& env, size_t agentIdx, const Constraints& constraints,
        const std::vector<PlanResult<State2, Action, Cost> >& solution)
        : m_env(env)
          // , m_agentIdx(agentIdx)
          // , m_constraints(constraints)
          ,
          m_solution(solution) {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State2& s) {
      return m_env.admissibleHeuristic(s);
    }

    Cost getHeatValue(const State2& s) {
      return m_env.getHeatValue(s);
    }

    Cost focalStateHeuristic(const State2& s, Cost gScore) {
      return m_env.focalStateHeuristic(s, gScore, m_solution);
    }

    Cost focalTransitionHeuristic(const State2& s1, const State2& s2,
                                  Cost gScoreS1, Cost gScoreS2) {
      return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
                                            m_solution);
    }

    bool isSolution(const State2& s) { return m_env.isSolution(s); }

    void getNeighbors(const State2& s,
                      std::vector<Neighbor<State2, Action, int> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State2& s, Cost fScore, Cost gScore) {
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State2& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
    }

   private:
    Environment& m_env;
    const std::vector<PlanResult<State2, Action, Cost> >& m_solution;
  };

 private:
  Environment& m_env;
  float m_w;
  typedef AStarEpsilon<State2, Action, Cost, LowLevelEnvironment>
      LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
