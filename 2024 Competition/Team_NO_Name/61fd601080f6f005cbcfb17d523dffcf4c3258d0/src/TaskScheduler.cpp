#include "TaskScheduler.h"
#include <Eigen/Dense> 
#include <chrono>
#include <vector>
#include <limits>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "scheduler.h"
#include "const.h"
#include "Hungarian.h"
#include <mutex>
#include <chrono>
#include <iostream>
#include<thread>
#include <atomic>
#include "dkm.hpp"
/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 *
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit
 * and adjust for a specified tolerance to account for potential timing errors.
 * It ensures that initialization does not exceed the allocated time.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_map<int, pair<bool, int>> task_values;
bool wait = false;
int cntt = 0;
struct block_area_initialize {
    int id;
    vector<int>random_data;

};
struct at_value {
    int agent_id;
    int task_id;
    long long value;
};
std::unordered_map<int, pair<int, int>>last_task_id;

struct Point2D {
    double x;
    double y;
};

struct area {
    int id;
    std::unordered_set<int> free_tasks_area;
    std::unordered_set<int> free_agents_area;
    Point2D centroid;
    double value() const {
        if (free_tasks_area.size() == 0 || free_agents_area.size() == 0) {
            return 0;
        }
        else {
            return free_tasks_area.size() * 1.0 / free_agents_area.size();
        }
    }

    static bool cmp(const area& a, const area& b) {
        return a.value() < b.value();
    }

    static bool cmp1(const area& a, const area& b) {
        return a.id < b.id;
    }
};

const double INF = 1e15;
struct Agv_imformation
{
    int solved_nums = 0;
    int searching_time = 0;
    int solving_time = 0;
    int id;
    int gettime = 0;
    int at_dist = 0;
}agv_imformation[10000];

struct Agv_value
{
    double v;
}agv_value[10000];

int max_solving_time = 0;
int max_searching_time = 0;
int max_solved_nums = 0;
double get_value(Agv_imformation t) {
    double v = 0;
    if (max_solved_nums != 0)v += t.solved_nums * 1.0 / max_solved_nums * 0.8;
    if (max_searching_time != 0)v += t.searching_time * 1.0 / max_searching_time * 0.1;
    if (max_solving_time != 0)v += t.solving_time * 1.0 / max_solving_time * 0.1;
    return v;
}
struct AgvCompare {
    bool operator()(const Agv_imformation& a, const Agv_imformation& b) const {
        if (a.solved_nums != b.solved_nums) {
            return a.solved_nums < b.solved_nums;
        }
        else if (a.searching_time != b.searching_time) {
            return a.searching_time < b.searching_time;
        }
        else {
            return a.solving_time < b.solving_time;
        }
    }
};

Eigen::MatrixXd buildSquareCostMatrix(
    const std::vector<int>& agentsVec,
    const std::vector<int>& tasksVec,
    const std::vector<std::vector<int>>& rawCost)
{
    int M = (int)agentsVec.size();
    int N = (int)tasksVec.size();
    int n = std::max(M, N);

    Eigen::MatrixXd squareMatrix = Eigen::MatrixXd::Constant(n, n, INF);

    // 把 rawCost 的 (M x N) 区域拷到 squareMatrix 前 M 行, 前 N 列
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            squareMatrix(i, j) = rawCost[i][j];
        }
    }

    return squareMatrix; // n x n
}


void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance 
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(1790000);
    int l = 0;
    int maxx = env->cols * env->rows;
    //block_area_initialize areas[9];
    //if (maxx == 32 * 32) {
    //    for (int i = 0; i < maxx; i++) {
    //        for (int j = 0; j < maxx; j++) {
    //            if (env->map[i] != 0 || env->map[j] != 0)continue;
    //            DefaultPlanner::get_h(env, i, j);
    //        }
    //    }
    //}
    //else if (maxx == 481 * 530) {
    //    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    //    DefaultPlanner::schedule_initialize(limit, env);
    //    return;
    //}
    //else if (maxx == 256 * 256) {
    //    block_area_initialize areas[9];
    //    for (int i = 0; i < maxx; i++) {
    //        if (env->map[i] != 0)continue;
    //        double row = i * 1.0 / env->cols;
    //        double col = i % env->cols * 1.0;

    //        if (row < env->rows / 3.0 && col < env->cols / 3.0) {
    //            areas[0].random_data.push_back(i);
    //        }
    //        else if (row < env->rows / 3.0 && col >= env->cols / 3.0 && col < 2 * env->cols / 3.0) {
    //            areas[1].random_data.push_back(i);
    //        }
    //        else if (row < env->rows / 3.0 && col >= 2 * env->cols / 3.0) {
    //            areas[2].random_data.push_back(i);
    //        }
    //        else if (row >= env->rows / 3.0 && row < 2 * env->rows / 3.0 && col < env->cols / 3.0) {
    //            areas[3].random_data.push_back(i);
    //        }
    //        else if (row >= env->rows / 3.0 && row < 2 * env->rows / 3.0 && col >= env->cols / 3.0 && col < 2 * env->cols / 3.0) {
    //            areas[4].random_data.push_back(i);
    //        }
    //        else if (row >= env->rows / 3.0 && row < 2 * env->rows / 3.0 && col >= 2 * env->cols / 3.0) {
    //            areas[5].random_data.push_back(i);
    //        }
    //        else if (row >= 2 * env->rows / 3.0 && col < env->cols / 3.0) {
    //            areas[6].random_data.push_back(i);
    //        }
    //        else if (row >= 2 * env->rows / 3.0 && col >= env->cols / 3.0 && col < 2 * env->cols / 3.0) {
    //            areas[7].random_data.push_back(i);
    //        }
    //        else if (row >= 2 * env->rows / 3.0 && col >= 2 * env->cols / 3.0) {
    //            areas[8].random_data.push_back(i);
    //        }

    //    }
    //    for (int i = 0; i < 9; i++) {
    //        for (auto k : areas[i].random_data) {
    //            for (int j = 0; j < 9; j++) {
    //                std::uniform_int_distribution<> dis(0, areas[j].random_data.size());
    //                int cnt = 3;
    //                while (cnt--) {
    //                    int random_index = dis(mt);
    //                    DefaultPlanner::get_h(env, areas[j].random_data[random_index], k);
    //                    if (std::chrono::steady_clock::now() > endtime)return;
    //                }
    //            }

    //        }
    //    }
    //}
    //else {
    //    unsigned int num_threads8 = std::thread::hardware_concurrency();
    //    if (num_threads8 <= 20) {
    //        _exit(124);
    //    }
    //    int block_rows = 14;  // 每个块的行数
    //    int block_cols = 5;  // 每个块的列数
    //    int total_rows = env->rows;  // 总行数
    //    int total_cols = env->cols;  // 总列数

    //    // 计算总的块数
    //    int num_blocks_x = (total_cols + block_cols - 1) / block_cols;  // 列方向的块数
    //    int num_blocks_y = (total_rows + block_rows - 1) / block_rows;  // 行方向的块数
    //    block_area_initialize areas[1000];
    //    for (int i = 0; i < maxx; i++) {
    //        if (env->map[i] != 0)continue;
    //        double row = i * 1.0 / env->cols;
    //        double col = i % env->cols * 1.0;
    //        int block_row = row / block_rows;  // 按行划分块
    //        int block_col = col / block_cols;  // 按列划分块
    //        int area_index = block_row * num_blocks_x + block_col;
    //        areas[area_index].random_data.push_back(i);
    //    }
    //    for (int i = 0; i < 1000; i++) {
    //        for (auto k : areas[i].random_data) {
    //            for (int j = 0; j < 1000; j++) {
    //                std::uniform_int_distribution<> dis(0, areas[j].random_data.size());
    //                int cnt = 2;
    //                while (cnt--) {
    //                    int random_index = dis(mt);
    //                    DefaultPlanner::get_h(env, areas[j].random_data[random_index], k);
    //                    if (std::chrono::steady_clock::now() > endtime)return;
    //                }
    //            }

    //        }
    //    }
    //}
}

/**
 * Plans a task schedule within a specified time limit.
 *
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 *
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    int maxx = env->cols * env->rows;
    if (maxx != 32 * 32 && maxx != 256 * 256 && maxx != 500 * 140) {
        if (env->curr_timestep == 0) {
            for (int i = 0; i < env->num_of_agents; i++) {
                agv_imformation[i].id = i;
            }
        }
        auto start_time = std::chrono::steady_clock::now();
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit - 400);
        free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
        free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
        int satisfied = 10;
        if (wait == true) {
            for (auto pairs : env->task_pool) {
                auto i = pairs.second;
                if (i.t_completed != -1)continue;
                else {
                    if (i.agent_assigned != -1) {
                        if (i.idx_next_loc == 0) {
                            if (agv_imformation[i.agent_assigned].gettime + satisfied < env->curr_timestep) {
                                free_tasks.insert(i.task_id);
                                free_agents.insert(i.agent_assigned);
                                proposed_schedule[i.agent_assigned] = -1;
                            }
                        }
                    }
                }
            }
        }
        else {
            if (free_agents.size() < 4000) {
                wait = true;
            }
        }
        int min_task_i, min_task_makespan, c_loc, count;
        double dist = 0;
        double a = 0.1;
        clock_t start = clock();
        int cnt = 0;
        if (env->curr_timestep != 0) {
            for (auto i : env->new_freeagents) {
                agv_imformation[i].solved_nums++;
            }
        }
        for (auto pairs : env->task_pool) {
            auto i = pairs.second;
            if (i.t_completed != -1)continue;
            else {
                if (i.agent_assigned != -1) {
                    if (i.idx_next_loc == 0) {
                        agv_imformation[i.agent_assigned].searching_time++;
                    }
                    else {
                        agv_imformation[i.agent_assigned].solving_time++;

                    }
                }
            }
        }
        std::vector<Agv_imformation> new_agent_pool;
        for (int j : free_agents) {
            new_agent_pool.push_back(agv_imformation[j]);
        }

        if (new_agent_pool.size() == 0)return;
        std::unordered_set<int> new_task_pool1;
        std::unordered_set<int> new_agent_pool1;
        sort(new_agent_pool.begin(), new_agent_pool.end(), AgvCompare());
        vector<int>need_erase;
        int flag1 = 0;
        for (auto it : new_agent_pool)
        {
            //keep assigning until timeout
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            int i = it.id;

            assert(env->curr_task_schedule[i] == -1);

            min_task_i = -1;
            min_task_makespan = INT_MAX;
            count = 0;

            // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
            for (int t_id : free_tasks)
            {
                //check for timeout every 10 task evaluations
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                dist = 0;
                c_loc = env->curr_states.at(i).location;
                flag1 = 0;
                // iterate over the locations (errands) of the task to compute the makespan to finish the task
                // makespan: the time for the agent to complete all the errands of the task t_id in order
                for (int loc : env->task_pool[t_id].locations) {
                    if (flag1 == 0) {
                        dist += DefaultPlanner::get_h(env, c_loc, loc);
                        flag1++;
                    }
                    else dist += DefaultPlanner::get_h(env, c_loc, loc) * a;
                    c_loc = loc;
                }

                // update the new minimum makespan
                if (dist < min_task_makespan) {
                    min_task_i = t_id;
                    min_task_makespan = dist;
                }
                count++;
            }

            // assign the best free task to the agent i (assuming one exists)
            if (min_task_i != -1) {
                free_tasks.erase(min_task_i);
                need_erase.push_back(i);
                proposed_schedule[i] = min_task_i;
                agv_imformation[i].gettime = env->curr_timestep;
                agv_imformation[i].at_dist = DefaultPlanner::get_h(env, env->curr_states.at(i).location, env->task_pool[min_task_i].locations[0]);
            }
            else {
                proposed_schedule[i] = -1;
            }
        }
        for (int i : need_erase) {
            free_agents.erase(i);
        }
        return;
    }
     else if (maxx == 32 * 32) {
          if (env->curr_timestep == 0) {
              for (int i = 0; i < env->num_of_agents; i++) {
                  last_task_id.insert({ i, {-1,0} });
              }
          }
          for (auto& i : proposed_schedule) {
              i = -1;
          }
          vector<at_value>ta_value;
          auto start_time = std::chrono::steady_clock::now();
          TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit - 300);
          free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
          free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
          std::unordered_set<int> new_task_pool;
          std::unordered_set<int> new_agent_pool;
          for (int i = 0; i < env->num_of_agents; i++) {
              new_agent_pool.insert(i);
          }
          int sum2 = 0;
          for (auto pairs : env->task_pool) {
              auto i = pairs.second;
              if (i.t_completed != -1)continue;
              else {
                  if (i.agent_assigned != -1) {
                      if (i.idx_next_loc == 0) {
                          new_task_pool.insert(i.task_id);
                          if ((last_task_id[i.agent_assigned].first) != i.task_id) {
                              last_task_id[i.agent_assigned].first = i.task_id;
                              last_task_id[i.agent_assigned].second = 2;
                          }
                          else {
                              last_task_id[i.agent_assigned].second += 2;
                              if (last_task_id[i.agent_assigned].second > 12) {
                                  last_task_id[i.agent_assigned].second = 12;
                              }
                          }
                          sum2++;
                      }
                      else {
                          proposed_schedule[i.agent_assigned] = i.task_id;
                          new_agent_pool.erase(i.agent_assigned);
                      }
                  }
                  else {
                      new_task_pool.insert(i.task_id);
                  }
              }
          }
          std::unordered_set<int>::iterator it = new_task_pool.begin();
          std::unordered_set<int> new_task_pool1;
          std::unordered_set<int> new_agent_pool1;
          new_task_pool1 = new_task_pool;
          new_agent_pool1 = new_agent_pool;
          int numTasks = new_task_pool.size();
          int desiredTasksPerCluster = 200;
          int k = std::max(1, numTasks / desiredTasksPerCluster);
          k = std::min(k, 250);
          // 2. 构造任务样本数据：以任务起点坐标作为样本数据
          std::vector<std::array<float, 2>> taskData;
          taskData.reserve(numTasks);
          std::vector<int> taskIds;
          taskIds.reserve(numTasks);
          for (int task : new_task_pool) {
              int loc = env->task_pool.at(task).locations[0];
              float row = static_cast<float>(loc / env->cols);
              float col = static_cast<float>(loc % env->cols);
              taskData.push_back({ row, col });
              taskIds.push_back(task);
          }
          // 3. 使用 DKM 库的 k-means 算法对任务进行聚类
          auto result = dkm::kmeans_lloyd(taskData, k);
          auto centroids = std::get<0>(result);  // 聚类中心，可能少于 k 个
          auto labels = std::get<1>(result);       // 每个任务对应的聚类标签
          // 4. 使用返回的实际聚类数（actual_k）来构造区域数组
          int actual_k = centroids.size();  // 实际的簇数
          std::vector<area> areas;
          areas.resize(actual_k);
          for (int i = 0; i < actual_k; i++) {
              areas[i].id = i;
              areas[i].free_agents_area.clear();
              areas[i].free_tasks_area.clear();
              // 从返回的 centroids 设置区域中心
              areas[i].centroid.x = centroids[i][0];
              areas[i].centroid.y = centroids[i][1];
          }
          // 5. 将每个任务根据聚类结果分配到对应区域中
          for (size_t i = 0; i < labels.size(); i++) {
              int clusterLabel = labels[i];
              // 检查 clusterLabel 是否在合法范围内
              if (clusterLabel < 0 || clusterLabel >= actual_k) {
                  // 如果不合法，可以选择跳过或者归入默认区域（例如区域0）
                  clusterLabel = 0;
              }
              int taskId = taskIds[i];
              areas[clusterLabel].free_tasks_area.insert(taskId);
          }
          // 6. 将每个 agent 根据其位置与各任务聚类中心的距离，分配到离其最近的区域
          for (int agent : new_agent_pool) {
              int loc = env->curr_states[agent].location;
              double row = static_cast<double>(loc / env->cols);
              double col = static_cast<double>(loc % env->cols);
              int bestCluster = 0;
              double minDist = std::numeric_limits<double>::max();
              for (int i = 0; i < actual_k; i++) {
                  double dx = row - areas[i].centroid.x;
                  double dy = col - areas[i].centroid.y;
                  double dist = dx * dx + dy * dy;
                  if (dist < minDist) {
                      minDist = dist;
                      bestCluster = i;
                  }
              }
              areas[bestCluster].free_agents_area.insert(agent);
          }
          for (int k = 0; k < areas.size(); k++) {
              it = areas[k].free_agents_area.begin();
              while (it != areas[k].free_agents_area.end()) {
                  if (std::chrono::steady_clock::now() > endtime) {
                      return;
                  }
                  int i = *it;
                  int min_cost = 1e9;
                  int task_id = -1;
                  for (auto j : areas[k].free_tasks_area) {
                      if (std::chrono::steady_clock::now() > endtime) {
                          return;
                      }
                      long long dist = 0;
                      int c_loc = env->curr_states.at(i).location;
                      for (int loc : env->task_pool[j].locations) {
                          dist += DefaultPlanner::get_h(env, c_loc, loc);
                          c_loc = loc;
                      }
                  }
                  it++;
              }
          }
          if (std::chrono::steady_clock::now() + std::chrono::milliseconds(100) > endtime)
          {
              return;
          }
          unsigned int num_threads = std::thread::hardware_concurrency();
          std::vector<std::thread> threads;
          std::atomic<int> current_area(0);
          int least = 0;
          for (unsigned int t = 0; t < num_threads; t++) {
              threads.push_back(std::thread([this, &proposed_schedule, endtime, &areas, &current_area, &least]() {
                  while (true) {
                      int area_id = current_area.fetch_add(1);
                      if (area_id >= (int)areas.size()) {
                          break;
                      }
                      SharedEnvironment* env = this->env;
                      int cnt1 = 0, cnt2 = 0;
                      int k = area_id;
                      std::vector<std::vector<int>> cost_matrix;
                      cost_matrix.resize(areas[k].free_agents_area.size(), std::vector<int>(areas[k].free_tasks_area.size(), 0));
                      cnt1 = 0;
                      auto it = areas[k].free_agents_area.begin();
                      while (it != areas[k].free_agents_area.end()) {
                          cnt2 = 0;
                          int i = *it;
                          for (auto j : areas[k].free_tasks_area) {
                              long long dist = 0;
                              int c_loc = env->curr_states.at(i).location;
                              for (int loc : env->task_pool[j].locations) {
                                  dist += DefaultPlanner::get_h(env, c_loc, loc);
                                  c_loc = loc;
                              }
                              if (j == last_task_id[i].first) {
                                  dist -= last_task_id[i].second;
                              }
                              if (dist < 0) dist = 0;
                              cost_matrix[cnt1][cnt2] = dist;
                              cnt2++;
                          }
                          it++;
                          cnt1++;
                      }
                      // 对区域内所有 agent 先置为未分配
                      for (auto i : areas[k].free_agents_area) {
                          proposed_schedule[i] = -1;
                      }
                      std::vector<int> free_agents_vector(areas[k].free_agents_area.begin(), areas[k].free_agents_area.end());
                      std::vector<int> free_tasks_vector(areas[k].free_tasks_area.begin(), areas[k].free_tasks_area.end());
                      int n = areas[k].free_agents_area.size();
                      int m = areas[k].free_tasks_area.size();
                      // 转换成本矩阵（double 类型）
                      std::vector<std::vector<double>> DistMatrix;
                      if (n == 0 || m == 0) continue;
                      DistMatrix.resize(n, std::vector<double>(m, 0.0));
                      for (int i = 0; i < n; i++) {
                          for (int j = 0; j < m; j++) {
                              DistMatrix[i][j] = cost_matrix[i][j];
                          }
                      }
                      HungarianAlgorithm HungAlgo;
                      std::vector<int> assignment;
                      double minCost = HungAlgo.Solve(DistMatrix, assignment);
                      // 根据匈牙利算法的结果更新调度方案
                      int idx = 0;
                      auto it1 = areas[k].free_agents_area.begin();
                      while (it1 != areas[k].free_agents_area.end()) {
                          if (idx >= (int)assignment.size()) break;
                          int agent = *it1;
                          if (assignment[idx] < 0 || assignment[idx] >= (int)free_tasks_vector.size()) {
                              proposed_schedule[agent] = -1;
                              it1++;
                          }
                          else {
                              int task_idd = free_tasks_vector[assignment[idx]];
                              if (areas[k].free_tasks_area.find(task_idd) == areas[k].free_tasks_area.end()) {
                                  proposed_schedule[agent] = -1;
                                  it1++;
                              }
                              else {
                                  proposed_schedule[agent] = task_idd;
                                  it1 = areas[k].free_agents_area.erase(it1);
                                  areas[k].free_tasks_area.erase(task_idd);
                              }
                          }
                          idx++;
                          least++;
                      }
                  }
                  }));
          }
          for (auto& t : threads) {
              t.join();
          }

     }
    else if (maxx == 256 * 256) {
        if (env->curr_timestep == 0) {
            for (int i = 0; i < env->num_of_agents; i++) {
                last_task_id.insert({ i, {-1,0} });
            }
        }
        for (auto& i : proposed_schedule) {
            i = -1;
        }
        vector<at_value>ta_value;
        auto start_time = std::chrono::steady_clock::now();
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit - 300);
        free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
        free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
        std::unordered_set<int> new_task_pool;
        std::unordered_set<int> new_agent_pool;
        for (int i = 0; i < env->num_of_agents; i++) {
            new_agent_pool.insert(i);
        }
        int sum2 = 0;
        for (auto pairs : env->task_pool) {
            auto i = pairs.second;
            if (i.t_completed != -1)continue;
            else {
                if (i.agent_assigned != -1) {
                    if (i.idx_next_loc == 0) {
                        new_task_pool.insert(i.task_id);
                        if ((last_task_id[i.agent_assigned].first) != i.task_id) {
                            last_task_id[i.agent_assigned].first = i.task_id;
                            last_task_id[i.agent_assigned].second = 2;
                        }
                        else {
                            last_task_id[i.agent_assigned].second += 2;
                            if (last_task_id[i.agent_assigned].second > 12) {
                                last_task_id[i.agent_assigned].second = 12;
                            }
                        }
                        sum2++;
                    }
                    else {
                        proposed_schedule[i.agent_assigned] = i.task_id;
                        new_agent_pool.erase(i.agent_assigned);
                    }
                }
                else {
                    new_task_pool.insert(i.task_id);
                }
            }
        }
        std::unordered_set<int>::iterator it = new_task_pool.begin();
        std::unordered_set<int> new_task_pool1;
        std::unordered_set<int> new_agent_pool1;
        new_task_pool1 = new_task_pool;
        new_agent_pool1 = new_agent_pool;

        int numTasks = new_task_pool.size();
        int desiredTasksPerCluster = 300;
        int k = std::max(1, numTasks / desiredTasksPerCluster);
        k = std::min(k, 250);

        // 2. 构造任务样本数据：以任务起点坐标作为样本数据
        std::vector<std::array<float, 2>> taskData;
        taskData.reserve(numTasks);
        std::vector<int> taskIds;
        taskIds.reserve(numTasks);
        for (int task : new_task_pool) {
            int loc = env->task_pool.at(task).locations[0];
            float row = static_cast<float>(loc / env->cols);
            float col = static_cast<float>(loc % env->cols);
            taskData.push_back({ row, col });
            taskIds.push_back(task);
        }

        // 3. 使用 DKM 库的 k-means 算法对任务进行聚类
        auto result = dkm::kmeans_lloyd(taskData, k);
        auto centroids = std::get<0>(result);  // 聚类中心，可能少于 k 个
        auto labels = std::get<1>(result);       // 每个任务对应的聚类标签

        // 4. 使用返回的实际聚类数（actual_k）来构造区域数组
        int actual_k = centroids.size();  // 实际的簇数
        std::vector<area> areas;
        areas.resize(actual_k);
        for (int i = 0; i < actual_k; i++) {
            areas[i].id = i;
            areas[i].free_agents_area.clear();
            areas[i].free_tasks_area.clear();
            // 从返回的 centroids 设置区域中心
            areas[i].centroid.x = centroids[i][0];
            areas[i].centroid.y = centroids[i][1];
        }

        // 5. 将每个任务根据聚类结果分配到对应区域中
        for (size_t i = 0; i < labels.size(); i++) {
            int clusterLabel = labels[i];
            // 检查 clusterLabel 是否在合法范围内
            if (clusterLabel < 0 || clusterLabel >= actual_k) {
                // 如果不合法，可以选择跳过或者归入默认区域（例如区域0）
                clusterLabel = 0;
            }
            int taskId = taskIds[i];
            areas[clusterLabel].free_tasks_area.insert(taskId);
        }

        // 6. 将每个 agent 根据其位置与各任务聚类中心的距离，分配到离其最近的区域
        for (int agent : new_agent_pool) {
            int loc = env->curr_states[agent].location;
            double row = static_cast<double>(loc / env->cols);
            double col = static_cast<double>(loc % env->cols);
            int bestCluster = 0;
            double minDist = std::numeric_limits<double>::max();
            for (int i = 0; i < actual_k; i++) {
                double dx = row - areas[i].centroid.x;
                double dy = col - areas[i].centroid.y;
                double dist = dx * dx + dy * dy;
                if (dist < minDist) {
                    minDist = dist;
                    bestCluster = i;
                }
            }
            areas[bestCluster].free_agents_area.insert(agent);
        }
        for (int k = 0; k < areas.size(); k++) {
            it = areas[k].free_agents_area.begin();
            while (it != areas[k].free_agents_area.end()) {
                if (std::chrono::steady_clock::now() > endtime) {
                    return;
                }
                int i = *it;
                int min_cost = 1e9;
                int task_id = -1;
                for (auto j : areas[k].free_tasks_area) {
                    if (std::chrono::steady_clock::now() > endtime) {
                        return;
                    }
                    long long dist = 0;
                    int c_loc = env->curr_states.at(i).location;
                    for (int loc : env->task_pool[j].locations) {
                        dist += DefaultPlanner::get_h(env, c_loc, loc);
                        c_loc = loc;
                    }
                }
                it++;
            }
        }


        if (std::chrono::steady_clock::now() + std::chrono::milliseconds(100) > endtime)
        {
            return;
        }
        unsigned int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        std::atomic<int> current_area(0);
        int least = 0;
        for (unsigned int t = 0; t < num_threads; t++) {
            threads.push_back(std::thread([this, &proposed_schedule, endtime, &areas, &current_area, &least]() {
                while (true) {
                    int area_id = current_area.fetch_add(1);
                    if (area_id >= (int)areas.size()) {
                        break;
                    }
                    SharedEnvironment* env = this->env;
                    int cnt1 = 0, cnt2 = 0;
                    int k = area_id;
                    std::vector<std::vector<int>> cost_matrix;
                    cost_matrix.resize(areas[k].free_agents_area.size(), std::vector<int>(areas[k].free_tasks_area.size(), 0));
                    cnt1 = 0;
                    auto it = areas[k].free_agents_area.begin();
                    while (it != areas[k].free_agents_area.end()) {
                        cnt2 = 0;
                        int i = *it;
                        for (auto j : areas[k].free_tasks_area) {
                            long long dist = 0;
                            int c_loc = env->curr_states.at(i).location;
                            for (int loc : env->task_pool[j].locations) {
                                dist += DefaultPlanner::get_h(env, c_loc, loc);
                                c_loc = loc;
                            }
                            if (j == last_task_id[i].first) {
                                dist -= last_task_id[i].second;
                            }
                            if (dist < 0) dist = 0;
                            cost_matrix[cnt1][cnt2] = dist;
                            cnt2++;
                        }
                        it++;
                        cnt1++;
                    }
                    // 对区域内所有 agent 先置为未分配
                    for (auto i : areas[k].free_agents_area) {
                        proposed_schedule[i] = -1;
                    }
                    std::vector<int> free_agents_vector(areas[k].free_agents_area.begin(), areas[k].free_agents_area.end());
                    std::vector<int> free_tasks_vector(areas[k].free_tasks_area.begin(), areas[k].free_tasks_area.end());
                    int n = areas[k].free_agents_area.size();
                    int m = areas[k].free_tasks_area.size();
                    // 转换成本矩阵（double 类型）
                    std::vector<std::vector<double>> DistMatrix;
                    if (n == 0 || m == 0) continue;
                    DistMatrix.resize(n, std::vector<double>(m, 0.0));
                    for (int i = 0; i < n; i++) {
                        for (int j = 0; j < m; j++) {
                            DistMatrix[i][j] = cost_matrix[i][j];
                        }
                    }
                    HungarianAlgorithm HungAlgo;
                    std::vector<int> assignment;
                    double minCost = HungAlgo.Solve(DistMatrix, assignment);
                    // 根据匈牙利算法的结果更新调度方案
                    int idx = 0;
                    auto it1 = areas[k].free_agents_area.begin();
                    while (it1 != areas[k].free_agents_area.end()) {
                        if (idx >= (int)assignment.size()) break;
                        int agent = *it1;
                        if (assignment[idx] < 0 || assignment[idx] >= (int)free_tasks_vector.size()) {
                            proposed_schedule[agent] = -1;
                            it1++;
                        }
                        else {
                            int task_idd = free_tasks_vector[assignment[idx]];
                            if (areas[k].free_tasks_area.find(task_idd) == areas[k].free_tasks_area.end()) {
                                proposed_schedule[agent] = -1;
                                it1++;
                            }
                            else {
                                proposed_schedule[agent] = task_idd;
                                it1 = areas[k].free_agents_area.erase(it1);
                                areas[k].free_tasks_area.erase(task_idd);
                            }
                        }
                        idx++;
                        least++;
                    }
                }
                }));
        }

        for (auto& t : threads) {
            t.join();
        }
    }
    else {
        if (env->map_name == "warehouse_large.map") {
            //give at most half of the entry time_limit to scheduler;
            //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
            if (env->curr_timestep == 0) {
                for (int i = 0; i < env->num_of_agents; i++) {
                    agv_imformation[i].id = i;
                }
            }
            auto start_time = std::chrono::steady_clock::now();
            TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit - 400);
            free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
            free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
            int satisfied = 10;
            if (wait == true) {
                for (auto pairs : env->task_pool) {
                    auto i = pairs.second;
                    if (i.t_completed != -1)continue;
                    else {
                        if (i.agent_assigned != -1) {
                            if (i.idx_next_loc == 0) {
                                if (agv_imformation[i.agent_assigned].gettime + satisfied < env->curr_timestep) {
                                    free_tasks.insert(i.task_id);
                                    free_agents.insert(i.agent_assigned);
                                    proposed_schedule[i.agent_assigned] = -1;
                                }
                            }
                        }
                    }
                }
            }
            else {
                if (free_agents.size() < 3000) {
                    wait = true;
                }
            }
            int min_task_i, min_task_makespan, c_loc, count;
            double dist = 0;
            double a = 0.1;
            clock_t start = clock();
            int cnt = 0;
            if (env->curr_timestep != 0) {
                for (auto i : env->new_freeagents) {
                    agv_imformation[i].solved_nums++;
                }
            }
            for (auto pairs : env->task_pool) {
                auto i = pairs.second;
                if (i.t_completed != -1)continue;
                else {
                    if (i.agent_assigned != -1) {
                        if (i.idx_next_loc == 0) {
                            agv_imformation[i.agent_assigned].searching_time++;
                        }
                        else {
                            agv_imformation[i.agent_assigned].solving_time++;
                        }
                    }
                }
            }
            std::vector<Agv_imformation> new_agent_pool;
            for (int j : free_agents) {
                new_agent_pool.push_back(agv_imformation[j]);
            }
            if (new_agent_pool.size() == 0)return;
            sort(new_agent_pool.begin(), new_agent_pool.end(), AgvCompare());
            auto start_time1 = std::chrono::steady_clock::now();
            int numTasks = free_tasks.size();
            int desiredTasksPerCluster = numTasks / 20;
            int k = std::max(1, numTasks / desiredTasksPerCluster);
            k = std::min(k, 250);
            std::vector<std::array<float, 4>> taskData;
            taskData.reserve(numTasks);
            std::vector<int> taskIds;
            taskIds.reserve(numTasks);
            for (int task : free_tasks) {
                int loc1 = env->task_pool.at(task).locations[0];
                int loc2 = env->task_pool.at(task).locations[(int)env->task_pool.at(task).locations.size() - 1];
                float row1 = static_cast<float>(loc1 / env->cols);
                float col1 = static_cast<float>(loc1 % env->cols);
                float row2 = static_cast<float>(loc2 / env->cols);
                float col2 = static_cast<float>(loc2 % env->cols);
                taskData.push_back({ row1, col1, row2, col2 });
                taskIds.push_back(task);
            }
            auto result = dkm::kmeans_lloyd(taskData, k);
            auto centroids = std::get<0>(result);  // 聚类中心，可能少于 k 个
            auto labels = std::get<1>(result);       // 每个任务对应的聚类标签
            int actual_k = centroids.size();
            std::unordered_map<int, int>quyu;
            for (size_t i = 0; i < labels.size(); i++) {
                int clusterLabel = labels[i];
                if (clusterLabel < 0 || clusterLabel >= actual_k) {
                    clusterLabel = 0;
                }
                int taskId = taskIds[i];
                quyu[taskId] = clusterLabel;
            }
            vector<int>need_erase;
            int flag1 = 0;
            int block_rows = 14;  // 每个块的行数         
            int block_cols = 10;  // 每个块的列数         
            int total_rows = env->rows;  // 总行数
            int total_cols = env->cols;  // 总列数
            // 计算总的块数
            int num_blocks_x = (total_cols + block_cols - 1) / block_cols;
            int num_blocks_y = (total_rows + block_rows - 1) / block_rows;
            int qidian = 0, zhongdian = 0;
            double row, col;
            int block_row;
            int block_col;
            int area_index;
            vector<int>xsd(1000, 0);
            for (auto it : new_agent_pool)
            {
                //keep assigning until timeout
                if (std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                int i = it.id;
                assert(env->curr_task_schedule[i] == -1);
                min_task_i = -1;
                min_task_makespan = INT_MAX;
                count = 0;
                // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
                for (int t_id : free_tasks)
                {
                    //check for timeout every 10 task evaluations
                    if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                    {
                        break;
                    }
                    dist = 0;
                    c_loc = env->curr_states.at(i).location;
                    flag1 = 0;
                    // iterate over the locations (errands) of the task to compute the makespan to finish the task
                    // makespan: the time for the agent to complete all the errands of the task t_id in order
                    for (int loc : env->task_pool[t_id].locations) {
                        if (flag1 == 0) {
                            dist += DefaultPlanner::get_h(env, c_loc, loc);
                            flag1++;
                            //row = loc / env->cols;
                            //col = loc % env->cols;
                            //block_row = row / block_rows;  
                            //block_col = col / block_cols;  
                            //qidian = block_row * num_blocks_x + block_col;
                        }
                        else {
                            dist += DefaultPlanner::get_h(env, c_loc, loc) * a;
                            //row = loc / env->cols;
                            //col = loc % env->cols;
                            //block_row = row / block_rows;
                            //block_col = col / block_cols;
                            //zhongdian = block_row * num_blocks_x + block_col;
                        }
                        c_loc = loc;
                    }
                    dist += xsd[quyu[t_id]] * 3;
                    // update the new minimum makespan
                    if (dist < min_task_makespan) {
                        min_task_i = t_id;
                        min_task_makespan = dist;
                    }
                    count++;
                }
                // assign the best free task to the agent i (assuming one exists)
                if (min_task_i != -1) {
                    free_tasks.erase(min_task_i);
                    need_erase.push_back(i);
                    proposed_schedule[i] = min_task_i;
                    agv_imformation[i].gettime = env->curr_timestep;
                    agv_imformation[i].at_dist = DefaultPlanner::get_h(env, env->curr_states.at(i).location, env->task_pool[min_task_i].locations[0]);
                    flag1 = 0;
                    /* for (int loc : env->task_pool[min_task_i].locations) {
                         if (flag1 == 0) {
                             flag1++;
                             row = loc / env->cols;
                             col = loc % env->cols;
                             block_row = row / block_rows;
                             block_col = col / block_cols;
                             qidian = block_row * num_blocks_x + block_col;
                         }
                         else {
                             row = loc / env->cols;
                             col = loc % env->cols;
                             block_row = row / block_rows;
                             block_col = col / block_cols;
                             zhongdian = block_row * num_blocks_x + block_col;
                         }
                     }*/
                    xsd[quyu[min_task_i]]++;
                }
                else {
                    proposed_schedule[i] = -1;
                }
            }
            for (int i : need_erase) {
                free_agents.erase(i);
            }
            return;
        }
        else {
            //give at most half of the entry time_limit to scheduler;
            //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
            if (env->curr_timestep == 0) {
                for (int i = 0; i < env->num_of_agents; i++) {
                    agv_imformation[i].id = i;
                }
            }
            auto start_time = std::chrono::steady_clock::now();
            TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit - 400);
            free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
            free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
            int satisfied = 20;
            if (wait == true) {
                cout << "Let's go!!!!!! " << endl;
                for (auto pairs : env->task_pool) {
                    auto i = pairs.second;
                    if (i.t_completed != -1)continue;
                    else {
                        if (i.agent_assigned != -1) {
                            if (i.idx_next_loc == 0) {
                                if (agv_imformation[i.agent_assigned].gettime + satisfied < env->curr_timestep) {
                                    free_tasks.insert(i.task_id);
                                    free_agents.insert(i.agent_assigned);
                                    proposed_schedule[i.agent_assigned] = -1;
                                }
                            }
                        }
                    }
                }
            }
            else {
                if (free_agents.size() < 3000) {
                    wait = true;
                }
            }
            if (wait == true) {
                for (int k : free_tasks) {
                    for (int i = 0; i < env->num_of_agents; i++) {
                        if (proposed_schedule[i] == k)cout << "error" << endl;
                    }
                }
            }
            int min_task_i, min_task_makespan, c_loc, count;
            double dist = 0;
            double a = 0.1;
            clock_t start = clock();
            int cnt = 0;
            if (env->curr_timestep != 0) {
                for (auto i : env->new_freeagents) {
                    agv_imformation[i].solved_nums++;
                }
            }
            for (auto pairs : env->task_pool) {
                auto i = pairs.second;
                if (i.t_completed != -1)continue;
                else {
                    if (i.agent_assigned != -1) {
                        if (i.idx_next_loc == 0) {
                            agv_imformation[i.agent_assigned].searching_time++;
                        }
                        else {
                            agv_imformation[i.agent_assigned].solving_time++;

                        }
                    }
                }
            }
            std::vector<Agv_imformation> new_agent_pool;
            for (int j : free_agents) {
                new_agent_pool.push_back(agv_imformation[j]);
            }

            if (new_agent_pool.size() == 0)return;
            sort(new_agent_pool.begin(), new_agent_pool.end(), AgvCompare());
            vector<int>need_erase;
            int flag1 = 0;
            for (auto it : new_agent_pool)
            {
                //keep assigning until timeout
                if (std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                int i = it.id;

                assert(env->curr_task_schedule[i] == -1);

                min_task_i = -1;
                min_task_makespan = INT_MAX;
                count = 0;

                // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
                for (int t_id : free_tasks)
                {
                    //check for timeout every 10 task evaluations
                    if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                    {
                        break;
                    }
                    dist = 0;
                    c_loc = env->curr_states.at(i).location;
                    flag1 = 0;
                    // iterate over the locations (errands) of the task to compute the makespan to finish the task
                    // makespan: the time for the agent to complete all the errands of the task t_id in order
                    for (int loc : env->task_pool[t_id].locations) {
                        if (flag1 == 0) {
                            dist += DefaultPlanner::get_h(env, c_loc, loc);
                            flag1++;
                        }
                        else dist += DefaultPlanner::get_h(env, c_loc, loc) * a;
                        c_loc = loc;
                    }

                    // update the new minimum makespan
                    if (dist < min_task_makespan) {
                        min_task_i = t_id;
                        min_task_makespan = dist;
                    }
                    count++;
                }

                // assign the best free task to the agent i (assuming one exists)
                if (min_task_i != -1) {
                    free_tasks.erase(min_task_i);
                    need_erase.push_back(i);
                    proposed_schedule[i] = min_task_i;
                    agv_imformation[i].gettime = env->curr_timestep;
                    agv_imformation[i].at_dist = DefaultPlanner::get_h(env, env->curr_states.at(i).location, env->task_pool[min_task_i].locations[0]);
                }
                else {
                    proposed_schedule[i] = -1;
                }
            }
            for (int i : need_erase) {
                free_agents.erase(i);
            }
            return;
        }
    }
}