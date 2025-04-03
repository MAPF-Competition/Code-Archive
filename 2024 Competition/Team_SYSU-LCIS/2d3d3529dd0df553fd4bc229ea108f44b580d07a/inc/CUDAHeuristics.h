#pragma once

#include <chrono>
#include <deque>
#include <iostream>
#include <unordered_set>

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
// cuda
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/replace.h>
#include <thrust/functional.h>

#include "SharedEnv.h"
#include "scheduler.h"

#define CHECK_RUNTIMING_API(call)                                                                                                         \
  {                                                                                                                                       \
    const cudaError_t error = call;                                                                                                       \
    if (error != cudaSuccess)                                                                                                             \
    {                                                                                                                                     \
      std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << ",code:" << error << ",reason:" << cudaGetErrorString(error) << std::endl; \
      exit(1);                                                                                                                            \
    }                                                                                                                                     \
  }

#define CHECK_DRIVER_API(call)                                                          \
  {                                                                                     \
    const CUresult error = call;                                                        \
    if (error != CUDA_SUCCESS)                                                          \
    {                                                                                   \
      std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << "get function error!\n"; \
      exit(1);                                                                          \
    }                                                                                   \
  }

enum Orientation
{
  LEFT = 2,
  DOWN = 3,
  RIGHT = 5,
  UP = 7
};

class CUDAHeuristics
{
private:
  SharedEnvironment *env;
  const int intSize = sizeof(int);

  dim3 BLOCK;
  int device_id;
  int savedSizeHost;

  int devPtrSize = 8;
  int **freeHeuristictablePtrDev;
  std::array<int*, 8> freeHeuristictableDev;
  int freeHeuristictableDevLen, *mapLenDev;

  int *xyDev, *MapDev, *savedSizeDev, *neighborsDev;
  int MapSize, neighborsSize, *neighborsHost, *xyHost;
  int *taskNumDev, *taskIDsDev, *agentNumDev, *scheduleAgentTaskNumDev;
  int *agentsLocDev, *tasks2DisDev, *tasksLocDev;
  int *mapsPtrDev;

  std::map<int, int> taskIDsHost, taskPtrsHost;

public:
  int free_state_num;
  std::unordered_set<int> map_in_cuda;
  std::unordered_set<int> freeTaskMapIDsSet;
  std::unordered_map<int, int> global_tasks_map;
  std::vector<int> loc_dict, loc_dict_rever, global_free_heuristictable;
  std::vector<std::vector<int>> global_heuristictable, global_neighbors, global_orientation, orientation_heuristictable;

  int get_h(int source, int target);
  void initialize();
  void cal_heuristic(std::vector<int> &cal_loc);
  void agent2cuda(std::vector<int> &agent_trans_ids, std::vector<int> &agent_locs);
  void task2cuda(std::vector<int> &map_ids, std::vector<int> &task_locs, std::vector<int> &task_cost);
  void copy2cuda();

  int *FGDC(std::vector<int> &cal_loc);
  std::pair<int *, int *> FGDC_orientation(std::vector<int> &cal_loc);

  std::mutex schedule_mtx;
  std::condition_variable schedule_cv;

  CUDAHeuristics(int device_id, SharedEnvironment *env) : BLOCK(512), device_id(device_id), env(env) {};
  ~CUDAHeuristics()
  {
    CHECK_RUNTIMING_API(cudaFree(xyDev));
    CHECK_RUNTIMING_API(cudaFree(MapDev));
    CHECK_RUNTIMING_API(cudaFree(savedSizeDev));
    CHECK_RUNTIMING_API(cudaFree(taskNumDev));
    CHECK_RUNTIMING_API(cudaFree(taskIDsDev));
    CHECK_RUNTIMING_API(cudaFree(agentNumDev));
    CHECK_RUNTIMING_API(cudaFree(scheduleAgentTaskNumDev));
    CHECK_RUNTIMING_API(cudaFree(agentsLocDev));
    CHECK_RUNTIMING_API(cudaFree(tasks2DisDev));
    CHECK_RUNTIMING_API(cudaFree(tasksLocDev));
    CHECK_RUNTIMING_API(cudaFree(mapsPtrDev));
  };

  int *initNeighbors();
  dim3 calGrid(int size);
  void freeTaskId(int t_id);
  int insertTaskId(int t_id);
  void setStateNum(int stateNum, SharedEnvironment *env);
  void locDevice(int *&dev, int *host, int bytes);
  void initDevice(SharedEnvironment *env);
  std::vector<int> *schedule(std::vector<int> &agentsHost, std::vector<int> &freeTasksID, std::vector<int> &unfinished_agent_cost);

  void printMap(int *map, int *xy);
};

__global__ void printfMap(int *map, int *xy);
__global__ void printCost(int *dist, int *scheduleAgentTaskNum);

__global__ void Agent2Cuda(int *agentIdsDev, int *agentLocsDev, int *agentsLocDev, int *agentLenDev);
__global__ void Task2Cuda(int *mapIdsDev, int *taskLocsDev, int *taskCostDev, int *tasksLocDev, int *tasks2DisDev, int *taskLenDev);

__global__ void hungary(int *dist, int *scheduleAgentTaskNum, int *min_val);
__global__ void initNeighborDevice(int *map, int *neighborsDev, int *xy);
__global__ void update(int *global_map, int *xy, int *locNum, int *finished);
__global__ void initGlobalMap(int *global_map, int *map, int *xy, int *locNum, int *cal_loc);
__global__ void calCost(int *dist, int **freeHeuristictablePtrDev, int *tasksLocDev, int *agents_loc,
                        int *task2_cost, int *global_tasks_id, int *global_agent_id, int *s_ATNum,
                        int *savedSizeDev);

__global__ void setGoalSignal(int *global_map, int *orientations, int *map, int *xy, int *locNum, int *cal_loc);
__global__ void initOrientationMap(int *global_map, int *orientations, int *map, int *xy, int *locNum, int *cal_loc);
__global__ void update_orientation(int *global_map, int *orientations, int *neighbors, int *xy, int *locNum, int *finished);
__global__ void copyMap2Cuda(int *transferHMap, int *transferIdx, int *tasksMap, int *xy, int *mapLenDev);

__global__ void updateUnfinished(int *dist, int *agents_loc, int *global_agent_id, int *s_ATNum,
                                 int *savedSizeDev, int *unfinishedAgentCostDev, int *unfinishedNumDev);
