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

class CUDAScheduler
{
private:
  const int intSize = sizeof(int);

  dim3 BLOCK;
  int device_id;
  int savedSizeHost;
  int *xyDev, *MapDev, *savedSizeDev;
  int MapSize, neighborsSize, *neighborsHost, *xyHost;
  int *taskNumDev, *taskIDsDev, *agentNumDev, *scheduleAgentTaskNumDev;
  int *agentsLocDev, *tasks2DisDev, *tasksMapDev;
  int *mapsPtrDev;

  std::map<int, int> taskIDsHost, taskPtrsHost;
  std::unordered_set<int> freeTaskMapIDsSet;

public:
  struct SchedulePair
  {
    int agent_id;
    int task_id;
  };
  std::mutex schedule_mtx;
  std::condition_variable schedule_cv;
  std::deque<SchedulePair> schedule_deque;

  CUDAScheduler(int device_id) : BLOCK(512), device_id(device_id) {};
  ~CUDAScheduler()
  {
    CHECK_RUNTIMING_API(cudaFree(xyDev));
    CHECK_RUNTIMING_API(cudaFree(MapDev));
    CHECK_RUNTIMING_API(cudaFree(taskNumDev));
    CHECK_RUNTIMING_API(cudaFree(taskIDsDev));
    CHECK_RUNTIMING_API(cudaFree(agentNumDev));
    CHECK_RUNTIMING_API(cudaFree(scheduleAgentTaskNumDev));
    CHECK_RUNTIMING_API(cudaFree(agentsLocDev));
    CHECK_RUNTIMING_API(cudaFree(tasks2DisDev));
    CHECK_RUNTIMING_API(cudaFree(tasksMapDev));
  };

  int *initNeighbors();
  dim3 calGrid(int size);
  void freeTaskId(int t_id);
  int insertTaskId(int t_id);
  void setStateNum(int stateNum, SharedEnvironment *env);
  void locDevice(int *&dev, int *host, int bytes);
  void initDevice(int devNum, SharedEnvironment *env);
  void agent2cuda(int devNum, int a_id, int agent_loc);
  void map2cuda(int devNum, std::vector<std::vector<int>> &map_loc);
  int *FGDC(int devNum, std::vector<int> &cal_loc);
  void task2cuda(int devNum, int t_id, std::vector<int> &h_map, int task_cost);
  thrust::host_vector<int> schedule(int devNum, std::vector<int> &agentsHost, std::vector<int> &freeTasksID);

  void printMap(int *map, int *xy);
};

__global__ void printfMap(int *map, int *xy);
__global__ void printCost(int *dist, int *scheduleAgentTaskNum);

__global__ void copyValue2Cuda(int *vec, int *value);
__global__ void initNeighborDevice(int *map, int *neighborsDev, int *xy);
__global__ void update(int *global_map, int *xy, int *locNum, int *finished);
__global__ void copyMap2Cuda(int *transferHMap, int *transferIdx, int *tasksMap, int *xy);
__global__ void initGlobalMap(int *global_map, int *map, int *xy, int *locNum, int *cal_loc);
__global__ void calCost(int *dist, int *tasks_map, int *agents_loc, int *task2_loc,
                        int *global_tasks_id, int *global_agent_id, int *scheduleAgentTaskNum, 
                        int *savedSizeDev);
