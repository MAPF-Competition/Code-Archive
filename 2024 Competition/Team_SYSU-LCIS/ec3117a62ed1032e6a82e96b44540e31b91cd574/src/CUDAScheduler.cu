#include "CUDAScheduler.h"

void CUDAScheduler::locDevice(int *&dev, int *host, int bytes)
{
  CHECK_RUNTIMING_API(cudaMalloc((void **)&dev, bytes));
  CHECK_RUNTIMING_API(cudaMemcpy(dev, host, bytes, cudaMemcpyHostToDevice));
}

void CUDAScheduler::printMap(int *map, int *xy)
{
  printf("============================================================\n");
  int a;
  for (int y = 0; y < xy[1]; y++)
  {
    printf("row %d: ", y);
    for (int x = 0; x < xy[0]; x++)
    {
      a = map[y * xy[0] + x];
      if (a == INT_MAX)
      {
        printf("-1 ");
      }
      else if (a == -1 || a > 9)
      {
        printf("%d ", a);
      }
      else
      {
        printf(" %d ", a);
      }
    }
    printf("\n");
  }
  printf("============================================================\n");
}

__global__ void printfMap(int *map, int *xy)
{
  printf("============================================================\n");
  int a;
  for (int y = 0; y < xy[1]; y++)
  {
    printf("row %d: ", y);
    for (int x = 0; x < xy[0]; x++)
    {
      a = map[y * xy[0] + x];
      if (a == -1 || a > 9)
      {
        printf("%d ", a);
      }
      else
      {
        printf(" %d ", a);
      }
    }
    printf("\n");
  }
  printf("============================================================\n");
}

void CUDAScheduler::initDevice(int devNum, SharedEnvironment *env)
{
  cudaDeviceProp deviceProp;
  CHECK_RUNTIMING_API(cudaGetDeviceProperties(&deviceProp, devNum));
  // std::cout << "Using device " << dev << ": " << deviceProp.name << std::endl;
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));

  int taskNum = env->num_of_agents * 1.5;
  for (int i = 0; i < taskNum; ++i)
  {
    freeTaskMapIDsSet.insert(i);
  }

  MapSize = intSize * env->map.size();
  neighborsSize = MapSize * 4;
  xyHost = new int[2]{env->cols, env->rows};
  neighborsHost = new int[env->map.size() * 4];

  locDevice(taskNumDev, &taskNum, intSize);
  locDevice(agentNumDev, &env->num_of_agents, intSize);
  locDevice(xyDev, xyHost, intSize * 2);
  locDevice(MapDev, env->map.data(), MapSize);
  CHECK_RUNTIMING_API(cudaMalloc((void **)&tasks2DisDev, taskNum * intSize));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&agentsLocDev, env->num_of_agents * intSize));
}

int *CUDAScheduler::initNeighbors()
{
  int *neighborsDev;
  CHECK_RUNTIMING_API(cudaMalloc((void **)&neighborsDev, neighborsSize));
  dim3 grid = calGrid(xyHost[0] * xyHost[1]);
  initNeighborDevice<<<grid, BLOCK>>>(MapDev, neighborsDev, xyDev);
  CHECK_RUNTIMING_API(cudaMemcpy(neighborsHost, neighborsDev, neighborsSize, cudaMemcpyDeviceToHost));
  CHECK_RUNTIMING_API(cudaFree(neighborsDev));
  return neighborsHost;
}

void CUDAScheduler::setStateNum(int stateNum, SharedEnvironment *env)
{
  savedSizeHost = stateNum;
  locDevice(savedSizeDev, &savedSizeHost, intSize);
  // CHECK_RUNTIMING_API(cudaMalloc((void **)&mapsPtrDev, savedSizeHost * savedSizeHost * intSize));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&tasksMapDev, env->num_of_agents * 1.5 * savedSizeHost * intSize));
}

dim3 CUDAScheduler::calGrid(int size)
{
  int gridSize = size / (BLOCK.x * BLOCK.y);
  return dim3((size - gridSize) > 0 ? gridSize + 1 : gridSize);
}

void CUDAScheduler::task2cuda(int devNum, int t_id, std::vector<int> &h_map, int task_cost)
{
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));
  int mapIdxHost = insertTaskId(t_id);

  int *transferIdxDev, *transferHMapDev, *transferValueDev;
  int transferIdxHost = mapIdxHost * savedSizeHost;
  locDevice(transferIdxDev, &transferIdxHost, intSize);
  locDevice(transferHMapDev, h_map.data(), intSize * h_map.size());
  copyMap2Cuda<<<calGrid(h_map.size()), BLOCK>>>(transferHMapDev, transferIdxDev, tasksMapDev, savedSizeDev);

  std::vector<int> transferValueHost{mapIdxHost, task_cost};
  locDevice(transferValueDev, transferValueHost.data(), intSize * 2);
  copyValue2Cuda<<<1, 1>>>(tasks2DisDev, transferValueDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(transferIdxDev));
  CHECK_RUNTIMING_API(cudaFree(transferHMapDev));
  CHECK_RUNTIMING_API(cudaFree(transferValueDev));
}

void CUDAScheduler::map2cuda(int devNum, std::vector<std::vector<int>> &map_loc)
{
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));
  for (int m_id = 0; m_id < map_loc.size(); ++m_id)
  {
    int *transferHMapDev, *transferIdxDev;
    locDevice(transferIdxDev, &m_id, intSize);
    locDevice(transferHMapDev, map_loc[m_id].data(), intSize * map_loc.size());
    copyMap2Cuda<<<calGrid(savedSizeHost), BLOCK>>>(transferHMapDev, transferIdxDev, mapsPtrDev, savedSizeDev);
  }
}

void CUDAScheduler::agent2cuda(int devNum, int a_id, int agent_loc)
{
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));
  int *transferValueDev;
  std::vector<int> transferValueHost{a_id, agent_loc};
  locDevice(transferValueDev, transferValueHost.data(), intSize * 2);
  copyValue2Cuda<<<1, 1>>>(agentsLocDev, transferValueDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(transferValueDev));
}

thrust::host_vector<int> CUDAScheduler::schedule(int devNum, std::vector<int> &agentsHost, std::vector<int> &freeTasksID)
{
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));
  int agentNum = agentsHost.size();
  int taskNum = freeTasksID.size();
  std::vector<int> s_ATNum{agentNum, taskNum};
  std::vector<int> tasksHost(taskNum);
  std::unordered_set<int> scheduledAgentsSet, scheduledTasksSet;
  for (int i = 0; i < taskNum; ++i)
  {
    tasksHost[i] = taskPtrsHost[freeTasksID[i]];
  }
  int *s_ATNumDev, *taskIDsDev, *agentIDsDev, *distDev;
  locDevice(s_ATNumDev, s_ATNum.data(), intSize * 2);
  locDevice(taskIDsDev, tasksHost.data(), intSize * taskNum);
  locDevice(agentIDsDev, agentsHost.data(), intSize * agentNum);
  CHECK_RUNTIMING_API(cudaMalloc((void **)&distDev, agentNum * taskNum * intSize));

  dim3 grid = calGrid(agentNum * taskNum);
  calCost<<<grid, BLOCK>>>(distDev, tasksMapDev, agentsLocDev, tasks2DisDev, taskIDsDev, agentIDsDev, s_ATNumDev, savedSizeDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  thrust::device_ptr<int> distDevPtr(distDev);
  thrust::device_vector<int> indices(agentNum * taskNum);
  thrust::sequence(indices.begin(), indices.end());
  thrust::sort_by_key(distDevPtr, distDevPtr + agentNum * taskNum, indices.begin());
  thrust::host_vector<int> minValuesHost = indices;
  CHECK_RUNTIMING_API(cudaFree(s_ATNumDev));
  CHECK_RUNTIMING_API(cudaFree(taskIDsDev));
  CHECK_RUNTIMING_API(cudaFree(agentIDsDev));
  CHECK_RUNTIMING_API(cudaFree(distDev));
  return minValuesHost;
}

void CUDAScheduler::freeTaskId(int t_id)
{
  freeTaskMapIDsSet.insert(taskPtrsHost[t_id]);
  taskPtrsHost.erase(t_id);
}

int CUDAScheduler::insertTaskId(int t_id)
{
  int mapIdxHost;
  if (!freeTaskMapIDsSet.empty())
  {
    mapIdxHost = *freeTaskMapIDsSet.begin();
    freeTaskMapIDsSet.erase(freeTaskMapIDsSet.begin());
  }
  else
  {
    throw std::runtime_error("No free task map IDs left!");
  }
  taskIDsHost[mapIdxHost] = t_id;
  taskPtrsHost[t_id] = mapIdxHost;
  return mapIdxHost;
}

int *CUDAScheduler::FGDC(int devNum, std::vector<int> &cal_loc)
{
  CHECK_RUNTIMING_API(cudaSetDevice(devNum));
  int *locNumDev, *locDev, *mapsDev, *finishedDev;
  int locNum = cal_loc.size();
  locDevice(locNumDev, &locNum, intSize);

  int *locHost = cal_loc.data();
  locDevice(locDev, locHost, intSize * locNum);

  long unsigned int mapsSize = MapSize * locNum;
  long unsigned int mapsHostSize = mapsSize / intSize;
  int *mapsHost = new int[mapsHostSize];
  CHECK_RUNTIMING_API(cudaMalloc((void **)&mapsDev, mapsSize));

  int *finishedHost = new int[2]{1, 0};
  locDevice(finishedDev, finishedHost, intSize * 2);

  dim3 grid = calGrid(xyHost[0] * xyHost[1] * locNum);
  initGlobalMap<<<grid, BLOCK>>>(mapsDev, MapDev, xyDev, locNumDev, locDev);
  int *finishedTransfer = new int[2]{1, 0};
  while (true)
  {
    update<<<grid, BLOCK>>>(mapsDev, xyDev, locNumDev, finishedDev);
    CHECK_RUNTIMING_API(cudaMemcpy(finishedTransfer, finishedDev, intSize * 2, cudaMemcpyDeviceToHost));
    if (finishedTransfer[1] == finishedHost[1])
    {
      break;
    }
    finishedHost[1] = finishedTransfer[1];
    finishedHost[0] += 1;
    CHECK_RUNTIMING_API(cudaMemcpy(finishedDev, finishedHost, intSize * 2, cudaMemcpyHostToDevice));
  }
  CHECK_RUNTIMING_API(cudaMemcpy(mapsHost, mapsDev, mapsSize, cudaMemcpyDeviceToHost));
  // printMap(mapsHost, xyHost);
  // printMap(mapsHost + xyHost[0] * xyHost[1], xyHost);
  CHECK_RUNTIMING_API(cudaFree(mapsDev));
  CHECK_RUNTIMING_API(cudaFree(locNumDev));
  CHECK_RUNTIMING_API(cudaFree(locDev));
  CHECK_RUNTIMING_API(cudaFree(finishedDev));
  free(finishedHost);
  free(finishedTransfer);
  return mapsHost;
}

__global__ void initGlobalMap(int *global_map, int *map, int *xy, int *locNum, int *cal_loc)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int global_idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int nxy = xy[0] * xy[1];
  int idx = global_idx % nxy;
  int total_idx = nxy * (*locNum);
  if (global_idx >= total_idx)
  {
    return;
  }
  if (map[idx] == 1)
  {
    global_map[global_idx] = INT_MAX;
  }
  else
  {
    global_map[global_idx] = 0;
  }
  int loc_idx = global_idx / nxy;
  int map_idx = global_idx - (loc_idx * nxy);
  if (global_map[global_idx] == 0 && cal_loc[loc_idx] == idx)
  {
    int y = map_idx / xy[0];
    int n0 = map_idx - xy[0];
    if (n0 >= 0 && map[n0] == 0)
    {
      global_map[global_idx - xy[0]] = 1;
    }
    int n1 = map_idx - 1;
    if (n1 >= 0 && int(n1 / xy[0]) == y && map[n1] == 0)
    {
      global_map[global_idx - 1] = 1;
    }
    int n2 = map_idx + xy[0];
    if (n2 < nxy && map[n2] == 0)
    {
      global_map[global_idx + xy[0]] = 1;
    }
    int n3 = map_idx + 1;
    if (n3 < nxy && int(n3 / xy[0]) == y && map[n3] == 0)
    {
      global_map[global_idx + 1] = 1;
    }
  }
}

__global__ void update(int *global_map, int *xy, int *locNum, int *finished)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int global_idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int nxy = xy[0] * xy[1];
  int total_idx = nxy * (*locNum);
  if (global_idx < total_idx && global_map[global_idx] == finished[0])
  {
    int loc_idx = global_idx / nxy;
    int map_idx = global_idx - (loc_idx * nxy);
    int y = map_idx / xy[0];
    int n0 = map_idx - xy[0];
    if (n0 >= 0 && global_map[global_idx - xy[0]] == 0)
    {
      global_map[global_idx - xy[0]] = finished[0] + 1;
      finished[1] -= 1;
    }
    int n1 = map_idx - 1;
    if (n1 >= 0 && int(n1 / xy[0]) == y && global_map[global_idx - 1] == 0)
    {
      global_map[global_idx - 1] = finished[0] + 1;
      finished[1] -= 1;
    }
    int n2 = map_idx + xy[0];
    if (n2 < nxy && global_map[global_idx + xy[0]] == 0)
    {
      global_map[global_idx + xy[0]] = finished[0] + 1;
      finished[1] -= 1;
    }
    int n3 = map_idx + 1;
    if (n3 < nxy && int(n3 / xy[0]) == y && global_map[global_idx + 1] == 0)
    {
      global_map[global_idx + 1] = finished[0] + 1;
      finished[1] -= 1;
    }
  }
}

__global__ void initNeighborDevice(int *map, int *neighborsDev, int *xy)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int nxy = xy[0] * xy[1];
  if (idx >= nxy)
  {
    return;
  }
  int neighbor_idx = idx * 4;
  neighborsDev[neighbor_idx] = neighborsDev[neighbor_idx + 1] = neighborsDev[neighbor_idx + 2] = neighborsDev[neighbor_idx + 3] = -1;
  if (map[idx] == 0)
  {
    int y = idx / xy[0];
    int n0 = idx - xy[0];
    if (n0 >= 0 && map[n0] == 0)
    {
      neighborsDev[neighbor_idx] = n0;
    }
    int n1 = idx - 1;
    if (n1 >= 0 && int(n1 / xy[0]) == y && map[n1] == 0)
    {
      neighborsDev[neighbor_idx + 1] = n1;
    }
    int n2 = idx + xy[0];
    if (n2 < nxy && map[n2] == 0)
    {
      neighborsDev[neighbor_idx + 2] = n2;
    }
    int n3 = idx + 1;
    if (n3 < nxy && int(n3 / xy[0]) == y && map[n3] == 0)
    {
      neighborsDev[neighbor_idx + 3] = n3;
    }
  }
}

__global__ void calCost(int *dist, int *tasks_map, int *agents_loc, int *task2_cost,
                        int *global_tasks_id, int *global_agent_id, int *s_ATNum,
                        int *savedSizeDev)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int task_num = s_ATNum[1];
  if (idx >= s_ATNum[0] * task_num)
  {
    return;
  }
  int agent_idx = idx / task_num;
  int task_idx = idx % task_num;
  int global_idx = global_tasks_id[task_idx] * (*savedSizeDev);
  dist[idx] = tasks_map[global_idx + agents_loc[global_agent_id[agent_idx]]] + task2_cost[global_tasks_id[task_idx]];
}

__global__ void copyValue2Cuda(int *vec, int *value)
{
  vec[value[0]] = value[1];
}

__global__ void copyMap2Cuda(int *transferHMap, int *transferIdx, int *tasksMap, int *savedSize)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  if (idx >= *savedSize)
  {
    return;
  }
  tasksMap[*transferIdx + idx] = transferHMap[idx];
}

__global__ void printCost(int *dist, int *s_ATNum)
{
  for (int idx = 0; idx < s_ATNum[0] * s_ATNum[1]; idx++)
  {
    int agent_idx = idx / s_ATNum[1];
    int task_idx = idx % s_ATNum[1];
    if (task_idx == 0)
    {
      printf("\nAgent %d: %d", agent_idx, dist[idx]);
    }
    else
    {
      printf(", %d", dist[idx]);
    }
  }
}
