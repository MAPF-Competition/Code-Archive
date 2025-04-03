#include "CUDAHeuristics.h"

void CUDAHeuristics::locDevice(int *&dev, int *host, int bytes)
{
  CHECK_RUNTIMING_API(cudaMalloc((void **)&dev, bytes));
  CHECK_RUNTIMING_API(cudaMemcpy(dev, host, bytes, cudaMemcpyHostToDevice));
}

void CUDAHeuristics::printMap(int *map, int *xy)
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

__global__ void printfTaskMap(int **map, int *xy, int *transferIdxDev, int *mapLenDev)
{
  printf("============================================================\n");
  int a;
  for (int i = 0; i < *xy; i++)
  {
    a = map[*transferIdxDev][i];
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
  printf("============================================================\n");
}

void CUDAHeuristics::initialize()
{
  clock_t total_time = clock();
  initDevice(env);
  int *neighborsHost = initNeighbors();
  // global_heuristictable.resize(env->map.size());
  // global_neighbors.resize(env->rows * env->cols);
  DefaultPlanner::global_heuristictable.resize(env->map.size());
  DefaultPlanner::global_neighbors.resize(env->rows * env->cols);

  loc_dict.resize(env->map.size(), -1);
  free_state_num = 0;
  for (int loc = 0; loc < env->map.size(); loc++)
  {
    if (env->map[loc] == 0)
    {
      loc_dict[loc] = free_state_num;
      loc_dict_rever.push_back(loc);
      free_state_num++;
      int neighbor_idx = 4 * loc;
      for (int i = 0; i < 4; i++)
      {
        if (neighborsHost[neighbor_idx + i] > -1)
        {
          // global_neighbors[loc].push_back(neighborsHost[neighbor_idx + i]);
          DefaultPlanner::global_neighbors[loc].push_back(neighborsHost[neighbor_idx + i]);
        }
      }
    }
  }
  free(neighborsHost);
  setStateNum(free_state_num, env);
  cout << "Free State Num: " << free_state_num << endl;

  long unsigned int cal_size = (1ULL << 28) / env->map.size();
  // global_heuristictable.resize(env->map.size());
  orientation_heuristictable.resize(free_state_num);
  // global_orientation.resize(free_state_num);
  // global_free_heuristictable.resize(free_state_num * free_state_num, -1);
  std::vector<int> cal_loc;
  cal_loc.resize(cal_size);
  long unsigned int idx = 0;
  for (int i = 0; i < env->map.size(); i++)
  {
    if (env->map[i] == 0)
    {
      // global_heuristictable.at(i).resize(env->map.size(), INT_MAX);

      cal_loc[idx] = i;
      idx++;
      if (idx == cal_size)
      {
        cal_heuristic(cal_loc);
        idx = 0;
      }
    }
  }
  if (idx > 0)
  {
    cal_loc.resize(idx);
    cal_heuristic(cal_loc);
  }
  copy2cuda();
  cout << "Init Time Usage: " << ((float)(clock() - total_time)) / CLOCKS_PER_SEC << endl;
  return;
}

void CUDAHeuristics::cal_heuristic(std::vector<int> &cal_loc)
{
  // int *h = FGDC(cal_loc);
  int idx_s = 0;
  int *h_ori, *ori;
  std::tie(h_ori, ori) = FGDC_orientation(cal_loc);
  for (int i = 0; i < cal_loc.size(); i++)
  {
    int idx_e = idx_s + env->map.size();
    int real_idx = cal_loc[i];
    // global_heuristictable.at(real_idx).assign(h + idx_s, h + idx_e);
    // global_heuristictable.at(real_idx)[real_idx] = 0;
    // DefaultPlanner::global_heuristictable.at(real_idx).htable.assign(h_ori + idx_s, h_ori + idx_e);
    // DefaultPlanner::global_heuristictable.at(real_idx).htable[real_idx] = 0;
    // DefaultPlanner::global_heuristictable.at(real_idx).htable.assign(h + idx_s, h + idx_e);
    // DefaultPlanner::global_heuristictable.at(real_idx).htable[real_idx] = 0;

    int free_idx = loc_dict[real_idx];
    orientation_heuristictable.at(free_idx).resize(free_state_num, INT_MAX);
    // global_orientation.at(free_idx).resize(free_state_num, 1);
    for (int j = 0; j < env->map.size(); j++)
    {
      int free_idx_ = loc_dict[j];
      if (free_idx_ == -1)
      {
        continue;
      }
      orientation_heuristictable[free_idx][free_idx_] = h_ori[j + idx_s];
      // global_orientation[free_idx][free_idx_] = ori[j + idx_s];
    }
    orientation_heuristictable.at(free_idx)[free_idx] = 0;
    // global_orientation.at(free_idx)[free_idx] = Orientation::DOWN * Orientation::UP * Orientation::LEFT * Orientation::RIGHT;
    idx_s = idx_e;
  }
  // free(h);
  free(h_ori);
  free(ori);
}

int CUDAHeuristics::get_h(int source, int target)
{
  return orientation_heuristictable[loc_dict[target]][loc_dict[source]];
  // return global_heuristictable[target][source];
}

void CUDAHeuristics::initDevice(SharedEnvironment *env)
{
  cudaDeviceProp deviceProp;
  CHECK_RUNTIMING_API(cudaGetDeviceProperties(&deviceProp, device_id));
  // std::cout << "Using device " << dev << ": " << deviceProp.name << std::endl;
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));

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

int *CUDAHeuristics::initNeighbors()
{
  CHECK_RUNTIMING_API(cudaMalloc((void **)&neighborsDev, neighborsSize));
  dim3 grid = calGrid(xyHost[0] * xyHost[1]);
  initNeighborDevice<<<grid, BLOCK>>>(MapDev, neighborsDev, xyDev);
  CHECK_RUNTIMING_API(cudaMemcpy(neighborsHost, neighborsDev, neighborsSize, cudaMemcpyDeviceToHost));
  return neighborsHost;
}

void CUDAHeuristics::setStateNum(int stateNum, SharedEnvironment *env)
{
  savedSizeHost = stateNum;
  locDevice(savedSizeDev, &savedSizeHost, intSize);
  // CHECK_RUNTIMING_API(cudaMalloc((void **)&mapsPtrDev, savedSizeHost * savedSizeHost * intSize));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&tasksLocDev, env->num_of_agents * 1.5 * intSize));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&freeHeuristictablePtrDev, sizeof(int **) * savedSizeHost));
  freeHeuristictableDevLen = savedSizeHost % devPtrSize == 0 ? savedSizeHost / devPtrSize : savedSizeHost / devPtrSize + 1;
  locDevice(mapLenDev, &freeHeuristictableDevLen, intSize);
  int saved_size = 0;
  for (int i = 0; i < devPtrSize; i++)
  {
    int size_i = saved_size + freeHeuristictableDevLen > savedSizeHost ? savedSizeHost - saved_size : freeHeuristictableDevLen;
    CHECK_RUNTIMING_API(cudaMalloc((void **)&freeHeuristictableDev[i], intSize * size_i * savedSizeHost));
    saved_size += size_i;
  }
}

dim3 CUDAHeuristics::calGrid(int size)
{
  int gridSize = size / (BLOCK.x * BLOCK.y);
  return dim3((size - (gridSize * BLOCK.x * BLOCK.y)) > 0 ? gridSize + 1 : gridSize);
}

void CUDAHeuristics::copy2cuda()
{
  int **freeHeuristictablePtr = (int **)malloc(sizeof(int *) * savedSizeHost);
  for (int i = 0; i < savedSizeHost; i++)
  {
    int section_id = i / freeHeuristictableDevLen;
    int map_id = i % freeHeuristictableDevLen;
    freeHeuristictablePtr[i] = freeHeuristictableDev[section_id] + savedSizeHost * map_id;
  }
  cudaMemcpy(freeHeuristictablePtrDev, freeHeuristictablePtr, sizeof(int *) * savedSizeHost, cudaMemcpyHostToDevice);
  free(freeHeuristictablePtr);

  int *transferHMapDev, *transferIdxDev;
  CHECK_RUNTIMING_API(cudaMalloc((void **)&transferHMapDev, intSize * savedSizeHost));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&transferIdxDev, intSize));
  for (int i = 0; i < free_state_num; i++)
  {
    CHECK_RUNTIMING_API(cudaMemcpy(transferHMapDev, orientation_heuristictable[i].data(), intSize * savedSizeHost, cudaMemcpyHostToDevice));
    CHECK_RUNTIMING_API(cudaMemcpy(transferIdxDev, &i, intSize, cudaMemcpyHostToDevice));
    int section_id = i / freeHeuristictableDevLen;
    copyMap2Cuda<<<calGrid(savedSizeHost), BLOCK>>>(transferHMapDev, transferIdxDev, freeHeuristictableDev[section_id], savedSizeDev, mapLenDev);
    CHECK_RUNTIMING_API(cudaDeviceSynchronize());
    // printf("section_id: %d, transferIdxDev: %d\n", section_id, i);
    // printfTaskMap<<<1, 1>>>(freeHeuristictablePtrDev, savedSizeDev, transferIdxDev, mapLenDev);
    // CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  }
  CHECK_RUNTIMING_API(cudaFree(transferHMapDev));
  CHECK_RUNTIMING_API(cudaFree(transferIdxDev));
}

void CUDAHeuristics::task2cuda(std::vector<int> &map_ids, std::vector<int> &task_locs, std::vector<int> &task_cost)
{
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));
  int *mapIdsDev, *taskLocsDev, *taskCostDev, *taskLenDev;
  int taskLen = map_ids.size();
  locDevice(taskLenDev, &taskLen, intSize);
  locDevice(mapIdsDev, map_ids.data(), intSize * taskLen);
  locDevice(taskLocsDev, task_locs.data(), intSize * taskLen);
  locDevice(taskCostDev, task_cost.data(), intSize * taskLen);
  Task2Cuda<<<calGrid(taskLen), BLOCK>>>(mapIdsDev, taskLocsDev, taskCostDev, tasksLocDev, tasks2DisDev, taskLenDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(mapIdsDev));
  CHECK_RUNTIMING_API(cudaFree(taskLocsDev));
  CHECK_RUNTIMING_API(cudaFree(taskCostDev));
  CHECK_RUNTIMING_API(cudaFree(taskLenDev));
}

void CUDAHeuristics::agent2cuda(std::vector<int> &agent_trans_ids, std::vector<int> &agent_locs)
{
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));
  int *agentIdsDev, *agentLocsDev, *agentLenDev;
  int agentLen = agent_trans_ids.size();
  locDevice(agentLenDev, &agentLen, intSize);
  locDevice(agentIdsDev, agent_trans_ids.data(), intSize * agentLen);
  locDevice(agentLocsDev, agent_locs.data(), intSize * agentLen);
  Agent2Cuda<<<calGrid(agentLen), BLOCK>>>(agentIdsDev, agentLocsDev, agentsLocDev, agentLenDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(agentIdsDev));
  CHECK_RUNTIMING_API(cudaFree(agentLocsDev));
  CHECK_RUNTIMING_API(cudaFree(agentLenDev));
}

std::vector<int> *CUDAHeuristics::schedule(std::vector<int> &agentsHost, std::vector<int> &freeTasksID, std::vector<int> &unfinished_agent_cost)
{
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));
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
  calCost<<<grid, BLOCK>>>(distDev, freeHeuristictablePtrDev, tasksLocDev, agentsLocDev, tasks2DisDev, taskIDsDev, agentIDsDev, s_ATNumDev, savedSizeDev);
  CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  thrust::device_ptr<int> distDevPtr(distDev);
  // thrust::device_vector<int> dist(distDevPtr, distDevPtr + agentNum * taskNum);
  thrust::device_vector<int> indices(agentNum * taskNum);
  thrust::sequence(indices.begin(), indices.end());
  thrust::sort_by_key(distDevPtr, distDevPtr + agentNum * taskNum, indices.begin());
  // std::vector<int> minValuesHost(dist.size());
  // thrust::copy(dist.begin(), dist.end(), minValuesHost.begin());
  std::vector<int> *minIndicesHost =  new std::vector<int>(indices.size());
  thrust::copy(indices.begin(), indices.end(), minIndicesHost->begin());
  CHECK_RUNTIMING_API(cudaFree(s_ATNumDev));
  CHECK_RUNTIMING_API(cudaFree(taskIDsDev));
  CHECK_RUNTIMING_API(cudaFree(agentIDsDev));
  CHECK_RUNTIMING_API(cudaFree(distDev));
  return minIndicesHost;
}

void CUDAHeuristics::freeTaskId(int t_id)
{
  freeTaskMapIDsSet.insert(taskPtrsHost[t_id]);
  taskPtrsHost.erase(t_id);
}

int CUDAHeuristics::insertTaskId(int t_id)
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

std::pair<int *, int *> CUDAHeuristics::FGDC_orientation(std::vector<int> &cal_loc)
{
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));
  int *locNumDev, *locDev, *mapsDev, *orientationsDev, *finishedDev;
  int locNum = cal_loc.size();
  locDevice(locNumDev, &locNum, intSize);

  int *locHost = cal_loc.data();
  locDevice(locDev, locHost, intSize * locNum);

  long unsigned int mapsSize = MapSize * locNum;
  long unsigned int mapsHostSize = mapsSize / intSize;
  int *mapsHost = new int[mapsHostSize];
  int *orientationsHost = new int[mapsHostSize];
  CHECK_RUNTIMING_API(cudaMalloc((void **)&mapsDev, mapsSize));
  CHECK_RUNTIMING_API(cudaMalloc((void **)&orientationsDev, mapsSize));

  int *finishedHost = new int[2]{1, 0};
  int *finishedTransfer = new int[2]{1, 0};
  locDevice(finishedDev, finishedHost, intSize * 2);

  dim3 grid = calGrid(xyHost[0] * xyHost[1] * locNum);
  initOrientationMap<<<grid, BLOCK>>>(mapsDev, orientationsDev, MapDev, xyDev, locNumDev, locDev);
  setGoalSignal<<<grid, BLOCK>>>(mapsDev, orientationsDev, MapDev, xyDev, locNumDev, locDev);
  while (true)
  {
    update_orientation<<<grid, BLOCK>>>(mapsDev, orientationsDev, neighborsDev, xyDev, locNumDev, finishedDev);
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
  CHECK_RUNTIMING_API(cudaMemcpy(orientationsHost, orientationsDev, mapsSize, cudaMemcpyDeviceToHost));
  // for (int i = 0; i < locNum; i++)
  // {
  //   printf("Location %d\n", cal_loc[i]);
  //   printMap(mapsHost + (i * xyHost[0] * xyHost[1]), xyHost);
  //   printMap(orientationsHost + (i * xyHost[0] * xyHost[1]), xyHost);
  // }
  // CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(mapsDev));
  CHECK_RUNTIMING_API(cudaFree(locNumDev));
  CHECK_RUNTIMING_API(cudaFree(locDev));
  CHECK_RUNTIMING_API(cudaFree(finishedDev));
  CHECK_RUNTIMING_API(cudaFree(orientationsDev));
  free(finishedHost);
  free(finishedTransfer);
  return {mapsHost, orientationsHost};
}

int *CUDAHeuristics::FGDC(std::vector<int> &cal_loc)
{
  CHECK_RUNTIMING_API(cudaSetDevice(device_id));
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
  int *finishedTransfer = new int[2]{1, 0};
  locDevice(finishedDev, finishedHost, intSize * 2);

  dim3 grid = calGrid(xyHost[0] * xyHost[1] * locNum);
  initGlobalMap<<<grid, BLOCK>>>(mapsDev, MapDev, xyDev, locNumDev, locDev);
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
  // CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  // printMap(mapsHost + xyHost[0] * xyHost[1], xyHost);
  // CHECK_RUNTIMING_API(cudaDeviceSynchronize());
  CHECK_RUNTIMING_API(cudaFree(mapsDev));
  CHECK_RUNTIMING_API(cudaFree(locNumDev));
  CHECK_RUNTIMING_API(cudaFree(locDev));
  CHECK_RUNTIMING_API(cudaFree(finishedDev));
  free(finishedHost);
  free(finishedTransfer);
  return mapsHost;
}

__global__ void copyMap2Cuda(int *transferHMap, int *transferIdx, int *tasksMap, int *savedSize, int *mapLenDev)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  if (idx >= *savedSize)
  {
    return;
  }
  int map_idx = (*transferIdx % *mapLenDev) * (*savedSize) + idx;
  tasksMap[map_idx] = transferHMap[idx];
}

__global__ void Task2Cuda(int *mapIdsDev, int *taskLocsDev, int *taskCostDev, int *tasksLocDev, int *tasks2DisDev, int *taskLenDev)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  if (idx >= *taskLenDev)
  {
    return;
  }
  tasksLocDev[mapIdsDev[idx]] = taskLocsDev[idx];
  tasks2DisDev[mapIdsDev[idx]] = taskCostDev[idx];
}

__global__ void Agent2Cuda(int *agentIdsDev, int *agentLocsDev, int *agentsLocDev, int *agentLenDev)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  if (idx >= *agentLenDev)
  {
    return;
  }
  agentsLocDev[agentIdsDev[idx]] = agentLocsDev[idx];
}

__global__ void updateUnfinished(int *dist, int *agents_loc, int *global_agent_id, int *s_ATNum,
                                 int *savedSizeDev, int *unfinishedAgentCostDev, int *unfinishedNumDev)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int task_num = s_ATNum[1];
  if (idx >= *unfinishedNumDev * task_num)
  {
    return;
  }
  int agent_num = s_ATNum[0];
  int start_idx = agent_num - *unfinishedNumDev;
  int agent_idx = idx / task_num;
  int task_map_idx = idx % task_num;
  int agent_map_idx = start_idx + agent_idx;
  int map_idx = agent_map_idx * task_num + task_map_idx;
  dist[map_idx] += unfinishedAgentCostDev[agent_idx];
}

__global__ void setGoalSignal(int *global_map, int *orientations, int *map, int *xy, int *locNum, int *cal_loc)
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
  int loc_idx = global_idx / nxy;
  int map_idx = global_idx - (loc_idx * nxy);
  if (global_map[global_idx] == 0 && cal_loc[loc_idx] == idx)
  {
    int y = map_idx / xy[0];
    int n0 = map_idx - xy[0];
    if (n0 >= 0 && map[n0] == 0)
    {
      global_map[global_idx - xy[0]] = 1;
      orientations[global_idx - xy[0]] = Orientation::DOWN;
    }
    int n1 = map_idx - 1;
    if (n1 >= 0 && int(n1 / xy[0]) == y && map[n1] == 0)
    {
      global_map[global_idx - 1] = 1;
      orientations[global_idx - 1] = Orientation::RIGHT;
    }
    int n2 = map_idx + xy[0];
    if (n2 < nxy && map[n2] == 0)
    {
      global_map[global_idx + xy[0]] = 1;
      orientations[global_idx + xy[0]] = Orientation::UP;
    }
    int n3 = map_idx + 1;
    if (n3 < nxy && int(n3 / xy[0]) == y && map[n3] == 0)
    {
      global_map[global_idx + 1] = 1;
      orientations[global_idx + 1] = Orientation::LEFT;
    }
  }
}

__global__ void initOrientationMap(int *global_map, int *orientations, int *map, int *xy, int *locNum, int *cal_loc)
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
  orientations[global_idx] = 1;
}

__global__ void update_orientation(int *global_map, int *orientations, int *neighbors, int *xy, int *locNum, int *finished)
{
  int blockId = blockIdx.x + blockIdx.y * gridDim.x;
  int threadId = (threadIdx.y * blockDim.x) + threadIdx.x;
  int global_idx = blockId * (blockDim.x * blockDim.y) + threadId;
  int nxy = xy[0] * xy[1];
  int total_idx = nxy * (*locNum);
  if (global_idx >= total_idx || global_map[global_idx] != 0)
  {
    return;
  }
  int loc_idx = global_idx / nxy;
  int loc0 = loc_idx * nxy;
  int neighbor_s = (global_idx - loc0) * 4;
  for (int i = 0; i < 4; i++)
  {
    if (neighbors[neighbor_s + i] != -1)
    {
      int neighbor_global_idx = neighbors[neighbor_s + i] + loc0;
      if (global_map[neighbor_global_idx] == finished[0])
      {
        if (i == 0 && orientations[neighbor_global_idx] % Orientation::UP == 0)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::UP;
          finished[1] -= 1;
        }
        else if (i == 1 && orientations[neighbor_global_idx] % Orientation::LEFT == 0)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::LEFT;
          finished[1] -= 1;
        }
        else if (i == 2 && orientations[neighbor_global_idx] % Orientation::DOWN == 0)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::DOWN;
          finished[1] -= 1;
        }
        else if (i == 3 && orientations[neighbor_global_idx] % Orientation::RIGHT == 0)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::RIGHT;
          finished[1] -= 1;
        }
      }
      else if (global_map[neighbor_global_idx] == finished[0] - 1 && global_map[neighbor_global_idx] != 0)
      {
        if (i == 0)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::UP;
          finished[1] -= 1;
        }
        else if (i == 1)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::LEFT;
          finished[1] -= 1;
        }
        else if (i == 2)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::DOWN;
          finished[1] -= 1;
        }
        else if (i == 3)
        {
          global_map[global_idx] = finished[0] + 1;
          orientations[global_idx] *= Orientation::RIGHT;
          finished[1] -= 1;
        }
      }
    }
  }
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

__global__ void calCost(int *dist, int **freeHeuristictablePtrDev, int *tasksLocDev, int *agents_loc,
                        int *task2_cost, int *global_tasks_id, int *global_agent_id, int *s_ATNum,
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
  int task_save_idx = global_tasks_id[task_idx];
  dist[idx] = freeHeuristictablePtrDev[tasksLocDev[task_save_idx]][agents_loc[global_agent_id[agent_idx]]] +
              task2_cost[task_save_idx];
  // printf("Agent %d, loc: %d, Task save id %d, loc: %d, Task cost %d, Total cost: %d\n",
  //        global_agent_id[agent_idx], agents_loc[global_agent_id[agent_idx]], task_save_idx, tasksLocDev[task_save_idx], task2_cost[task_save_idx], dist[idx]);
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
