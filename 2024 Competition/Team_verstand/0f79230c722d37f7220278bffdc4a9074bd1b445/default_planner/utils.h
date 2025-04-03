
#ifndef utils_hpp
#define utils_hpp

#include "Types.h"

namespace DefaultPlanner{
    /**
* @brief: 根据两个地图点之间的 索引差值 (diff)，计算并返回两点之间的 方向编号
* @param int diff: 两个地图点之间的 索引差值 (diff)
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return int: 0: 右。1: 下。2: 左。3: 上
 */
int get_d(int diff, const SharedEnvironment* env)  ;

/**
 * @brief: 验证从位置 loc 到位置 loc2 的移动是否有效。
 * @param int loc: 移动起点
 * @param int loc2: 移动终点
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return bool: true valid; false invalid.
 */
bool validateMove(int loc, int loc2, const SharedEnvironment* env);

/**
 * @brief: 计算从位置 loc 到位置 loc2 的曼哈顿距离。
 * @param int loc: 起点
 * @param int loc2: 终点
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return int: 从位置 loc 到位置 loc2 的曼哈顿距离。
 */
int manhattanDistance(int loc, int loc2,const SharedEnvironment* env);

void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

/**
 * @brief: 计算指定位置的邻居节点。
 * @param const Neighbors* ns: global_neighbors
 * @param std::vector<int>& neighbors: 邻居节点保存处
 * @param int location: 对象位置。
 * @return void。
 */
void getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) ;

/**
 * @brief: 计算指定位置的邻居节点。
 * @param const Neighbors* ns: global_neighbors
 * @param int neighbors[]: 邻居节点保存处
 * @param int location: 对象位置。
 * @return void。
 */
void getNeighborLocs(const Neighbors* ns, int neighbors[], int location) ;
}

#endif