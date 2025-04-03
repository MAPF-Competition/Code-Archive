//
// Created by take_ on 25-1-10.
//

#ifndef DISTANCE_H
#define  DISTANCE_H

// #include "Types.h"
#include "basic_type.h"
// #include "utils.h"
#include "usage.h"
#include <queue>
// #include "TrajLNS.h"
#include "path.h"
// #include "search_node.h"
#include "astar_node.h"

/**
* @brief: 初始化每个地图点的邻居
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @return void
*/
void init_heuristics(SharedEnvironment* env);

/**
* @brief: 根据地图（env->map）的结构初始化全局邻居信息表（global_neighbors），为每个节点记录其可到达的邻居节点。
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @return void
* comment: 这里的可到达没有考虑动态障碍物(other agent)
*/
void init_neighbor(SharedEnvironment* env);

/**
* @brief: 初始化地图中每个点和目标点的启发式距离
* @param HeuristicTable& ht: 地图所有点到所有点的启发式距离
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param int goal_location: 目标点。
* @return void
*/
void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

/**
* @brief: 通过启发式搜索计算从给定源位置 (source) 到目标位置的启发式代价（heuristic）
* @param HeuristicTable& ht: 存储地图两点间的启发式距离。实际代入的参数是global_heuristictable, 因此一对位置只计算一次。
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param int source: 源位置（source），即从这个位置开始计算启发式值。
* @param Neighbors* ns: 提供某个位置的邻居信息（即可以直接到达的相邻位置）。实际上是只考虑静态障碍物的global_neighbors.
* @return curr.value: 给定源位置 (source) 到目标位置的启发式代价（heuristic）
*/
int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

/**
* @brief: hash中有就读取, 没有就计算
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param int source: 源位置（source），即从这个位置开始计算启发式值。
* @param int target: 目标位置位置（target），即从这个位置开始计算启发式值。
* @return int distance: 给定源位置 (source) 到目标位置的启发式代价（heuristic）
*/
int get_h(SharedEnvironment* env, int source, int target);

/**
* @brief: 初始化地图中每个点到路径末尾的距离, 计算对象路径中每个点到路径末尾的距离。
* @param Dist2Path& dp: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param Traj& path: 对象路径。
* @return void
*/
void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

/**
* @brief: 从某个起始点（source）找到该点到预定义路径终点的距离信息
* @param Dist2Path& dp: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param int source: 源位置（source），即从这个位置开始计算启发式值。
* @param Neighbors* ns: 提供获取当前点的相邻点（邻居）的功能。
* @return std::pair<int, int>: 距离代价（cost）, 剩余步数（togo）。
*/
std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

/**
* @brief: 计算地图上某个点 source 到目标路径终点的 总距离 = 到路径最近点的代价 (cost) + 从路径最近点到路径终点的剩余步数 (togo)。
* @param Dist2Path& dp: 路径距离信息的结构体，包含了所有地图点到路径的距离数据。
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @param int source: 源位置（source），即从这个位置开始计算启发式值。
* @param Neighbors* ns: 提供获取当前点的相邻点（邻居）的功能。
* @return int: 点 source 到目标路径终点的 总距离
* comment: 有点像highway模型了, 不知道能不能用起来
*/
int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);



#endif // DISTANCE_H
