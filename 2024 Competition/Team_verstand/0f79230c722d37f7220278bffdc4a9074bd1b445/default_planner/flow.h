
#ifndef flow_hpp
#define flow_hpp
#include "Types.h"
#include "search.h"
#include "TrajLNS.h"
#include "heuristics.h"

#include <random>
#include <unordered_set>

namespace DefaultPlanner{

/**
* @brief: 从给定的轨迹中移除agent对 flow 的影响: 减少路径上的每一步流量，从而更新流量表。
 * @param TrajLNS& lns: 保存移除agent后的SOC和flow。
 * @param int agent: 需要移除其轨迹影响的具体智能体编号。
 * @return void
 * comment: remove flow for each location's outgoing edge according to the traj
 */
void remove_traj(TrajLNS& lns, int agent);

/**
* @brief: 将某个智能体（agent）的轨迹（traj）信息添加到 TrajLNS 对象中，从而更新与路径相关的代价和流量数据。
 * @param TrajLNS& lns: 保存加入agent后的SOC和flow。
 * @param int agent: 需要加入其轨迹影响的具体智能体编号。
 * @return void
 */
void add_traj(TrajLNS& lns, int agent);

/**
* @brief: 检测哪些智能体的当前位置已经偏离了其轨迹（轨迹偏差，deviation），并根据偏离程度对这些智能体进行排序。
 * @param TrajLNS& lns: 保存加入agent后的SOC和flow。
 * @return void
 * comment: 感觉可以用在highway
 */
void get_deviation(TrajLNS& lns);

/**
* @brief: 更新agent从当前位置到预设轨迹的距离
 * @param TrajLNS& lns: 保存更新后的fw_metrics。
 * @return void
 */
void update_fw_metrics(TrajLNS& lns);

/**
* @brief: 利用一种 Frank-Wolfe 样式的迭代方法，为一组智能体（agents）重新规划轨迹，同时考虑路径上的偏离程度、上次重规划时间等因素。
 * 在给定的时间限制内（timelimit），它会逐步更新需要重规划的智能体的路径。
 * @param TrajLNS& lns: 存储智能体的路径（trajs）、流量信息（flow）、启发式（heuristics）、当前的轨迹偏差（fw_metrics）等。
 * @param std::unordered_set<int>& updated: 保存被更新轨迹的智能体 ID 集合。
 * @param TimePoint timelimit: 时间限制，函数运行的时间不能超过这个限制。
 * @return void
 */
void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit);

/**
* @brief: 更新第 i 个智能体的路径到末尾距离数据。
 * @param TrajLNS& lns: 保存更新后的第 i 个智能体的路径到末尾距离数据。
 * @return void
 */
void update_dist_2_path(TrajLNS& lns, int i);

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount);

/**
* @brief: update traj and distance table for agent i
 * @param TrajLNS& lns: 保存更新后的第 i 个智能体的路径到末尾距离数据。
 * @param int i: 指定的agent index。
 * @return void
 */
void update_traj(TrajLNS& lns, int i);


}
#endif