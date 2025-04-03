#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include "heuristics.h"
#include <iostream>

#include <set>

// 将所有代码封装在 DefaultPlanner 命名空间中，避免与其他模块的名称冲突。
namespace DefaultPlanner{

    // 定义一种自适应策略枚举，用于选择轨迹优化时的策略。
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};
// RANDOM：随机选择策略。
// CONGESTION：基于拥塞情况的策略。
// DEVIATION：基于偏离轨迹的策略。
// COUNT：标志枚举值的总数。

extern std::vector<HeuristicTable> global_heuristictable; // 全局启发式查找表，存储地图点之间的距离。
extern Neighbors global_neighbors; // 全局邻接表，存储每个地图点的邻居信息，用于路径规划。

// 存储偏离规划路径的长度, 和上次规划的时间。
// 与任务拥塞的启发式无关。偏离程度和上次规划时间和拥塞能有啥关系？
struct FW_Metric{
    int id; // 代理或任务的标识符。
    int deviation; // 记录轨迹偏离的程度。
    int last_replan_t; // 上次重新规划的时间步。
    int rand;

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

// 定义一个启发式评估器，用于路径规划中的流量计算。
// 可与任务拥塞的启发式没啥关系。你看都没用到。
struct FlowHeuristic{
    HeuristicTable* h; // 指向启发式表的指针。
    int target; // 目标位置
    int origin; // 起点位置
    pqueue_min_of open; // 优先队列，用于存储待处理的节点。
    MemoryPool mem; // 内存池，对应每一个地图点。


    bool empty(){
        return mem.generated() == 0;
    }

    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

// TrajLNS 是这段代码的核心部分，代表了一种用于轨迹规划的局部搜索机制。
class TrajLNS{
    public:
    SharedEnvironment* env; // 共享环境指针
    std::vector<int> tasks; // 每个agent当前的目标位置

    TimePoint start_time; // 轨迹优化的起始时间
    int t_ms=0; // 当前时间步

    std::vector<Traj> trajs; // 每个agent的轨迹

    std::vector<std::pair<int,int>> deviation_agents; // 偏离轨迹的代理

    std::vector<Int4> flow; // 流量信息
    std::vector<HeuristicTable>& heuristics; // 地图点两两之间的距离
    std::vector<Dist2Path> traj_dists; // agent当前位置到规划路径的距离
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics; // agent偏离规划路径的距离, 和上次规划的时间
    Neighbors& neighbors; // 地图点的邻居


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0; // 轨迹总成本

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size()); // 创造地图节点数量个search node
    }

    // constructor
    TrajLNS(SharedEnvironment* env, std::vector<HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(heuristics),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors){
        };

    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};
};
}
#endif