//
// Created by take_ on 25-1-10.
//

#ifndef BASIC_TYPE_H
#define BASIC_TYPE_H

#include <limits.h>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>


#define MAX_TIMESTEP INT_MAX/2

#include "SharedEnv.h"
#include "ActionModel.h"



// 定义任务完成状态的枚举类型。
enum DoneState{
    NOT_DONE,
    DONE
};

struct Int4{
    int d[4];
};

struct Int2{
    int d[4];
};

// 描述一个节点的位置和状态。
struct DCR{
    int loc;
    int state;
};

// 一个整数序列，用于表示路径中的节点顺序。
typedef std::vector<int> Traj;

// 存储路径规划中的节点信息，主要用于启发式算法。
struct PIBT_C{
    int location;
    int heuristic;
    int orientation;  // 0:east, 1:south, 2:west, 3:north
    int tie_breaker;

    //constructor
    PIBT_C(int location, int heuristic, int orientation, int tie_breaker):
            location(location), heuristic(heuristic), orientation(orientation), tie_breaker(tie_breaker) {};
};

// 表示路径规划中的一个节点（通常用于优先队列）。
struct HNode
{
    int label;
    int location;
    int direction;
    int value; // 与规划相关的值（如代价）。
    int other;

    unsigned int priority;
    unsigned int get_priority() const { return priority; }
    void set_priority(unsigned int p) { priority = p; }


    HNode() = default;
    HNode(int location,int direction, int value) : location(location), direction(direction), value(value) {}
    // the following is used to compare nodes in the OPEN list
    struct compare_node
    {
        bool operator()(const HNode& n1, const HNode& n2) const
        {
            return n1.value < n2.value;
        }
    };  // used by OPEN (open) to compare nodes (top of the open has min f-val, and then highest g-val)
};

// 表示启发式值的查找表。
struct HeuristicTable{
    std::vector<int> htable;
    std::deque<HNode> open; // 优先队列，用于存储待处理节点。不过居然不是最小堆?

    bool empty(){
        return htable.empty();
    }
};

// element of Dist2Path
// 代表了单个路径点与目标路径之间的距离或状态信息
struct d2p{
    int label = 0;
    int id = -1;
    int cost = -1;
    int togo = -1; // 当前点到目标路径的剩余步数（即反向步数，从终点到当前点的距离）。

    d2p(int label, int id, int cost, int togo):label(label), id(id), cost(cost), togo(togo){};
};

// 整个地图上所有点到某条目标路径的信息
struct Dist2Path{
    int label = 0;
    std::vector<d2p> dist2path; // 存储地图中每个点的 d2p 信息，表示该点到目标路径的距离状态。
    std::deque<d2p> open;

    bool empty(){
        return dist2path.empty();
    }
};

typedef std::vector<std::vector<int>> Neighbors;

#endif // BASIC_TYPE_H
