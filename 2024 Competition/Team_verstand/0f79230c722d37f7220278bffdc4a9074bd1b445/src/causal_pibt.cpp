//
// Created by take_ on 25-1-10.
//
#include "causal_pibt.h"


/**
* @brief: 提供的不同启发式策略，选择合适的方法来估计从target点到ai路径终点的距离。
* @param TrajLNS& lns: 被计算路径保存的地方
* @param int ai: agent ai的路径。
* @param int target: 被估计的点。
* @return void
*/
int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

    // 如果都不为空, 调用get_dist_2_path
    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
        min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));
    else if (!lns.heuristics[lns.tasks.at(ai)].empty()) // 如果启发式表已经计算
        min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
    else // 如果啥都没计算, 就用曼哈顿距离
        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);

    return min_heuristic;
}

/**
* @brief: 在一个路径规划算法中为某个agent分配下一步的移动位置。
* @param int _curr_id: 被规划的agent id。
* @param int _higher_id: 表示当前机器人是否受某个优先级更高的机器人阻塞。如果为 -1，表示没有受其他机器人约束。
* @param std::vector<State>& _prev_states: 每个agent下达指令前的位置
* @param std::vector<State>& _next_states: 每个agent下达指令后的位置。
* @param std::vector<int>& _prev_decision: 每个地图点下达指令前被哪个agent占据
* @param std::vector<int>& _decision: 每个地图点下达指令后被哪个agent占据
* @param std::vector<bool>& _occupied: 每个地图点是否被占据
* @param TrajLNS& _lns: 储存各种信息
* @return bool: true, 规划成功; false, 规划失败。
*/
bool causalPIBT(int _curr_id, int _higher_id,
                std::vector<State>& _prev_states, std::vector<State>& _next_states,
                std::vector<int>& _prev_decision, std::vector<int>& _decision,
                std::vector<bool>& _occupied, TrajLNS& _lns
){
    // The PIBT works like a causal PIBT when using MAPF-T model. But a normal PIBT when using MAPF model.

    assert(_next_states[_curr_id].location == -1);
    int prev_loc = _prev_states[_curr_id].location; // agent curr id下达指令前的位置
    int prev_orientation = _prev_states[_curr_id].orientation; // agent curr id下达指令前的方向
    // 邻居位置
    int next[4] = { prev_loc + 1, prev_loc + _lns.env->cols, prev_loc - 1, prev_loc - _lns.env->cols};
    int orien_next_v = next[prev_orientation]; // 根据方向计算出可能的邻居位置

    assert(prev_loc >= 0 && prev_loc < _lns.env->map.size());

    // agent curr id被分配的目标点
    int target = _lns.tasks.at(_curr_id);

    // for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
    std::vector<int> neighbors;
    std::vector<PIBT_C> successors; // 每个邻居都是可能的后继, 记录每一个后继的启发式距离
    getNeighborLocs(&(_lns.neighbors), neighbors, prev_loc); // 计算agent curr id下达指令前的位置的邻居节点。
    for (auto& neighbor: neighbors){

        assert(validateMove(prev_loc, neighbor, _lns.env));

        // 计算邻居节点和agent curr id路径终点的启发式距离
        int min_heuristic = get_gp_h(_lns, _curr_id, neighbor);

        successors.emplace_back(neighbor,min_heuristic,-1,rand());
    }

    // 停在当前位置也是可能的后继
    int wait_heuristic = get_gp_h(_lns, _curr_id, prev_loc);
    successors.emplace_back(prev_loc, wait_heuristic,-1,rand());

    // 3. 排序候选节点
    std::sort(successors.begin(), successors.end(),
              [&](PIBT_C& a, PIBT_C& b)
              {
                  int diff[4] = {1, _lns.env->cols, -1, -_lns.env->cols};
                  if (a.heuristic == b.heuristic){
                      // tie-break on prefer moving forward 优先选择前进方向的节点
                      if (a.location==orien_next_v && b.location!=orien_next_v)
                          return true;
                      if (a.location!=orien_next_v && b.location==orien_next_v)
                          return false;
                      // random tie-break
                      return a.tie_breaker < b.tie_breaker;
                  }
                  return a.heuristic < b.heuristic; // 启发式距离较近的后继优先。
              });

    // 4. 从后继中选择可能移动到的地图点
    // 后继中有可能位置和方向都和现在不同, 也就说移动了两个时间步的action
    for (auto& next: successors){
        // 如果候选地图点已经被占用, 跳过
        if(_occupied[next.location])
            continue;
        assert(validateMove(prev_loc, next.location, _lns.env));

        if (next.location == -1)
            continue;
        if (_decision[next.location] != -1){  // 如果后继结点已经被占用, 跳过
            continue;
        }
        // 如果后继结点之前被更高优先级的占用, 跳过
        if (_higher_id != -1 && _prev_decision[next.location] == _higher_id){
            continue;
        }

        // 把agent curr id的后继状态初始化为不发生冲突的邻居节点
        _next_states.at(_curr_id) = State(next.location, -1, -1);
        _decision.at(next.location) = _curr_id;

        // 如果当前候选节点被分配给另一个机器人（但该机器人尚未分配位置），递归调用 causalPIBT, 把它推开.
        if (_prev_decision.at(next.location) != -1 &&
            _next_states.at(_prev_decision.at(next.location)).location == -1){
            int lower_id = _prev_decision.at(next.location);
            if (!causalPIBT(lower_id, _curr_id, _prev_states, _next_states, _prev_decision, _decision, _occupied, _lns)){
                continue;
            }
        }

        return true;
    }

    // 如果所有候选节点都无法分配，默认分配到原位置：
    _next_states.at(_curr_id) = State(prev_loc, -1 , -1);;
    _decision.at(prev_loc) = _curr_id;

#ifndef NDEBUG
    std::cout<<"false: "<< _next_states[_curr_id].location<<","<<_next_states[_curr_id].orientation <<std::endl;
#endif

    return false;
}

/**
* @brief: 根据当前状态 (prev) 和目标位置 (next_loc)，计算出适合的 行动（Action）
* @param State& prev: 当前状态，包括当前的位置 (location) 和当前的朝向 (orientation)。
* @param State& next: 下个状态，包括下个的位置 (location) 和下个的朝向 (orientation)。
* @return Action: 返回适合的行动(Action)
*/
Action getAction(State& prev, State& next){
    if (prev.location == next.location && prev.orientation == next.orientation){
        return Action::W;
    }
    if (prev.location != next.location && prev.orientation == next.orientation){
        return Action::FW;
    }
    if (next.orientation  == (prev.orientation+1)%4){
        return Action::CR;
    }
    if (next.orientation  == (prev.orientation+3)%4){
        return Action::CCR;
    }
    assert(false);
    return Action::W;
}

/**
* @brief: 根据当前状态 (prev) 和目标位置 (next_loc)，计算出适合的 行动（Action）
* @param State& prev: 当前状态，包括当前的位置 (location) 和当前的朝向 (orientation)。
* @param int next_loc: 智能体希望移动到的目标位置。
* @param SharedEnvironment* env: 共享的环境对象，提供一些地图信息，比如列数 (cols)。
* @return Action: 返回适合的行动(Action)
*/
Action getAction(State& prev, int next_loc, SharedEnvironment* env){
    // 如果智能体的当前位置（prev.location）已经等于目标位置（next_loc），它就无需移动，返回 Action::W（等待）
    if (prev.location == next_loc){
        return Action::W;
    }

    // 计算位置差分
    int diff = next_loc - prev.location; // 目标位置与当前当前位置的差值，用来判断目标位置与当前位置的相对位置。
    int orientation;
    if (diff == 1){
        orientation = 0; // 向右
    }
    if (diff == -1){
        orientation = 2; // 向左
    }
    if (diff == env->cols){
        orientation = 1; // 向下
    }
    if (diff == -env->cols){
        orientation = 3; // 向上
    }

    // 没有后退, 没有左移右移, 只有前进和转向; 转向完成后才能前进
    if (orientation == prev.orientation){
        return Action::FW; // 目标方向与当前朝向相同，直接前进
    }
    if (orientation  == (prev.orientation+1)%4){
        return Action::CR; // 目标方向在当前朝向顺时针方向，转向
    }
    if (orientation  == (prev.orientation+3)%4){
        return Action::CCR; // 目标方向在当前朝向逆时针方向，转向
    }
    if (orientation  == (prev.orientation+2)%4){
        return Action::CR; // 目标方向与当前朝向相反，转身
    }
    assert(false);
}

/**
* @brief: 检查agent是否移动。
* @param int id: 当前智能体的 ID。
* @param std::vector<bool>& checked: 记录每个智能体是否已经被检查过，避免重复检查（递归中的循环调用）。
* @param std::vector<DCR>& decided: 存储每个智能体的决策结果，其中包含目标位置（loc）。
* @param std::vector<Action>& actions: 当前智能体的行动列表，例如：Action::FW（前进）、Action::W（等待）。
* @param std::vector<int>& prev_decision: 每个目标位置的决策记录，存储目标位置被哪个智能体占据。如果目标位置没有被占据，值为 -1。
* @return bool: 返回一个行动（例如移动）是否可以被执行
*/
bool moveCheck(int id, std::vector<bool>& checked,
               std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision){
    // 记录每个智能体是否已经被检查过，避免重复检查（递归中的循环调用）。
    if (checked.at(id) && actions.at(id) == Action::FW)
        return true;
    checked.at(id) = true; // 将当前智能体标记为已检查。

    // 如果当前智能体的动作不是 "前进"（Action::FW），则返回 false，表示不移动。
    if (actions.at(id) != Action::FW)
        return false;

    // move forward
    int target = decided.at(id).loc;
    assert(target != -1);

    // 根据之前的决策, 当前地图点不会被占据, 因此可以移动
    int na = prev_decision[target];
    if (na == -1)
        return true;

    // 如果目标位置已经被其他智能体 na 占据，则递归调用 moveCheck 函数，检查智能体 na 是否能够完成它的动作。
    // 如果智能体 na 的行动可以成功，则当前智能体也可以完成行动，返回 true。
    // 这个就是push的思想了
    if (moveCheck(na,checked,decided,actions,prev_decision))
        return true;
    actions.at(id) = Action::W; // 如果递归返回 false，说明冲突无法解决，当前智能体的动作被修改为 Action::W（等待）。
    return false;
}
