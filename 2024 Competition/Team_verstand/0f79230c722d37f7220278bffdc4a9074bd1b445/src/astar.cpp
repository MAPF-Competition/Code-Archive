//
// Created by take_ on 25-1-9.
//
#include "astar.h"




    std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

    /**
* @brief: 考虑流量的寻路
* @param SharedEnvironment* env: 共享环境信息
     * @param std::vector<Int4>& flow: 地图点们的流量数据
     * @param HeuristicTable& ht: 地图点到其他地图点的启发式距离
     * @param Traj& traj: 保存求出的路径
     * @param MemoryPool& mem: 任务池
     * @param int start: 起点
     * @param int goal: 目标点
     * @param Neighbors* ns: 每个地图点的邻居节点
* @return s_node: 指定节点是否已生成。
     * comment: 在扩展邻居结点时选择g+h+flow最小的邻居
*/
s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
                             HeuristicTable& ht, Traj& traj,
                             const vector<vector<int>>& _extra_weights,
                             MemoryPool& mem, int start, int goal, Neighbors* ns)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h;

    // 如果启发式表为空，则使用曼哈顿距离估算启发值（h），否则从启发式表中获取。
    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);

    // 使用起点信息生成一个根节点（s_node），其深度为 0，代价为 0。
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    // 如果起点等于目标点，直接返回路径。
    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    pqueue_min_of open;
    re_of re;

    // 把根节点加入队列
    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];

    while (open.size() > 0){
        s_node* curr = open.pop();
        curr->close();

        // 查看pop的节点是否达到目标
        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        // 已有的邻居填进去, 没有的邻居以-1的形式挂在末尾
        getNeighborLocs(ns,neighbors,curr->id); // 求出当前节点的邻居

        // 扩展邻居
        for (int i=0; i<4; i++)
        {
            int next = neighbors[i];
            if (next == -1){
                continue;
            }

            cost = curr->g+1;

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1; // depth是搜索深度, 或者说扩展邻居的层数

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(ht.empty())
                h = manhattanDistance(next,goal,env);
            else
                h = get_heuristic(ht,env, next, ns);

            diff = next - curr->id;
            d = get_d(diff,env);
            // cout << "d" << d << endl;
            // cout << "extra weights: " << _extra_weights[curr->id][d] << endl;
            cost += _extra_weights[curr->id][d]; // 不鼓励agent沿highway反向移动

            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d!=d)
                    tie_breaker = 0.1;
                else
                    tie_breaker = 0;
                //tie breaking on prefering moving forward
            }

            // 对向流量相乘
            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);
            // temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]) * 4; // 加大对向惩罚

            //all vertex flow
            //the sum of all outgoing edge flow is the same as the total number of vertex visiting.
            // 某顶点四个方向的流量之和
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];
            }

            op_flow += temp_op; // 把对向流量相乘后相加

            all_vertex_flow+= (temp_vertex-1) /2; // 顶点流量除以2相加

            p_diff = 0; // 确定agent是否在地图点移动
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow); // 设置顶点流量和对向流量
            // temp_node.set_all_flow(0, 0); // 去掉flow, 看看结果如何

            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else{
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
                }
                else{

                    if (re(temp_node,*existing)){
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }

                }
            }
        }
    }

    // 找不到路
    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }

    // 通过回溯, 保存求出的路径
    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}

// 无解的时候不要让整个代码停止运行, 留下改正的余地
bool astar_highway(SharedEnvironment* env, std::vector<Int4>& flow,
             HeuristicTable& ht, Traj& traj,
             const vector<vector<int>>& _extra_weights,
             MemoryPool& mem, int start, int goal, Neighbors* ns)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h;

    // 如果启发式表为空，则使用曼哈顿距离估算启发值（h），否则从启发式表中获取。
    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);

    // 使用起点信息生成一个根节点（s_node），其深度为 0，代价为 0。
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    // 如果起点等于目标点，直接返回路径。
    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return true;
    }

    pqueue_min_of open;
    re_of re;

    // 把根节点加入队列
    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];

    while (open.size() > 0){
        s_node* curr = open.pop();
        curr->close();

        // 查看pop的节点是否达到目标
        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        // 已有的邻居填进去, 没有的邻居以-1的形式挂在末尾
        getNeighborLocs(ns,neighbors,curr->id); // 求出当前节点的邻居

        // 扩展邻居
        for (int i=0; i<4; i++)
        {
            int next = neighbors[i];
            if (next == -1){
                continue;
            }

            cost = curr->g+1;

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1; // depth是搜索深度, 或者说扩展邻居的层数

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(ht.empty())
                h = manhattanDistance(next,goal,env);
            else
                h = get_heuristic(ht,env, next, ns);

            diff = next - curr->id;
            d = get_d(diff,env);
            // cout << "d" << d << endl;
            // cout << "extra weights: " << _extra_weights[curr->id][d] << endl;
            cost += _extra_weights[curr->id][d]; // 不鼓励agent沿highway反向移动

            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d!=d)
                    tie_breaker = 0.1;
                else
                    tie_breaker = 0;
                //tie breaking on prefering moving forward
            }

            // 对向流量相乘
            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);
            // temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]) * 4; // 加大对向惩罚

            //all vertex flow
            //the sum of all outgoing edge flow is the same as the total number of vertex visiting.
            // 某顶点四个方向的流量之和
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];
            }

            op_flow += temp_op; // 把对向流量相乘后相加

            all_vertex_flow+= (temp_vertex-1) /2; // 顶点流量除以2相加

            p_diff = 0; // 确定agent是否在地图点移动
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow); // 设置顶点流量和对向流量
            // temp_node.set_all_flow(0, 0); // 去掉flow, 看看结果如何

            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else{
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
                }
                else{

                    if (re(temp_node,*existing)){
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }

                }
            }
        }
    }

    // 找不到路。说明highway不连通
    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        return false;
    }

    // 通过回溯, 保存求出的路径
    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return true;
}

