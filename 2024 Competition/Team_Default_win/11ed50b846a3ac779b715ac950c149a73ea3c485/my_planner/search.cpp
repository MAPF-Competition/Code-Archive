


#include "search.h"


namespace MyPlanner{
std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

int calculateDirection(int curr_id, int next_id, const SharedEnvironment* env) {
    if (next_id - curr_id == 1) {
        return 0; // 向右
    } else if (next_id - curr_id == -1) {
        return 2; // 向左
    } else if (next_id - curr_id == env->cols) {
        return 1; // 向下
    } else if (next_id - curr_id == -env->cols) {
        return 3; // 向上
    } else {
        // 错误处理
        return -1; // 返回 -1 表示无效方向
    }
}

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h;

    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);
    

    
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    pqueue_min_of open;
    re_of re;

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
        if (curr->is_closed()){
            continue;
        }   
        curr->close();
        
        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        getNeighborLocs(ns,neighbors,curr->id);
        
        for (int i=0; i<4; i++){
            int next = neighbors[i];
            if (next == -1){
                continue;
            }
            
            // 获取当前节点和父节点的位置
            int curr_id = curr->id;
            int parent_id = curr->parent ? curr->parent->id : curr_id; // 如果没有父节点，使用当前节点位置

            // 计算当前节点的方向
            int curr_direction = calculateDirection(parent_id, curr_id, env);

            // 计算next节点的方向
            int next_direction = calculateDirection(curr_id, next, env);

            // 计算邻居节点的方向差
            int dir_diff = (next_direction - curr_direction + 4) % 4;

            // 根据方向差计算成本
            int cost;
            switch (dir_diff) {
                case 0: // 正前方
                    cost = curr->g + 1;
                    break;
                case 1: // 顺时针旋转或逆时针旋转
                case 3:
                    cost = curr->g + 2;
                    break;
                case 2: // 后方
                    cost = curr->g + 3;
                    break;
                default:
                    // 错误处理
                    break;
            }
            if (mem.has_node(next)){
                if (mem.get_node(next)->is_closed()){
                    continue;
                }
            }
            //cost = curr->g+1;
            //cout <<"-----------测试测试-------------"<<endl;
            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(ht.empty())
                h = manhattanDistance(next,goal,env);
            else
                h = get_heuristic(ht,env, next, ns);
            //h = get_heuristic(ht,env, next, ns);
            diff = next - curr->id;
            d = get_d(diff,env);
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d!=d)
                    tie_breaker = 0.1;
                else
                    tie_breaker = 0;
                //tie breaking on prefering moving forward
            }


            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];                
            }

            op_flow += temp_op;
        
            all_vertex_flow+= (temp_vertex-1) /2;

            p_diff = 0;
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);

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


    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }

    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}
}

