#include "Astar.h"
#include <map>


namespace DefaultPlanner{
//std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

std::vector<int> backtrack(s_node *H)
{
  std::vector<int> plan;
  auto _H = H;
  while (_H != nullptr) {
    plan.push_back(_H->id);
    _H = _H->parent;
  }
  std::reverse(plan.begin(), plan.end());
  return plan;
}

bool isRepeated(const std::vector<std::vector<int>>& bigVec,
                    const std::vector<int>& smallVec)
    {
        int count = 0;
        for (const auto& row : bigVec) {
            if (row == smallVec) {
                ++count;
                if (count > 0) {
                    return true; 
                }
            }
        }
        return false;
    }

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
    }
};

// std::vector<int> my_Astar_constraint(int agent_id ,SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
//                                     std::vector<std::vector<int>>& V_Containts,
//                                     std::vector<std::vector<int>>& E_Containts){

//     int h, cost, timesteps;
//     std::vector<int> path = {start};

//     if(ht.empty())
//         h = manhattanDistance(start,goal,env);
//     else
//         h = get_heuristic(ht,env, start, ns);
    
//     std::priority_queue<my_node> open;
//     std::vector<std::vector<int>> closed;

//     for(int i=0;i<V_Containts.size();i++){
//         if(agent_id == V_Containts[i][0])
//             closed.push_back({V_Containts[i][1],V_Containts[i][2]}); // location, timesteps;
//     }

//     // for(int j=0;j<closed.size();j++){
//     //     std::cout << "location: "<< closed[j][0] <<" timesteps: " << closed[j][1] << endl;
//     // }

//     int neighbors[5];

//     my_node root(start,0,h,path);

//     open.push(root);
//     while (open.size() > 0){
//         my_node curr = open.top();
//         open.pop();
//         if(!isRepeated(closed,{curr.location, curr.timesteps})){
//             closed.push_back({curr.location,curr.timesteps});
//             if(curr.location == goal){
//                 return curr.paths;
//             }
//             getNeighborLocs(ns,neighbors,curr.location);
//             neighbors[4] = curr.location;
//             for (int i=0; i<5; i++){
//                 int next = neighbors[i];
//                 if (next == -1)
//                     continue;
//                 path = curr.paths;

//                 path.push_back(next);
//                 bool edge_test = false;
//                 for(int i=0;i<E_Containts.size();i++){
//                     int pre_loc = E_Containts[i][1];
//                     int loc = E_Containts[i][2];
//                     int cons_timestep = E_Containts[i][3];
//                     //std::cout << "path size:" << curr.paths.size() << " " << cons_timestep << std::endl;
//                     if (curr.paths.size() > cons_timestep && curr.paths[cons_timestep] == loc && curr.paths[cons_timestep-1] == pre_loc && agent_id == E_Containts[i][0]){
//                         edge_test = true;
//                         break;
//                     }
//                 }
//                 if(edge_test == true)
//                     continue;
//                 timesteps = curr.timesteps + 1;
//                 if(ht.empty())
//                     h = manhattanDistance(next,goal,env);
//                 else
//                     h = get_heuristic(ht,env, next, ns);
//                 my_node new_node(next,timesteps,h,path);
//                 //my_node new_node(next,timesteps,h,path,0,0,0);
//                 open.push(new_node);
//             }
//         }
//         else{
//             continue;
//         }
//     }
//     return {start};
// }

// std::vector<int> my_Astar(SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht){

//     int h, cost, timesteps;
//     std::vector<int> path = {start};

//     if(ht.empty())
//         h = manhattanDistance(start,goal,env);
//     else
//         h = get_heuristic(ht,env, start, ns);
    
//     std::priority_queue<my_node> open;
//     std::vector<std::vector<int>> closed;
//     std::map<int, int> best_g;

//     int neighbors[4];
//     int next_neighbors[4];

//     my_node root(start,0,h,path);

//     open.push(root);
//     while (open.size() > 0){
//         my_node curr = open.top();

//         // std::cout << "curr path:" << std::endl;
//         // for(int i=0;i< curr.paths.size();i++){
//         //     std::cout << curr.paths[i] << " ";
//         // }
//         // std::cout << "   " << std::endl;
//         open.pop();
//         auto it = best_g.find(curr.location);
//         if(it == best_g.end() || best_g[curr.location] > curr.timesteps){
//             best_g[curr.location] = curr.timesteps;
//             if(curr.location == goal){
//                 return curr.paths;
//             }
//             getNeighborLocs(ns,neighbors,curr.location);
//             for (int i=0; i<4; i++){
//                 int next = neighbors[i];
//                 if (next == -1)
//                     continue;
//                 path = curr.paths;
//                 path.push_back(next);
//                 timesteps = curr.timesteps + 1;
//                 if(ht.empty())
//                     h = manhattanDistance(next,goal,env);
//                 else
//                     h = get_heuristic(ht,env, next, ns);

//                 my_node new_node(next,timesteps,h,path);
//                 open.push(new_node);
//             }
//         }
//         else{
//             continue;
//         }
//     }
//     return {start};
// }

std::vector<int> my_Astar_constraint(int agent_id ,SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
                                    std::vector<Int4> flow,
                                    std::vector<std::vector<int>>& V_Containts,
                                    std::vector<std::vector<int>>& E_Containts
                                    ){

    int h, cost, timesteps, diff,d;
    int temp_vertex, p_diff, p_d;
    double op_flow, all_vertex_flow, tie_breaker, temp_op;
    std::vector<int> path = {start};

    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);
    
    std::priority_queue<my_node> open;
    std::vector<std::vector<int>> closed;

    for(int i=0;i<V_Containts.size();i++){
        if(agent_id == V_Containts[i][0])
            closed.push_back({V_Containts[i][1],V_Containts[i][2]}); // location, timesteps;
    }

    // for(int j=0;j<closed.size();j++){
    //     std::cout << "location: "<< closed[j][0] <<" timesteps: " << closed[j][1] << endl;
    // }

    int neighbors[5];

    my_node root(start,0,h,path,0.0,0.0,0.0);

    open.push(root);
    int count = 0;
    while (open.size() > 0){
        count+=1;
        // std::cout << "count: "<< count << endl;
        my_node curr = open.top();
        open.pop();
        if(!isRepeated(closed,{curr.location, curr.timesteps})){
            closed.push_back({curr.location,curr.timesteps});
            if(curr.location == goal){
                // std::cout << "count: "<< count << endl;
                return curr.paths;
            } 
            // else if (count > 4000 && curr.paths.size() > 1){
            //     // std::cout << "count: "<< count << endl;
            //     return curr.paths;
            // }
            

            getNeighborLocs(ns,neighbors,curr.location);
            neighbors[4] = curr.location;
            for (int i=0; i<5; i++){
                int next = neighbors[i];
                if (next == -1)
                    continue;
                path = curr.paths;

                // tie breaker
                op_flow = 0.0;
                all_vertex_flow = 0.0;
                diff = next - curr.location;
                tie_breaker = 0.0;

                d = get_d(diff,env);
                if(path.size() > 2){
                    p_diff = path[path.size()-1] - path[path.size()-2];
                    p_d = get_d(p_diff,env);
                    if (p_d!=d)
                        tie_breaker = 0.1;
                }


                temp_op =  (flow[curr.location].d[d]+1) * flow[next].d[(d+2)%4];
                op_flow += 0.01*temp_op;
                op_flow += curr.op_flow; //op_flow is contra flow

                // temp_vertex = 1;
                // for (int j=0; j<4; j++){
                //     temp_vertex += flow[next].d[j];                
                // }

                // all_vertex_flow += 0.01*(temp_vertex-1) /2;
                // all_vertex_flow += curr.all_vertex_flow;

                tie_breaker += curr.tie_breaker;

                // tie breaker end



                path.push_back(next);
                bool edge_test = false;
                for(int i=0;i<E_Containts.size();i++){
                    int pre_loc = E_Containts[i][1];
                    int loc = E_Containts[i][2];
                    int cons_timestep = E_Containts[i][3];
                    if (curr.paths[cons_timestep] == loc && curr.paths[cons_timestep-1] == pre_loc && agent_id == E_Containts[i][0]){
                        edge_test = true;
                        break;
                    }
                }
                if(edge_test == true)
                    continue;
                timesteps = curr.timesteps + 1;
                if(ht.empty())
                    h = manhattanDistance(next,goal,env);
                else
                    h = get_heuristic(ht,env, next, ns);
                
                // std::cout << "cost: "<< timesteps << endl;
                // std::cout << "h: "<< h << endl;
                // std::cout << "all_vertex_flow: "<< all_vertex_flow  << endl;
                // std::cout << "op_flow: "<< op_flow << endl;

                my_node new_node(next,timesteps,h,path,all_vertex_flow,op_flow,tie_breaker);
                //my_node new_node(next,timesteps,h,path,0,0,0);
                open.push(new_node);
            }
        }
        else{
            continue;
        }
    }
    // std::cout << "count: "<< count << endl;
    return path;
}

std::vector<int> my_Astar(SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
                            std::vector<Int4> flow){
    int h, cost, timesteps, diff,d;
    int temp_vertex, p_diff, p_d;
    double op_flow, all_vertex_flow, tie_breaker, temp_op;
    std::vector<int> path = {start};

    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);
    
    std::priority_queue<my_node> open;
    std::vector<std::vector<int>> closed;
    std::map<int, int> best_g;

    int neighbors[4];
    int next_neighbors[4];

    my_node root(start,0,h,path,0.0,0.0,0.0);

    open.push(root);
    int count = 0;
    while (open.size() > 0){
        count+=1;
        // std::cout << "count: "<< count << endl;
        my_node curr = open.top();
        open.pop();
        auto it = best_g.find(curr.location);
        if(it == best_g.end() || best_g[curr.location] > curr.timesteps){
            best_g[curr.location] = curr.timesteps;
            if(curr.location == goal){
                // open = std::priority_queue<my_node>();
                // std::cout << "count 2: "<< count << endl;
                return curr.paths;
            } 
            // else if (count > 3000 && curr.paths.size() > 1){
            //     // std::cout << "count 2: "<< count << endl;
            //     return curr.paths;
            // }
            getNeighborLocs(ns,neighbors,curr.location);
            for (int i=0; i<4; i++){
                int next = neighbors[i];
                if (next == -1)
                    continue;

                // tie_breaker
                op_flow = 0.0;
                all_vertex_flow = 0.0;
                diff = next - curr.location;
                d = get_d(diff,env);
                path = curr.paths;

                if(path.size() > 1){
                    p_diff = path[path.size()-1] - path[path.size()-2];
                    p_d = get_d(p_diff,env);
                    if (p_d!=d)
                        tie_breaker = 0.1;
                    else
                        tie_breaker = 0.0;
                }

                temp_op = ( (flow[curr.location].d[d]+1) * flow[next].d[(d+2)%4]);
                op_flow += temp_op;
                op_flow += curr.op_flow; //op_flow is contra flow

                // temp_vertex = 1;
                // for (int j=0; j<4; j++){
                //     temp_vertex += flow[next].d[j];                
                // }
                // all_vertex_flow+= (temp_vertex-1) /2;

                // // p_diff = 0;
                // // if(path.size() > 2){
                // //     p_diff = path[path.size()-1] - path[path.size()-2];
                // // }

                // all_vertex_flow += curr.all_vertex_flow;
                
                tie_breaker += curr.tie_breaker;

                // tie breaker end

                path.push_back(next);
                timesteps = curr.timesteps + 1;
                if(ht.empty())
                    h = manhattanDistance(next,goal,env);
                else
                    h = get_heuristic(ht,env, next, ns);
                // if(all_vertex_flow != 0.0 || op_flow != 0.0) {
                //     std::cout << "cost: "<< timesteps << endl;
                //     std::cout << "h: "<< h << endl;
                //     std::cout << "all_vertex_flow: "<< all_vertex_flow  << endl;
                //     std::cout << "op_flow: "<< op_flow << endl;

                // }
                // all_vertex_flow *= 0.1;
                // op_flow *= 0.1;
                my_node new_node(next,timesteps,h,path,all_vertex_flow,op_flow,tie_breaker);
                //my_node new_node(next,timesteps,h,path,0,0,0);
                open.push(new_node);
            }
        }
        else{
            continue;
        }
    }
    return path;
}



int h3(MCTSNode& node, Neighbors* ns) {
    // Vertex conflict check
    int num_of_cof = 0;
    std::vector<std::vector<int>> Constraint;
    int max_timesteps = 0,timestep = 0,cost=0;
    
    for(int i=0;i<node.paths.size();i++){
        timestep = 0;
        for(int j=0;j<node.paths[i].size();j++){
            timestep ++;
            cost ++;
            if (max_timesteps < timestep){
                max_timesteps = timestep;
            }
        }
    }

    std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> pair;
    int r = 0;
    for(int t = 0; t < max_timesteps; t++){
        std::unordered_map<int, int> poss; 
        //std::cout << "new agent" << std::endl;
        for(int i=0;i<node.paths.size();i++){
            int pos;
            if (node.paths[i].size() > t) {
                pos =node.paths[i][t]; 
            } else {
                continue;
            }
            if (poss.find(pos) != poss.end()) {
                if(pair.find({i,poss[pos]}) == pair.end() && pair.find({poss[pos],i}) == pair.end()){
                    num_of_cof ++;
                    
                    Constraint.push_back({node.priority[i], i, poss[pos], pos, t});
                    if (r == 0) {
                        node.far_timestpes = t;
                        r = 1;
                    }
                    pair[{i, poss[pos]}] = 1;
                    pair[{poss[pos],i}] = 1;
                }
            }
            poss[pos] = i; 
        }
    }


    // // Edge conflict check

    for(int t = 1; t < max_timesteps; t++){
        std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> edges;
        for(int i=0; i <node.paths.size(); i++){
            //std::cout << " agent id: "<< i << std::endl;
            if (node.paths[i].size() > t) {
                int pre_pos = node.paths[i][t - 1]; 
                int pos = node.paths[i][t];  

                int record = 0;
                int record_2 = 0;
                std::vector<int> neighbors_pre;
                getNeighborLocs(ns,neighbors_pre,pre_pos);
                if (neighbors_pre.size() == 1) {
                    record = 1;
                }



                std::vector<int> neighbors_cur;
                getNeighborLocs(ns,neighbors_cur,pos);
                if (neighbors_cur.size() == 1) {
                    // after_record = 2;
                    record_2 = 2;
                }   

                if (edges.find({pos, pre_pos}) != edges.end()) {
                    if(pair.find({i,edges[{pos, pre_pos}]}) == pair.end() && pair.find({edges[{pos, pre_pos}],i}) == pair.end()){
                        num_of_cof ++;
                        if (node.priority[i] <= node.priority[edges[{pos, pre_pos}]]) {
                            Constraint.push_back({node.priority[i], i, edges[{pos, pre_pos}], pre_pos, pos, record, record_2, t});
                        } else {
                            Constraint.push_back({node.priority[edges[{pos, pre_pos}]], i, edges[{pos, pre_pos}], pre_pos, pos, record, record_2, t});
                        }

                        if (t < node.far_timestpes) {
                            node.far_timestpes = t;
                        }
                        pair[{i, edges[{pos, pre_pos}]}] = 1;
                        pair[{edges[{pos, pre_pos}],i}] = 1;
                    }
                }
                edges[{pre_pos, pos}] = i; 
            }
        }
    }

    node.constraint = Constraint;
    node.cost = cost;
    node.num_cof = num_of_cof;


    return num_of_cof;
}

std::vector<int> validate(a_node& trajs) {
    // Vertex conflict check
    int num_of_cof = 0;
    trajs.num_cof = 0; 
    std::vector<int> Constrant;
    std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> pair;
    for(int t = 0; t < trajs.max_steps; t++){
        std::unordered_map<int, int> poss; 
        //std::cout << "new agent" << std::endl;

        for(int i=0;i<trajs.Paths.size();i++){
            int pos;
            if (trajs.Paths[i].size() > t) {
                pos = trajs.Paths[i][t]; 
            } else {
                continue;
            }
            //std::cout << pos << " ";
            if (poss.find(pos) != poss.end()) {
                num_of_cof ++;
                //std::cout << " find a vertex conflict"<< std::endl;
                if(pair.find({i,poss[pos]}) == pair.end() && pair.find({poss[pos],i}) == pair.end()){
                    trajs.num_cof++;
                    pair[{i, poss[pos]}] = 1;
                    pair[{poss[pos],i}] = 1;
                }
                if (Constrant.empty()){
                    Constrant = {i, poss[pos], pos, t};
                    trajs.far_timestpes = t;
                }
            }
            poss[pos] = i; 
        }
    }

    // // Edge conflict check

    for(int t = 1; t < trajs.max_steps; t++){
        std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> edges;
        for(int i=0; i <trajs.Paths.size(); i++){
            //std::cout << " agent id: "<< i << std::endl;
            if (trajs.Paths[i].size() > t) {
                int pre_pos = trajs.Paths[i][t - 1]; 
                int pos = trajs.Paths[i][t];        

                if (edges.find({pos, pre_pos}) != edges.end()) {
                    num_of_cof ++;
                    if(pair.find({i,edges[{pos, pre_pos}]}) == pair.end() && pair.find({edges[{pos, pre_pos}],i}) == pair.end()){
                        trajs.num_cof++;
                        pair[{i, edges[{pos, pre_pos}]}] = 1;
                        pair[{edges[{pos, pre_pos}],i}] = 1;
                    }

                    //std::cout << " find a edge conflict"<< std::endl;
                    if (Constrant.empty()){
                        Constrant = {i, edges[{pos, pre_pos}], pre_pos, pos, t};
                        trajs.far_timestpes = t;
                    }
                    else if(Constrant.size() == 4 && Constrant[3] > t){
                        Constrant = {i, edges[{pos, pre_pos}], pre_pos, pos, t};
                        trajs.far_timestpes = t;
                    }
                    else if(Constrant.size() == 5 && Constrant[4] > t){
                        Constrant = {i, edges[{pos, pre_pos}], pre_pos, pos, t};
                        trajs.far_timestpes = t;
                    }

                }
                edges[{pre_pos, pos}] = i; 
            }
        }
    }

    //std::cout << "trajs.num_cof: "<< trajs.num_cof << " num_of_conf: "<< num_of_cof << std::endl;


    return Constrant;
}

s_node Astar(SharedEnvironment* env, std::vector<Int4>& flow,
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

            cost = curr->g+1;

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
        std::cout <<"error in astar: no path found "<< start<<","<<goal << std::endl;
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

std::vector<int> Astar_constraint(int agent_id, SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns,
    std::vector<std::vector<int>>& V_Containts,
    std::vector<std::vector<int>>& E_Containts)
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
        return backtrack(root);
    }

    pqueue_min_of open;
    re_of re;

    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    int pre_loc,loc,timestep;
    double tie_breaker, decay_factor;
    bool edge_test = false;


    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];

    bool Constraints_test = false;

    

    while (open.size() > 0){

        s_node* curr = open.pop();
        curr->close();
        Constraints_test = false;
        //std::vector<int> path;
        std::vector<int> path = backtrack(curr);

        // for(int j=0;j<path.size();j++){
        //   std::cout << path[j] << " ";
        // }

        //std::cout << " " << std::endl;

        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        getNeighborLocs(ns,neighbors,curr->id);
        for (int i=0; i<4; i++){
            //std::cout << " zhelima?: " << std::endl;
            path = backtrack(curr);
            int next = neighbors[i];
            if (next == -1){
                continue;
            }
            path.push_back(next);
            edge_test = false;
            Constraints_test = false;
            for(int i=0;i<E_Containts.size();i++){
                pre_loc = E_Containts[i][1];
                loc = E_Containts[i][2];
                timestep = E_Containts[i][3];
                if (path[timestep] == loc && path[timestep-1] == pre_loc && agent_id == E_Containts[i][0]){
                    edge_test = true;
                    break;
                }
            }

            for(int i=0;i<V_Containts.size();i++){
                if(path[V_Containts[i][2]] == V_Containts[i][1] && agent_id == V_Containts[i][0]){
                    Constraints_test = true;
                    break;
                }
            }

            if(edge_test == true ||  Constraints_test == true){
                continue;
            }

            cost = curr->g+1;

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
        //std::cout << "agent:"<< agent_id << " "<< "error in astar: no path found "<< start<<","<<goal << std::endl;
        return {};
        //assert(false);
        //exit(1);
    }

    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }
    return backtrack(goal_node);
}
}

