#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"

#include "Astar.h"
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <future>


namespace DefaultPlanner{


    //default planner data
    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    TrajLNS trajLNS;
    std::mt19937 mt1;

    // my data
    std::vector<int> path;
    std::vector<int>  goal_indexes;
    std::mutex mtx;
    a_node goal_node;
    a_node pre_goal_node;
    int my_limit_time;
    double w = 1.2;


    /**
     * @brief Default planner initialization
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            goal_indexes.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);

            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
            return;
    };

    void remove_flow(std::vector<int> path, std::vector<Int4>& flow, SharedEnvironment* env){
        if (path.size() <= 1){
            return;
        }
        int loc, prev_loc, diff, d, to;

        to = path.size();

        for (int j = 1; j < to; j++){
            loc = path[j];
            prev_loc = path[j-1];
            diff = loc - prev_loc;
            d = get_d(diff, env);
            flow[prev_loc].d[d] -= 1;
        }
    }

    void add_flow(std::vector<int> path, std::vector<Int4>& flow, SharedEnvironment* env){
        if (path.size() <= 1){
            return;
        }
        int loc, prev_loc, diff, d, to;

        to = path.size();

        for (int j = 1; j < to; j++){
            loc = path[j];
            prev_loc = path[j-1];
            diff = loc - prev_loc;
            d = get_d(diff, env);

            flow[prev_loc].d[d] += 1;
        }
    }

    void update_node_size_4(MCTSNode& new_node_1, std::vector<int> Constrant,SharedEnvironment* env){
        int p_i = Constrant[0];
        int pos = Constrant[1];
        int t = Constrant[2];
        new_node_1.V_Constraints.push_back(Constrant);
        int start = env->curr_states[p_i].location;
        int goal = goal_indexes[p_i];
        remove_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        // std::cout << "1" << std::endl;
        // std::cout << "V size: " << new_node_1.V_Constraints.size() << std::endl;
        // std::cout << "E size: " << new_node_1.E_Constraints.size() << std::endl;
        new_node_1.paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
                                                        new_node_1.flow, new_node_1.V_Constraints, new_node_1.E_Constraints);
        // std::cout << "2" << std::endl;
        add_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        // std::cout << "3" << std::endl;
        h3(new_node_1, &global_neighbors);

    }

    int update_node_size_6(MCTSNode& new_node_1, std::vector<int> Constrant,SharedEnvironment* env){
        int p_i = Constrant[0];
        int pos1 = Constrant[1];
        int pos2 = Constrant[2];
        int t = Constrant[3];
        int record = Constrant[4];
        int p_j = Constrant[5];
        int new_c = 0;

        // new_node_1.E_Constraints.push_back({p_i,pos1,pos2,t});
        // int start = env->curr_states[p_i].location;
        // int goal = goal_indexes[p_i];
        // remove_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        // new_node_1.paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
        //                                             new_node_1.flow, new_node_1.V_Constraints, new_node_1.E_Constraints);
        // add_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        // h3(new_node_1, &global_neighbors);

        // if (!isRepeated(new_node_1.E_Constraints,{p_i,pos1,pos2,t})) {
        //     new_node_1.E_Constraints.push_back({p_i,pos1,pos2,t});
        //     int start = env->curr_states[p_i].location;
        //     int goal = goal_indexes[p_i];
        //     remove_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        //     new_node_1.paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
        //                                                 new_node_1.V_Constraints, new_node_1.E_Constraints);
        //     add_flow(new_node_1.paths[p_i], new_node_1.flow, env);
        //     h3(new_node_1, &global_neighbors);
        // }

        if (record == 0) {

            if (!isRepeated(new_node_1.V_Constraints,{p_i,pos2,t})) {
                new_node_1.V_Constraints.push_back({p_i,pos2,t});
                new_c = 1;
            }

            if (!isRepeated(new_node_1.V_Constraints,{p_i,pos1,t})) {
                new_node_1.V_Constraints.push_back({p_i,pos1,t});
                new_c = 1;
            }

            if (new_c == 1) {
                int start = env->curr_states[p_i].location;
                int goal = goal_indexes[p_i];
                remove_flow(new_node_1.paths[p_i], new_node_1.flow, env);
                // std::cout << "1" << std::endl;
                // std::cout << "V size: " << new_node_1.V_Constraints.size() << std::endl;
                // std::cout << "E size: " << new_node_1.E_Constraints.size() << std::endl;
                new_node_1.paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
                                                            new_node_1.flow, new_node_1.V_Constraints, new_node_1.E_Constraints);
                // std::cout << "2" << std::endl;
                add_flow(new_node_1.paths[p_i], new_node_1.flow, env);
                // std::cout << "3" << std::endl;
                h3(new_node_1, &global_neighbors);
            }

        } else if (record == 1) {
            int new_c = 0;
            if (!isRepeated(new_node_1.V_Constraints,{p_j,pos2,t})) {
                new_node_1.V_Constraints.push_back({p_j,pos2,t});
                new_c = 1;
            }

            if (!isRepeated(new_node_1.V_Constraints,{p_j,pos1,t})) {
                new_node_1.V_Constraints.push_back({p_j,pos1,t});
                new_c = 1;
            }

            if (new_c == 1) {
                int start = env->curr_states[p_j].location;
                int goal = goal_indexes[p_j];
                remove_flow(new_node_1.paths[p_j], new_node_1.flow, env);
                // std::cout << "1" << std::endl;
                // std::cout << "V size: " << new_node_1.V_Constraints.size() << std::endl;
                // std::cout << "E size: " << new_node_1.E_Constraints.size() << std::endl;
                new_node_1.paths[p_j] = my_Astar_constraint(p_j,env,start,goal, &global_neighbors, global_heuristictable[goal],
                                                            new_node_1.flow, new_node_1.V_Constraints, new_node_1.E_Constraints);
                // std::cout << "2" << std::endl;
                add_flow(new_node_1.paths[p_j], new_node_1.flow, env);
                // std::cout << "3" << std::endl;
                h3(new_node_1, &global_neighbors);
            }
        }
        return new_c;
    }


    void expand(MCTSNode* root, SharedEnvironment* env, std::vector<MCTSNode*>& mcts_node_list, int LB, a_node& goal_node, int my_limit_time, const std::chrono::steady_clock::time_point& while_start) {
        // std::cout << root->constraint.empty() << std::endl;
        // auto now = std::chrono::steady_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
        // std::cout << "elapsed: " << elapsed << std::endl;
        // std::cout << "my_limit_time: " << my_limit_time << std::endl;
        // if (elapsed >= my_limit_time) {
        //     return;
        // }

        if (root->constraint.empty()) return;

        auto earliest_conflict = *std::min_element(
            root->constraint.begin(), 
            root->constraint.end(),
            [](const std::vector<int>& a, const std::vector<int>& b) {
                if (a.back() == b.back()) {
                    // if (a[0] == b[0]) {
                    //     return (rand() % 2) == 0;
                    // } else {
                    //     return a[0] < b[0];
                    // }
                    return (rand() % 2) == 0;
                }
                return a.back() < b.back();
            }
        );

        MCTSNode new_node_1 = MCTSNode(*root);
        MCTSNode new_node_2 = MCTSNode(*root);

        if (earliest_conflict.size() == 5) {
            int p_i = earliest_conflict[1];
            int p_j = earliest_conflict[2];
            int pos = earliest_conflict[3];
            int t = earliest_conflict[4];

            auto now_1 = std::chrono::steady_clock::now();
            if((!isRepeated(new_node_1.V_Constraints,{p_i, pos, t}))){
                update_node_size_4(new_node_1, {p_i, pos, t}, env);
                MCTSNode* new_node_1_ptr = new MCTSNode(new_node_1);
                if(new_node_1_ptr->cost < LB * w) {
                    root->children.insert(new_node_1_ptr);
                    mcts_node_list.push_back(new_node_1_ptr);
                    new_node_1_ptr->parent = root;

                }
            }

            auto now_2 = std::chrono::steady_clock::now();
            auto diff_1 = std::chrono::duration_cast<std::chrono::milliseconds>(now_2 - now_1).count();
            auto diff_2 = std::chrono::duration_cast<std::chrono::milliseconds>(now_2 - while_start).count();

            if (diff_1 + diff_2 >= my_limit_time) {
                return;
            }

            if((!isRepeated(new_node_2.V_Constraints,{p_j, pos, t}))){
                update_node_size_4(new_node_2, {p_j, pos, t}, env);
                MCTSNode* new_node_2_ptr = new MCTSNode(new_node_2);
                if(new_node_2_ptr->cost < LB * w) {
                    root->children.insert(new_node_2_ptr);
                    mcts_node_list.push_back(new_node_2_ptr);
                    new_node_2_ptr->parent = root;

                }
            }

        } else if (earliest_conflict.size() == 8) {
            int p_i = earliest_conflict[1];
            int p_j = earliest_conflict[2];
            int pos1 = earliest_conflict[3];
            int pos2 = earliest_conflict[4];
            int record = earliest_conflict[5];
            int record_2 = earliest_conflict[6];
            int t = earliest_conflict[7];

            auto now_1 = std::chrono::steady_clock::now();
            int new_c_1 = update_node_size_6(new_node_1, {p_i, pos1, pos2, t, record, p_j}, env);

            if (new_c_1 == 1) {
                MCTSNode* new_node_1_ptr = new MCTSNode(new_node_1);
                if(new_node_1_ptr->cost < LB * w) {
                    root->children.insert(new_node_1_ptr);
                    mcts_node_list.push_back(new_node_1_ptr);
                    new_node_1_ptr->parent = root;

                }

            }

            // auto now_2 = std::chrono::steady_clock::now();
            // auto diff_1 = std::chrono::duration_cast<std::chrono::milliseconds>(now_2 - now_1).count();
            // auto diff_2 = std::chrono::duration_cast<std::chrono::milliseconds>(now_2 - while_start).count();

            // if (diff_1 + diff_2 >= my_limit_time) {
            //     return;
            // }

            int new_c_2 = update_node_size_6(new_node_2, {p_j, pos2, pos1, t, record_2, p_i}, env);

            if (new_c_2 == 1) {
                MCTSNode* new_node_2_ptr = new MCTSNode(new_node_2);
                if(new_node_2_ptr->cost < LB * w) {
                    root->children.insert(new_node_2_ptr);
                    mcts_node_list.push_back(new_node_2_ptr);
                    new_node_2_ptr->parent = root;

                }

            }

            // if (!isRepeated(new_node_1.E_Constraints,{p_i,pos1,pos2,t})) {
            //     update_node_size_6(new_node_1, {p_i, pos1, pos2, t, record, p_j}, env);
            //     MCTSNode* new_node_1_ptr = new MCTSNode(new_node_1);
            //     if(new_node_1_ptr->cost < LB * w) {
            //         root->children.insert(new_node_1_ptr);
            //         mcts_node_list.push_back(new_node_1_ptr);
            //         new_node_1_ptr->parent = root;

            //     }

            // }

            // if((!isRepeated(new_node_2.E_Constraints,{p_j,pos2,pos1,t}))){
            //     update_node_size_6(new_node_2, {p_j, pos2, pos1, t, record_2, p_i}, env);
            //     MCTSNode* new_node_2_ptr = new MCTSNode(new_node_2);

            //     if(new_node_2_ptr->cost < LB * w) {
            //         root->children.insert(new_node_2_ptr);
            //         mcts_node_list.push_back(new_node_2_ptr);
            //         new_node_2_ptr->parent = root;

            //     }

            // }


            // if(goal_node.num_cof != 0 && new_node_1_ptr->cost <= goal_node.cost) {
            //     if(new_node_1_ptr->cost < LB * w) {
            //         root->children.insert(new_node_1_ptr);
            //         mcts_node_list.push_back(new_node_1_ptr);
            //         new_node_1_ptr->parent = root;

            //     }

            // }

            // if(goal_node.num_cof != 0 && new_node_2_ptr->cost <= goal_node.cost) {
            //     if(new_node_2_ptr->cost < LB * w) {
            //         root->children.insert(new_node_2_ptr);
            //         mcts_node_list.push_back(new_node_2_ptr);
            //         new_node_2_ptr->parent = root;

            //     }
            // }

        }

    }

    void Selection(MCTSNode& node, SharedEnvironment* env, a_node& goal_node, std::vector<MCTSNode*>& mcts_node_list, int LB, int my_limit_time, const std::chrono::steady_clock::time_point& while_start) {
        MCTSNode* current_node = &node;
        expand(current_node, env, mcts_node_list, LB, goal_node, my_limit_time, while_start);
        // std::cout << "444" << std::endl;
        
        goal_node.Paths = current_node->paths;
        goal_node.num_cof = current_node->num_cof;
        goal_node.cost = current_node->cost;
        goal_node.far_timestpes = current_node->far_timestpes;
    }


    void update_path(a_node& final_solution) {
        // Vertex conflict check

        for(int t = 1; t < 2; t++){
            std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> edges;
            for(int i=0; i <final_solution.Paths.size(); i++){
                //std::cout << " agent id: "<< i << std::endl;
                if (final_solution.Paths[i].size() > t) {
                    int pre_pos = final_solution.Paths[i][t - 1]; 
                    int pos = final_solution.Paths[i][t];     

                    if (edges.find({pos, pre_pos}) != edges.end()) {
                        final_solution.Paths[i] = {final_solution.Paths[i][0]};
                        int j  = edges[{pos, pre_pos}];
                        final_solution.Paths[j] = {final_solution.Paths[j][0]};
                    }
                    edges[{pre_pos, pos}] = i; 
                }
            }
        }

        for(int t = 0; t < 2; t++){ // only check next step
            std::unordered_map<int, int> poss; 
            //std::cout << "new agent" << std::endl;
            for(int i=0;i<final_solution.Paths.size();i++){
                int pos;
                if (final_solution.Paths[i].size() > t) {
                    pos =final_solution.Paths[i][t]; 
                } else {
                    continue;
                }

                if (poss.find(pos) != poss.end()) {
                    final_solution.Paths[i] = {final_solution.Paths[i][0]};
                }
                poss[pos] = i; 
            }
        }
    }


    void mcts_backtrack(SharedEnvironment* env, MCTSNode* current_node, a_node& final_solution, std::vector<MCTSNode*>& mcts_node_list, int LB, int max_back_steps = 5) {
        MCTSNode* best = current_node;
        int steps = 0;

        while (mcts_node_list.size() != 0 && steps < max_back_steps) {
            auto best_it = std::min_element(
                mcts_node_list.begin(), 
                mcts_node_list.end(), 
                [](const MCTSNode* a, const MCTSNode* b) {
                    if (a->num_cof == b->num_cof) {
                        if (a->far_timestpes == b->far_timestpes){
                            if (a->cost == b->cost) {
                                return (rand() % 2) == 0;
                            } else {
                                return a->cost < b->cost;
                            }
                        } else {
                            return a->far_timestpes > b->far_timestpes;
                        }

                    } else {
                        return (a->num_cof) < (b->num_cof);
                    }
                }
            );

            // auto best_it = std::min_element(
            //     mcts_node_list.begin(), 
            //     mcts_node_list.end(), 
            //     [](const MCTSNode* a, const MCTSNode* b) {
            //         // return (a->cost + a->num_cof) < (b->cost + b->num_cof);
            //         if (a->num_cof == b->num_cof) {
            //             if (a->cost == b->cost){
            //                 if (a->far_timestpes == b->far_timestpes) {
            //                     return (rand() % 2) == 0;
            //                 } else {
            //                     return a->far_timestpes > b->far_timestpes;
            //                 }
            //             } else {
            //                 return a->cost < b->cost;
            //             }

            //         } else {
            //             return (a->num_cof) < (b->num_cof);
            //         }
            //     }
            // );

            MCTSNode* current = *best_it;
            mcts_node_list.erase(best_it);
            
            if (current->num_cof <= best->num_cof && current->cost <= best->cost) {

                final_solution.Paths = current->paths;
                final_solution.num_cof = current->num_cof;
                final_solution.cost = current->cost;
                best = current;
            }

            steps++;
        }
    }

    a_node planner(SharedEnvironment* env) {
        std::vector<Int4> flow;
        flow.resize(env->map.size(),Int4({0,0,0,0}));
        std::vector<std::vector<int>> Paths;
        std::vector<int> priority;
        std::vector<MCTSNode*> mcts_node_list;

        a_node goal_node;
        auto while_start = std::chrono::steady_clock::now();
        // int num_completed_task = (env->new_tasks).size();
        // std::cout << "check timeout 1" << std::endl;
        for(int i=0;i<env->num_of_agents;i++){
            int start = env->curr_states[i].location;
            int goal = goal_indexes[i];
            int h;
            // if(global_neighbors.empty())
            //     h = manhattanDistance(start,goal,env);
            // else
            //     h = get_heuristic(global_heuristictable[goal], env, start, &global_neighbors);
             h = manhattanDistance(start,goal,env);
            priority.push_back(h);

            if (pre_goal_node.Paths.size() > 1 && pre_goal_node.Paths[i].size() > 1){
                if(start == pre_goal_node.Paths[i][1]){
                    path = std::vector<int>(pre_goal_node.Paths[i].begin() + 1, pre_goal_node.Paths[i].end());
                }
                else if(start == pre_goal_node.Paths[i][0] && pre_goal_node.Paths[i].size() > 1){
                    path = pre_goal_node.Paths[i];
                }
            }
            else{
                path = my_Astar(env,start,goal, &global_neighbors, global_heuristictable[goal], flow);
            }
            
            add_flow(path, flow, env);
            // priority.push_back(path.size());
            Paths.push_back(path);
        }

        // MCTSNode root = MCTSNode(Paths);
        // h3(root);
        // std::cout << "check timeout 2" << std::endl;
        MCTSNode* root = new MCTSNode(Paths, priority, flow);
        h3(*root, &global_neighbors);
        // std::cout << "low level A* cost: " << root->cost << std::endl;
        mcts_node_list.push_back(root);
        goal_node.Paths = root->paths;
        goal_node.num_cof = root->num_cof;
        goal_node.cost = root->cost;
        goal_node.far_timestpes = root->far_timestpes;

        MCTSNode* current_node;
        int closed_cof = 0;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();

        while (!mcts_node_list.empty() && elapsed < my_limit_time) {
            // std::cout << "check reach" << std::endl;
            // MCTSNode* current_node = *std::min_element(
            //     mcts_node_list.begin(),
            //     mcts_node_list.end(),
            //     [](const MCTSNode* a, const MCTSNode* b) {
            //         // return (a->cost + a->num_cof) < (b->cost + b->num_cof);
            //         return (a->num_cof) < (b->num_cof);
            //     }
            // );

            auto best_it = std::min_element(
                mcts_node_list.begin(), 
                mcts_node_list.end(), 
                [](const MCTSNode* a, const MCTSNode* b) {
                    // return (a->cost + a->num_cof) < (b->cost + b->num_cof);
                    if (a->num_cof == b->num_cof) {
                        if (a->far_timestpes == b->far_timestpes){
                            if (a->cost == b->cost) {
                                return (rand() % 2) == 0;
                            } else {
                                return a->cost < b->cost;
                            }
                        } else {
                            return a->far_timestpes > b->far_timestpes;
                        }

                    } else {
                        return (a->num_cof) < (b->num_cof);
                    }
                }
            );

            // auto best_it = std::min_element(
            //     mcts_node_list.begin(), 
            //     mcts_node_list.end(), 
            //     [](const MCTSNode* a, const MCTSNode* b) {
            //         // return (a->cost + a->num_cof) < (b->cost + b->num_cof);
            //         if (a->num_cof == b->num_cof) {
            //             if (a->cost == b->cost){
            //                 if (a->far_timestpes == b->far_timestpes) {
            //                     return (rand() % 2) == 0;
            //                 } else {
            //                     return a->far_timestpes > b->far_timestpes;
            //                 }
            //             } else {
            //                 return a->cost < b->cost;
            //             }

            //         } else {
            //             return (a->num_cof) < (b->num_cof);
            //         }
            //     }
            // );

            current_node = *best_it;
            mcts_node_list.erase(best_it);

            int LB = current_node->cost;

            // std::cout << "111"  << std::endl;

            if (current_node->num_cof == 0 && current_node->cost <= goal_node.cost) {
                goal_node.Paths = current_node->paths;
                goal_node.num_cof = current_node->num_cof;
                goal_node.cost = current_node->cost;
                goal_node.far_timestpes = current_node->far_timestpes;
                // return goal_node;
                // std::cout << "got optimal solution"  << std::endl;
                break;
            } 
            // else if (current_node->num_cof <= goal_node.num_cof) {
            //     goal_node.Paths = current_node->paths;
            //     goal_node.num_cof = current_node->num_cof;
            //     goal_node.cost = current_node->cost;
            //     goal_node.far_timestpes = current_node->far_timestpes;
            // }
            
            // else if (closed_cof < current_node->far_timestpes) {// sub goal
            //     goal_node.Paths = current_node->paths;
            //     goal_node.num_cof = current_node->num_cof;
            //     goal_node.cost = current_node->cost;
            //     closed_cof = current_node->far_timestpes;
            //     goal_node.far_timestpes = current_node->far_timestpes;
            // }


            // expand(current_node, env);
            Selection(*current_node, env, goal_node, mcts_node_list, LB, my_limit_time, while_start);

            // std::cout << "pre cof num: "  << current_node->num_cof << std::endl;
            // if (current_node->num_cof == 0) {
            //     return goal_node;
            // }

            
            //  std::cout << "cof num: "  << current_node->num_cof << std::endl;


            now = std::chrono::steady_clock::now();
            elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
        }

        // int LB = current_node->cost;
        // std::cout << "before backtracking cost: " << goal_node.cost << std::endl;
        // mcts_backtrack(env, current_node, goal_node, mcts_node_list, LB);
        // std::cout << "after backtracking cost: " << goal_node.cost << std::endl;
        return goal_node;
    }


    using PlannerFunc = std::function<a_node(SharedEnvironment*)>;

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){
        // std::cout << "check timeout 3" << std::endl;
        auto init_start = std::chrono::steady_clock::now();


        // std::cout << "time limit: " << time_limit << std::endl;
    

        const int numRuns = 1;
        my_limit_time = time_limit/(numRuns+0.6);

        // std::cout << "my_limit_time: " << my_limit_time << std::endl;

        // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        // update the status of each agent and prepare for planning
        int count = 0;

        for(int i=0; i<env->num_of_agents; i++){
            for(int j=0; j<env->goal_locations[i].size(); j++){
                int goal_loc = env->goal_locations[i][j].first;
                if(global_heuristictable.at(goal_loc).empty()){
                    init_heuristic(global_heuristictable[goal_loc],env,goal_loc);
                    count++;
                }
            }
            
            // path = {prev_states[i].location,env->curr_states[i].location};
            // add_flow(path, flow, env);
            
            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                goal_indexes[i] = dummy_goals.at(i);
            }
            // else if (env->curr_timestep > 0 && prev_states[i].location == env->curr_states[i].location &&  pre_goal_node.Paths[i].size() > 1){
            //      goal_indexes[i] = pre_goal_node.Paths[i][1];
            // }
            else{
                goal_indexes[i] = env->goal_locations[i].front().first;
            }


            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            prev_decision[env->curr_states[i].location] = i; 
        }
    
        auto init_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> init_elapsed_seconds = init_end - init_start;
        
        goal_node = planner(env);
        pre_goal_node = goal_node;
        update_path(goal_node);

        // std::cout << "Final goal cost: " << goal_node.cost << std::endl;
        // std::cout << "Final num_cof: " << goal_node.num_cof << std::endl;
        // std::cout << "Final far cof: " << goal_node.far_timestpes << std::endl;


        actions.resize(env->num_of_agents);

        for(int i = 0; i < env->num_of_agents;i++){
            if(goal_node.Paths[i].size() > 1){
                decided[i] = DCR({goal_node.Paths[i][1],DONE::NOT_DONE});
                // for(int k=0;k<goal_node.Paths[i].size() - 1;k++){
                //     if (env->curr_states[i].location == goal_node.Paths[i][k])
                //         decided[i] = DCR({goal_node.Paths[i][k+1],DONE::NOT_DONE});
                //     continue;
                // }
            }
            else if (goal_node.Paths[i].size() == 1){
                decided[i] = DCR({goal_node.Paths[i][0],DONE::NOT_DONE});
            }
            actions[i] = getAction(prev_states[i],decided[i].loc, env);
            checked[i] = false;
        }

        // // recursively check if the FW action can be executed by checking whether all agents in the front of the agent can move forward
        // if any agent cannot move foward due to turning, all agents behind the turning agent will not move forward.
        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,actions,prev_decision);
            }
        }
        
        prev_states = env->curr_states;
        // pre_goal_node = goal_node;

        return;

    };


}