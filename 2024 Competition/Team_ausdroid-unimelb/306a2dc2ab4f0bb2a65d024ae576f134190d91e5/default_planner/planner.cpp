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

    // void update_node_size_4(MCTSNode* new_node_1, std::vector<int> Constrant,SharedEnvironment* env){
    //     int p_i = Constrant[0];
    //     int pos = Constrant[1];
    //     int t = Constrant[2];
    //     new_node_1->V_Constraints.push_back(Constrant);
    //     int start = env->curr_states[p_i].location;
    //     int goal = goal_indexes[p_i];
    //     new_node_1->paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_1->V_Constraints, new_node_1->E_Constraints);
                                        
    //     h3(new_node_1);

    // }

    // void update_node_size_5(MCTSNode* new_node_1, std::vector<int> Constrant,SharedEnvironment* env){
    //     int p_i = Constrant[0];
    //     int pos1 = Constrant[1];
    //     int pos2 = Constrant[2];
    //     int t = Constrant[3];
    //     new_node_1->E_Constraints.push_back(Constrant);
    //     int start = env->curr_states[p_i].location;
    //     int goal = goal_indexes[p_i];
    //     new_node_1->paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_1->V_Constraints, new_node_1->E_Constraints);
    //     // std::cout << "Path improved: " << std::endl;
    //     // for(int i =0;i< new_node_1->paths[p_i].size();i++){
    //     //     std::cout << new_node_1->paths[p_i][i] << " ";
    //     // }
    //     //  std::cout << " "<< std::endl;
    //     h3(new_node_1);

    // }


    // void expand(MCTSNode* root, SharedEnvironment* env){
    //     std::vector<int> Constrant = {};
    //     double LB =  1.01 * root->cost;
    //     //std::cout << "expand depth:"<< root->depth  << std::endl;
    //     //std::cout << "expand parent depth:"<< root->parent->depth  << std::endl;
    //     std::vector<MCTSNode*> children; 
    //     for(int i=0;i<root->constraint.size();i++){
    //         MCTSNode* new_node_1 = new MCTSNode(root);
    //         MCTSNode* new_node_2 = new MCTSNode(root);
    //         //std::cout << "expand child depth:"<< new_node_1->parent->depth  << std::endl;
    //         Constrant =  root->constraint[i];
    //         if (Constrant.size() == 4){
    //             int p_i = Constrant[0];
    //             int p_j = Constrant[1];
    //             int pos = Constrant[2];
    //             int t = Constrant[3];
    //             //std::cout << "Paths Constraint " << p_i << " " << p_j << " " << pos << " " << t <<std::endl;

    //             if((!isRepeated(new_node_1->V_Constraints,{p_i,pos,t}))){
    //                 update_node_size_4(new_node_1,{p_i,pos,t},env);
    //                 if(new_node_1->cost < LB){
    //                     children.push_back(new_node_1);                       
    //                 }
    //             }
                
    //             if((!isRepeated(new_node_2->V_Constraints,{p_j,pos,t}))){
    //                 update_node_size_4(new_node_2,{p_j,pos,t},env);
    //                 if(new_node_2->cost < LB){
    //                     children.push_back(new_node_2);
    //                 }
    //             }
    //         }
    //         else if (Constrant.size() == 5){
    //             int p_i = Constrant[0];
    //             int p_j = Constrant[1];
    //             int pos1 = Constrant[2];
    //             int pos2 = Constrant[3];
    //             int t = Constrant[4];
    //             //std::cout << "Paths Constraint " << p_i << " " << p_j << " " << pos1 << " " << pos2 << " " << t <<std::endl;
    //             if((!isRepeated(new_node_1->E_Constraints,{p_i,pos1,pos2,t}))){
    //                 update_node_size_5(new_node_1,{p_i,pos1,pos2,t},env);
    //                 if(new_node_1->cost < LB){
    //                     children.push_back(new_node_1);
    //                 }

    //             }
    //             if((!isRepeated(new_node_2->E_Constraints,{p_j,pos2,pos1,t}))){
    //                 update_node_size_5(new_node_2,{p_j,pos2,pos1,t},env);
    //                 if(new_node_2->cost < LB){
    //                     children.push_back(new_node_2);
    //                 }
    //             }
    //         }
    //     }
    //     root->children = children;

    // }

    

    // void Selection(MCTSNode* node, SharedEnvironment* env, a_node& goal_node){
    //     int min_cost = std::numeric_limits<int>::max();
    //     std::vector<a_node> results;
    //     a_node result;
    //     h3(node);
    //     if(node->children.size() == 0){
    //         expand(node,env);
    //     }
    //     std::sort(node->children.begin(), node->children.end(), [](MCTSNode* a, MCTSNode* b) {
    //         return a->uctValue(1.4) > b->uctValue(1.4); 
    //     });
    //     MCTSNode* new_node = node->children[0];
    //     goal_node.Paths = new_node->paths;
    //     goal_node.num_cof = new_node->num_cof;
    //     goal_node.cost = new_node->cost;
    //     goal_node.far_timestpes = new_node->closest_timestpes;

    //     int cont = 0;
    //     double best_score = 0;
    //     while(new_node->children.size() > 0){
    //         // std::cout << "cont: " <<  cont << std::endl;
    //         // std::cout << "current depth:"<< new_node->depth  << std::endl;
    //         std::sort(new_node->children.begin(), new_node->children.end(), [](MCTSNode* a, MCTSNode* b) {
    //             return a->uctValue(1.4) > b->uctValue(1.4); 
    //         });
    //         std::cout << "new_node children depth: "  << std::endl;
    //         for (auto it = new_node->children.begin(); it != new_node->children.end(); ++it) {
    //             std::cout << (*it)->depth << " "; 
    //         }
    //         std::cout << " "  << std::endl;

    //         // std::cout << "UCT value: "  << std::endl;
    //         // for (auto it = new_node->children.begin(); it != new_node->children.end(); ++it) {
    //         //     std::cout << (*it)->uctValue(1.4) << " "; 
    //         // }
    //         // std::cout << " "  << std::endl;
    //         cont ++;
    //         new_node = new_node->children[0];
    //         // std::cout << "current UCT:"<< new_node->uctValue(1.4)  << std::endl;
    //         // std::cout << "current parent depth:"<< new_node->parent->depth  << std::endl;

    //         result.Paths = new_node->paths;
    //         result.num_cof = new_node->num_cof;
    //         result.cost = new_node->cost;
    //         new_node->parent->depth ++;
    //         std::cout << "result cost size: " <<  result.cost << std::endl;
    //         std::cout << "result num_cof size: " <<  result.num_cof << std::endl;
    //         result.far_timestpes = new_node->closest_timestpes;
    //         // if(result.cost + result.num_cof - 0.1 *  result.far_timestpes < min_cost && result.far_timestpes != 1){
    //         //     results = {result};
    //         //     min_cost = result.cost + result.num_cof - 0.1* result.far_timestpes;
    //         // }
    //         // if(result.cost + result.num_cof - 0.1 * result.far_timestpes == min_cost && result.far_timestpes != 1){
    //         //     results.push_back(result);
    //         // }
    //         if( result.num_cof - 0.1 * result.far_timestpes < min_cost){
    //             results = {result};
    //             min_cost = result.num_cof - 0.1* result.far_timestpes;
    //         }
    //         if(result.num_cof - 0.1 * result.far_timestpes == min_cost){
    //             results.push_back(result);
    //         }
    //     }
    //     expand(new_node,env);
    //     //node = *node1;
    //     //std::cout << "result size: " <<  results.size() << std::endl;
    //     //node.depth = node.depth + 1;
    //     if (results.size()>0){
    //         a_node minNode = *std::min_element(results.begin(), results.end(), 
    //         [](const a_node& a, const a_node& b) {
    //             return a.cost < b.cost;
    //         });
    //         goal_node = minNode;
    //     }
    //         //std::cout << " closest_timestpes : "<< goal_node.far_timestpes    << std::endl;
    // }

    void checkLowLevel(std::vector<my_node> Paths, SharedEnvironment* env){
        for(int i=0;i<Paths.size();i++){
            //if (i == 73 || i == 38){
            std::cout << "initial Paths: " << i << std::endl;
            std::cout <<"start:" << env->curr_states[i].location << endl; 
            std::cout <<"goal_indexes:"<< goal_indexes[i] << endl;
            for(int j=0;j< Paths[i].paths.size();j++){
                std::cout << Paths[i].paths[j] << " ";
            }
            std::cout << " "<< endl; 

            std::cout << "initial Actions: " << i << std::endl;
            for(int j=0;j<Paths[i].actions.size();j++){
                std::cout << Paths[i].actions[j] << " ";
            }
            std::cout <<" " << endl; 
            //std::cout <<"  "<<  endl;
            //}
        }
    }

    a_node low_level(SharedEnvironment* env, a_node pre_goal_node){
        a_node root;
        my_node paths;
        std::vector<my_node> Paths;
        int max_timesteps, timesteps;
        timesteps = 0;
        max_timesteps = 0;
        std::vector<Action> actions;

        for(int i=0;i<env->num_of_agents;i++){
            int start = env->curr_states[i].location;
            int goal = goal_indexes[i];
            if (env->curr_timestep > 0 && pre_goal_node.Paths[i].actions.size() > 0 && pre_goal_node.Paths[i].paths[0] == start){
                path = pre_goal_node.Paths[i].paths;
                actions = pre_goal_node.Paths[i].actions;
                paths = my_node(start,pre_goal_node.Paths[i].timesteps - 1, pre_goal_node.Paths[i].cost,
                                     path,env->curr_states[i].orientation,actions);
                
                //                 path = std::vector<int>(pre_goal_node.Paths[i].paths.begin() + 1, pre_goal_node.Paths[i].paths.end());
                // actions = std::vector<Action>(pre_goal_node.Paths[i].actions.begin() + 1, pre_goal_node.Paths[i].actions.end());
                // paths = my_node(start,pre_goal_node.Paths[i].timesteps - 1, pre_goal_node.Paths[i].cost,
                //                 path,env->curr_states[i].orientation,actions);
                if (path.size() < 2 || actions.size() < 1){
                    paths = my_Astar(i, env,start,goal, &global_neighbors, global_heuristictable[goal]);
                }
            }
            else if(start == goal){
                path = {start,start};
                actions = {Action::W};
                paths = my_node(start, 1,1,path,env->curr_states[i].orientation,actions);
            }
            else{
                paths = my_Astar(i, env,start,goal, &global_neighbors, global_heuristictable[goal]);
            }
            Paths.push_back(paths);
            timesteps += paths.paths.size();
            if (max_timesteps < paths.paths.size())
                max_timesteps = paths.paths.size();
        }
        
        //checkLowLevel(Paths,env);

        root.Paths = Paths;
        root.cost = timesteps;
        root.num_cof = std::numeric_limits<int>::max();
        root.max_steps = max_timesteps;
        root.far_timestpes = 1;
        return root;
    }

    void update_node_size_4(a_node& new_node_1, std::vector<int> Constrant, SharedEnvironment* env){
        int p_i = Constrant[0];
        int pos = Constrant[1];
        int t = Constrant[2];
        new_node_1.V_Constraints.push_back(Constrant);
        int start = env->curr_states[p_i].location;
        int goal = goal_indexes[p_i];
        new_node_1.cost -= new_node_1.Paths[p_i].paths.size();
        //std::cout << "prev path"<< endl; 
        // for(int j=0;j<new_node_1.Paths[p_i].paths.size();j++){
        //     std::cout << new_node_1.Paths[p_i].paths[j] << " ";
        //     }
        // std::cout << " "<< endl; 

        new_node_1.Paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
                                                        new_node_1.V_Constraints, new_node_1.E_Constraints);

        
        // std::cout << "Improved path"<< endl; 
        // std::cout << new_node_1.Paths[p_i].paths[0] << " ";
        // // for(int j=0;j<new_node_1.Paths[p_i].paths.size();j++){
        // //     std::cout << new_node_1.Paths[p_i].paths[j] << " ";
        // //     }
        // std::cout << " "<< endl; 
        int path_length = new_node_1.Paths[p_i].paths.size();
        new_node_1.cost += path_length;
        if(new_node_1.max_steps <  path_length)
            new_node_1.max_steps = path_length;
    }
    
    void update_node_size_5(a_node& new_node_1, std::vector<int> Constrant, SharedEnvironment* env){
        int p_i = Constrant[0];
        int pos1 = Constrant[1];
        int pos2 = Constrant[2];
        int t = Constrant[3];
        new_node_1.E_Constraints.push_back(Constrant);
        int start = env->curr_states[p_i].location;
        int goal = goal_indexes[p_i];
        new_node_1.cost -= new_node_1.Paths[p_i].paths.size();
        new_node_1.Paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
                                                        new_node_1.V_Constraints, new_node_1.E_Constraints);
        // std::cout << "Improved path"<< endl; 
        // std::cout << new_node_1.Paths[p_i].paths[0] << " ";
        // // for(int j=0;j<new_node_1.Paths[p_i].paths.size();j++){
        // //     std::cout << new_node_1.Paths[p_i].paths[j] << " ";
        // //     }
        // std::cout << " "<< endl; 
        int path_length = new_node_1.Paths[p_i].paths.size();
        new_node_1.cost += path_length;
        if(new_node_1.max_steps <  path_length)
            new_node_1.max_steps = path_length;
    }

    int generate_d(Action action, int orientation){
        int d = 0;
        if (action == Action::CR)
            d = (orientation+1)%4;
        if (action == Action::CCR)
            d = (orientation+3)%4;
        if (action == Action::W || action == Action::FW)
            d = orientation;
        return d;
    }

    a_node high_level(std::priority_queue<a_node> open, a_node root, SharedEnvironment* env){
        std::cout << "start high_level" << endl; 
        a_node goal_node;
        goal_node.cost = std::numeric_limits<int>::max();
        goal_node.far_timestpes = 0;
        a_node subgoal_node;
        int min_num_cof = std::numeric_limits<int>::max();
        int far_timestpes = 0;
        auto while_start = std::chrono::steady_clock::now();
        std::vector<int> Constrant; 
        int LB;
        bool find = false;
        while (!open.empty()) {
            a_node node = open.top();
            open.pop();
            if(find == true && node.cost > goal_node.cost)
                continue;
            if(min_num_cof > node.num_cof){
                min_num_cof = node.num_cof;
            }
            for(int i=0;i<env->num_of_agents;i++){
                if (node.Paths[i].actions.size() == 0){
                    node.Paths[i] = root.Paths[i];
                 }
            }
            // std::cout << "check path"<< endl; 
            // std::cout << node.Paths[1].paths[0] << " ";
            // std::cout << " "<< endl; 
            // for(int j=0;j<node.Paths[1].paths.size();j++){
            //     std::cout << node.Paths[1].paths[j] << " ";
            // }
            //std::cout << node.max_steps << endl; 
            Constrant = validate(node);
            if(far_timestpes < node.far_timestpes){
                subgoal_node  = node;
                far_timestpes = node.far_timestpes;
            }
            else if(far_timestpes == node.far_timestpes && subgoal_node.cost > node.cost){
                subgoal_node  = node;
            }
            LB = node.cost;
            
            if (node.num_cof == 0){
                if(find == false){
                    goal_node = node;
                    goal_node.far_timestpes = 0;
                    find = true;
                }
                else if(find == true && node.cost < goal_node.cost){
                    goal_node = node;
                    goal_node.far_timestpes = 0;
                    std::cout << "Level up!" << endl; 

                }


            }
            a_node new_node_1(node);
            a_node new_node_2(node);
            if (Constrant.size() == 4){
                int p_i = Constrant[0];
                int p_j = Constrant[1];
                int pos = Constrant[2];
                int t = Constrant[3];
                if((!isRepeated(new_node_1.V_Constraints,{p_i,pos,t}))){
                    update_node_size_4(new_node_1,{p_i,pos,t},env);
                    if(new_node_1.Paths[p_i].actions.size() > 0){
                        validate(new_node_1);
                        //if(new_node_1.cost < LB * 1.1)
                            open.push(new_node_1);                       
                        //}
                    }
                }
                if((!isRepeated(new_node_2.V_Constraints,{p_j,pos,t}))){
                    update_node_size_4(new_node_2,{p_j,pos,t},env);
                    if(new_node_2.Paths[p_j].actions.size() > 0){
                        validate(new_node_2);
                        //if(new_node_2.cost < LB * 1.1)
                        open.push(new_node_2);                       
                        //}
                    }
                }

            }
            else if (Constrant.size() == 5){
                int p_i = Constrant[0];
                int p_j = Constrant[1];
                int pos1 = Constrant[2];
                int pos2 = Constrant[3];
                int t = Constrant[4];   

                if((!isRepeated(new_node_1.E_Constraints,{p_i,pos1,pos2,t}))){
                    update_node_size_5(new_node_1,{p_i,pos1,pos2,t},env);
                    if(new_node_1.Paths[p_i].actions.size() > 0){
                        validate(new_node_1);   
                        //if(new_node_1.cost < LB * 1.1)
                        open.push(new_node_1);                       
                        //}
                    }
                }
                if((!isRepeated(new_node_2.E_Constraints,{p_j,pos2,pos1,t}))){
                    update_node_size_5(new_node_2,{p_j,pos2,pos1,t},env);
                    if(new_node_2.Paths[p_j].actions.size() > 0){
                        validate(new_node_2);
                        //if(new_node_2.cost < LB * 1.1)
                        open.push(new_node_2);                       
                        //}
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
            //std::cout << "check time: " << (elapsed >= my_limit_time) << std::endl;
            if (elapsed >= my_limit_time) {
                break;
            }
        }
        if (find != true){
            goal_node = subgoal_node;
            goal_node.far_timestpes = far_timestpes;
            std::cout << "use subgoal" << endl; 
        }
        

        // std::priority_queue<a_node> next_open;
        // std::vector<Action> actions;
        // std::vector<my_node> Paths;
        // int max_timesteps, timesteps;
        // my_node paths;
        
        // while (!open.empty()){  
        //     a_node node = open.top();
        //     open.pop();
        //     bool test = true;
        //     for(int i =0;i<node.Paths.size();i++){
        //         if (node.Paths[i].actions.size() < 2 || node.Paths[i].actions[0] != goal_node.Paths[i].actions[0])
        //             test = false;
        //         else{
        //             node.Paths[i].paths = std::vector<int>(node.Paths[i].paths.begin() + 1, node.Paths[i].paths.end());
        //             node.Paths[i].actions = std::vector<Action>(node.Paths[i].actions.begin() + 1, node.Paths[i].actions.end());
        //         }
        //     }
        //     if(test == true){
        //         next_open.push(node);
        //     }
        // }
        // std::cout << "next_open size: "<< next_open.size() << endl; 
        // goal_node.next_open = next_open;
        return goal_node;
    }

    a_node planner(SharedEnvironment* env){
        std::priority_queue<a_node> open;
        std::vector<int> Constrant;
        //open = pre_goal_node.next_open;
        auto for_start = std::chrono::steady_clock::now();
        a_node root;
        a_node goal_node;
        root = low_level(env,pre_goal_node);
        open.push(root);
        goal_node = high_level(open,root,env);
        // std::cout << "check path"<< endl; 
        // std::cout << goal_node.Paths[1].paths[0] << " ";
        // std::cout << " " << endl; 
        return goal_node;
    }

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
        auto init_start = std::chrono::steady_clock::now();


        std::cout << "time limit: " << time_limit << std::endl;

        const int numRuns = 1;
        my_limit_time = time_limit/(numRuns+0.5);

        std::cout << "my_limit_time: " << my_limit_time << std::endl;

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

        std::cout << "Final goal cost: " << goal_node.cost << std::endl;
        std::cout << "Final num_cof: " << goal_node.num_cof << std::endl;
        std::cout << "Final far cof: " << goal_node.far_timestpes << std::endl;


        actions.resize(env->num_of_agents);
    


        for(int i = 0; i < env->num_of_agents;i++){
            // std::cout << "Agent: " << i << std::endl;
            // std::cout <<"start:" << env->curr_states[i].location << endl; 
            // std::cout <<"goal_indexes:"<< goal_indexes[i] << endl;  
            // for(int j=0;j<goal_node.Paths[i].paths.size();j++){
            //     std::cout << goal_node.Paths[i].paths[j] << " ";
            // }
            // std::cout << " "<< endl; 
            // std::cout << "Actions: " << i << std::endl;
            // for(int j=0;j<goal_node.Paths[i].actions.size();j++){
            //     std::cout << goal_node.Paths[i].actions[j] << " ";
            // }
            // std::cout << " "<< endl; 
            // std::cout << " "<< endl; 
            if(goal_node.Paths[i].actions.size() > 0){
                actions[i] = goal_node.Paths[i].actions[0];
                goal_node.Paths[i].actions.erase(goal_node.Paths[i].actions.begin());
                goal_node.Paths[i].paths.erase(goal_node.Paths[i].paths.begin());
            }

        }

        // recursively check if the FW action can be executed by checking whether all agents in the front of the agent can move forward
        // if any agent cannot move foward due to turning, all agents behind the turning agent will not move forward.
        // for (int id=0;id < env->num_of_agents ; id++){
        //     if (!checked.at(id) && actions.at(id) == Action::FW){
        //         moveCheck(id,checked,decided,actions,prev_decision);
        //     }
        // }
        
        prev_states = env->curr_states;
        pre_goal_node = goal_node;

        return;

    };


    // a_node planner(SharedEnvironment* env){

    //     std::lock_guard<std::mutex> lock(mtx);

    //     // low level
    //     std::vector<my_node> Paths;
    //     my_node paths;

    //     int timesteps = 0;
    //     int max_timesteps = 0;
    //     int timestep = 0;

    //     std::vector<std::vector<int>> emptyConstraints;
    //     vector<Action>  actions; 
        

    //     auto for_start = std::chrono::steady_clock::now();




    //     for(int i=0;i<env->num_of_agents;i++){
    //         int start = env->curr_states[i].location;
    //         int goal = goal_indexes[i];
 
    //         if (pre_goal_node.Paths.size() > 1 && pre_goal_node.Paths[i].actions.size() > 1 && start == pre_goal_node.Paths[i].paths[1]){
    //             path = std::vector<int>(pre_goal_node.Paths[i].paths.begin() + 1, pre_goal_node.Paths[i].paths.end());
    //             actions = std::vector<Action>(pre_goal_node.Paths[i].actions.begin() + 1, pre_goal_node.Paths[i].actions.end());
    //             paths = my_node(start,pre_goal_node.Paths[i].timesteps - 1, pre_goal_node.Paths[i].cost,
    //                             path,env->curr_states[i].orientation,actions);
    //             if (path[path.size()-1] != goal){
    //                 paths = my_Astar(i, env,start,goal, &global_neighbors, global_heuristictable[goal]);
    //             }
    //         }
    //         else{
    //             paths = my_Astar(i, env,start,goal, &global_neighbors, global_heuristictable[goal]);
    //         }
            
    //         // if (i == 0){
    //         // std::cout << "curr_states[i].orientation: " << env->curr_states[i].orientation << std::endl;
    //         // std::cout << "Paths: " << i << std::endl;
    //         // for(int j=0;j<paths.paths.size();j++){   
    //         //     std::cout << paths.paths[j] << " ";
    //         // }
    //         // std::cout << " "<<  endl; 

    //         // std::cout << "Actions: " << i << std::endl;
    //         // for(int j=0;j<paths.actions.size();j++){
    //         //     std::cout << paths.actions[j] << " ";
    //         // }
    //         // std::cout << " "<< endl; 
    //         // }
            

    //         Paths.push_back(paths);
    //         timestep = 0;
    //         for(int j=0;j<Paths[i].paths.size();j++){
    //             timestep ++;
    //             timesteps ++;
    //             if (max_timesteps < timestep){
    //                 max_timesteps = timestep;
    //             }
    //         }
    //     }

    //     //std::cout << max_timesteps << std::endl;
    //     auto for_end = std::chrono::steady_clock::now();
    //     std::chrono::duration<double> for_elapsed_seconds = for_end - for_start;
    
    //     //std::cout << "low level cost: " << for_elapsed_seconds.count() << " s\n";

    //     // std::cout << " " << std::endl;

    //     // for(int i=0;i<Paths.size();i++){
    //     //     std::cout << "Conflict path "<< i << " : "<< " ";
    //     //     for(int j=0;j<Paths[i].size();j++){
    //     //         std::cout << Paths[i][j] << " ";
    //     //     }
    //     //     std::cout << " " << std::endl;
    //     // }

    //     // std::cout << " " << std::endl;

    //     // for(int i=0;i<Paths.size();i++){
    //     //     if (i == 73 || i == 38){
    //     //         std::cout << "initial Paths: " << i << std::endl;
    //     //         for(int j=0;j< Paths[i].paths.size();j++){
    //     //             std::cout << Paths[i].paths[j] << " ";
    //     //         }
    //     //         std::cout << " "<< endl; 

    //     //         std::cout << "initial Actions: " << i << std::endl;
    //     //         for(int j=0;j<Paths[i].actions.size();j++){
    //     //             std::cout << Paths[i].actions[j] << " ";
    //     //         }
    //     //         std::cout << " "<< endl; 
    //     //         std::cout <<"start:" << env->curr_states[i].location << endl; 
    //     //         std::cout <<"goal_indexes:"<< goal_indexes[i] << endl;
    //     //     }
    //     // }
        


    //     a_node root;
    //     int LB = timesteps;
    //     double w = 1.01;
    //     a_node goal_node;
    //     a_node subgoal_node;
    //     root.Paths = Paths;
    //     root.cost = timesteps;
    //     root.num_cof = 0;
    //     root.max_steps = max_timesteps;
    //     root.far_timestpes = 0;
    //     goal_node.cost = std::numeric_limits<int>::max();
    //     goal_node.num_cof = std::numeric_limits<int>::max();



    //     std::priority_queue<a_node> open;
    //     open.push(root);

    //     int far_timestpes = 0;

    //     std::vector<int> Constrant;        
    //     auto while_start = std::chrono::steady_clock::now();

    //     while (!open.empty()) {
    //         a_node node = open.top();
    //         open.pop();
    //         Constrant = validate(node);


    //         LB = node.cost;
    //         // std::cout << "pop node: " << std::endl;
    //         // std::cout << "Constrant size: " << Constrant.size() << std::endl;
    //         // std::cout << "conflict number: "<< node.num_cof << std::endl;
    //         // if (node.num_cof == 0){
    //         //     goal_node = node;
    //         //     //break;
    //         // }
            
    //         if(node.num_cof == 0 && goal_node.cost > node.cost){
    //             std::cout << "pop node: " << std::endl;
    //             goal_node = node;
    //         }

    //         a_node new_node_1, new_node_2;
    //         new_node_1.V_Constraints = node.V_Constraints;
    //         new_node_1.E_Constraints = node.E_Constraints;

    //         new_node_2.V_Constraints = node.V_Constraints;
    //         new_node_2.E_Constraints = node.E_Constraints;

    //         new_node_1.Paths = node.Paths;
    //         new_node_2.Paths = node.Paths;


    //         if (Constrant.size() == 4){
    //             int p_i = Constrant[0];
    //             int p_j = Constrant[1];
    //             int pos = Constrant[2];
    //             int t = Constrant[3];
    //             if (t > far_timestpes){
    //                 subgoal_node = node;
    //                 far_timestpes = t;
    //                 //std::cout << "fatest timestepst: " << far_timestpes << std::endl;
    //             }


    //             // std::cout << "Conflict time and pos: " << std::endl;
    //             // std::cout << "timestime: "<< t << " ";
    //             // std::cout << "pos: "<< pos << " ";
    //             // std::cout << " " << std::endl;


    //             // std::cout << "Conflict path 1: " << std::endl;
    //             // for(int j=0;j<node.Paths[p_i].size();j++){
    //             //     std::cout << node.Paths[p_i][j] << " ";
    //             // }
    //             // std::cout << " " << std::endl;

    //             // std::cout << "Conflict path 2: " << std::endl;
                
    //             // for(int j=0;j<node.Paths[p_j].size();j++){
    //             //     std::cout << node.Paths[p_j][j] << " ";
    //             // }
    //             // std::cout << " " << std::endl;

    //             if((!isRepeated(new_node_1.V_Constraints,{p_i,pos,t}))){
    //                 new_node_1.V_Constraints.push_back({p_i,pos,t});
    //                 int start = env->curr_states[p_i].location;
    //                 int goal = goal_indexes[p_i];
                    
    //                 new_node_1.Paths[p_i] = my_Astar_constraint(p_i,env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_1.V_Constraints, new_node_1.E_Constraints);
                    
                    
    //                 // Astar_constraint(p_i,trajLNS.env,trajLNS.flow, 
    //                 //                         trajLNS.heuristics[goal],trajLNS.trajs[p_i],trajLNS.mem,start,goal, &(trajLNS.neighbors),
    //                 //                         new_node_1.V_Constraints, new_node_1.E_Constraints);
    //                 timesteps = 0;
    //                 max_timesteps = 0;
    //                 timestep = 0;

    //                 for(int i=0;i<new_node_1.Paths.size();i++){
    //                     timestep = 0;
    //                     for(int j=0;j<new_node_1.Paths[i].paths.size();j++){
    //                         timestep ++;
    //                         timesteps ++;
    //                         if (max_timesteps < timestep){
    //                             max_timesteps = timestep;
    //                         }
    //                     }
    //                 }
                    
    //                 new_node_1.cost = timesteps;
    //                 new_node_1.max_steps = max_timesteps;

    //                 // std::cout << "fixed path 1: " << std::endl;
                    
    //                 // for(int j=0;j<new_node_1.Paths[p_i].size();j++){
    //                 //     std::cout << new_node_1.Paths[p_i][j] << " ";
    //                 // }
    //                 // std::cout << " " << std::endl;

    //                 if(new_node_1.Paths[p_i].paths.size() > 1){
    //                     validate(new_node_1);
    //                     if(new_node_1.cost < LB * w)
    //                         open.push(new_node_1);
    //                 }
    //             }
                
    //             if(!isRepeated(new_node_2.V_Constraints,{p_j,pos,t})){
    //                 new_node_2.V_Constraints.push_back({p_j,pos,t});        
    //                 int start = env->curr_states[p_j].location;
    //                 int goal = goal_indexes[p_j];
                    
    //                 new_node_2.Paths[p_j] = my_Astar_constraint(p_j,env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_2.V_Constraints, new_node_2.E_Constraints);

    //                 timesteps = 0;
    //                 max_timesteps = 0;
    //                 timestep = 0;

    //                 for(int i=0;i<new_node_2.Paths.size();i++){
    //                     timestep = 0;
    //                     for(int j=0;j<new_node_2.Paths[i].paths.size();j++){
    //                         timestep ++;
    //                         timesteps ++;   
    //                         if (max_timesteps < timestep){
    //                             max_timesteps = timestep;
    //                         }
    //                     }
    //                 }
                    
    //                 new_node_2.cost = timesteps;
    //                 new_node_2.max_steps = max_timesteps;

    //                 // std::cout << "fixed path 2: " << std::endl;
                    
    //                 // for(int j=0;j<new_node_2.Paths[p_j].size();j++){
    //                 //     std::cout << new_node_2.Paths[p_j][j] << " ";
    //                 // }

    //                 // std::cout << " " << std::endl;

    //                 if(new_node_2.Paths[p_j].paths.size() > 1){
    //                     validate(new_node_2);
    //                     if(new_node_2.cost < LB * w)
    //                         open.push(new_node_2);
    //                 }
    //             }

    //         }
    //         else if (Constrant.size() == 5){
    //             int p_i = Constrant[0];
    //             int p_j = Constrant[1];
    //             int pos1 = Constrant[2];
    //             int pos2 = Constrant[3];
    //             int t = Constrant[4];

    //             if (t > far_timestpes){
    //                 subgoal_node = node;
    //                 far_timestpes = t;
    //                 //std::cout << "fatest timestepst: " << far_timestpes << std::endl;
    //             }
                
    //             // std::cout << "Conflict time and pos: " << std::endl;
    //             // std::cout << "timestime: "<< t << " ";
    //             // std::cout << "pos1: "<< pos1 << " " << "pos2: "<< pos2 << " ";
    //             // std::cout << " " << std::endl;


    //             // std::cout << "Conflict path 1: " << std::endl;
    //             // for(int j=0;j<node.Paths[p_i].size();j++){
    //             //     std::cout << node.Paths[p_i][j] << " ";
    //             // }
    //             // std::cout << " " << std::endl;

    //             // std::cout << "Conflict path 2: " << std::endl;
                
    //             // for(int j=0;j<node.Paths[p_j].size();j++){
    //             //     std::cout << node.Paths[p_j][j] << " ";
    //             // }
    //             // std::cout << " " << std::endl;

    //             if(!isRepeated(new_node_1.E_Constraints,{p_i,pos1,pos2,t})){
    //                 int start = env->curr_states[p_i].location;
    //                 int goal = goal_indexes[p_i];
    //                 new_node_1.E_Constraints.push_back({p_i,pos1,pos2,t});

    //                 new_node_1.Paths[p_i] = my_Astar_constraint(p_i, env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_1.V_Constraints, new_node_1.E_Constraints);
    //                 timesteps = 0;
    //                 max_timesteps = 0;
    //                 timestep = 0;
    //                 for(int i=0;i<new_node_1.Paths.size();i++){
    //                     timestep = 0;
    //                     for(int j=0;j<new_node_1.Paths[i].paths.size();j++){
    //                         timestep ++;
    //                         timesteps ++;
    //                         if (max_timesteps < timestep){
    //                             max_timesteps = timestep;
    //                         }
    //                     }
    //                 }
    //                 new_node_1.cost = timesteps;
    //                 new_node_1.max_steps = max_timesteps;

    //                 // std::cout << "fixed path 1: " << std::endl;
                    
    //                 // for(int j=0;j<new_node_1.Paths[p_i].size();j++){
    //                 //     std::cout << new_node_1.Paths[p_i][j] << " ";
    //                 // }
    //                 // std::cout << " " << std::endl;

    //                 if(new_node_1.Paths[p_i].paths.size() > 1){
    //                     validate(new_node_1);
    //                     if(new_node_1.cost < LB * w)
    //                         open.push(new_node_1);
    //                 }
    //             }

    //             if(!isRepeated(new_node_2.E_Constraints,{p_j,pos2,pos1,t})){
    //                 new_node_2.E_Constraints.push_back({p_j,pos2,pos1,t});        
    //                 int start = env->curr_states[p_j].location;
    //                 int goal = goal_indexes[p_j];

    //                 new_node_2.Paths[p_j] = my_Astar_constraint(p_j,env,start,goal, &global_neighbors, global_heuristictable[goal],
    //                                                     new_node_2.V_Constraints, new_node_2.E_Constraints);

    //                 timesteps = 0;
    //                 max_timesteps = 0;
    //                 timestep = 0;

    //                 for(int i=0;i<new_node_2.Paths.size();i++){
    //                     timestep = 0;
    //                     for(int j=0;j<new_node_2.Paths[i].paths.size();j++){
    //                         timestep ++;
    //                         timesteps ++;   
    //                         if (max_timesteps < timestep){
    //                             max_timesteps = timestep;
    //                         }
    //                     }
    //                 }
                    
    //                 new_node_2.cost = timesteps;
    //                 new_node_2.max_steps = max_timesteps;

    //                 // std::cout << "fixed path 2: " << std::endl;
                    
    //                 // for(int j=0;j<new_node_2.Paths[p_j].size();j++){
    //                 //     std::cout << new_node_2.Paths[p_j][j] << " ";
    //                 // }
    //                 // std::cout << " " << std::endl;

    //                 if(new_node_2.Paths[p_j].paths.size() > 1){
    //                     validate(new_node_2);
    //                     if(new_node_2.cost < LB * w)
    //                         open.push(new_node_2);
    //                 }
    //             }
    //         }
    //         // std::cout << " " << std::endl;
    //         // std::cout << " " << std::endl;
    //         auto now = std::chrono::steady_clock::now();
    //         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
    //         if (elapsed >= my_limit_time) {
    //             break;
    //         }

    //     }


        
    //     auto end = std::chrono::steady_clock::now();
    //     std::chrono::duration<double> elapsed_seconds = end - while_start;
    
    //     std::cout << "high level time cost: " << elapsed_seconds.count() << " s, timestepts: " << goal_node.cost << endl;
        
    //     if (goal_node.num_cof > subgoal_node.num_cof){
    //         goal_node = subgoal_node;
    //         goal_node.far_timestpes = far_timestpes;
    //     //     // std::cout << "timeout, use subgoal: " << std::endl;
    //     //     // std::cout << "fatest timestepst: " << far_timestpes << std::endl;
    //     //     // std::cout << "num of conflict: " << goal_node.num_cof << std::endl;
    //     }

    //     return goal_node;
    // }


    // a_node planner(SharedEnvironment* env){
    //     std::vector<std::vector<int>> Paths;
    //     // int max_timesteps; 
    //     // int timestep, timesteps;
    //     a_node goal_node;
    //     auto while_start = std::chrono::steady_clock::now();



    //     for(int i=0;i<env->num_of_agents;i++){
    //         int start = env->curr_states[i].location;
    //         int goal = goal_indexes[i];
    //             if (pre_goal_node.Paths.size() > 1 && pre_goal_node.Paths[i].size() > 1){
    //                 if(start == pre_goal_node.Paths[i][1]){
    //                     path = std::vector<int>(pre_goal_node.Paths[i].begin() + 1, pre_goal_node.Paths[i].end());
    //                 }
    //                 else if(start == pre_goal_node.Paths[i][0] && pre_goal_node.Paths[i].size() > 1){
    //                     path = pre_goal_node.Paths[i];
    //                 }
    //             }
    //             else{
    //                 path = my_Astar(env,start,goal, &global_neighbors, global_heuristictable[goal]);
    //             }
            
    //         //path = my_Astar(env,start,goal, &global_neighbors, global_heuristictable[goal]);
    //         Paths.push_back(path);
    //     }
    //     MCTSNode* root = new MCTSNode(Paths);
    //     auto now = std::chrono::steady_clock::now();
    //     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
    //     //MCTSNode* node = new MCTSNode(root);

    //     while(elapsed < my_limit_time){
    //         //goal_node.Paths = Selection(root,env)->paths;
    //         Selection(root,env, goal_node);
    //         //std::cout << "node size: " <<  node.children.size() << std::endl;
    //         now = std::chrono::steady_clock::now();
    //         elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - while_start).count();
    //     }


    //     return goal_node;

    // }


    // using PlannerFunc = std::function<a_node(SharedEnvironment*)>;




}