#include <MAPFPlanner.h>
#include <random>
#include <iostream>
#include <stdexcept>

#define PLAN_DIST 5
#define PLAN_KEEP 3
#define STARTING_DELTA 5

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    
    cout << "rows " << env->rows << ", cols " << env->cols << endl;

    map.init(env->map, env->rows, env->cols);

    // print the map
    // cout << "printing map" << endl;
    // int zeroes = 0;
    // for (int i = 0; i < env->rows; i++){
    //     for (int j = 0; j < env->cols; j++){
    //         cout << env->map[i*env->cols + j];
    //         if (env->map[i*env->cols + j] == 0){
    //             zeroes += 1;
    //         }
    //     }
    //     cout << endl;
    // }
    // cout << "zeroes: " << zeroes << endl;
    cout << "init done" << endl;

    // map.bfs_all_nodes();
    // std::cout << "bfs done" << std::endl;
    cout << "planner initialize done" << endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // loop over the agents. Do a bfs on the goal of each agent
    for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++){
        directional_pos goal = get_goal(agent_id);
        map.bfs(goal);
    }

    if (first_solve){
        solve();
        first_solve = false;
    }
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    get_next_step(actions);
    std::cout << last_plan_time_ << std::endl;
    return;
}

directional_pos MAPFPlanner::get_start(int agent_id){
    return std::pair(env->curr_states[agent_id].location, Orientation(env->curr_states[agent_id].orientation));
}

directional_pos MAPFPlanner::get_goal(int agent_id){
    // Orientation best = Orientation(0);
    // directional_pos start = get_start(agent_id);
    // int min_dist = std::numeric_limits<int>::max();

    // for (int i = 0; i<4 ; i++){
    //     directional_pos pos = std::pair(env->goal_locations[agent_id][0].first, Orientation(i));
    //     int dist = map.distance_between(start, pos);
    //     if (dist < min_dist){
    //         min_dist = dist;
    //         best = Orientation(i);
    //     }
    // }

    // return std::pair(env->goal_locations[agent_id][0].first, best);
    return std::pair(env->goal_locations[agent_id][0].first, Orientation(0));
}

int MAPFPlanner::get_horizon(int agent_id){

    directional_pos start = get_start(agent_id);
    directional_pos goal = get_goal(agent_id);
    return map.distance_between(goal, start);
}

void MAPFPlanner::add_instance(Clingo::Backend& bck, unsigned int delta){
    
    std::unordered_set<Node*> vertices;
    std::unordered_map <unsigned int, std::unordered_set<Node*> > agent_vertices;
    std::unordered_map<Node*,std::unordered_set<Node*>> edges;

    for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++){
        directional_pos start = get_start(agent_id);
        directional_pos goal = get_goal(agent_id);

        // add start, goal and agent facts
        auto start_xy = map.deserialize_2d(start.first);
        auto xy_func = Clingo::Function("", {Clingo::Number(start_xy.first), Clingo::Number(start_xy.second)});
        auto dir_start = Clingo::Function("", {xy_func, Clingo::Number(start.second)});

        auto start_atm = bck.add_atom(Clingo::Function("start", {Clingo::Number(agent_id), dir_start}));

        // goal
        auto goal_xy = map.deserialize_2d(start.first);
        auto gxy_func = Clingo::Function("", {Clingo::Number(goal_xy.first), Clingo::Number(goal_xy.second)});
        auto dir_goal = Clingo::Function("", {gxy_func, Clingo::Number(start.second)});

        auto goal_atm = bck.add_atom(Clingo::Function("goal", {Clingo::Number(agent_id), dir_goal}));

        // agent
        auto agent_atm = bck.add_atom(Clingo::Function("agent", {Clingo::Number(agent_id)}));

        // add facts
        bck.rule(false, {start_atm}, {});
        bck.rule(false, {goal_atm}, {});
        bck.rule(false, {agent_atm}, {});

        // Add reach atoms for each agent
        auto reach = map.reachable_nodes_within_distance(PLAN_DIST, get_horizon(agent_id)+delta, start, goal);
        for (int time = 0 ; time < reach.size() ; time++){
            for (Node* node : reach[time]){
                agent_vertices[agent_id].insert(node);

                auto vertex = map.deserialize_2d(node->name.first);
                auto xy_reach_func = Clingo::Function("", {Clingo::Number(vertex.first), Clingo::Number(vertex.second)});
                auto dir_reach = Clingo::Function("", {xy_reach_func, Clingo::Number(node->name.second)});
                
                // reach atom
                auto atm = bck.add_atom(Clingo::Function("reach", {Clingo::Number(agent_id), dir_reach, Clingo::Number(time)}));
                bck.rule(false, {atm}, {});

                vertices.insert(node);
                std::vector<Node*> neighbors = node->out;
                neighbors.push_back(node);
                for (Node* neighbor : neighbors){
                    // if we are not in the last step and the neighbor is reachable in the next step
                    if (time < reach.size() - 1 &&  reach[time+1].count(neighbor) == 1){
                        if (edges.count(node) == 0){
                            edges[node] = std::unordered_set<Node*>();
                        }
                        edges[node].insert(neighbor);
                    }
                }
            }
        }
    }
    // now we add atoms for node and edge
    for (Node* node : vertices){
        auto vertex = map.deserialize_2d(node->name.first);
        
        auto xy_func = Clingo::Function("", {Clingo::Number(vertex.first), Clingo::Number(vertex.second)});
        auto dir_vertex = Clingo::Function("", {xy_func, Clingo::Number(node->name.second)});


        auto atm = bck.add_atom(Clingo::Function("vertex", {dir_vertex}));
        bck.rule(false, {atm}, {});
    }

    for ( auto& [agent_id, vertices] : agent_vertices){
        for (Node* node : vertices){
            auto vertex = map.deserialize_2d(node->name.first);
            
            auto xy_func = Clingo::Function("", {Clingo::Number(vertex.first), Clingo::Number(vertex.second)});
            auto dir_vertex = Clingo::Function("", {xy_func, Clingo::Number(node->name.second)});

            unsigned int dist = map.distance_between(get_goal(agent_id), node->name);

            auto atm = bck.add_atom(Clingo::Function("dist", {Clingo::Number(agent_id),dir_vertex, Clingo::Number(dist)}));
            bck.rule(false, {atm}, {});
        }
    }
    
    for (auto& [nodefrom, nodeto_vec] : edges){
        auto vertex_from = map.deserialize_2d(nodefrom->name.first);
        auto xy_func_from = Clingo::Function("", {Clingo::Number(vertex_from.first), Clingo::Number(vertex_from.second)});
        auto dir_func_from = Clingo::Function("", {xy_func_from, Clingo::Number(nodefrom->name.second)});

        for ( auto& nodeto : nodeto_vec){
            auto vertex_to = map.deserialize_2d(nodeto->name.first);
            auto xy_func_to = Clingo::Function("", {Clingo::Number(vertex_to.first), Clingo::Number(vertex_to.second)});
            auto dir_func_to = Clingo::Function("", {xy_func_to, Clingo::Number(nodeto->name.second)});

            auto atm = bck.add_atom(Clingo::Function("edge", {dir_func_from, dir_func_to}));
            bck.rule(false, {atm}, {});
        }
    }
}

void MAPFPlanner::solve(){

    std::unordered_map< unsigned int, std::vector<directional_pos> > positions;

    for (int i = 0; i < PLAN_DIST+1; i++){
        positions[i] = std::vector<directional_pos>(env->curr_states.size(), std::pair(0,Orientation(0)));
    }

    bool model_found = false;
    unsigned int delta = STARTING_DELTA;

    while (!model_found){
        Clingo::Control ctl{{"--opt-strategy=usc"}};
        Clingo::Backend bck = ctl.backend();

        add_instance(bck, delta);

        // add the encoding
        ctl.load("encodings/paths-turns-soc-fixed.lp");

        ctl.ground({{"base", {}}});
        for(auto& m : ctl.solve()){
            std::cout << "model found! Final delta is: " << delta << endl;
            // set to true to get out of the loop
            model_found = true;

            for(auto& atom : m.symbols()){
                // std::cout << atom << " ";
                if ( atom.match("at", 3) ){
                    // could only get the rest if time == 1 (the first move)
                    int time = atom.arguments()[2].number();
                    int agent_id = atom.arguments()[0].number();
                    int x = atom.arguments()[1].arguments()[0].arguments()[0].number();
                    int y = atom.arguments()[1].arguments()[0].arguments()[1].number();
                    int dir = atom.arguments()[1].arguments()[1].number();

                    positions[time][agent_id] =  std::make_pair(map.serialize_2d(x,y), Orientation(dir));                    
                    
                }
            }
        }
        // std::cout << endl;
        delta += 1;

        // stats
        total_calls_ += 1;
        total_time_ += ctl.statistics()["summary"]["times"]["total"];
        total_solve_time_ += ctl.statistics()["summary"]["times"]["solve"];
        total_choices_ += ctl.statistics()["solving"]["solvers"]["choices"];
        total_conflicts_ += ctl.statistics()["solving"]["solvers"]["conflicts"];
    }

    // populate the moves vector
    for (int time = 0; time < PLAN_KEEP; time++){
        std::vector<Action> actions = std::vector<Action>(env->curr_states.size(), Action::W);
        for (int agent_id = 0; agent_id < env->curr_states.size(); agent_id++){
            actions[agent_id] = convert_action(positions[time][agent_id], positions[time+1][agent_id]);
        }
        computed_plans_.push_back(actions);
    }

    std::cout << "total calls: " << total_calls_ << std::endl;
    std::cout << "total time: " << total_time_ << std::endl;
    std::cout << "total solve time: " << total_solve_time_ << std::endl;
    std::cout << "total choices: " << total_choices_ << std::endl;
    std::cout << "total conflicts: " << total_conflicts_ << std::endl;

}

void MAPFPlanner::get_next_step(std::vector<Action> & actions){

    if (last_plan_time_ > computed_plans_.size()-1){
        cout << "solving" << endl;
        solve();
    }

    actions = computed_plans_[last_plan_time_];
    last_plan_time_ += 1;

}

Action MAPFPlanner::convert_action(directional_pos prev, directional_pos curr){
    // could be done with some hash map to make it more efficient and clear
    // but this was easier to write for me 

    if (prev == curr){
        return Action::W;
    }
    // if the position is not the same but direction is the same, it went forward
    Orientation prevdir = prev.second;
    Orientation currdir = curr.second;

    if (prevdir == currdir){
        return Action::FW;
    }

    if (prevdir < currdir && prevdir + 1 == currdir){
        return Action::CR;
    }
    // if curr is highest but difference is not 1 then it went from 0(right) to 3(up)
    if (prevdir == 0 && currdir == 3){
        return Action::CCR;
    }

    if (prevdir > currdir && currdir + 1 == prevdir){
        return Action::CCR;
    }

    if (prevdir == 3 && currdir == 0){
        return Action::CR;
    }

    // if nothing works, raise an error
    std::cout << "ERROR: could not convert directions" << (int)prevdir << (int)currdir << std::endl;
    // throw an error
    std::runtime_error("could not convert  directions");
    
}
