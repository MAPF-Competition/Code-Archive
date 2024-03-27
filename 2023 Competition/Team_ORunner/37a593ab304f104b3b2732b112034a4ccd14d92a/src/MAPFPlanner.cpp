#include <MAPFPlanner.h>
#include <ActionModel.h>
#include <random>
#include <boost/unordered_map.hpp>
#include <chrono>

struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};

void MAPFPlanner::printPath(list<pair<int,int>> path)
{
    std::list<pair<int,int>>::iterator it = path.begin();
    int step_id = 1;
    for (it = path.begin(); it != path.end(); ++it){
        std::cout << step_id + env->curr_timestep <<" " <<it->first/env->cols << " " << it->first%env->cols << " " << it->second<< " " << it->first<<std::endl;
        step_id += 1;
    }
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2, int direction)
{
    int loc1_y = loc1/env->cols;
    int loc1_x = loc1%env->cols;
    int loc2_y = loc2/env->cols;
    int loc2_x = loc2%env->cols;
    int additional_costs = 0;
    if (loc1_x != loc2_x && loc1_y != loc2_y){
        additional_costs += 1;
    }
    //     if (loc2_x > loc1_x && loc2_y < loc1_y)
    //     {
    //         if (direction == 1 || direction == 2)
    //         {
    //             additional_costs += 1;
    //         }
    //     }else if(loc2_x > loc1_x && loc2_y > loc1_y)
    //     {
    //         if (direction == 2 || direction == 3)
    //         {
    //             additional_costs += 1;
    //         }
    //     }else if(loc2_x < loc1_x && loc2_y > loc1_y)
    //     {
    //         if (direction == 3 || direction == 0)
    //         {
    //             additional_costs += 1;
    //         }
    //     }else if(loc2_x < loc1_x && loc2_y < loc1_y)
    //     {
    //         if (direction == 0 || direction == 1)
    //         {
    //             additional_costs += 1;
    //         }
    //     }

    // }else if (loc1_x != loc2_x)
    // {
    //     if (loc1_x > loc2_x)
    //     {
    //         if (direction != 0)
    //         {
    //             additional_costs = 1;
    //         }else if (direction == 2)
    //         {
    //             additional_costs = 2;
    //         }
    //     }

    //     if (loc1_x < loc2_x)
    //     {
    //         if (direction != 2)
    //         {
    //             additional_costs = 1;
    //         }else if (direction == 0)
    //         {
    //             additional_costs = 2;
    //         }
    //     }
        
    // }else if (loc1_y != loc2_y)
    // {
    //     if (loc1_y > loc2_y)
    //     {
    //         if (direction != 3)
    //         {
    //             additional_costs = 1;
    //         }else if (direction == 1)
    //         {
    //             additional_costs = 2;
    //         }
    //     }

    //     if (loc1_y < loc2_y)
    //     {
    //         if (direction != 1)
    //         {
    //             additional_costs = 1;
    //         }else if (direction == 3)
    //         {
    //             additional_costs = 2;
    //         }
    //     }
    // }

    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y) + additional_costs;
}

void MAPFPlanner::printPositionFromLocation(int loc)
{
    std::cout<<"Y-axis: " << loc/env->cols << " X-axis: " << loc%env->cols <<std::endl;
}


bool MAPFPlanner::validateMove(int loc, int loc2, int curt)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1 || all_vertex_occupied_vector[curt].find(loc) != all_vertex_occupied_vector[curt].end() || (all_edge_occupied_vector[curt].find({loc2, loc}) != all_edge_occupied_vector[curt].end()))
        return false;   

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;
}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction, int curt, int agent_idx)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location, curt)){
        neighbors.emplace_back(make_pair(forward,new_direction));
    }
    bool stay_possible = false;

    if (all_vertex_occupied_vector[curt].count(location) == 0){
        stay_possible = true;
    }else if ((all_vertex_occupied_vector[curt].at(location) == agent_idx)){
        stay_possible = true;
    }


    if (stay_possible){
        // turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.emplace_back(make_pair(location,new_direction));
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.emplace_back(make_pair(location,new_direction));
        neighbors.emplace_back(make_pair(location,direction)); //wait
    }

    return neighbors;
}

void MAPFPlanner::deletePath(int agent_idx, int num_epochs, int path_block_limit)
{
    int prevLocation = env->curr_states[agent_idx].location;
    int t = env->curr_timestep + 1;
    int maxIteration = std::min<int>(vectorOfPaths[agent_idx].size(), num_epochs - env->curr_timestep-1);
    //std::cout<<"Hello 3"<<std::endl;
    for (int j = 0; j < maxIteration; j++){
        std::list<pair<int,int>>::iterator it2 = vectorOfPaths[agent_idx].begin();
        std::advance(it2, j); // Advance the iterator to the element at index j
        all_vertex_occupied_vector[t].erase(it2->first);
        all_edge_occupied_vector[t].erase({it2->first, prevLocation});
        std::advance(it2, j); // Advance the iterator to the element at index j
        prevLocation = it2->first;
        t += 1;
    }
    //std::cout<<"Hello 4"<<std::endl;

    for (int k = env->curr_timestep + maxIteration + 1; k < path_block_limit; k++)
    {
        all_vertex_occupied_vector[k].erase(vectorOfPaths[agent_idx].back().first);
    }
}

////////////////////// New ////////////////////

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    path_length_factor = 100;
    time_limit_milliseconds = 940;
    if (env->num_of_agents < 100)
    {
        time_limit_milliseconds = 980;
        path_length_factor = 3000;
    }else if (env->num_of_agents < 500)
    {
        time_limit_milliseconds = 980;
        path_length_factor = 1500;
    }else if (env->num_of_agents < 2000)
    {
        time_limit_milliseconds = 980;
        path_length_factor = 500;
    }else if (env->num_of_agents < 4000)
    {
        time_limit_milliseconds = 980;
        path_length_factor = 200;
    }else if (env->num_of_agents < 8000)
    {
        time_limit_milliseconds = 950;
        path_length_factor = 150;
    }
    

    minimum_iteration_limit_astar = 50;
    minimum_path_length = 20;
    time_steps = 5001;
    vectorOfPaths.resize(0);
    for (int j = 0; j < env->num_of_agents; j++){
        list<pair<int,int>> path_of_agent;
        vectorOfPaths.push_back(path_of_agent);
    }

    all_vertex_occupied_vector.resize(0);
    all_edge_occupied_vector.resize(0);
    for (int i = 0; i < time_steps; i++) {
        boost::unordered_map<int, int> vertex_occupied;
        unordered_map<pair<int, int>, int> edge_occupied;
        // Add the initialized maps to the vectors
        all_vertex_occupied_vector.push_back(vertex_occupied);
        all_edge_occupied_vector.push_back(edge_occupied);
    }
    cout << "planner initialize done" << endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int max_path_size = std::min<int>(path_length_factor, time_steps-env->curr_timestep);
    max_path_size = std::max<int>(minimum_path_length, max_path_size);
    int path_block_limit = std::min<int>(env->curr_timestep+max_path_size, time_steps);
    int iteration_limit_astar = std::max<int>(1.25*max_path_size, minimum_iteration_limit_astar);
    std::cout<<"Iteration limit: "<<iteration_limit_astar<<" Path limit: "<<path_length_factor<<std::endl;

    actions = std::vector<Action>(env->curr_states.size(), Action::W);

    // In the beginning, since we dont know when a vertex becomes availables, each currently occupied vertex is blocked until the end of time 
    std::vector<std::tuple<int, int, int, int>> heuristic_distance_to_goal;
    for (int j = 0; j < env->num_of_agents; j++)  
    {
        int new_route_needed = 0;
        if (vectorOfPaths[j].size() < 1)
        {
            new_route_needed = 1;
            for (int k = env->curr_timestep; k < path_block_limit; k++)
            {
                if (all_vertex_occupied_vector[k].count(env->curr_states[j].location) == 0)
                {
                    all_vertex_occupied_vector[k][env->curr_states[j].location] = j;
                }
            }   
        }
        //actions[i] = giveAction(i);
        heuristic_distance_to_goal.push_back(std::make_tuple(j, getManhattanDistance(env->curr_states[j].location, env->goal_locations[j].front().first, env->curr_states[j].orientation),0, new_route_needed));  
    }

    auto end_time2 = std::chrono::high_resolution_clock::now();
    std::cout<<"Time 1: "<<((end_time2 - start_time)/std::chrono::milliseconds(1))<<std::endl;

    // Sort distances increasing
    sort(heuristic_distance_to_goal.begin(), heuristic_distance_to_goal.end(), [=](std::tuple<int, int, int, int>& a, std::tuple<int, int, int, int>& b){return std::get<1>(a) > std::get<1>(b);});
    int agent_counter = 0;
    
    
    while (!heuristic_distance_to_goal.empty()){
        agent_counter += 1;
        std::tuple<int, int, int, int> agent = heuristic_distance_to_goal.back();
        heuristic_distance_to_goal.pop_back();
        int i = std::get<0>(agent);
        list<pair<int,int>> path_list;
        list<pair<int,int>> path_empty;
        int iterations_needed;
        std::pair<std::list<pair<int,int>>,int> path;
        if (env->goal_locations[i].empty()) 
        {
            path_list = {std::make_pair(env->curr_states[i].location, env->curr_states[i].orientation)};
            vectorOfPaths[i] = path_list;
        } 
        else if (vectorOfPaths[i].size() < 1 || vectorOfPaths[i].back().first != env->goal_locations[i].front().first || std::get<2>(agent) == 1) 
        {   
            deletePath(i, time_steps, path_block_limit);
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first,
                                    iteration_limit_astar,
                                    max_path_size,
                                    std::get<1>(agent),
                                    i);        
      

            
            if (path.first.size()< 1)
            {
                deletePath(i, time_steps, path_block_limit);
                actions[i] = Action::W;
                vectorOfPaths[i] = {std::make_pair(env->curr_states[i].location, env->curr_states[i].orientation)};
                path_list = vectorOfPaths[i];
                continue;
            }else{
                vectorOfPaths[i] = path.first;
                path_list = path.first;
            }


        }else
        {
            path_list = vectorOfPaths[i];
        }
  
        actions[i] = giveAction(i);
        int maxIteration = std::min<int>(path_list.size(), time_steps-env->curr_timestep-1);
        // if we calculated a new route:
        if (std::get<3>(agent) == 1)
        {
            // We do not block that current vertex anymore.
            for (int k = env->curr_timestep; k < path_block_limit; k++)
            {
                if (all_vertex_occupied_vector[k].count(env->curr_states[i].location) != 0)
                {
                    if (all_vertex_occupied_vector[k][env->curr_states[i].location] == i)
                    {
                        all_vertex_occupied_vector[k].erase(env->curr_states[i].location);
                    }
                }
                
                // block the goal until the end if we reach it(for now)
                if (std::get<1>(agent)+env->curr_timestep < time_steps)
                {
                    if (k+maxIteration < time_steps)
                    {
                        if (all_vertex_occupied_vector[k+maxIteration].count(vectorOfPaths[i].back().first) == 0)
                        {
                            all_vertex_occupied_vector[k+maxIteration][vectorOfPaths[i].back().first] = i;
                        }
                    }
                }
            }
        }
        
        // Add the path to the two vectors 
        int prevLocation = env->curr_states[i].location;
        int t = 1 + env->curr_timestep;

        for (int j = 0; j < maxIteration; j++){
            std::list<pair<int,int>>::iterator it2 = path_list.begin();
            std::advance(it2, j); // Advance the iterator to the element at index j
            all_vertex_occupied_vector[t][it2->first] = i;
            all_edge_occupied_vector[t][{it2->first, prevLocation}] = i;
            prevLocation = it2->first;
            t += 1;
        }


        if (vectorOfPaths[i].size() > 0)
        {
            vectorOfPaths[i].pop_front();
        }   
        auto end_time = std::chrono::high_resolution_clock::now();
        if (((end_time - start_time)/std::chrono::milliseconds(1)) > time_limit_milliseconds){
            break;
        }
    }
    
    std::cout<<"Number of agents: " << agent_counter<<" "<<env->curr_timestep<<std::endl;
    if (agent_counter < env->num_of_agents)
    {
        path_length_factor -= 2;
    }else
    {
        path_length_factor += 2;
    }
    // Check for robots with errors and let all robots causing errors wait for now
    auto start_time2 = std::chrono::high_resolution_clock::now();

    int error_size = 1;
    while (error_size > 0)
    {
        std::vector<int> robots_with_errors = is_valid(env->curr_states, actions);
        std::cout<<"Error size: "<<error_size<<std::endl;
        error_size = robots_with_errors.size();
        for (auto & robot : robots_with_errors)
        {
            actions[robot] = Action::W;
            deletePath(robot, time_steps, path_block_limit);
            vectorOfPaths[robot] = {};
        }
    }
    auto end_time3 = std::chrono::high_resolution_clock::now();
    std::cout<<"Time 2: "<<((end_time3 - start_time2)/std::chrono::milliseconds(1))<<std::endl;
    

  return;
}


pair<list<pair<int,int>>, int> MAPFPlanner::single_agent_plan(int start,int start_direct,int end, int iterationLimit, int max_path_size, int start_manhattan_distance, int agent_idx)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<std::tuple<int, int>, AstarNode*> all_nodes;
    unordered_set<std::tuple<int, int>> close_list;
    unordered_map<int, int> vertex_occupied_empty;
    AstarNode* s = new AstarNode(start, start_direct, 0, start_manhattan_distance, 0, nullptr);
    AstarNode* bestNode = s;
    open_list.push(s);
    all_nodes[std::make_tuple(start*4 + start_direct, s->t)] = s;
    int vector_size = all_vertex_occupied_vector.size();
    
    int curr_iteration = 0;

    while (!open_list.empty())
    {
        curr_iteration += 1;
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(std::make_tuple(curr->location*4 + curr->direction, curr->t));
 
        // If we reached the goal or iteration limit, we leave the loop. This can be the case, as we included the possibility to wait.
        if (curr->location == end || curr->t > max_path_size || curr->t >= vector_size-env->curr_timestep-1 || curr_iteration > iterationLimit)
        {
            // if (curr->location != end){
            //     AstarNode* curr = bestNode;
            // }
            
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }

        int epoch = curr->t + env->curr_timestep + 1;
        list<pair<int,int>> neighbors;
        neighbors = getNeighbors(curr->location, curr->direction, epoch, agent_idx);

        if (neighbors.size() > 0)
        {
            for (const pair<int,int>& neighbor: neighbors)
            {
                if (close_list.find(std::make_tuple(neighbor.first*4 + neighbor.second, epoch)) != close_list.end())
                    continue;
                if (all_nodes.find(std::make_tuple(neighbor.first*4 + neighbor.second, epoch)) != all_nodes.end())
                {
                    AstarNode* old = all_nodes[std::make_tuple(neighbor.first*4 + neighbor.second, epoch)];
                    if ((curr->g + 1 < old->g))
                    {
                        old->g = curr->g+1;
                        old->f = old->h+old->g;
                        old->parent = curr;
                    }
                }
                else
                {
                    AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second, curr->g+1,getManhattanDistance(neighbor.first, end, neighbor.second), curr->t+1, curr);
                    if (next_node->h < bestNode->h){
                        AstarNode* bestNode = next_node;
                    }
                    open_list.push(next_node);
                    all_nodes[std::make_tuple(neighbor.first*4+neighbor.second, epoch)] = next_node;
                }
            }
        }
    }

    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return make_pair(path, curr_iteration);
}

std::vector<int> MAPFPlanner::is_valid(const vector<State>& prev, const vector<Action> & actions)
{
    vector<State> next = result_states(prev, actions, env->cols);
    unordered_map<int, int> vertex_occupied;
    unordered_map<pair<int, int>, int> edge_occupied;
    int cols = env->cols;
    int rows = env->rows;
    std::vector<int> robots_with_errors;

    for (int i = 0; i < prev.size(); i ++) 
    {
        
        if (next[i].location < 0 || next[i].location >= env->map.size() || 
            (abs(next[i].location / cols - prev[i].location/cols) + abs(next[i].location % cols - prev[i].location %cols) > 1 ))
        {
            //cout << "ERROR: agent " << i << " moves out of map size. " << endl;
            robots_with_errors.push_back(i);
        }
        if (env->map[next[i].location] == 1) {
            //cout << "ERROR: agent " << i << " moves to an obstacle. " << endl;
            robots_with_errors.push_back(i);
        }

        if (vertex_occupied.find(next[i].location) != vertex_occupied.end()) {
            //cout << "ERROR: agents " << i << " and " << vertex_occupied[next[i].location] << " have a vertex conflict. " << endl;
            robots_with_errors.push_back(i);
            robots_with_errors.push_back(vertex_occupied[next[i].location]);
        }

        int edge_idx = (prev[i].location + 1) * rows * cols +  next[i].location;

        if (edge_occupied.find({prev[i].location, next[i].location}) != edge_occupied.end()) {
            //cout << "ERROR: agents " << i << " and " << edge_occupied[{prev[i].location, next[i].location}] << " have an edge conflict. " << endl;
            robots_with_errors.push_back(i);
            robots_with_errors.push_back(edge_occupied[{prev[i].location, next[i].location}]);
        }
        

        vertex_occupied[next[i].location] = i;
        int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
        edge_occupied[{next[i].location, prev[i].location}] = i;
    }

    return robots_with_errors;
}

Action MAPFPlanner::giveAction(int agent_index){
    
    if (vectorOfPaths[agent_index].size() > 0)
    {
        if (vectorOfPaths[agent_index].front().first != env->curr_states[agent_index].location)
            {
                return Action::FW; //forward action
            } 
            else if (vectorOfPaths[agent_index].front().second!= env->curr_states[agent_index].orientation)
            {
                int incr = vectorOfPaths[agent_index].front().second - env->curr_states[agent_index].orientation;
                if (incr == 1 || incr == -3)
                {
                    return Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    return Action::CCR; //CCR--clockwise rotate
                } 
            }
    }

    return Action::W;
}
