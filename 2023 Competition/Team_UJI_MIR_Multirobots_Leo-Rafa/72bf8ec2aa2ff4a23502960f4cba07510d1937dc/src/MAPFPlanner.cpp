#include <MAPFPlanner.h>
#include <random>


struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
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



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "planner initialize done" << endl;
    // initializes the agent_stuck vector
    agent_stuck = std::vector<int>(env->num_of_agents, 0);
    random_goal_vector = std::vector<int>(env->num_of_agents, 0);
    random_goal_time = std::vector<int>(env->num_of_agents, 0);
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    // List for storing the future calculated locations of the agents
    list<int> agent_future_locations(env->num_of_agents, -1);
    // Array with the current locations of the agents
    vector<int> agent_current_locations(env->num_of_agents, -1);

    // Stuck vector
    // vector<int> stuck(env->num_of_agents, 0);

    // random goal vector
    // vector<int> random_goal_vector(env->num_of_agents, 0);
    // vector<int> random_goal_time(env->num_of_agents, 0);


    // Creates an array with the same shape as env->map
    // and fills it with ceros
    vector<int> occupancy_grid(env->map.size(), 0);



    for (int i = 0; i < env->num_of_agents; i++) 
    {
        agent_current_locations[i] = env->curr_states[i].location;
        occupancy_grid[env->curr_states[i].location] = 1;
    }

    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        occupancy_grid[env->goal_locations[i].front().first] = 0;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else if (agent_stuck[i] >= 8)
        {
            // cout << "Agent " << i << " is stuck" << endl;
            // generate a random goal location within the map different from the 
            // map spaces that have obstacles and other agents
            if (random_goal_vector[i] == 0)
            {
            int random_goal = rand() % env->map.size();
            while (env->map[random_goal] == 1 || 
                   std::find(agent_current_locations.begin(), 
                   agent_current_locations.end(), random_goal) != agent_current_locations.end())
            {
                random_goal = rand() % env->map.size();
            }
            random_goal_vector[i] = random_goal;
            };
            if (random_goal_time[i] == 0)
            {
                // cout << "Agent " << i << " is stuck for the first time with time" << env->curr_timestep << endl;
                random_goal_time[i] = env->curr_timestep;
            }
            // cout << "Agent " << i << " has a been stuck since " << random_goal_time[i] << endl;
            // cout << "Agent " << i << " has being stuck for " << env->curr_timestep-random_goal_time[i] << " timesteps" << endl;
            if (env->curr_timestep-random_goal_time[i] == 8)
            {
                agent_stuck[i] = 0;
                random_goal_time[i] = 0;
                random_goal_vector[i] = 0;
                // cout << "Agent " << i << " is NO MORE stuck" << endl;

            }
            

            // set a new goal 
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    random_goal_vector[i],
                                    occupancy_grid);
        }
        else 
        {
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first,
                                    occupancy_grid);
        }


        // if the location doesn't exist in the list, add it to the list
        // First iteration, it just looks if the first destiny in the path is already busy or not 
        // by another robot in the next timestamp. If it is, it adds a wait action
        bool already_taken = (
            std::find(agent_future_locations.begin(), 
            agent_future_locations.end(), path.front().first) != agent_future_locations.end());
            
        
        // Second iteration, it looks the current position of different robots and checks if one of those
        // is in the path of the current robot. If it is, it adds a CR action        
        bool edge_collision = std::find(std::begin(agent_current_locations), 
                                        std::end(agent_current_locations), 
                                        path.front().first) != std::end(agent_current_locations);

        // Gets the position of path.front().first in the agent_future_locations list
        auto it = std::find(agent_future_locations.begin(), agent_future_locations.end(), path.front().first);
        if (it != agent_future_locations.end())
        {
            int index = std::distance(agent_future_locations.begin(), it);
            if (env->curr_states[i].orientation == env->curr_states[index].orientation)
            {
                edge_collision = false;
            }
        }


        if (path.front().first == env->curr_states[i].location)
        {
            // cout << "Agent " << i << " is stuck" << endl;
            agent_stuck[i] += 1;
            // cout << "Stuck counter " << stuck[i] << endl;
        }



        // if the location is already taken, add a wait action
        // cout << "Agent future locations " << agent_future_locations << endl;
        //cout << "agent " << i  << " Future location: " << path.front().first << endl;
        //cout << "Future X " << path.front().first/env->cols << ", Future Y " << path.front().first%env->cols << endl;
        //cout << "Current location " << env->curr_states[i].location << endl;
        //cout << "Coordinates X " << env->curr_states[i].location/env->cols << ", Coordinates Y " << env->curr_states[i].location%env->cols << endl;
        //cout << "Already taken flag " << already_taken << endl;

        // if (path.front().first == 0)
        // {
        //     cout << "Ooopsie, there's a Cero in agent " << i << endl;
        // }

        if (already_taken || path.front().first == 0) 
        {
            actions[i] = Action::W;
            // pass to the next agent
            continue;
        } 
        else 
        {
            agent_future_locations.emplace_front(path.front().first);
        }

        if (edge_collision)
        {
            actions[i] = Action::CR;
            continue;
        }

        if (path.front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
            if (agent_stuck[i] < 8)
            {
                agent_stuck[i] = 0;
            }
        } 
        else if (path.front().second!= env->curr_states[i].orientation)
        {
            int incr = path.front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C-- clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR-- counter clockwise rotate
            } 
        }

    }
  return;
}


list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end, vector<int> occupancy_grid)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, occupancy_grid, start, start_direct);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int loc, int loc2, vector<int> occupancy_grid, int start, int start_direct)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    int start_x = start/env->cols;
    int start_y = loc%env->cols;

    // prints the value of the occupancy grid
    bool close_col = false;

    int search_area = 5;

    // if ( loc_y == start_y && loc_x <= start_x + search_area && loc_x >= start_x - search_area )
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }
    // else if ( loc_x == start_x && loc_y <= start_y + search_area && loc_y >= start_y - search_area)
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }

    

    // if (start_direct == 0 && loc_y == start_y && loc_x <= start_x + search_area && loc_x >= start_x - search_area )
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }
    // else if (start_direct == 1 && loc_x == start_x && loc_y <= start_y + search_area && loc_y >= start_y - search_area)
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }
    // else if (start_direct == 2 && loc_y == start_y && loc_x >= start_x - search_area && loc_x <= start_x + search_area)
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }
    // else if (start_direct == 3 && loc_x == start_x && loc_y >= start_y - search_area && loc_y <= start_y + search_area)
    // {
    //     close_col = (occupancy_grid[loc] == 1) ? true : false;
    // }
    
    


    // Adds the position of the other robots as obstacles to the occupancy grid
    // if they're in a search_are x search_are of the current robot
    if (loc_x >= start_x - search_area 
        && loc_x <= start_x + search_area 
        && loc_y >= start_y - search_area 
        && loc_y <= start_y + search_area ) 
        close_col = (occupancy_grid[loc] == 1) ? true : false; 

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1 || close_col)
        return false;


    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction, vector<int> occupancy_grid, int start, int start_direct)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location,occupancy_grid, start, start_direct))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
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
    return neighbors;
}
