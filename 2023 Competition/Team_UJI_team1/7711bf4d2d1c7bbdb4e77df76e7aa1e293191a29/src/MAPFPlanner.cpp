#include <MAPFPlanner.h>
#include <random>

vector<int> occupymap_tp1;


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
}



// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // bool occupancy_map_tp1[env->cols][env->rows] = {};
    
    pair<int,int> occupancy_map_tp1[env->num_of_agents];
    pair<int,int> occupancy_map_t[env->num_of_agents];

    actions = std::vector<Action>(env->curr_states.size(), Action::W);

    // occupy_map_tp1.clear();
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first);
        }
        // update occupancy map
        int x_tp1 = (path.front().first)/(env->cols);
        int y_tp1 = (path.front().first)%(env->cols);
        // occupancy_map_tp1[x_tp1][y_tp1] = true;
        occupancy_map_tp1[i].first = x_tp1;
        occupancy_map_tp1[i].second = y_tp1;

        int _pos = env->curr_states[i].location;
        int x_t = (env->curr_states[i].location)/(env->cols);
        int y_t = (env->curr_states[i].location)%(env->cols);
        occupancy_map_t[i].first = x_t;
        occupancy_map_t[i].second = y_t;
        
        if (occupymap_tp1.size() < env->num_of_agents)
            occupymap_tp1.push_back(_pos);
        else
            occupymap_tp1[i] = _pos;
    }


    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first);
        }
        if (path.front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
            for (int j = 0; j < env->num_of_agents; j++)
            {
                if (occupancy_map_tp1[i].first == occupancy_map_tp1[j].first && 
                occupancy_map_tp1[i].second == occupancy_map_tp1[j].second  && 
                i != j)
                {
                    actions[i] = Action::W; //wait action
                }
            }
            for (int j = 0; j < env->num_of_agents; j++)
            {
                if (occupancy_map_tp1[i].first == occupancy_map_t[j].first && 
                occupancy_map_tp1[i].second == occupancy_map_t[j].second  && 
                i != j)
                {
                    actions[i] = Action::W; //wait action
                }
            }
        } 
        else if (path.front().second!= env->curr_states[i].orientation)
        {
            int incr = path.front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }

    }
    return;
}


list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
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
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
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

bool MAPFPlanner::validateCollision(int loc, int loc2)
{
    // for (int i = 0; i < occupy_map_t.size(); i++)
    // {   
    //     if (occupy_map_t[i] != loc2 && occupy_map_t[i] == loc)
    //     {
    //         return false;
    //     }
    // }
    // for (int i = 0; i < occupy_map_tp1.size(); i++)
    // {
    //     if (occupy_map_t[i] != loc2 && occupy_map_tp1[i] == loc)
    //         return false;
    // }

    for (int i = 0; i < occupymap_tp1.size(); i++)
    {
        if (occupymap_tp1[i] != loc2 && occupymap_tp1[i] == loc)
            return false;
    }
    return true;

}

bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location) && validateCollision(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn rightconst pair<int,int>& neighbor: neighbors)
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}
