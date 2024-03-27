#include <MAPFPlanner.h>
#include <random>
using namespace std::chrono;

struct AstarNode
{
    int location;
    int direction;
    int f, g, h;
    AstarNode *parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location, int _direction, int _g, int _h, AstarNode *_parent) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent) {}
    AstarNode(int _location, int _direction, int _g, int _h, int _t, AstarNode *_parent) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), t(_t), parent(_parent) {}
};

struct cmp
{
    bool operator()(AstarNode *a, AstarNode *b)
    {
        if (a->f == b->f)
            return a->g <= b->g;
        else
            return a->f > b->f;
    }
};

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "planner initialize done" << endl;
    old_paths.clear();
    shift_id = 0;
    max_time_limit = 980000;
    if (max(env->cols, env->rows) > 200)
    {
        max_time_limit = 950000;
    }
    if(max(env->cols, env->rows) > 400){
        max_time_limit = 920000;
    }
    // if (min(env->cols, env->rows) > 400)
    // {
    //     max_time_limit = 850000;
    // }
    map = vector<std::set<pair<int, int>>>(env->map.size(), std::set<pair<int, int>>());
    closed_list = vector<std::set<int>>(env->map.size() * 4, std::set<int>());
    for (int i = 0; i < 4; ++i)
    {
        blocked_edges[i] = vector<std::set<int>>(env->map.size(), std::set<int>());
    }
}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit, vector<Action> &actions)
{
    start_plan_time = high_resolution_clock::now();
    if (old_paths.size() == 0)
    {
        old_paths = vector<list<pair<pair<int, int>, pair<int, int>>>>(env->num_of_agents, list<pair<pair<int, int>,pair<int, int>>>());
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            ++env->map[env->curr_states[i].location];
        }
    }
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    // if(env->curr_timestep > 100)return;
    int tmp_shift_id = shift_id;
    int id = 0;
    for (auto &it : old_paths)
    {
        // if(it.size()>0){
        //     assert(it.front().first.first == env->curr_states[id].location);
        //     assert(it.front().first.second == env->curr_states[id].orientation);
        //     assert(it.front().second.first == env->curr_timestep);
        // }
        if (it.size() > 1 || (it.size()>0 && it.front().second.first<it.front().second.second))
        {
            pair<int, int> tmp;
            if(it.front().second.first<it.front().second.second){
                tmp = it.front().first;
            }
            else{
                tmp = (++it.begin())->first;
            }
            // assert(env->map[path.back().first] == 1);
            if (tmp.first != env->curr_states[id].location)
            {
                actions[id] = Action::FW; // forward action
            }
            else if (tmp.second != env->curr_states[id].orientation)
            {
                int incr = tmp.second - env->curr_states[id].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[id] = Action::CR; // C--counter clockwise rotate
                }
                else if (incr == -1 || incr == 3)
                {
                    actions[id] = Action::CCR; // CCR--clockwise rotate
                }
            }
        }
        ++id;
    }
    for (int ii = 0; ii < env->num_of_agents; ii++)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = (ii+tmp_shift_id);
        if(i >= env->num_of_agents){
            i -= env->num_of_agents;
        }
        if (!env->goal_locations[i].empty() && old_paths[i].size() <= 1)
        {
            list<pair<pair<int, int>, pair<int, int>>> path;
            // assert(env->map[env->curr_states[i].location] == 1);
            env->map[env->curr_states[i].location] = 0;
            del_first_reservation(old_paths[i]);
            single_agent_plan(env->curr_states[i].location,
                                     env->curr_states[i].orientation,
                                     env->goal_locations[i].front().first, path);
            if (path.size() > 0)
            {
                env->map[env->goal_locations[i].front().first] = 1;
                old_paths[i] = path;
                reserve(path);
            }
            else
            {
                env->map[env->curr_states[i].location] = 1;
            }
            if (path.size() > 1 || (path.size()>0 && path.front().second.first<path.front().second.second))
            {
                pair<int, int> tmp;
                if(path.front().second.first<path.front().second.second){
                    tmp = path.front().first;
                }
                else{
                    tmp = (++path.begin())->first;
                }
                if (tmp.first != env->curr_states[i].location)
                {
                    actions[i] = Action::FW; // forward action
                }
                else if (tmp.second != env->curr_states[i].orientation)
                {
                    int incr = tmp.second - env->curr_states[i].orientation;
                    if (incr == 1 || incr == -3)
                    {
                        actions[i] = Action::CR; // C--counter clockwise rotate
                    }
                    else if (incr == -1 || incr == 3)
                    {
                        actions[i] = Action::CCR; // CCR--clockwise rotate
                    }
                }
            }
        }
        shift_id = i;
    }
    for (int ii = 0; ii < env->num_of_agents; ii++)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = (ii+tmp_shift_id);
        if(i >= env->num_of_agents){
            i -= env->num_of_agents;
        }
        if (!env->goal_locations[i].empty())
        {
            list<pair<pair<int, int>, pair<int, int>>> path;
            // assert(env->map[env->curr_states[i].location] == 1);
            if(old_paths[i].size()){
                env->map[old_paths[i].back().first.first] = 0;
            }
            else{
                env->map[env->curr_states[i].location] = 0;
            }
            del_reservations(old_paths[i]);
            single_agent_plan(env->curr_states[i].location,
                                     env->curr_states[i].orientation,
                                     env->goal_locations[i].front().first, path);
            if(path.size()==0 || (old_paths[i].size()>0 && path.back().second.second >= old_paths[i].back().second.second)){
                if(old_paths[i].size()){
                    env->map[old_paths[i].back().first.first] = 1;
                }
                else{
                   env->map[env->curr_states[i].location] = 1; 
                }
                reserve(old_paths[i]);
                shift_id = i;
                continue;
            }

            // assert(path.back().first == env->goal_locations[i].front().first);
            env->map[env->goal_locations[i].front().first] = 1;
            old_paths[i] = path;
            reserve(path);
            if (path.size() > 1 || (path.size()>0 && path.front().second.first<path.front().second.second))
            {
                pair<int, int> tmp;
                if(path.front().second.first<path.front().second.second){
                    tmp = path.front().first;
                }
                else{
                    tmp = (++path.begin())->first;
                }
                if (tmp.first != env->curr_states[i].location)
                {
                    actions[i] = Action::FW; // forward action
                }
                else if (tmp.second != env->curr_states[i].orientation)
                {
                    int incr = tmp.second - env->curr_states[i].orientation;
                    if (incr == 1 || incr == -3)
                    {
                        actions[i] = Action::CR; // C--counter clockwise rotate
                    }
                    else if (incr == -1 || incr == 3)
                    {
                        actions[i] = Action::CCR; // CCR--clockwise rotate
                    }
                }
                else{
                    actions[i] = Action::W;
                }
            }
        }
        shift_id = i;
    }
    int cnt = 0;
    for (int i = 0; i < old_paths.size(); ++i)
    {
        if (old_paths[i].size() > 0)
        {
            del_first_reservation(old_paths[i]);
            ++cnt;
        }
    }
    // cout<<"cnt = "<<cnt<<endl;
    // cout<<shift_id%env->cols<<endl;
    // int cnt1 = 0;
    // for(auto it:env->map){
    //     cnt1 += it;
    // }
    // cout<<"cnt1 = "<<cnt1<<endl;
    return;
}

void MAPFPlanner::single_agent_plan(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path)
{
    if(env->map[start] || env->map[end]){
        return;
    }
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, env->curr_timestep, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    while (!open_list.empty())
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        AstarNode *curr = open_list.top();
        open_list.pop();
        int id = curr->location * 4 + curr->direction;
        if (curr->location == end && (map[curr->location].empty() || map[curr->location].upper_bound({curr->g,curr->g}) == map[curr->location].end()))
        {
            path.emplace_front(make_pair(make_pair(curr->location, curr->direction), make_pair(curr->g, curr->g)));
            while (curr->parent != NULL)
            {
                auto pa = curr->parent;
                path.emplace_front(make_pair(make_pair(pa->location, pa->direction), make_pair(pa->g, curr->g-1)));
                curr = pa;
            }
            break;
        }
        // Check visited
        if (!closed_list[id].empty())
        {
            auto it_node_prev = closed_list[id].upper_bound(curr->g);
            if (it_node_prev != closed_list[id].begin())
            {
                --it_node_prev; // node in closed_list which have time less or equal curr->g
                if (map[curr->location].empty())
                { // no block times at this location.
                    continue;
                }
                auto it_block_prev = map[curr->location].upper_bound({curr->g,curr->g});
                if (it_block_prev == map[curr->location].begin())
                { // no block time at this location less than curr->g
                    continue;
                }
                --it_block_prev; // block time less than curr->g
                if (*it_node_prev > it_block_prev->second)
                    continue;
            }
        }
        closed_list[id].insert(curr->g);
        list<pair<int, pair<int, int>>> neighbors = getNeighbors(curr->location, curr->direction, curr->g);
        for (const pair<int, pair<int, int>> &neighbor : neighbors)
        {
            AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second.first,
                                                 neighbor.second.second, getManhattanDistance(neighbor.first, end), curr);
            open_list.push(next_node);
            all_nodes.push_back(next_node);
        }
    }
    for (auto n : all_nodes)
    {
        closed_list[n->location * 4 + n->direction].clear();
        delete n;
    }
    all_nodes.clear();
    return;
}

void MAPFPlanner::reserve(std::list<pair<pair<int, int>,pair<int, int>>> &path)
{
    if(path.size()==0)return;
    auto pre_loc = path.front().first;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
        // if (it != it_prev_end)
        map[it->first.first].insert(it->second);
        if (it->first.first != pre_loc.first)
        {
            blocked_edges[pre_loc.second][pre_loc.first].insert(it->second.first - 1);
        }
        pre_loc = it->first;
    }
    // }
}

void MAPFPlanner::del_reservations(std::list<pair<pair<int, int>, pair<int, int>>> &path)
{
    if(path.size()==0){
        return;
    }
    auto pre_loc = path.front().first;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
        // if (it != it_prev_end)
        map[it->first.first].erase(it->second);
        if (it->first.first != pre_loc.first)
        {
            blocked_edges[pre_loc.second][pre_loc.first].erase(it->second.first - 1);
        }
        pre_loc = it->first;
    }
}

void MAPFPlanner::del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path)
{
    if(path.size()==0){
        return;
    }
    auto loc = path.front();
    map[loc.first.first].erase(map[loc.first.first].find(loc.second));
    path.pop_front();
    loc.second.first++;
    if(loc.second.first <= loc.second.second){
        path.push_front(loc);
        map[loc.first.first].insert(loc.second);
    }
    else if(path.size()>0){
        if(loc.first.first != path.front().first.first){
            blocked_edges[loc.first.second][loc.first.first].erase(env->curr_timestep);
        }
    }
    // }
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1 / env->cols;
    int loc1_y = loc1 % env->cols;
    int loc2_x = loc2 / env->cols;
    int loc2_y = loc2 % env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2, int t)
{
    int loc_x = loc / env->cols;
    int loc_y = loc % env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols)
        return false;

    int loc2_x = loc2 / env->cols;
    int loc2_y = loc2 % env->cols;
    if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1)
        return false;
    return true;
}

bool MAPFPlanner::check(list<pair<int, int>> &path)
{
    int t = 1;
    for (auto it : path)
    {
        if (env->map[it.first])
        {
            return false;
        }
        ++t;
        if (t == 2)
            break;
    }
    return true;
}

bool MAPFPlanner::is_edge(int loc)
{
    int x = loc % env->cols;
    int y = loc / env->cols;
    if (x == 0 || x == env->cols - 1 || y == 0 || y == env->rows - 1)
    {
        return true;
    }
    return false;
}

list<pair<int, pair<int, int>>> MAPFPlanner::getNeighbors(int location, int direction, int t)
{
    list<pair<int, pair<int, int>>> neighbors;
    // forward
    int candidates[4] = {location + 1, location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward >= 0 && forward < env->map.size() && (location % env->cols || direction != 2) && (location % env->cols != env->cols - 1 || direction != 0))
    {
        // if(static_map[forward]==0 && (t>=2 || env->map[forward]==0)){
        if (env->map[forward] == 0)
        {
            int t_u = 1e9;
            auto it = map[location].upper_bound({t,t});
            if (it != map[location].end())
            {
                t_u = it->first - 1;
            }
            if (map[forward].size() > 0)
            {
                // it = map[forward].upper_bound()
                int t_prev = 0;
                for (auto it1 = map[forward].begin(); it1 != map[forward].end(); ++it1)
                {
                    int t_l = max(t + 1, t_prev + 1);
                    if (t_l < it1->first && t_l - 1 <= t_u)
                    {
                        if (blocked_edges[(direction + 2) % 4][forward].find(t_l - 1) == blocked_edges[(direction + 2) % 4][forward].end())
                        {
                            neighbors.push_back({forward, {direction, t_l}});
                        }
                    }
                    if (t_l - 1 > t_u)
                        break;
                    t_prev = it1->second + 1;
                }
                int t_l = max(t + 1, t_prev + 1);
                if (t_l - 1 <= t_u)
                {
                    if (blocked_edges[(direction + 2) % 4][forward].find(t_l - 1) == blocked_edges[(direction + 2) % 4][forward].end())
                    {
                        neighbors.push_back({forward, {direction, t_l}});
                    }
                }
            }
            else
            {
                neighbors.push_back({forward, {direction, t + 1}});
            }
        }
    }

    int t_u = 1e9;
    auto it = map[location].upper_bound({t,t});
    if (it != map[location].end())
    {
        t_u = it->first - 1;
    }
    if (t + 1 <= t_u)
    {
        // turn left
        new_direction = direction - 1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.push_back({location, {new_direction, t + 1}});
        // turn right
        new_direction = direction + 1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.push_back({location, {new_direction, t + 1}});
    }
    return neighbors;
}