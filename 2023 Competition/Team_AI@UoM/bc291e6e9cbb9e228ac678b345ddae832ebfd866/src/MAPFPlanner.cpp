#include <MAPFPlanner.h>
#include <random>
#include <chrono>
#include <thread>


struct AstarNode {
    int location;
    int direction;
    int f, g, h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    int size;

    AstarNode(int _location, int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent), size(1) {}
    AstarNode(int _location, int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), t(_t), parent(_parent), size(1) {}

    void updateSize() {
        size = 1; // Start with the current node's size
        if (parent)
            size += parent->size; // Add the parent's size recursively
    }
};


struct MapLocation {
    char state;

    MapLocation():
       state('0') {}

    bool is_occupied(){
        return state != '0' && state != '1';
    }

    bool is_obstacle(){
        return state == '1';
    }
};


struct cmp {
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};


static const int MAX_RUN_TIME = 800; // In ms.
static const int MAX_TIMESTEPS = 20000;

static int num_of_agents;

vector<vector<MapLocation>> map;
vector<vector<pair<int,int>>> all_paths;
vector<int> to_validate; // Agents that need path validation.
vector<int> to_recalculate; // Agents that need path Re:calculation.


void MAPFPlanner::initialize(int preprocess_time_limit) {
    map.resize(MAX_TIMESTEPS, vector<MapLocation>(env->rows * env->cols));
    num_of_agents = env->num_of_agents;
    all_paths.resize(num_of_agents);
    

    // Map initialization
    for (int i = 0; i < MAX_TIMESTEPS; i++) { 
        for(int row = 0; row < env->rows; row++){
            for(int col = 0; col < env->cols; col++){
                map[i][row * env->cols + col].state = char(env->map[row * env->cols + col] + 48);
            }
        }
    }
}


void MAPFPlanner::plan(int time_limit, vector<Action> & actions) {

    auto start_time = std::chrono::high_resolution_clock::now(); // Timer start.


    actions = std::vector<Action>(env->curr_states.size(), Action::W); // Sets default action of agents to wait.


    // Sets current agent locations to the map.
    for(int i = 0; i < num_of_agents; i++) 
        map[env->curr_timestep][env->curr_states[i].location].state = char(env->curr_states[i].orientation + 2 + 48);


    // Main path planner.
    for (int i = 0; i < num_of_agents; i++) {
        if(!all_paths[i].empty()){
            continue;
        }
        else if (env->goal_locations[i].empty()) {
            all_paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        }
        else {
            all_paths[i] = single_agent_plan(env->curr_states[i].location,
                                            env->curr_states[i].orientation,
                                            env->goal_locations[i].front().first, start_time);
            
            if(to_validate.empty() || (!to_validate.empty() && i > to_validate.back()))
                to_validate.push_back(i);
        }
    }


    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
    // cout << "Time A*: " << duration.count() << endl;

    // Path validator.
    while(!to_validate.empty()){
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if(duration.count() > MAX_RUN_TIME){
            map.insert(map.begin() + env->curr_timestep, map[env->curr_timestep]);
            for(int i = 0; i < all_paths.size(); i++){
                all_paths[i].insert(all_paths[i].begin(), {env->curr_states[i].location, env->curr_states[i].orientation});
            }

            break;
        }

 
        validate_path(to_validate.front(), to_recalculate);
        to_validate.erase(to_validate.begin());


        if(!to_recalculate.empty()){
            recalculate_path(to_recalculate.front(), start_time);
            to_validate.push_back(to_recalculate.front());
            to_recalculate.erase(to_recalculate.begin());
        }
    }

    // print_map(env->curr_timestep);

    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
    // cout << "Time WH: " << duration.count() << endl;

    // Actions planner.
    set_actions(actions);

    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
    // cout << "Time SA: " << duration.count() << endl;
}

void MAPFPlanner::set_actions(vector<Action>& actions){

    for(int i = 0; i < num_of_agents; i++) {
        if (all_paths[i].front().first != env->curr_states[i].location) {
            actions[i] = Action::FW; // Forward action.
        }
        else if (all_paths[i].front().second != env->curr_states[i].orientation) {
            int incr = all_paths[i].front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3) { // Clockwise rotate.
                actions[i] = Action::CR; 
            }
            else if (incr == -1 || incr == 3) { // Counter clockwise rotate.
                actions[i] = Action::CCR; 
            }
        }

        all_paths[i].erase(all_paths[i].begin());
    }
}


// Prints map. Used for debugging.
void MAPFPlanner::print_map(int ts){
    int count = 0;
    cout << endl << "TIMESTEP " << ts;
    for(int row = 0; row < env->rows; row++){
        cout << endl;
        for(int col = 0; col < env->cols; col++){
            char value = map[ts][row * env->cols + col].state;
            
            if(value == '0')
                cout << "\033[47m";
            else if(value == '1')
                cout << "\033[41m";
            else{
                cout << "\033[1;46m";
                count++;
            }
            cout << value << " " << "\033[0m";
        }
    }
    cout << endl << "PLANNED AGENTS: " << count << endl;
}


void MAPFPlanner::recalculate_path(int index, auto start_time){
    for(int j = 0; j < all_paths[index].size(); j++)
        map[env->curr_timestep + j + 1][all_paths[index][j].first].state = '0';    
    map[env->curr_timestep][env->curr_states[index].location].state = '0';

    all_paths[index] = single_agent_plan(env->curr_states[index].location,
                                        env->curr_states[index].orientation,
                                        env->goal_locations[index].front().first, start_time);
}


void MAPFPlanner::validate_path(int index, vector<int>& to_recalculate){

    for(int i = 0; i < num_of_agents; i++){
        if(i == index)
            continue;

        int smallest_path_size = (all_paths[index].size() < all_paths[i].size() ? all_paths[index].size() : all_paths[i].size());
        for(int ts = 0; ts < smallest_path_size; ts++){

            // Vertex conflict.
            if(all_paths[index][ts].first == all_paths[i][ts].first){
                to_recalculate.emplace(to_recalculate.begin(), index);
                to_recalculate.emplace(to_recalculate.begin(), i);
                break;
            }

            // Edge conflict.
            if(ts + 1 < smallest_path_size && all_paths[index][ts + 1].first == all_paths[i][ts].first && all_paths[index][ts].first == all_paths[i][ts + 1].first){
                to_recalculate.emplace(to_recalculate.begin(), index);
                to_recalculate.emplace(to_recalculate.begin(), i);
                break;
            }
        }
    }
}


// A* pathfinding algorithm with collision detection.
vector<pair<int,int>> MAPFPlanner::single_agent_plan(int start, int start_direct, int end, auto start_time) {

    vector<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>, cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty()) {

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if(duration.count() > MAX_RUN_TIME) // In milliseconds.
            return path;
        

        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);

        curr->updateSize();
        int ts = env->curr_timestep + curr->size - 1;
        if(ts >= MAX_TIMESTEPS - 1)
            continue;

        if (curr->location == end) {
            while(curr->parent != NULL) { // Sets all agent's future positions to the map.
                path.insert(path.begin(), make_pair(curr->location, curr->direction));
                map[ts][curr->location].state = char(curr->direction + 2 + 48);
                ts--;
                curr = curr->parent;
            }

            map[ts][curr->location].state = char(curr->direction + 2 + 48); // Sets agent's current location to the map.

            break;
        }

        vector<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, ts, end);
        for (const pair<int,int>& neighbor : neighbors) {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end()) {
                AstarNode* old = all_nodes[neighbor.first * 4 + neighbor.second];
                int new_g = curr->g + 1;
                if (new_g < old->g) {
                    old->g = new_g;
                    old->f = old->h + new_g;
                    old->parent = curr;
                }
            }
            else {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                                                    curr->g + 1, getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }


        // If no path is found, the planner returns the closest path to the finish, so no agent is fully unplanned.
        if(open_list.empty()){
            while(curr->parent != NULL) { // Sets all agent's future positions to the map.
                path.insert(path.begin(), make_pair(curr->location, curr->direction));
                map[ts][curr->location].state = char(curr->direction + 2 + 48);
                ts--;
                curr = curr->parent;
            }

            if(curr != NULL)
                map[ts][curr->location].state = char(curr->direction + 2 + 48); // Sets agent's current location to the map.

            break;
        }
    }

    if(path.empty()){ // No path found. Agent will wait for a timestep.
        path.push_back({start, start_direct});
        map[env->curr_timestep + 1][start].state = char(start_direct + 2 + 48);
    }

    for (auto n: all_nodes) {
        delete n.second;
    }

    all_nodes.clear();

    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {

    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;

    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int next_loc, int curr_loc, int direction, int timestamp, int end) {

    if(map[timestamp + 1][next_loc].is_obstacle()) // Wall.
        return false;

    if(map[timestamp + 1][next_loc].is_occupied()) // Vertex Conflict.
        return false;
    
    if(map[timestamp][next_loc].is_occupied() && abs(map[timestamp][next_loc].is_occupied() - char(direction + 2 + 48))) // Edge Conflict.
        return false;

    if (getManhattanDistance(next_loc, curr_loc) > 1)
        return false;

    return true;
}


vector<pair<int,int>> MAPFPlanner::getNeighbors(int location, int direction, int timestamp, int end){

    vector<pair<int,int>> neighbors;
     
    int candidates[4] = {location + 1,location + env->cols, location - 1, location - env->cols};

    int forward = candidates[direction];
    int new_direction = direction;

    if ((forward >= 0 && forward < map[timestamp].size()) && validateMove(forward, location, direction, timestamp, end))
        neighbors.push_back(make_pair(forward,new_direction)); // Forward.
        
    if(map[timestamp + 1][location].is_occupied()) // Agent cannot stay at the same location.
        return neighbors;

    // Turn left.
    new_direction = direction - 1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.push_back(make_pair(location, new_direction));

    // Turn right.
    new_direction = direction + 1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.push_back(make_pair(location, new_direction));

    neighbors.push_back(make_pair(location, direction)); // Wait.

    return neighbors;
}