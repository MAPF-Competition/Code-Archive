#include <MAPFPlanner.h>
#include <random>
#include <omp.h>

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

void MAPFPlanner::initialize(int preprocess_time_limit) {
    auto start_t = std::chrono::steady_clock::now();
    // current_goal_locations = std::vector<int>(env->num_of_agents,-1);
    // next_goal_locations = std::vector<int>(env->num_of_agents,-1);
    // getCostMap();
    if (!solver) {
        solver = std::make_unique<cbs::ECBSSolver>(env->map, env->rows,
                                                  env->cols, costmaps,predecessors);
        solver->setHorizon(ecbs_horizon);
    }
    auto end_t = std::chrono::steady_clock::now();
    cout << "planner initialize done, time: "
         << std::chrono::duration_cast<std::chrono::milliseconds>(end_t -
                                                                  start_t)
                .count()
         << " ms " << endl;
    // exit(1);
    plan_for_all = true;

}

void MAPFPlanner::plan_all(){
    std::cout<<"plan for all agents"<<std::endl;
    std::vector<std::pair<State, State>> tasks;
    State start, end;
    for (int i = 0; i < env->num_of_agents; i++) {
        start = State(env->curr_states[i].location, 0,
                      env->curr_states[i].orientation);
        end = start;
        if (!env->goal_locations[i].empty() &&
            env->curr_states[i].location !=
                env->goal_locations[i].front().first) {
          end = State(env->goal_locations[i].front().first, 0, 0);
        }
        tasks.push_back({start, end});
    }
    solver->setHorizon(ecbs_horizon);
    sol = solver->Solve(tasks);  //

    for (int i = 0; i < env->num_of_agents; i++) {
        auto& path = sol.at(i);
        if (path.empty()) {
          return;
        }
        path.pop_front();
    }
    // printAllPaths();
}

void MAPFPlanner::printAllPaths(){
    // for debug
    std::cout<<"MAPFPlanner::printAllPaths"<<std::endl;
    for (int i = 0; i < env->num_of_agents; i++) {
        const auto& path = sol.at(i);
        State start, end;
        start = State(env->curr_states[i].location, 0,
                      env->curr_states[i].orientation);
        end = State(env->goal_locations[i].front().first, 0, 0);
        std::cout << start<<" to "<<end << " ~ agent[" << i
                  << "]'s future path: " << Path{path.begin(), path.end()};
    }
}

void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
    // if (plan_for_all || replan_timer <= 0) {
    //     plan_all();
    //     plan_for_all = false;
    //     replan_timer = REPLAN_THR;
    // }

    for(auto p: sol){
        if(p.empty()){
            plan_for_all = true;
        }
    }
    if(plan_for_all){
        plan_all();
    }

    actions = std::vector<Action>(env->curr_states.size(), Action::W);

    // // fullfill plans
    // for (int i = 0; i < env->num_of_agents; i++) {
    //     auto& path = sol.at(i);
    //     State start, end;
    //     start = State(env->curr_states[i].location, 0,
    //                   env->curr_states[i].orientation);
    //     end = State(env->goal_locations[i].front().first, 0, 0);
    //     if (path.empty()) {  // no point left to follow
    //       std::cout << "agent " << i
    //                 << "'s path empty. start plan. start: " << start
    //                 << std::endl;
    //       auto res =
    //           solver->SearchOnePath({start, end}, sol, env->curr_states, i);
    //       if (!res.empty()) {
    //         path = res;
    //         std::cout << "plan success:" << std::endl;
    //         std::cout << "\t" << start << " ~ agent[" << i
    //                   << "]'s future path: " << Path{path.begin(), path.end()};
    //       } else {
    //         std::cout << "plan fail" << std::endl;
    //         // single plan failed, plan for all
    //         plan_for_all = true;
    //         break;
    //       }
    //     }
    // }

    // handle single plan failed or force replan
    // if (plan_for_all) {
    //     plan_all();
    //     plan_for_all = false;
    //     replan_timer = REPLAN_THR;
    // }
    // the path nodes left shouldnot be empty
    // printAllPaths();
    // then start actions
    for (int i = 0; i < env->num_of_agents; i++) {
        auto& path = sol.at(i);
        if(path.empty()){
            std::cout<<"plan failed"<<std::endl;
            return;
        }
#ifdef DEBUG
        if (path.front().location != env->curr_states[i].location &&
            path.front().orientation != env->curr_states[i].orientation) {
          std::cout << "  ******* agent: " << i
                    << " current: " << env->curr_states[i]
                    << " path node: " << path.front() << std::endl;
          throw std::runtime_error(
              "location and orientation change at the same time");
        }
#endif
        if (path.front().location != env->curr_states[i].location) {
        //   assert(fabs(path.front().location - env->curr_states[i].location) <=
        //          env->cols);
          actions[i] = Action::FW;  // forward action
        } else if (path.front().orientation != env->curr_states[i].orientation) {
          int incr = path.front().orientation - env->curr_states[i].orientation;
          if (incr == 1 || incr == -3) {
            actions[i] = Action::CR;  // C--counter clockwise rotate
          } else if (incr == -1 || incr == 3) {
            actions[i] = Action::CCR;  // CCR--clockwise rotate
          }
          else{
#ifdef DEBUG
            throw std::runtime_error("angle change invalid!");
#endif
          }
        }
        path.pop_front();
    }

    std::cout<<" ------ plan end ------ "<<plan_count<<std::endl;
    plan_count++;
    replan_timer--;
}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan_sample(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
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


// return: <grid_index, direction>
list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
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

// for each point as a goal, calculate the costmap
void MAPFPlanner::getCostMap() {
    costmaps.resize(env->map.size());
    predecessors.resize(env->map.size());
    int num_threads = omp_get_num_procs();
    omp_set_num_threads(num_threads);
    std::cout<<"omp_get_num_procs() "<<num_threads<<std::endl;
#pragma omp parallel for
    for (int i = 0; i < env->map.size(); i++) {
        std::vector<int> distances(
            std::vector<int>(env->map.size(), INT32_MAX));
        std::vector<int> pred(
            std::vector<int>(env->map.size(), -1));
        int goal = i;
        if (env->map[i] == 1) {
            continue;
        }
        std::queue<int> q;
        q.push(goal);
        distances[goal] = 0;
        pred[goal] = goal;
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        while (!q.empty()) {
            auto current = q.front();
            auto current_cord =
                std::pair<int, int>{current / env->cols, current % env->cols};
            q.pop();
            for (auto dir : directions) {
                int newRow = current_cord.first + dir.first;
                int newCol = current_cord.second + dir.second;
                int newLoc = newRow * env->cols + newCol;
                if (newRow >= 0 && newRow < env->rows && newCol >= 0 &&
                    newCol < env->cols && distances[newLoc] == INT32_MAX &&
                    env->map[newLoc] == 0) {
                    distances[newLoc] = distances[current] + 1;
                    q.push(newLoc);
                    pred[newLoc] = current;
                }
            }
        }
        costmaps[i] = distances;
        predecessors[i] = pred;
    }

}
