#include "KivaGraph.h"
#include "PBSNode.h"
#include "SingleAgentSolver.h"
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
    cout << env->curr_states.size() << endl;
    cout << env->map.size() << endl;
    cout << "------" << endl;
    // init map
    paths.resize(env->num_of_agents);
    simulation_window = 5;
    planning_window = 9;
    timestep = 0;
    plan_timestep = 0;
    has_plan = false;
    KivaGrid* g = new KivaGrid;
    G  = g;
    G->load_env_map(env);
    
    SingleAgentSolver* path_planner = new SIPP();
    PBS* p = new PBS(*G, *path_planner);
    pbs = p;
    pbs->lazyPriority = false;
    pbs->prioritize_start = true;
    pbs->setRT(true, true);
    pbs->window = planning_window;
    pbs->k_robust = 0;
    pbs->hold_endpoints = false;
    pbs->screen = 0;
    pbs->solution_found = false;

	pbs->initial_rt.hold_endpoints = true;
	pbs->initial_rt.map_size = env->map.size();
	pbs->initial_rt.k_robust = 0;
	pbs->initial_rt.window = INT_MAX;
    cout << "planner initialize done" << endl;
}

void MAPFPlanner::update_start_locations()
{
    starts.resize(env->curr_states.size());
    for (int i = 0; i < env->num_of_agents; i++)
    {
        starts[i] = State(env->curr_states[i].location, 0, env->curr_states[i].orientation);
    }
}

void MAPFPlanner::update_goal_locations()
{
    for (auto goals : env->goal_locations)
    {
        for (auto goal : goals)
        {
            int goal_location = goal.first;
            G->computeGoalHeuristics(goal_location);
        }
    }

}

void MAPFPlanner::plan2(int time_limit,vector<Action> & actions) 
{
    // update

}

void MAPFPlanner::solve()
{
    pbs->clear();
    update_start_locations();
    update_goal_locations();
    bool sol = pbs->run(starts, env->goal_locations, 100);
    cout << "sol " << sol << endl;
    has_plan = true;
    plan_timestep = timestep;
}


void MAPFPlanner::updatePaths(vector<Path> solver_paths, int max_timestep)
{
    for (int k = 0; k < env->num_of_agents; k++)
    {
        int length = min(max_timestep, (int) solver_paths[k].size());
        paths[k].resize(timestep + length);
        for (int t = 0; t < length; t++)
        {
            paths[k][timestep + t] = solver_paths[k][t];
            paths[k][timestep + t].timestep = timestep + t;
        }
    }
}


bool MAPFPlanner::needReplan()
{
    if (timestep >= plan_timestep + 4 || !has_plan) return true;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (paths[i].size() <= timestep + 1) return true;

        if (paths[i][timestep] != env->curr_states[i]) return true;
    }
    skip_time++;
    return false;
}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // t->clear();
    timestep = env->curr_timestep;
    cout << "timestep " << timestep  << "skip time" << skip_time << endl;
    if (needReplan())
    {
    solve();
    cout << "solution length" << pbs->solution.size() << endl;
    updatePaths(pbs->solution, 10000);
    }


    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    cout << "inline" << endl;
    int curr_timestep = env->curr_timestep;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (env->goal_locations[i].empty()) continue;
        auto agent_path = paths[i];
        if (agent_path[curr_timestep+1].location != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
        }
        else if (agent_path[curr_timestep+1].orientation != env->curr_states[i].orientation)
        {
            int incr = agent_path[curr_timestep+1].orientation - env->curr_states[i].orientation;
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
