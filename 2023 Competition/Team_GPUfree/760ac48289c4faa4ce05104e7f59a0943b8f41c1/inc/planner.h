#pragma once
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <iostream>
#include <random>
#include "SharedEnv.h"
#define INF 10000000

struct Node {
    explicit Node(int location=-1, int orientation=-1, float g = INF, float h = 0) : locaction(location), orientation(orientation), g(g), f(g+h){}
    int locaction;
    int orientation;
    float g;
    float f;
    int get_id(int gridsize) const { return orientation * gridsize + locaction; }
    bool operator<(const Node& other) const {
        return this->f < other.f or (std::fabs(this->f - other.f) < 1e-5 and this->g < other.g);
    }
    bool operator>(const Node& other) const {
        return this->f > other.f or (std::fabs(this->f - other.f) < 1e-5 and this->g > other.g);
    }
    bool operator==(const Node& other) const {
        return this->locaction == other.locaction and this->orientation == other.orientation;
    }
    bool operator==(int other_loc) const {
        return this->locaction == other_loc;
    }
};

class planner
{
public:
    planner(SharedEnvironment* env):env(env)
    {

        num_occupations = std::vector<int>(env->map.size(), 0);
        bad_cells = std::vector<int>(env->map.size(), 0);
        penalties = std::vector<float>(env->map.size(), 1);
        use_precalc_cost = true;
        use_dynamic_cost = true;
        reset_dynamic_cost = true;
        gridsize = static_cast<int>(env->map.size());
        start_location = -1;
        start_orientation = -1;
        goal_location = -1;
    }
    int start_location;
    int start_orientation;
    int goal_location;
    int gridsize;
    SharedEnvironment* env;
    std::priority_queue<Node, std::vector<Node>, std::greater<>> OPEN;
    std::vector<int> bad_cells;
    std::vector<int> num_occupations;
    std::vector<float> penalties;
    std::vector<float> g_values;
    std::vector<float> h_values;
    std::vector<int> parents;
    bool use_precalc_cost;
    bool use_dynamic_cost;
    bool reset_dynamic_cost;
    std::default_random_engine re;
    std::uniform_real_distribution<> dis;

    bool validate_move(int loc, int loc2)
    {
        int loc_x = loc/env->cols;
        int loc_y = loc%env->cols;
        int loc2_x = loc2/env->cols;
        int loc2_y = loc2%env->cols;
        if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
            return false;
        return true;
    }

    int get_loc(int location, int direction, Action action)
    {
        if(action == Action::FW)
        {
            std::vector<int> candidates = { location + 1,location + env->cols, location - 1, location - env->cols};
            int forward = candidates[direction];
            if (forward>=0 && forward < env->map.size() && env->map[forward] == 0)
                if(validate_move(location, forward))
                    return forward;
            return -1;
        }
        return location;
    }

    list<pair<int,int>> get_neighbors(int location, int direction)
    {
        list<pair<int,int>> neighbors;
        //forward
        std::vector<int> candidates = { location + 1,location + env->cols, location - 1, location - env->cols};
        int forward = candidates[direction];
        int new_direction = direction;
        if (forward>=0 && forward < env->map.size() && env->map[forward] == 0)
            if(validate_move(location, forward))
                neighbors.emplace_back(forward,new_direction);
        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.emplace_back(location,new_direction);
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.emplace_back(location,new_direction);
        //neighbors.emplace_back(location,direction); //wait
        return neighbors;
    }

    void compute_shortest_path()
    {
        Node current;
        while(!OPEN.empty() and current.locaction != goal_location) {
            current = OPEN.top();
            OPEN.pop();
            if(g_values[current.get_id(gridsize)] < current.g)
                continue;
            for(const auto& n: get_neighbors(current.locaction, current.orientation)) {
                float cost(1);
                int id = n.second*gridsize+n.first;
                if(use_precalc_cost)
                    cost = penalties[n.first];
                if(use_dynamic_cost)
                    cost += num_occupations[n.first];
                if(g_values[id] > current.g + cost) {
                    OPEN.emplace(n.first, n.second, current.g + cost, h_values[id]);
                    g_values[id] = current.g + cost;
                    parents[id] = current.orientation*gridsize + current.locaction;
                }
            }
        }
    }

    float get_avg_distance(int loc)
    {
        std::queue<int> fringe;
        fringe.emplace(loc);
        auto result = std::vector<int>(gridsize,-1);
        result[loc] = 0;
        std::vector<int> moves = {1, -1, env->cols, -env->cols};
        int total_dist(0), total_nodes(1);
        while(!fringe.empty()) {
            loc = fringe.front();
            fringe.pop();
            for(const auto& move: moves) {
                int new_loc(loc + move);
                if(new_loc < 0 || new_loc >= gridsize)
                    continue;
                if(env->map[new_loc] == 0 && result[new_loc] < 0) {
                    result[new_loc] = result[loc] + 1;
                    fringe.emplace(new_loc);
                    total_nodes++;
                    total_dist += result[new_loc];
                }
            }
        }
        return float(total_dist)/float(total_nodes);
    }

    void update_h_values(int g)
    {
        std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
        h_values = std::vector<float>(gridsize*4, INF);
        for(int i = 0; i < 4; i++) {
            h_values[i*gridsize + g] = 0;
            open.emplace(g, i, 0, 0);
        }
        while(!open.empty()) {
            Node current = open.top();
            open.pop();
            if(h_values[current.get_id(gridsize)] < current.g)
                continue;
            int inverted_orientation = current.orientation - 2;
            if(inverted_orientation < 0)
                inverted_orientation += 4;
            for(const auto& n: get_neighbors(current.locaction, inverted_orientation)) {
                float cost(1);
                if(use_precalc_cost)
                    cost = penalties[current.locaction];
                int id = n.second*gridsize+n.first;
                if(h_values[id] > current.g + cost) {
                    open.emplace(n.first, n.second, current.g + cost, 0);
                    h_values[id] = current.g + cost;
                }
            }
        }
    }

    int get_goal_orientation(int goal_pos)
    {
        for(int i = 0; i < 4; i++)
            if(g_values[i*env->map.size()+goal_pos] < INF)
                return i;
        return -1;
    }

    void reset()
    {
        parents = std::vector<int>(gridsize*4, -1);
        g_values = std::vector<float>(gridsize*4,INF);
        OPEN = std::priority_queue<Node, std::vector<Node>, std::greater<>>();
        Node s(start_location, start_orientation, 0, h_values[start_orientation*gridsize+start_orientation]);
        OPEN.push(s);
        g_values[start_orientation*gridsize+start_orientation] = 0;
    }

    std::vector<float> precompute_penalty_matrix()
    {
        penalties = std::vector<float>(env->map.size(), 0);
        float max_avg_dist(0);
        for(size_t i = 0; i < env->map.size(); i++)
            if(env->map[i] == 0) {
                penalties[i] = get_avg_distance(i);
                if(penalties[i] > max_avg_dist)
                    max_avg_dist = penalties[i];
            }
        for(size_t i = 0; i < env->map.size(); i++)
            if(env->map[i] == 0)
                penalties[i] = max_avg_dist / penalties[i];
        return penalties;
    }

    void set_penalties(std::vector<float> _penalties)
    {
        penalties = std::move(_penalties);
    }

    void update_occupied_cells(int cur_goal)
    {
        if(reset_dynamic_cost)
            if(goal_location != cur_goal)
                num_occupations = std::vector<int>(env->map.size(), 0);
        for(const auto& s: env->curr_states)
            num_occupations[s.location]++;
        num_occupations[start_location]--;
    }

    void update_bad_cells()
    {
        bad_cells = std::vector<int>(env->map.size(), 0);
        for(const auto& s:env->curr_states)
            bad_cells[s.location] = 1;
    }

    void update_path(int cur_location, int cur_orientation, int _goal_location)
    {
        start_location = cur_location;
        start_orientation = cur_orientation;
        if(goal_location != _goal_location)
            update_h_values(_goal_location);
        update_occupied_cells(_goal_location);
        goal_location = _goal_location;
        reset();
        compute_shortest_path();
    }

    std::list<int> get_path()
    {
        std::list<int> path;
        int next_node = get_goal_orientation(goal_location)*gridsize + goal_location;
        int start_id = start_orientation*gridsize + start_location;
        if(next_node < 0)
            return {start_id, start_id};
        if(g_values[next_node] > 0 and next_node != start_id) {
            while (next_node != start_id) {
                path.push_front(next_node);
                next_node = parents[next_node];
            }
            path.push_front(next_node);
        }
        parents.clear();
        g_values.clear();
        return path;
    }

    std::pair<int, int> get_next_node()
    {
        int next_node = get_goal_orientation(goal_location)*gridsize + goal_location;
        int start_id = start_orientation*gridsize + start_location;
        if(next_node < 0)
            return {start_id, start_id};
        if(g_values[next_node] > 0 and next_node != start_id) {
            while (parents[next_node] != start_id)
                next_node = parents[next_node];
            return {start_id, next_node};
        }
        return {-1, -1};
    }

    bool path_found()
    {
        int goal_pos = get_goal_orientation(goal_location)*gridsize + goal_location;
        return g_values[goal_pos] > 0;
    }
};