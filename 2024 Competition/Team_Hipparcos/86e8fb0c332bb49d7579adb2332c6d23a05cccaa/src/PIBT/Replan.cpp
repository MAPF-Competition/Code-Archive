#include "PIBT/Replan.h"
#include <bits/stdc++.h>

struct AstarNode {
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

struct cmp {
    bool operator()(AstarNode* a, AstarNode* b) {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};

bool Replan::replan(std::vector<Agent2*>& agents) {
    agentsCopy.clear();
    std::random_device rd;
    std::mt19937 g(rd());

    for (int i {0}; i < agents.size(); ++i) {
        Agent2* agent = new Agent2(agents[i]->id, env, nullptr);
        agent->locations = agents[i]->locations;
        agent->p = i;
        agentsCopy.push_back(agent);
    }

    std::shuffle(agentsCopy.begin(), agentsCopy.end(), g);

    for (int i {1}; i < reservationsCopy.size(); ++i) {
        for (auto a : agentsCopy) {
            if (i < a->locations.size()) {
                reservationsCopy[i][a->locations[i].first] = -1;
            }
        }
    }

    for (auto a : agentsCopy) {
        auto start = a->locations.front();
        auto goal = a->locations.back();
        a->locations.resize(1);
        if (start.first == goal.first) { 
            if(reservations[1][start.first] == -1) {
                reservations[1][start.first] = a->id;
                a->locations.push_back(start);
                continue;
            }
            else {
                return false;
            }
        }
        auto result = singleAgentPlan(a->id, start.first, start.second, goal.first, goal.second);
        if (result.first == false) {return false;}
        std::copy(result.second.begin(), result.second.end(), std::back_inserter(a->locations));
    }

    int prevSum {0};
    int currSum {0};
    for (int i {0}; i < agents.size(); ++i) {
        agentsCopy[i]->goalTime = agentsCopy[i]->locations.size() - 1;
        currSum += agentsCopy[i]->goalTime;
        prevSum += agents[i]->goalTime;
    }

    if (currSum < prevSum) {
        reservations = reservationsCopy;

        for (auto a : agentsCopy) {
            agents[a->p]->locations = a->locations;
            agents[a->p]->goalTime = a->goalTime;
        }

        return true;
    }

    return false;
}

std::pair<bool,std::list<std::pair<int,int>>> Replan::singleAgentPlan(int id, int start, int startDir,int end, int endDir) {
    std::list<pair<int,int>> path;
    std::priority_queue<AstarNode*,vector<AstarNode*>,cmp> openList;
    std::unordered_map<int,AstarNode*> allNodes;
    std::unordered_set<int> closeList;
    AstarNode* s = new AstarNode(start, startDir, 0, getManhattanDistance(start,end), nullptr);
    openList.push(s);
    allNodes[start*4 + startDir] = s;
    bool success {false};

    while (!openList.empty()) {
        AstarNode* curr = openList.top();
        openList.pop();
        closeList.emplace(curr->location*4 + curr->direction);
        if (curr->location == end && curr->direction == endDir) {
            while(curr->parent!=nullptr) {
                path.emplace_front(make_pair(curr->location, curr->direction));
                assert(reservationsCopy[curr->g][curr->location] == -1);
                reservationsCopy[curr->g][curr->location] = id;
                curr = curr->parent;
            }
            success = true;
            break;
        }
        else if  (curr->f > 40) { break; }

        if (reservationsCopy.size() <= curr->g + 1) {
            reservationsCopy.push_back(std::vector<int>(env->map.size(), -1));
        }
        std::list<std::pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, curr->g + 1);
        for (const std::pair<int,int>& neighbor: neighbors) {
            if (closeList.find(neighbor.first*4 + neighbor.second) != closeList.end()) {
                continue;
            }
            if (allNodes.find(neighbor.first*4 + neighbor.second) != allNodes.end()) {
                AstarNode* old = allNodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g) {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second, curr->g+1, getManhattanDistance(neighbor.first,end), curr);
                openList.push(next_node);
                allNodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: allNodes) {
        delete n.second;
    }
    return {success, path};
}

int Replan::getManhattanDistance(int loc1, int loc2) {
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool Replan::validateMove(int loc, int loc2) {
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1) {
        return false;
    }

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1) {
        return false;
    }
    return true;
}

std::list<std::pair<int,int>> Replan::getNeighbors(int loc, int dir, int time) {
    std::list<std::pair<int,int>> neighbors;
    //forward
    int candidates[4] = { loc + 1, loc + env->cols, loc - 1, loc - env->cols };
    int forward = candidates[dir];
    int newDir = dir;

    if (forward>=0 && forward < env->map.size() && validateMove(forward,loc)) {
        int id = reservationsCopy[time-1][forward];
        int id2 = reservationsCopy[time][loc];
        if (reservationsCopy[time][forward] == -1 && (id == -1 || id != id2)) {
            neighbors.emplace_back(std::make_pair(forward,newDir));
        }
    }
    
    if (reservationsCopy[time][loc] == -1) {
        //turn left
        newDir = dir-1;
        if (newDir == -1) {
            newDir = 3;
        }
        neighbors.emplace_back(std::make_pair(loc,newDir));
        //turn right
        newDir = dir+1;
        if (newDir == 4) {
            newDir = 0;
        }
        neighbors.emplace_back(std::make_pair(loc,newDir));
        neighbors.emplace_back(std::make_pair(loc,dir)); //wait
    }

    return neighbors;
}

Replan::~Replan() {
    for (auto a : agentsCopy) {
        delete a;
    }
}
