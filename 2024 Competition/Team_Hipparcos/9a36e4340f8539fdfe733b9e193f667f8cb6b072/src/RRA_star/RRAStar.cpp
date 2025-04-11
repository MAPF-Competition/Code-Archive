#include <math.h>
#include <limits>
#include "RRA_star/RRAstar.h"

void RRAstar::initialize(int start, int startDir, int goal) {
    this->start = start;
    for (const std::pair<int,int>& neighbor : getGoalNeighbors(goal)) {
        RRAstarNode* n = new RRAstarNode(neighbor.first, neighbor.second, 1, getManhattanDist(neighbor.first, start));
        open.push(n);
        allNodes[neighbor.first * 4 + neighbor.second] = n;
    }

    for (int i {0}; i < 4; ++i) {
        RRAstarNode* s = new RRAstarNode(goal, i, 0, getManhattanDist(goal, start));
        closed[goal * 4 + i] = s;
        allNodes[goal * 4 + i] = s;
    }

    if (start != goal) {
        resume(start, startDir);
    }
}

int RRAstar::abstractDist(int loc, int dir) {
    int i {loc * 4 + dir};
    auto it = closed.find(i);

    if (it != closed.end()) {
        return it->second->g;
    }

    if (resume(loc, dir) == true) {
        return closed[i]->g;
    }

    return std::numeric_limits<int>::max();
}

bool RRAstar::resume(int loc, int dir) {
    while (!open.empty()) {
        RRAstarNode* curr = open.top();
        open.pop();
        closed[curr->location * 4 + curr->direction] = curr;

        if (loc == curr->location && dir == curr->direction) {
            return true;
        }

        for (const std::pair<int,int>& neighbor: getNeighbors(curr->location, curr->direction))
        {
            int i {neighbor.first * 4 + neighbor.second};
            if (closed.find(i) != closed.end()) {
                continue;
            }

            if (allNodes.find(i) != allNodes.end()) {
                RRAstarNode* old = allNodes[i];
                if (curr->g + 1 < old->g) {
                    old->g = curr->g + 1;
                }
            } 
            else {
                RRAstarNode* nextNode = new RRAstarNode(neighbor.first, neighbor.second, curr->g + 1, getManhattanDist(neighbor.first, start));
                open.push(nextNode);
                allNodes[i] = nextNode;
            }
        }
    }

    return false;
}

void RRAstar::reset() {
    for (auto n: allNodes) {
        delete n.second;
    }

    allNodes.clear();
    closed.clear();
    open = std::priority_queue<RRAstarNode*, std::vector<RRAstarNode*>, RRAstarNode::cmp>();
}

int RRAstar::getManhattanDist(int loc1, int loc2) const {
    int loc1_x {loc1 / env->cols};
    int loc1_y {loc1 % env->cols};
    int loc2_x {loc2 / env->cols};
    int loc2_y {loc2 % env->cols};
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

std::list<std::pair<int,int>> RRAstar::getNeighbors(int loc,int dir) const
{
    std::list<std::pair<int,int>> neighbors;
    //forward
    int candidates[4] {loc - 1, loc - env->cols, loc + 1, loc + env->cols};
    int forward {candidates[dir]};
    int new_dir {dir};
    if (validateMove(forward,loc)) {
        neighbors.emplace_back(std::make_pair(forward, new_dir));
    }
    //turn left
    new_dir = dir-1;
    if (new_dir == -1) {
        new_dir = 3;
    }
    neighbors.emplace_back(std::make_pair(loc, new_dir));
    //turn right
    new_dir = dir+1;
    if (new_dir == 4) {
        new_dir = 0;
    }
    neighbors.emplace_back(std::make_pair(loc, new_dir));

    return neighbors;
}

std::list<std::pair<int,int>> RRAstar::getGoalNeighbors(int loc) const {
    std::list<std::pair<int,int>> neighbours;
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i {0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], (i + 2) % 4));
        }
    }

    return neighbours;
}

bool RRAstar::validateMove(int loc1, int loc2) const
{
    int loc_x {loc1 / env->cols};
    int loc_y {loc1 % env->cols};
    if (env->map[loc1] == 1 || loc_x >= env->rows || loc_y >= env->cols || loc_x < 0 || loc_y < 0) {
        return false;
    }

    int loc2_x {loc2 / env->cols};
    int loc2_y {loc2 % env->cols};
    if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1) {
        return false;
    }

    return true;
}

RRAstar::~RRAstar() {
       for (auto n: allNodes) {
        delete n.second;
    }
}