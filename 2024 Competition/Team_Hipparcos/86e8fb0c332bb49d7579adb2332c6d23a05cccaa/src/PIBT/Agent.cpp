#include <chrono>   
#include "PIBT/Agent.h"

void Agent::setGoal() {
    if (env->goal_locations[id].empty()) {
        goal = getLoc();
    }
    else {
        goal = env->goal_locations[id].front().first;
    }

    int areaFrom {maputils->areaMap[getLoc()]};
    int areaTo {maputils->areaMap[goal]};
    nextArea = maputils->basePointDistances[areaFrom][areaTo].second.cbegin();
    endArea = maputils->basePointDistances[areaFrom][areaTo].second.cend();

    initializeHeuristic(areaFrom, areaTo);

    goalDist = maputils->basePointDistances[areaFrom][areaTo].first + heuristic.abstractDist(getLoc(), getDir());
    p = goalDist;
}

void Agent::setArea() {
    int areaFrom {maputils->areaMap[getLoc()]};

    if (nextArea != endArea && areaFrom == *nextArea) {
        int areaTo {maputils->areaMap[goal]}; 
        initializeHeuristic(areaFrom, areaTo);
    }
}

int Agent::getLoc() const {
    return env->curr_states[id].location;
}

int Agent::getDir() const {
    return env->curr_states[id].orientation;
}

bool Agent::isNewGoal() const {
    if (env->goal_locations[id].empty()) {
        return goal == -1;
    }

    return goal != env->goal_locations[id].front().first;
}

std::vector<std::pair<int,int>> Agent::getNeighborsWithDist() {
    std::vector<std::pair<int,int>> neighbours;
    int loc {getLoc()};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], heuristic.abstractDist(candidates[i], i)));
        }
    }

    neighbours.emplace_back(std::make_pair(loc, heuristic.abstractDist(loc, getDir())));

    return neighbours;
}

void Agent::calculateNeighborDists() {
    setArea();

    int loc {getLoc()};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            heuristic.abstractDist(candidates[i], i);
        }
    }

    heuristic.abstractDist(loc, getDir());
}

bool Agent::validateMove(int loc1, int loc2) const {
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

void Agent::initializeHeuristic(int areaFrom, int areaTo) {
    heuristic.reset();

    if (areaFrom == areaTo) {
        nextArea = endArea;
        heuristic.initialize(getLoc(), getDir(), goal);
    }
    else {
        ++nextArea;
        heuristic.initialize(getLoc(), getDir(), *nextArea, maputils);
    }
}