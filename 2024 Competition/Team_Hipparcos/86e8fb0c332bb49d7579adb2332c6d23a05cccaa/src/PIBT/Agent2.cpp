#include <chrono>   
#include "PIBT/Agent2.h"

void Agent2::setGoal() {
    if (env->goal_locations[id].empty()) {
        goal = getLoc(0);
    }
    else {
        goal = env->goal_locations[id].front().first;
    }

    int areaFrom {maputils->areaMap[getLoc(0)]};
    int areaTo {maputils->areaMap[goal]};
    nextArea = maputils->basePointDistances[areaFrom][areaTo].second.cbegin();
    endArea = maputils->basePointDistances[areaFrom][areaTo].second.cend();

    initializeHeuristic(areaFrom, areaTo, 0);

    goalDist = maputils->basePointDistances[areaFrom][areaTo].first + heuristic.abstractDist(getLoc(0), getDir(0));
    p = goalDist;
}

void Agent2::setArea(int time) {
    int areaFrom {maputils->areaMap[getLoc(time)]};

    if (nextArea != endArea && areaFrom == *nextArea) {
        int areaTo {maputils->areaMap[goal]}; 
        initializeHeuristic(areaFrom, areaTo, time);
    }
}

int Agent2::getLoc(int time) const {
    if (locations.size() < time) {
        assert(false);
        return -1;
    }
    return locations[time].first;
}

int Agent2::getDir(int time) const {
    if (locations.size() <= time) {
        assert(false);
        return -1;
    }
    return locations[time].second;
}

bool Agent2::isNewGoal() const {
    if (env->goal_locations[id].empty()) {
        return goal == -1;
    }

    return goal != env->goal_locations[id].front().first;
}

std::vector<std::pair<int,int>> Agent2::getNeighborsWithDist(int time) {
    std::vector<std::pair<int,int>> neighbours;
    int loc {getLoc(time)};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], heuristic.abstractDist(candidates[i], i)));
        }
    }

    neighbours.emplace_back(std::make_pair(loc, heuristic.abstractDist(loc, getDir(time))));

    return neighbours;
}

void Agent2::calculateNeighborDists(int time) {
    setArea(time);

    int loc {getLoc(time)};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            heuristic.abstractDist(candidates[i], i);
        }
    }

    heuristic.abstractDist(loc, getDir(time));
}

bool Agent2::validateMove(int loc1, int loc2) const {
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

void Agent2::initializeHeuristic(int areaFrom, int areaTo, int time) {
    heuristic.reset();

    if (areaFrom == areaTo) {
        nextArea = endArea;
        heuristic.initialize(getLoc(time), getDir(time), goal);
    }
    else {
        ++nextArea;
        heuristic.initialize(getLoc(time), getDir(time), *nextArea, maputils);
    }
}