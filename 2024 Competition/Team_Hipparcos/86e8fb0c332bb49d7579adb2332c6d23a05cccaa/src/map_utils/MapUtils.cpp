#include "map_utils/MapUtils.h"
#include <random>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

void MapUtils::eraseDeadEnds() {
    deadEndMap = env->map;
    bool changed {true};
    int cnt {0};

    while (changed) {
        changed = false;
        for (int i {0}; i < deadEndMap.size(); ++i) {
            if (deadEndMap[i] > 0) { continue; }

            if (isDeadLoc(i)) {
                deadEndMap[i] = DEADLOC;
                changed = true;
                ++cnt;
            }
        }
    }
}

bool MapUtils::isDeadLoc(int location) const {
    int cnt1 {0};
    int cnt2 {0};

    int x {location / env->cols};
    int y {location % env->cols};

    // right
    if (y + 1 < env->cols) {
        if (deadEndMap[location + 1] == 1) { ++cnt1; }
        else if (deadEndMap[location + 1] == DEADLOC) { ++cnt2; }
    }
    else { ++cnt1; }

    // left
    if (y - 1 >= 0) {
        if (deadEndMap[location - 1] == 1) { ++cnt1; }
        else if (deadEndMap[location - 1] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    // top
    if (x - 1 >= 0) {
        if (deadEndMap[location - env->cols] == 1) { ++cnt1; }
        else if (deadEndMap[location - env->cols] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    // bottom
    if (x + 1 < env->rows) {
        if (deadEndMap[location + env->cols] == 1) { ++cnt1; }
        else if (deadEndMap[location + env->cols] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    return cnt1 == 3 || (cnt1 == 2 && cnt2 == 1);
}

void MapUtils::divideIntoAreas(int distance) {
    markBasePoints(distance);
    createAreasAroundBasePoints();
}

void MapUtils::markBasePoints(int distance) {
    areaMap.resize(env->map.size(), EMPTY);
    int id {0};

    for (int i {distance / 2}; i < env->rows; i += distance) {
        for (int j {distance / 2}; j < env->cols; j += distance) {
            if (markRandomPoint(i, j, distance / 3, id)) { ++id; }
        }
    }

    std::cout << "Base point cnt: " << id << '\n';
}

bool MapUtils::markRandomPoint(int row, int col, int distance, int id) {
    std::vector<int> candidates, out;

    for (int i {max(row - distance, 0)}; i < min(row + distance, env->rows); ++i) {
        for (int j {max(col - distance, 0)}; j < min(col + distance, env->cols); ++j) {
            int loc {i * env->cols + j};
            if (env->map[loc] == 0) {
                candidates.push_back(loc);
            }
        }
    }

    if (!candidates.empty()) {
        std::sample(candidates.begin(), candidates.end(), std::back_inserter(out), 1, std::mt19937{std::random_device{}()});
        basePoints.push_back(out[0]);
        return true;
    }

    return false;
}

void MapUtils::createAreasAroundBasePoints() {
    std::vector<std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::cmp>> open(basePoints.size());
    std::vector<std::unordered_map<int, DijkstraNode*>> closed(basePoints.size());
    std::vector<std::unordered_map<int, DijkstraNode*>> allNodes(basePoints.size());

    for (int i {0}; i < basePoints.size(); ++i) {
        DijkstraNode* s = new DijkstraNode(basePoints[i], EMPTY, 0);
        open[i].push(s);
        allNodes[i][s->location] = s;
    }

    int distance {0};
    bool running {true};
    while (running) {
        running = false;
        for (int i {0}; i < basePoints.size(); ++i) {
            if (open[i].empty()) { continue; }

            running = true;
            DijkstraNode* curr {open[i].top()};
            while (!open[i].empty() && curr->g <= distance) {
                open[i].pop();
                closed[i][curr->location] = curr;

                for (const auto& neighbor: getNeighbors(curr->location, curr->parentLocation)) {
                    if (closed[i].find(neighbor.first) != closed[i].end()) { continue; }

                    if (allNodes[i].find(neighbor.first) != allNodes[i].end()) {
                        DijkstraNode* old = allNodes[i][neighbor.first];
                        if (curr->g + neighbor.second < old->g) {
                            old->g = curr->g + neighbor.second;
                            old->parentLocation = curr->location;
                        }
                    } 
                    else if (areaMap[neighbor.first] == EMPTY) {
                        DijkstraNode* nextNode = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second);
                        open[i].push(nextNode);
                        allNodes[i][nextNode->location] = nextNode;
                    }
                }

                if (areaMap[curr->location] == EMPTY) {
                    areaMap[curr->location] = i; // i == basePoint id
                    areaLocations[i].emplace(curr->location);
                }

                if (!open[i].empty()) {
                    curr = open[i].top();
                }
            }
        }

        ++distance;
    }
    
    for (auto& i : allNodes) {
        for (auto& n : i) {
            delete n.second;
        }
    }
            
    for (int i {0}; i < basePoints.size(); ++i) {
        createAreaBorders(i);
    }
}

void MapUtils::createAreaBorders(int areaId) {
    for (const auto& areaLocation : areaLocations[areaId]) {
        for (const std::pair<int,int>& neighbor : getGoalNeighbors(areaLocation)) {
            if (areaMap[neighbor.first] != areaId) { 
                areaBorders[areaId].emplace(neighbor.first, neighbor.second);
            }
        }
    }
}

std::list<std::pair<int,int>> MapUtils::getGoalNeighbors(int loc) const {
    std::list<std::pair<int,int>> neighbours;
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i {0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], (i + 2) % 4));
        }
    }

    return neighbours;
}

std::list<std::pair<int,int>> MapUtils::getNeighbors(int loc, int parent_loc) const {
    std::list<std::pair<int,int>> neighbors;
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int candidate : candidates) {
        if (candidate == parent_loc) { continue; }
        if (validateMove(candidate, loc)) {
            int cost;
            if (candidate == loc + (loc - parent_loc) || parent_loc == EMPTY) {
                cost = 1;
            } else {
                cost = 2;
            }

            neighbors.emplace_back(std::make_pair(candidate, cost));
        }
    }

    return neighbors;
}

bool MapUtils::validateMove(int loc1, int loc2) const {
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

void MapUtils::calculateDistanceBetweenAreas() {
    basePointDistances.resize(basePoints.size(), std::vector<std::pair<int,std::list<int>>>(basePoints.size(), {EMPTY, {}}));
    boost::asio::thread_pool pool(boost::thread::hardware_concurrency());

    for (int i {0}; i < basePoints.size(); ++i) {
        int start {basePoints[i]};
        boost::asio::post(pool, [this, start, i]() {
            dijkstra(start, i);
        });
    }

    pool.join();
}

void MapUtils::dijkstra(int startLoc, int id) {
    std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::cmp> open;
    std::unordered_map<int, DijkstraNode*> closed;
    std::unordered_map<int, DijkstraNode*> allNodes;

    DijkstraNode* s = new DijkstraNode(startLoc, EMPTY, 0, nullptr);
    open.push(s);
    allNodes[s->location] = s;

    while (!open.empty()) {
        DijkstraNode* curr = open.top();
        open.pop();
        closed[curr->location] = curr;

        for (const auto& neighbor: getNeighbors(curr->location, curr->parentLocation)) {
            if (closed.find(neighbor.first) != closed.end()) {
                continue;
            }

            if (allNodes.find(neighbor.first) != allNodes.end()) {
                DijkstraNode* old = allNodes[neighbor.first];
                if (curr->g + neighbor.second < old->g) {
                    old->g = curr->g + neighbor.second;
                    old->parentLocation = curr->location;
                    old->parent = curr;
                }
            } 
            else {
                DijkstraNode* nextNode = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second, curr);
                open.push(nextNode);
                allNodes[nextNode->location] = nextNode;
            }
        }
    }

    for (int i {0}; i < basePoints.size(); ++i) {
        auto it {closed.find(basePoints[i])};
        if (it != closed.end()) {
            basePointDistances[id][i].first = it->second->g;
            backTrack(id, i, it->second);
        }
    }

    for (auto& n : allNodes) {
        delete n.second;
    }
}

void MapUtils::backTrack(int from, int to, DijkstraNode* current) {
    std::unordered_set<int> seen;
    std::list<int>& route {basePointDistances[from][to].second};
    route.push_front(to);
    //seen.insert(to);
    int areaId {to};

    // keep >>first<< or last occurence ??

    while (areaId != from) {
        current = current->parent;
        areaId = areaMap[current->location];

        if (/*seen.find(areaId) == seen.end() &&*/ route.front() != areaId) {
           route.push_front(areaId);
            // seen.insert(areaId);
        }
    }

    auto it {route.begin()};
    while (it != route.end()) {
        if (seen.find(*it) != seen.end()) {
            it = route.erase(it);
        } else {
            seen.insert(*it);
            ++it;
        }
    }
}