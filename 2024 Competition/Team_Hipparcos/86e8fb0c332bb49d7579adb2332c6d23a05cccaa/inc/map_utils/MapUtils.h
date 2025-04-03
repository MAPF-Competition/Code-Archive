#ifndef MAPUTILS_H
#define MAPUTILS_H

#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include "SharedEnv.h"
#include "map_utils/DijkstraNode.h"

class MapUtils {
public:
    MapUtils(SharedEnvironment* _env): env(_env) { }

    static constexpr int EMPTY  { -1 };
    static constexpr int DEADLOC { 2 };

    std::vector<int> deadEndMap;
    std::vector<int> basePoints;
    std::vector<int> areaMap;
    std::vector<std::vector<std::pair<int,std::list<int>>>> basePointDistances;
    std::unordered_map<int, std::unordered_set<int>> areaLocations;

    struct hashFunction { 
        size_t operator()(const pair<int,int> &x) const { 
            return x.first ^ x.second; 
        } 
    };
    std::unordered_map<int, std::unordered_set<std::pair<int,int>,hashFunction>> areaBorders;

    void divideIntoAreas(int distance);
    void calculateDistanceBetweenAreas();

    void eraseDeadEnds();

private:
    const int MAP_PRINT_WIDTH { 550 };
    const int MAP_PRINT_OFFSET { 0 };

    SharedEnvironment* env;

    bool isDeadLoc(int location) const;

    void markBasePoints(int distance);
    bool markRandomPoint(int row, int col, int distance, int id);
    void createAreasAroundBasePoints();
    void createAreaBorders(int areaId);
    std::list<std::pair<int,int>> getGoalNeighbors(int loc) const;

    void dijkstra(int startLoc, int id);
    void backTrack(int from, int to, DijkstraNode* current);

    std::list<std::pair<int,int>> getNeighbors(int loc, int parent_loc) const;
    bool validateMove(int loc1, int loc2) const;
};

#endif // MAPUTILS_H
