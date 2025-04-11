#ifndef REDUCEMAP_H
#define REDUCEMAP_H

#include <vector>
#include <unordered_map>
#include "SharedEnv.h"

class ReduceMap {
public:
    ReduceMap(SharedEnvironment* _env): env(_env) { }

    void reduceMapStart();
    bool reduceMap(bool keepSalient);

    void reduceMapWaypointsStart();
    bool reduceReduceMapWaypoints(int distance);

    void printReducedMap() const;
    void printReducedMapWaypoints() const;

private:
    const int MAP_PRINT_WIDTH = 550;
    const int MAP_PRINT_OFFSET = 0;

    SharedEnvironment* env;
    std::vector<int> reducedMap;
    std::vector<std::unordered_map<int,int>> reducedMapWaypoints;

    void reduceMapUpdate();

    bool areNeighboursTraversable(int location) const;;
    bool isNorthEastEmpty(int location, int x, int y) const;
    bool isSouthEastEmpty(int location, int x, int y) const;
    bool isNorthWestEmpty(int location, int x, int y) const;
    bool isSouthWestEmpty(int location, int x, int y) const;

    int countReducedVerticalNeighbours(int location) const;
    int countReducedHorizontalNeighbours(int location) const;
};

#endif // REDUCEMAP_H
