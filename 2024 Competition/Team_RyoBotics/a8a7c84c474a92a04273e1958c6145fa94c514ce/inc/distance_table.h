#ifndef DISTANCE_TABLE_H
#define DISTANCE_TABLE_H

#include "Types.h"
#include "utils.h"
#include "FastDistanceTable.h"
#include <string>
#include <fstream>
#include <unordered_map>

namespace DefaultPlanner
{
    class DistanceTable
    {
    public:
        static void computeAndSave(SharedEnvironment *env, const std::string &filepath);
        static FastDistanceTable load(const std::string &filepath);
        static FastDistanceTable computeAllPairsShortestPaths(SharedEnvironment *env);

    private:
        static void floydWarshall(std::vector<std::vector<int>> &dist, int V);
    };
}

#endif