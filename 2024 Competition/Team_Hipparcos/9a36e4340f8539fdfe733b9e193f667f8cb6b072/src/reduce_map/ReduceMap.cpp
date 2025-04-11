#include "reduce_map/ReduceMap.h"

void ReduceMap::reduceMapStart()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : distance + 1; 
            reducedMap.push_back(distance);
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0; y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}

void ReduceMap::reduceMapUpdate()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}

bool ReduceMap::reduceMap(bool keepSalient)
{
    bool changed = false;
    if(keepSalient)
    {
        for(int i = 0; i < reducedMap.size(); i++)
        {
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && areNeighboursTraversable(i) && ((countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) > 1))
            {
                changed = true;
                reducedMap[i] = 0;
            }
        }
    }
    else
    {
        vector<int> tmpReducedMap = reducedMap;
        for(int i = 0; i < reducedMap.size(); i++)
        {   
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && (countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) == 1)
            {
                changed = true;
                tmpReducedMap[i] = 0;
            }
        }
        reducedMap = tmpReducedMap;
    }

    reduceMapUpdate();
    return changed;
}

bool ReduceMap::areNeighboursTraversable(int location) const
{
    int x = location/env->cols;
    int y = location%env->cols;
    int horizontalNeighbourCount = countReducedHorizontalNeighbours(location);
    int verticalNeighbourCount = countReducedVerticalNeighbours(location);
    //Bridge Node
    if((verticalNeighbourCount == 0 && horizontalNeighbourCount == 2) || 
       (verticalNeighbourCount == 2 && horizontalNeighbourCount == 0))
    {
        return false;
    }
    //North + East + NorthEast
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isNorthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + East + SouthEast
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isSouthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + West + SouthWest
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isSouthWestEmpty(location, x, y))
    {
        return false;
    }
    //North + West + NorthWest
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isNorthWestEmpty(location, x, y))
    {
        return false;
    }
    return true;
}

bool ReduceMap::isNorthEastEmpty(int location, int x, int y) const
{
    return !(x > 0 && y + 1 < env->cols && reducedMap[location-env->cols+1] > 0);
}
bool ReduceMap::isSouthEastEmpty(int location, int x, int y) const
{
    return !(x + 1 < env->rows && y + 1 < env->cols && reducedMap[location+env->cols+1] > 0);
}
bool ReduceMap::isNorthWestEmpty(int location, int x, int y) const
{
    return !(x > 0 && y > 0 && reducedMap[location-env->cols-1] > 0);
}
bool ReduceMap::isSouthWestEmpty(int location, int x, int y) const
{
    return !(x + 1 < env->rows && y + 1 > 0 && reducedMap[location+env->cols-1] > 0);
}

int ReduceMap::countReducedVerticalNeighbours(int location) const
{
    int count = 0;
    if(location >= env->cols && reducedMap[location-env->cols] > 0) {count++;}
    if(location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) {count++;}
    return count;
}

int ReduceMap::countReducedHorizontalNeighbours(int location) const
{
    int count = 0;
    int y = location % env->cols;
    if(y > 0 && reducedMap[location-1] > 0) {count++;}
    if(y+1 < env->cols && reducedMap[location+1] > 0) {count++;}
    return count;
}

void ReduceMap::reduceMapWaypointsStart()
{
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
        if(reducedMap[i] > 0)
        {
            int locY = i % env->cols;
            if(i >= env->cols && reducedMap[i-env->cols] > 0)
            {
                int n = i-env->cols;
                reducedMapWaypoints[i][n] = 1;
            }
            if(locY + 1 < env->cols && reducedMap[i+1] > 0)
            {
                int e = i+1;
                reducedMapWaypoints[i][e] = 1;
            }
            if(i+env->cols < env->map.size() && reducedMap[i+env->cols] > 0)
            {
                int s = i+env->cols;
                reducedMapWaypoints[i][s] = 1;
            }
            if(locY > 0 && reducedMap[i-1] > 0)
            {
                int w = i-1;
                reducedMapWaypoints[i][w] = 1;
            }
        }
    }
}

bool ReduceMap::reduceReduceMapWaypoints(int distance)
{
    bool changed = false;
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> neighbours = reducedMapWaypoints[i];
        if(reducedMap[i] > 0 && neighbours.size() == 2)
        {
            vector<int> keys;
            for(auto kv : neighbours)
            {
                keys.push_back(kv.first);
            }
            int n0 = keys[0];
            int n1 = keys[1];
            int newCost = 999999;
            if (reducedMapWaypoints[n0].find(n1) == reducedMapWaypoints[n0].end())
            {
                int locToN1Cost = reducedMapWaypoints[i][n1];
                int n0ToLocCost = reducedMapWaypoints[n0][i];
                newCost = n0ToLocCost + locToN1Cost;
                if(newCost == 2 && (countReducedVerticalNeighbours(i) != 2) && (countReducedHorizontalNeighbours(i) != 2))
                {
                    newCost++;
                }
                if(newCost < distance)
                {
                    reducedMap[i] = 0;
                    reducedMapWaypoints[i].clear();
                    for (auto it = reducedMapWaypoints[n0].begin(); it != reducedMapWaypoints[n0].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n0].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n0][n1] = newCost;

                    for (auto it = reducedMapWaypoints[n1].begin(); it != reducedMapWaypoints[n1].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n1].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n1][n0] = newCost;
                    changed = true;
                }
            }
        }
    }
    return changed;
}

void ReduceMap::printReducedMap() const
{
    std::string blackBox = "■" ;
    std::string whiteBox = "□" ;

    for(int x = 0; x < env->rows; x++)
    {
        std::string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET); y++)
        {
            int loc = x * env->cols + y;
            if(reducedMap[loc] == 0)
            {
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else 
            {
                temp += std::to_string(reducedMap[loc]);
            }
        }
        std::cout << temp << '\n';
    }
    std::cout << "\n\n";
}

void ReduceMap::printReducedMapWaypoints() const
{
    std::string blackBox = "■" ;
    std::string whiteBox = "□" ;
    int cnt {0};

    for(int x = 0; x < env->rows; x++)
    {
        std::string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET); y++)
        {
            int loc = x * env->cols + y;
            std::unordered_map<int,int> tmpMap = reducedMapWaypoints[loc];
            if(reducedMapWaypoints[loc].size() == 0)
            { 
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else
            {
                ++cnt;
                temp += whiteBox;
            }
        }
        std::cout << temp << '\n';
    }
    std::cout << "Waypoint count: " << cnt << '\n';
    std::cout << "\n\n";
}