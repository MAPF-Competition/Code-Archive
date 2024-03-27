#include <MAPFPlanner.h>
//#include "./../inc/MAPFPlanner.h"
#include <random>
#include <set>



/* TASKS
*  - create blocks to lower to CPU time of search algorithm - Voronoi-diagram ? 
        ! hpa*
*  - create a maximum tenants of blocks (needed for dead ends)
*  - label each node with directions where its okey to move from that node:
*           Go from Top left corner, when u visit a field check 
*           Have to take care of one ways but need to be accessed by both directions
*  - agents should prioritize forward movements
*
*
*/

enum Direction {East, South, West, North};
typedef enum {MIDDLE_ENTRANCE, END_ENTRANCE} EntranceStyle;

const int COST_ONE = 100;
const int MAP_PRINT_WIDTH = 230;
const int MAP_PRINT_OFFSET = 90;

const int REDUCEDMAP_CHUNCK_SIZE = 26;
const int CLOSENESS = 20; //REDUCEDMAP_CHUNCK_SIZE * 0.17;
const int TOO_CLOSE_ON_REDUCEDMAPWAYPOINTS = 14;

const int BEST_REDUCEDMAP_CHUNK_SIZE = 18;
const int BEST_CLOSENESS = 8;

const int CLUSTER_SIZE = 40;
const int MAX_ENTRANCE_WIDTH = 6;

EntranceStyle entranceStyle = END_ENTRANCE;

struct AstarNode
{
    int location; // (row * total number of colums) + (column)
    int direction; // 0 -> east | 1 -> south | 2 -> west | 3 -> north
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    int occupingAgent = -1;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::sortAgents()
{
    for(int i = 0; i < env->num_of_agents-1; i++)
    {
        int min = i;
        for(int j = i + 1; j < env->num_of_agents;j++)
        {
            if(paths[j].empty() && !paths[min].empty())
            {
                min = j;
            }
            else if(!paths[j].empty() && !paths[min].empty())
            {
                if(getManhattanDistance(env->curr_states[j].location, env->goal_locations[j].front().first) < getManhattanDistance(env->curr_states[min].location, env->goal_locations[min].front().first))
                {
                    min = j;
                }
            }
        }
        std::swap(agentPriority[i], agentPriority[min]);
    }
}

void MAPFPlanner::printMap()
{
    string blackBox = "■" ;
    string whiteBox = "□" ;

    string arrowLeft = "←" ;
    string arrowRight = "→" ;
    string arrowDown = "↓" ;
    string arrowUp = "↑" ;

    string arrowLeftDown = "┐" ;
    string arrowRightDown = "┌" ;
    string arrowLeftTop = "┘";
    string arrowRightTop = "└";

    string triUp = "┴";//"▲";
    string triRight = "├";//"►"; 
    string triLeft = "┤";//"◄";
    string triDown = "┬";//"▼";
    string allSides = "┼";//"◆";

    cout << endl << endl;

    NormalNode* currNode;
    for(int x=0; x < env->rows ; x++)
    {
        for(int y=MAP_PRINT_OFFSET; y<min(env->cols,MAP_PRINT_WIDTH+MAP_PRINT_OFFSET); y++)
        {
            
            int loc = x* env->cols + y;
            if(env->map[loc] == 1)
            {
                cout << blackBox;
            }
            else 
            {
                currNode = getNormalNodeByLocation(loc);
                bool dir_right = false;
                bool dir_down = false;
                bool dir_left = false;
                bool dir_up = false;

                for(auto dir : currNode->directions)
                {
                    switch(dir)
                    {
                        case 0:
                            dir_right = true;
                            break;
                        case 1:
                            dir_down = true;
                            break;
                        case 2:
                            dir_left = true;
                            break;
                        case 3:
                            dir_up = true;
                            break;
                    } 
                }
                if(dir_right && dir_down && dir_left && dir_up){cout << allSides;}
                else if(dir_right && dir_down && dir_left && !dir_up){cout << triDown;}
                else if(dir_right && dir_down && dir_up){cout << triRight;}
                else if(dir_right && dir_up && dir_left){cout << triUp;}
                else if(dir_down && dir_up && dir_left){cout << triLeft;}
                else if(dir_down && dir_left){cout <<arrowLeftDown; }
                else if(dir_down && dir_right){cout <<arrowRightDown; }
                else if(dir_up && dir_left){cout <<arrowLeftTop; }
                else if(dir_up && dir_right){cout <<arrowRightTop; }
                else if(dir_up){cout << arrowUp;}
                else if(dir_down){cout << arrowDown;}
                else if(dir_left){cout << arrowLeft;}
                else if(dir_right){cout << arrowRight;}
            }
        }
        cout << endl;
    }
    cout << endl << endl;
}

void MAPFPlanner::printReducedMap()
{
    string blackBox = "■" ;
    string whiteBox = "□" ;

    for(int x = 0; x < env->rows;x++)
    {
        string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET) ;y++)
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
            //temp += reducedMap[loc] == 0 ? blackBox : whiteBox;
            //temp += reducedMap[loc] == 0 ? blackBox : " ";
        }
        cout << temp << endl;
    }
    cout << endl << endl;
}

void MAPFPlanner::printReducedMapWaypoints()
{
    string blackBox = "■" ;
    string whiteBox = "□" ;

    for(int x = 0; x < env->rows;x++)
    {
        string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET) ;y++)
        {
            int loc = x * env->cols + y;
            unordered_map<int,int> tmpMap = reducedMapWaypoints[loc];
            if(reducedMapWaypoints[loc].size() == 0)
            { 
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else
            {
                temp += whiteBox;
            }
            //temp += reducedMap[loc] == 0 ? blackBox : whiteBox;
            //temp += reducedMap[loc] == 0 ? blackBox : " ";
        }
        cout << temp << endl;
    }
    cout << endl << endl;
}

void MAPFPlanner::printRunningMap()
{
    string blackBox = "■" ;
    string whiteBox = "□" ;

    string arrowLeft = "←" ;
    string arrowRight = "→" ;
    string arrowDown = "↓" ;
    string arrowUp = "↑" ;
    string candidates[4] = { arrowRight, arrowDown, arrowLeft, arrowUp  };

    for(int x = 0; x < env->rows;x++)
    {
        string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET) ;y++)
        {
            int loc = x * env->cols + y;
            if(auto it = agentMap.find(loc); it != agentMap.end())
            {  

                temp += candidates[env->curr_states[it->second].orientation];
            }
            else
            { 
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            //temp += reducedMap[loc] == 0 ? blackBox : whiteBox;
            //temp += reducedMap[loc] == 0 ? blackBox : " ";
        }
        cout << temp << endl;
    }
    cout << endl << endl;
}


void MAPFPlanner::initNormalNodesMap()
{
    // Initialize all custom NormalNodes
    for(int i = 0; i < env->map.size(); i++)
    {
        if(env->map[i] == 0)
        {
            NormalNode* node = new NormalNode(i);
            pair<int, std::set<int> > neighborCountWithDirections = getNodeNeighborCountWithAvailableDirections(i);
            node->directions = neighborCountWithDirections.second;
            allNormalNodes.try_emplace(i,node);
        }

    }
    cout << "created nodes for all" << endl;
}

void MAPFPlanner::connectNeighborNodes()
{
    NormalNode* currNode;
    // Connect all NormalNodes with neighbouring NormalNodes
    for(int i = 0; i < env->map.size(); i++)
    {
        if(env->map[i] == 0)
        {
            if (auto iterator = allNormalNodes.find(i); iterator != allNormalNodes.end())
            {
                currNode = iterator->second;
            }
            

            // Set Node on right side
            if( i+1 < env->map.size()) 
            {
                if(env->map[i+1] == 0)
                {
                    if (auto node = allNormalNodes.find(i+1); node != allNormalNodes.end())
                    {
                        currNode->rightNode = node->second;
                    }
                }
            }

            // Set Node on bottom side
            if( i+env->cols < env->map.size()) 
            {
                if(env->map[i+env->cols] == 0)
                {
                    if (auto node = allNormalNodes.find(i+env->cols); node != allNormalNodes.end())
                    {
                        currNode->bottomNode = node->second;
                    }
                }
            }

            // Set Node on left side
            if( i-1 >= 0) 
            {
                if(env->map[i-1] == 0)
                {
                    if (auto node = allNormalNodes.find(i-1); node != allNormalNodes.end())
                    {
                        currNode->leftNode = node->second;
                    }
                }
            }

            // Set Node on top side
            if( i-env->cols >= 0) 
            {
                if(env->map[i-env->cols] == 0)
                {
                    if (auto node = allNormalNodes.find(i-env->cols); node != allNormalNodes.end())
                    {
                        currNode->topNode = node->second;
                    }
                }
            }
        }
    }
    cout << "connected all neighbors" << endl;
}

void MAPFPlanner::fullOneWay()
{
    NormalNode* currNode;
    for(int loc=0; loc<env->map.size();loc++)
    {
        if(env->map[loc] == 0)
        {
            currNode = getNormalNodeByLocation(loc);
            int locX = loc/env->cols;
            int locY = loc%env->cols;
            int candidates[4] = { loc + 1,loc + env->cols, loc - 1, loc - env->cols};
            for(int i = 0; i<4;i++)
            {
                int forward_loc = candidates[i];
                if(forward_loc >= 0 && forward_loc < env->map.size() && validateMove(forward_loc, loc))
                {
                    if( 
                        (locX % 2) == 0 && i == 0 ||
                        (locX % 2) != 0 && i == 2 ||
                        (locY % 2) == 0 && i == 1 ||
                        (locY % 2) != 0 && i == 3 ||
                        ((locX % 2) == 0 && !validateMove(candidates[0], loc) && (locY % 2) == 0 && !validateMove(candidates[1], loc)) ||
                        ((locX % 2) == 0 && !validateMove(candidates[0], loc) && (locY % 2) != 0 && !validateMove(candidates[3], loc)) ||
                        ((locX % 2) != 0 && !validateMove(candidates[2], loc) && (locY % 2) == 0 && !validateMove(candidates[1], loc)) ||
                        ((locX % 2) != 0 && !validateMove(candidates[2], loc) && (locY % 2) != 0 && !validateMove(candidates[3], loc))
                     )
                    {
                        currNode->directions.emplace(i);
                    } 
                }
            }
        }
    }
}

void MAPFPlanner::switchingLimitedOneWayWarehouseSetup()
{
    NormalNode* currNode;
    bool changeDirectionForColumn = false;
    bool changeDirectionForRow = false;
    bool rowFound, colFound;

    for(int x = 0; x < env->rows; x++)
    {
        if(rowFound)
        {
            changeDirectionForRow = !changeDirectionForRow;
        }
        rowFound = false;
        changeDirectionForColumn = false;
        for(int y = 0; y < env->cols; y++)
        {
            if(colFound)
            {
                changeDirectionForColumn = !changeDirectionForColumn;
            }
            colFound = false;
            int loc = x* env->cols + y;
            if(env->map[loc] == 0)
            {
                currNode = getNormalNodeByLocation(loc);
                pair<int, std::set<int> > neighborCountWithDirections = getNodeNeighborCountWithAvailableDirections(loc);
                //currNode->directions = neighborCountWithDirections.second;
                if(neighborCountWithDirections.second.size() == 2) // single-roads
                {
                    int dirSum = 0;
                    for(auto& dir : neighborCountWithDirections.second)
                    {
                        dirSum += dir;
                    }
                    if(dirSum % 2 == 0) // Straight
                    {
                        if(dirSum == 2)     // Horizontal
                        {
                            //go right only
                            std::set<int> newDirections;
                            if(!changeDirectionForRow){ newDirections.emplace(0);}
                            else{ newDirections.emplace(2);}
                            currNode->directions = newDirections;
                            currNode->isOneWay = true;
                            rowFound = true;

                        }
                        else if(dirSum == 4) // Vertical
                        {
                            //go down only
                            std::set<int> newDirections;
                            if(!changeDirectionForColumn){ newDirections.emplace(1);}
                            else{ newDirections.emplace(3);}
                            currNode->directions = newDirections;
                            currNode->isOneWay = true;
                            colFound = true;
                        }     
                    }
                }
            }
        }
    }

}

void MAPFPlanner::limitedOneWayWarehouseSetup()
{
    NormalNode* currNode;
    for(int i = 0; i < env->map.size(); i++)
    {
        if(env->map[i] == 0)
        {
            currNode = getNormalNodeByLocation(i);
            pair<int, std::set<int> > neighborCountWithDirections = getNodeNeighborCountWithAvailableDirections(i);
            //currNode->directions = neighborCountWithDirections.second;
            if(neighborCountWithDirections.second.size() == 1) // dead-end
            {
                
            } 
            else if(neighborCountWithDirections.second.size() == 2) // single-roads
            {

                int dirSum = 0;
                for(auto& dir : neighborCountWithDirections.second)
                {
                    dirSum += dir;
                }
                if(dirSum % 2 == 0) // Straight
                {
                    if(dirSum == 2)     // Horizontal
                    {
                        //go right only
                        std::set<int> newDirections;
                        newDirections.emplace(0);
                        currNode->directions = newDirections;
                        currNode->isOneWay = true;

                    }
                    else if(dirSum == 4) // Vertical
                    {
                        //go down only
                        std::set<int> newDirections;
                        newDirections.emplace(1);
                        currNode->directions = newDirections;
                        currNode->isOneWay = true;
                    }

                }
                else // Corner
                {

                }
            }
            else if(neighborCountWithDirections.second.size() == 3)  // junction
            {

            }
            else if(neighborCountWithDirections.second.size() == 4 && neighborCountWithDirections.first == 4) // cross-roads
            {
            
            }
            else // plain
            {
                //currNode->directions = neighborCountWithDirections.second;
            }
        }
    }
}

void MAPFPlanner::reduceMapStart()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows;x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols;y++)
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
    for(int y = env->cols-1; y >= 0;y--)
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

void MAPFPlanner::reduceMapUpdate()
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

bool MAPFPlanner::reduceMap(bool keepSalient)
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

bool MAPFPlanner::areNeighboursTraversable(int location)
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

bool MAPFPlanner::isNorthEastEmpty(int location, int x, int y)
{
    if(x > 0 && y + 1 < env->cols && reducedMap[location-env->cols+1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isSouthEastEmpty(int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 < env->cols && reducedMap[location+env->cols+1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isNorthWestEmpty(int location, int x, int y)
{
    if(x > 0 && y > 0 && reducedMap[location-env->cols-1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isSouthWestEmpty(int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 > 0 && reducedMap[location+env->cols-1] > 0){return false;}
    return true;
}

int MAPFPlanner::countReducedVerticalNeighbours(int location)
{
    int count = 0;
    if(location >= env->cols && reducedMap[location-env->cols] > 0) {count++;}
    if(location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) {count++;}
    return count;
}

int MAPFPlanner::countReducedHorizontalNeighbours(int location)
{
    int count = 0;
    int y = location % env->cols;
    if(y > 0 && reducedMap[location-1] > 0) {count++;}
    if(y+1 < env->cols && reducedMap[location+1] > 0) {count++;}
    return count;
}

void MAPFPlanner::reduceMapWaypointsStart()
{
    for(int i = 0; i < env->map.size(); i++)
    {
        unordered_map<int,int> locationDistanceMap;
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

bool MAPFPlanner::reduceReduceMapWaypoints(int distance)
{
    bool changed = false;
    for(int i = 0; i < env->map.size();i++)
    {
        unordered_map<int,int> neighbours = reducedMapWaypoints[i];
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
/*
void MAPFPlanner::createAbstractMap()
{
    buildClustersAndEntrances();
    createAbstractGraph();
}

int MAPFPlanner::getLocation(int row, int column)
{
    assert(row >= 0 && row < env->rows);
    assert(column >= 0 && column < env->cols);
    return row * env->cols + column;
}

int MAPFPlanner::getNode(int row, int column)
{
    int location = getLocation(row, column)
    return env->map[location];
}

void MAPFPlanner::buildClustersAndEntrances()
{
    int row=0, col=0, clusterId = 0;
    int horizontalSize, verticalSize;
    int clusterSize = CLUSTER_SIZE;
    int entranceId = 0;

    for(int y = 0; y < env->cols; y += clusterSize)
    {
        col = 0;
        for(int x = 0; x < env->rows; x+= clusterSize)
        {
            horizontalSize = min(clusterSize, env->rows - x);
            verticalSize = min(clusterSize, env->cols - y );
            Cluster cluster(clusterId++, row, col, x, y, horizontalSize, verticalSize);
            clusters.push_back(cluster);

            if(y > 0 && y < env->cols)
            {
                createHorizontalEntrances(x, x + horizontalSize-1, y-1, row-1, col, &entranceId);
            }
            if(x > 0 && x < env->rows)
            {
                createVerticalEntrances(y, y + verticalSize -1, x-1, row, col - 1, &entranceId);
            }
            col++;
        }
        row++;
    }
    
}

void MAPFPlanner::createHorizontalEntrances(int start, int end, int latitude, int row, int col, int *lastId)
{
    int node1, node2,
    for(int i = start; i <= end; i++)
    {
        node1 = getNode(latitude, i);
        node2 = getNode(latitude+1, i);
        if(node1 == 1 || node2 == 1)
        {
            continue;
        }
        int entranceStart = i;
        while(true)
        {
            i++;
            if( i >= end)
            {
                break;
            }
            node1 = getNode(latitude, i);
            node2 = getNode(latitude+1, i);
            if(node1 == 1 || node2 == 1 || i >= end)
            {
                break;
            }
        }
        if(entranceStyle == END_ENTRANCE && (i - entranceStart) > MAX_ENTRANCE_WIDTH)
        {
            //create two new entrances one for each end
            Entrance entrance1((*lastId)++, -1, -1, latitude, entranceStart, 
                                getLocation(latitude, entranceStart),
                                getLocation(latitude+1, entranceStart),
                                row, col, 1,  0);
            entrances.push_back(entrance1);

            Entrance entrance2((*lastId)++, -1, -1, latitude, (i-1), 
                                getLocation(latitude, i-1),
                                getLocation(latitude+1, i-1),
                                row, col, 1,  0);
             entrances.push_back(entrance2);
        }
        else
        {
            Entrance entrance1((*lastId)++, -1, -1, latitude, ((i - 1) + entranceStart)/2, 
                                getLocation(latitude, ((i - 1) + entranceStart)/2),
                                getLocation(latitude+1, ((i - 1) + entranceStart)/2),
                                row, col, (i - entranceStart),  0);
            entrances.push_back(entrance1);

        }
    }
}

void MAPFPlanner::createVerticalEntrances(int start, int end, int meridian, int row, int col, int *lastId)
{
    int node1, node2,
    for(int i = start; i <= end; i++)
    {
        node1 = getNode(i, meridian);
        node2 = getNode(i, meridian +1);
        if(node1 == 1 || node2 == 1)
        {
            continue;
        }
        int entranceStart = i;
        while(true)
        {
            i++;
            if( i >= end)
            {
                break;
            }
            node1 = getNode(i, meridian);
            node2 = getNode(i, meridian+1);
            if(node1 == 1 || node2 == 1 || i >= end)
            {
                break;
            }
        }
        if(entranceStyle == END_ENTRANCE && (i - entranceStart) > MAX_ENTRANCE_WIDTH)
        {
            //create two new entrances one for each end
            Entrance entrance1((*lastId)++, -1, -1, entranceStart, meridian, 
                                getLocation(entranceStart, meridian),
                                getLocation(entranceStart, meridian + 1),
                                row, col, 1,  1);
            entrances.push_back(entrance1);

            Entrance entrance2((*lastId)++, -1, -1, (i-1), meridian, 
                                getLocation(i-1, meridian),
                                getLocation(i-1, meridian + 1),
                                row, col, 1,  1);
             entrances.push_back(entrance2);
        }
        else
        {
            Entrance entrance1((*lastId)++, -1, -1, ((i - 1) + entranceStart)/2, meridian, 
                                getLocation(((i - 1) + entranceStart)/2, meridian),
                                getLocation(((i - 1) + entranceStart)/2, meridian + 1),
                                row, col, (i - entranceStart),  1);
            entrances.push_back(entrance1);
        }
    }
}

void MAPFPlanner::linkEntrancesAndClusters()
{
    
}
*/

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "start to initialize" << endl;
    initNormalNodesMap();
    connectNeighborNodes();

    heatMap = std::vector<int>(env->map.size(), 0);
    selfTime = -1;
    reducedMap.clear();
    l0reducedMap.clear();
    l1reducedMap.clear();
    l2reducedMap.clear();
    reducedMapWaypoints.clear();
    l1reducedMapWaypoints.clear();
    l2reducedMapWaypoints.clear();

    

    if(env->map.size() > 2000 || env->num_of_agents > 250)
    {
        fullOneWay();
        cout << "Original map:" << endl;
        //printMap();
        reduceMapStart();

        // reduceNormalWithSalients
        /*
        bool reducing = reduceMap(true);
        while(reducing)
        {
            //printReducedMap();
            reducing = reduceMap(true);
            //reduceMap(false);
        }
        printReducedMap();
        */

        //Waypoints
        
        while(reduceMap(false))
        {}
        while(reduceMap(true))
        {
            reduceMap(false);
        }
        l0reducedMap = reducedMap;
        cout << "l0 reduced map:" << endl; 
        printReducedMap();

        reduceMapWaypointsStart();
        while(reduceReduceMapWaypoints(8))
        {}
        l1reducedMap = reducedMap;
        l1reducedMapWaypoints = reducedMapWaypoints;
        cout << "l1 reduced map:" << endl;
        printReducedMap();
        //cout << "reducedMapWaypoints: " << endl << endl;
        //printReducedMapWaypoints();
        /*
        while(reduceReduceMapWaypoints(100))
        {}
        l2reducedMap = reducedMap;
        l2reducedMapWaypoints = reducedMapWaypoints;
        cout << "l2 reduced map:" << endl;
        printReducedMap();
        */
    }
    else
    {
        //limitedOneWayWarehouseSetup();
        //switchingLimitedOneWayWarehouseSetup();
        //fullOneWay();

        printMap();
    }
    

    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,pair<int,int>>> path; // location and orientation
        paths.push_back(path);
        pair<int,int> waypointTmp = make_pair(-1,-1);
        vector<pair<int, int> > waypointDest;
        waypointDest.push_back(waypointTmp);
        waypointDestinations.push_back(waypointDest);
        simpleWaypointDestinations.push_back(-1);
        list<int> waypoint;
        waypoints.push_back(waypoint);
    }

    /////////////////////////////////
    ///INFOS ABOUT MAP AND AGENTS////
    /////////////////////////////////

    cout << "number of agents: " << env->num_of_agents << endl;
    cout << "map name: " << env->map_name << endl;
    cout << "map size: " << env->rows << "x" << env->cols << " == " << env->map.size() << endl;
    int counter = 0;
    for(int i = 0; i < env->map.size(); i++)
    {
        if(env->map[i] == 0)
        {
            counter++;
        }
    }
    cout << "map has traversable nodes: " << counter << endl;
    cout << "map has NON-traversable nodes: " << env->map.size() - counter << endl;

    cout << "planner initialize done" << endl;

    //string allSides = "➹"; cout << allSides << endl;

}


/*
    num_of_robots:
    int, the total team size.
    rows: int, the number of rows of the map.
    cols: int, the number of columns of the map.
    map_name: string, the map file name.
    map: vector of int, stores the map. 0: traversable ; 1: non-traversable
    file_storage_path: string, use for indicating the path for file storage, refer to section 'Local Preprocessing and Large Files'.
    goal locations, vector of vector of pair <int,int>: current tasks locations allocate to each robot.
    current_timestep: int, the current time step that our system already simulated the robots' actions.
    curr_states: vector of State, the current state for each robot at the current time step,

*/
int debuggingTimeStep = 49;
// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{   
    const auto startPlanning{std::chrono::steady_clock::now()};
    selfTime++;
    //planWithSingleIteration(time_limit, actions);
    if(env->map.size() > 2000 || env->num_of_agents > 250)
    {
        //planWithMultipleIterationConflictChecksOnReducedMap(time_limit, actions);
        planWithMultipleIterationConflictChecksOnReducedMapWaypoints(time_limit, actions);
        //planWithMultipleIterationConflictChecksOnReducedMapWaypointsImprovedConflictResolution(time_limit, actions);
        
        //printRunningMap();

    }else
    {
        planWithMultipleIterationConflictChecks(time_limit, actions);
    }
    const auto endPlanning{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> planningSeconds {endPlanning - startPlanning};
    cout << "PLANNING FOR: " << env->num_of_agents << " agents took: " << planningSeconds.count() << endl;
    return;
}


void MAPFPlanner::planWithSingleIteration(int time_limit, std::vector<Action> & actions)
{
    NormalNode* currNode;
    NormalNode* nextNode;
    unordered_map<int,int> locationTakenByAgent;

    vector<pair<int,int>> nextStepForEachAgent;
    vector<int> retryFindPathAgents;
    
    //DEBUGGING
    debug(49, 787);
    debug(49, 788);
    debug(50, 788);


    //PLANNING
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        bool emptyTurn = false;
        currNode = getNormalNodeByLocation(env->curr_states[i].location);
        pair<int,int> nextStep = make_pair(-1,-1);
        
        if (env->goal_locations[i].empty()) 
        {
            //nextStep = make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
            continue;
        } 
        else 
        {   
            if(paths[i].empty())
            {
                emptyTurn = true;
                cout << "PLANNING NEW SINGLE PATH FOR ROBOT: " << i  << endl;
                paths[i] = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first);

                //IF NOT FOUND ANY PATH
                if(paths[i].empty())
                {
                    cout << "DIDN'T FIND PATH FOR ROBOT " << i <<  ". TURN AND RETRY NEXT ROUND." << endl; 
                    actions[i] = currNode->isOneWay ? Action::W : Action::CR;
                    if(auto it = locationTakenByAgent.find(env->curr_states[i].location); it != locationTakenByAgent.end())
                    {
                        for(int j = 0; j < i; j++)
                        {
                            if(nextStepForEachAgent[j].first == it->first)
                            {
                                cout << "Robot " << j << " makes correction and WAITES. Original action would have been " << actions[j] << endl;
                                clearPath(paths[j]);
                                actions[j] = Action::W;
                                /*
                                pair<bool,int> isNextPlaceTakenByOtherAgent;
                                int TESTcounter = 0;
                                vector<int> backUpdateAgents;
                                int tmpLoc = env->curr_states[i].location;
                                do
                                {

                                    if(TESTcounter > 1000){break;}
                                    isNextPlaceTakenByOtherAgent = isOtherAgentTakenPlace(locationTakenByAgent, tmpLoc);
                                    
                                    if(isNextPlaceTakenByOtherAgent.first)
                                    {
                                        clearPath(paths[isNextPlaceTakenByOtherAgent.second]);
                                        actions[isNextPlaceTakenByOtherAgent.second] = Action::W;
                                        tmpLoc = env->curr_states[isNextPlaceTakenByOtherAgent.second].location;
                                        //IT DOES NOT BACKUPDATE ! NEED TO FIGURE OUT IF EVEN NEED BACK-CHAIN UPDATE THE NEW NEXT LOCATIONS
                                        //locationTakenByAgent[tmpLoc] = isNextPlaceTakenByOtherAgent->second;
                                        backUpdateAgents.push_back(isNextPlaceTakenByOtherAgent.second);
                                    }
                                    TESTcounter++ 
                                } while(isNextPlaceTakenByOtherAgent.first);
                                
                                // BACKUPDATE ALL BUT LAST
                                for(auto agentNum : backUpdateAgents)
                                {
                                    locationTakenByAgent[env->curr_states[agentNum].location] = agentNum;
                                }
                                //LAST IN CHAIN
                                for(int j = 0;j < env->num_of_agents; j++)
                                {
                                    if(env->curr_states[j].location == tmpLoc)
                                    {
                                        locationTakenByAgent[tmpLoc] = j;
                                    }
                                }
                                */
                            }
                        }
                    }
                    else
                    {
                        locationTakenByAgent[env->curr_states[i].location] = i;
                    }
                    nextStepForEachAgent.push_back(nextStep);
                    continue;
                }
                else
                {
                    cout << "PATH FOUND FOR : " << env->goal_locations[i].front().first  << endl;
                    for (auto pa : paths[i])
                    {
                        cout << "       TIMESTEP: " << pa.first << " <-> LOCATION: " << pa.second.first << endl;
                    }
                }
            }
        }

        nextStep = paths[i].front().second;
        paths[i].pop_front();
        nextStepForEachAgent.push_back(nextStep);
        
        ////////////////////////////
        /// ASSIGNING THE MOVES ////
        ////////////////////////////
        if(nextStep.first > 0)
        {
            if (nextStep.first != env->curr_states[i].location)
            {
                actions[i] = Action::FW; //forward action
            } 
            else if (nextStep.second!= env->curr_states[i].orientation)
            {
                int incr = nextStep.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; //CCR--clockwise rotate
                } 
            }
            


            if(auto it = locationTakenByAgent.find(nextStep.first); it != locationTakenByAgent.end())
            {
                if(actions[i] == 1 || actions[i] == 2)
                {
                    for(int j = 0; j < i; j++)
                    {
                        cout << j << " -> " << nextStepForEachAgent[j].first << " && " << actions[j] << endl;
                        if(nextStepForEachAgent[j].first == it->first)
                        {
                            cout << "Robot " << j << " makes correction and WAITES. Original action would have been " << actions[j] << endl;
                            clearPath(paths[j]);
                            actions[j] = Action::W;
                            locationTakenByAgent[env->curr_states[j].location] = j;



                            break;
                        }
                    }
                }
                else
                {
                    cout << "THIS ROBOT WAITS! For robot " << i << " the next step is: " << nextStep.first << ", " << nextStep.second << " which would have been a " << actions[i] << " for " << paths[i].front().first << " time step. The size of its path is " << paths[i].size() << endl;
                    clearPath(paths[i]);
                    actions[i] = Action::W;
                    pair<bool,int> isNextPlaceTakenByOtherAgent;
                    vector<int> backUpdateAgents;
                    int tmpLoc = env->curr_states[i].location;
                    do
                    {
                        isNextPlaceTakenByOtherAgent = isOtherAgentTakenPlace(locationTakenByAgent, tmpLoc);
                        
                        if(isNextPlaceTakenByOtherAgent.first)
                        {
                            clearPath(paths[isNextPlaceTakenByOtherAgent.second]);
                            actions[isNextPlaceTakenByOtherAgent.second] = Action::W;
                            tmpLoc = env->curr_states[isNextPlaceTakenByOtherAgent.second].location;
                            //IT DOES NOT BACKUPDATE ! NEED TO FIGURE OUT IF EVEN NEED BACK-CHAIN UPDATE THE NEW NEXT LOCATIONS
                            //locationTakenByAgent[tmpLoc] = isNextPlaceTakenByOtherAgent->second;
                            backUpdateAgents.push_back(isNextPlaceTakenByOtherAgent.second);
                        } 
                    } while(isNextPlaceTakenByOtherAgent.first);
                    
                    // BACKUPDATE ALL BUT LAST
                    for(auto agentNum : backUpdateAgents)
                    {
                        locationTakenByAgent[env->curr_states[agentNum].location] = agentNum;
                    }
                    //LAST IN CHAIN
                    for(int j = 0;j < env->num_of_agents; j++)
                    {
                        if(env->curr_states[j].location == tmpLoc)
                        {
                            locationTakenByAgent[tmpLoc] = j;
                        }
                    }
                    continue;
                }
            }
            else
            {
                locationTakenByAgent[nextStep.first] = i;
            }
            cout << "Robot " << i << " move to: " << nextStep.first << ", " << nextStep.second << " which is a " << actions[i] << " for " << paths[i].front().first -1 << " time step. The size of its path is " << paths[i].size() << endl;
        }
    }
}

void MAPFPlanner::planWithMultipleIterationConflictChecks(int time_limit, std::vector<Action> & actions)
{
    //clocking
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    reservation.clear();
    
    // setup for the reservations
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,pair<int,int>>> path = paths[i];
        int lastLocation = -1;
        int t = 1;
        for(auto p : path)
        {
            reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
            if(lastLocation != -1)
            {
                reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
            }
            lastLocation = p.second.first;
            t++;
        }
    }

    updateAgentMap();

    // updating paths for agents without any goal locations
    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(env->goal_locations[i].empty())
        {
            list<pair<int,pair<int,int>>> path;
            pair<int,int> tmpPair = make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
            path.emplace_front(make_pair(selfTime + 1, tmpPair));
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
            paths[i] = path;
        }
    }

    // making new paths 
    for(int i = 0; i < env->num_of_agents; i++)
    {
        list<pair<int,pair<int,int>>> path = paths[i];
        if(paths[i].empty())
        {
            if(!env->goal_locations[i].empty())
            {
                path = spaceTimeSingleAgentAstar(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                env->goal_locations[i].front().first,
                                                reservation);
            }
            int lastLocation = -1;
            int t = 1;
            for(auto p : path)
            {
                reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
                if(lastLocation != -1)
                {
                    reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
                }
                lastLocation = p.second.first;
                t++;
            }
            paths[i] = path; 
        }
        if(!path.empty())
        {
            if (path.front().second.first != env->curr_states[i].location)
            {
                actions[i] = Action::FW; //forward action
            } 
            else if (path.front().second.second != env->curr_states[i].orientation)
            {
                int incr = path.front().second.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; //CCR--clockwise rotate
                } 
            }
        }
    }
    
    pair<bool, std::vector<bool>> possibleConflicts = findConflicts();
    while(possibleConflicts.first)
    {
        for(int i = 0; i < env->num_of_agents; i++)
        {
            list<pair<int,pair<int,int>>> path = paths[i];
            if(path.empty() || possibleConflicts.second[i])
            {
                int lastLocation = -1;
                int t = 1;
                for(auto p : path)
                {
                    //remove tuple from set
                    std::tuple<int,int,int> tup = make_tuple(p.second.first, -1, selfTime + t);
                    for (auto it = reservation.begin(); it != reservation.end();)
                    {
                    if (*it == tup)
                        it = reservation.erase(it);
                    else
                        ++it;
                    }

                    if(lastLocation != -1)
                    {
                        tup = make_tuple(lastLocation, p.second.first, selfTime + t);
                        for (auto it = reservation.begin(); it != reservation.end();)
                        {
                        if (*it == tup)
                            it = reservation.erase(it);
                        else
                            ++it;
                        }
                    }
                    lastLocation = p.second.first;
                    t++;
                }
                path.clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                path.emplace_front(make_pair(selfTime, make_pair(env->curr_states[i].location, nextOrientation)));
                paths[i] = path;
                reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
                actions[i] = Action::CR;
            }
        }

        possibleConflicts = findConflicts();
    }

    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(!paths[i].empty())
        {
            paths[i].pop_front();
        }
    }

    return;
}

void MAPFPlanner::planWithMultipleIterationConflictChecksOnReducedMap(int time_limit, std::vector<Action> & actions)
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    reservation.clear();

    // setup for the reservations
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,pair<int,int>>> path = paths[i];
        int lastLocation = -1;
        int t = 1;
        for(auto p : path)
        {
            reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
            if(lastLocation != -1)
            {
                reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
            }
            lastLocation = p.second.first;
            t++;
        }
    }

     updateAgentMap();

    // updating paths for agents without any goal locations
    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(env->goal_locations[i].empty())
        {
            list<pair<int,pair<int,int>>> path;
            pair<int,int> tmpPair = make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
            path.emplace_front(make_pair(selfTime + 1, tmpPair));
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
            paths[i] = path;
        }
    }

    // making new paths 
    for(int i = 0; i < env->num_of_agents; i++)
    {
        list<pair<int,pair<int,int>>> path = paths[i];
        if(waypointDestinations[i] != env->goal_locations[i])
        {
            //cout << "generating waypoints for agent: " << i << endl;
            int waypointStart = BFStoReducedMap(env->curr_states[i].location);
            int waypointEnd = BFStoReducedMap(env->goal_locations[i][0].first);
            waypoints[i] = reducedMapAStar(waypointStart, waypointEnd,REDUCEDMAP_CHUNCK_SIZE);
            //cout << "generated " << waypoints[i].size() << " waypoints for agent: " << i << endl;
            /*for(auto waypoint : waypoints[i])
            {
                cout << "      location: " << waypoint << "  which is this far from agent: " << getManhattanDistance(env->curr_states[i].location, waypoint) << endl;
            }*/
            waypointDestinations[i] = env->goal_locations[i];
            paths[i].clear();
        }

        if(paths[i].empty())
        {
            if(!env->goal_locations[i].empty())
            {
                while(!waypoints[i].empty() && (getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < CLOSENESS))
                {
                    waypoints[i].pop_front();
                }
                if(!waypoints[i].empty())
                {
                    //cout << "started planning for next waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstar(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                waypoints[i].front(),
                                                reservation);
                }
                else
                {
                    //cout << "started planning for last waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstar(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                env->goal_locations[i].front().first,
                                                reservation);
                }
            }
            int lastLocation = -1;
            int t = 1;
            for(auto p : path)
            {
                reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
                if(lastLocation != -1)
                {
                    reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
                }
                lastLocation = p.second.first;
                t++;
            }
            paths[i] = path; 
        }
        if(!path.empty())
        { 
            if (path.front().second.first != env->curr_states[i].location)
            {
                actions[i] = Action::FW; //forward action
            } 
            else if (path.front().second.second != env->curr_states[i].orientation)
            {
                int incr = path.front().second.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; //CCR--clockwise rotate
                } 
            }
        }
    }
    
    pair<bool, std::vector<bool>> possibleConflicts = findConflicts();
    while(possibleConflicts.first)
    {
        for(int i = 0; i < env->num_of_agents; i++)
        {
            list<pair<int,pair<int,int>>> path = paths[i];
            if(path.empty() || possibleConflicts.second[i])
            {
                int lastLocation = -1;
                int t = 1;
                for(auto p : path)
                {
                    //remove tuple from set
                    std::tuple<int,int,int> tup = make_tuple(p.second.first, -1, selfTime + t);
                    for (auto it = reservation.begin(); it != reservation.end();)
                    {
                    if (*it == tup)
                        it = reservation.erase(it);
                    else
                        ++it;
                    }

                    if(lastLocation != -1)
                    {
                        tup = make_tuple(lastLocation, p.second.first, selfTime + t);
                        for (auto it = reservation.begin(); it != reservation.end();)
                        {
                        if (*it == tup)
                            it = reservation.erase(it);
                        else
                            ++it;
                        }
                    }
                    lastLocation = p.second.first;
                    t++;
                }
                path.clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                path.emplace_front(make_pair(selfTime, make_pair(env->curr_states[i].location, nextOrientation)));
                paths[i] = path;
                reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
                actions[i] = Action::CR;
            }
        }

        possibleConflicts = findConflicts();
    }

    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(!paths[i].empty())
        {
            paths[i].pop_front();
        }
    }

    return;

}

void MAPFPlanner::planWithMultipleIterationConflictChecksOnReducedMapWaypoints(int time_limit, std::vector<Action> & actions)
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    reservation.clear();
    agentMap.clear();
    vector<int> agentPriority;

    // setup for the reservations
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        int emptyNeighbour = 0;
        agentMap[env->curr_states[i].location] = i;
        list<pair<int,pair<int,int>>> path = paths[i];

        int location = env->curr_states[i].location;
        int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
        if (auto iterator = agentMap.find(location+1); iterator != agentMap.end()){ if(env->map[location+1] == 0){emptyNeighbour++;}}
        if (auto iterator = agentMap.find(location+env->cols); iterator != agentMap.end()){ if(env->map[location+env->cols] == 0){emptyNeighbour++;}}
        if (auto iterator = agentMap.find(location-1); iterator != agentMap.end()){ if(env->map[location-1] == 0){emptyNeighbour++;}}
        if (auto iterator = agentMap.find(location-env->cols); iterator != agentMap.end()){ if(env->map[location-env->cols] == 0){emptyNeighbour++;}}
        agentEmptySpaceMap[i] = emptyNeighbour;

        agentPriority.push_back(i);
        int lastLocation = -1;
        int t = 1;
        for(auto p : path)
        {
            reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
            if(lastLocation != -1)
            {
                reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
            }
            lastLocation = p.second.first;
            t++;
        }
        /*if(path.empty())
        {
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
        }*/
    }
    
    std::sort(agentPriority.begin(), agentPriority.end(), [this](int i, int j)
    {
        if(this->paths[i].empty() && this->paths[j].empty())
        {
            return this->agentEmptySpaceMap[i] < this->agentEmptySpaceMap[j];
        }
        return this->paths[i].size() > this->paths[j].size();
    });
    /*
    std::sort(agentPriority.begin(), agentPriority.end(), [this](int i, int j)
    {
        if(this->paths[i].empty() && this->paths[j].empty())
        {
            return this->getManhattanDistance(this->env->curr_states[i].location, this->env->goal_locations[i].front().first) < this->getManhattanDistance(this->env->curr_states[j].location, this->env->goal_locations[j].front().first);
        }
        return this->paths[i].size() > this->paths[j].size();
    });
    */
    //updateAgentMap();

    // updating paths for agents without any goal locations
   
    /*for(int i = 0; i < env->num_of_agents; i++)
    {
        if(env->goal_locations[i].empty())
        {
            list<pair<int,pair<int,int>>> path;
            pair<int,int> tmpPair = make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
            path.emplace_front(make_pair(selfTime + 1, tmpPair));
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
            paths[i] = path;
        }
    }*/

    // making new paths 
    const auto startPathPlanning{std::chrono::steady_clock::now()};
    const auto checkForSkipStart{std::chrono::steady_clock::now()};
    bool skipPathfinding = false;
    int agentMapCalc = 0;
    for(int i : agentPriority)
    //for(int i = 0; i < env->num_of_agents; i++)
    {
        //cout << i <<  " -> " <<  paths[i].size() << " -> " << getManhattanDistance(env->curr_states[i].location, env->goal_locations[i].front().first) << endl ;
        list<pair<int,pair<int,int>>> path = paths[i];
        if(skipPathfinding && path.empty())
        {
            paths[i].clear();
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
            continue;
        }
        if(simpleWaypointDestinations[i] != env->goal_locations[i][0].first)
        {
            if(selfTime == 0 && agentMapCalc > 1000 ){continue;}
            if(selfTime == 1 && agentMapCalc > 2000 ){continue;}
            if(selfTime == 2 && agentMapCalc > 2000 ){continue;}
            if(selfTime == 3 && agentMapCalc > 2000 ){continue;}
            if(selfTime == 4 && agentMapCalc > 2000 ){continue;}
            if(selfTime == 5 && agentMapCalc > 3000 ){continue;}
            //cout << "generating waypoints for agent: " << i << endl;
            int waypointStart = BFStoReducedMap(env->curr_states[i].location);
            int waypointEnd = BFStoReducedMap(env->goal_locations[i][0].first);
            waypoints[i] = reducedMapWaypointsAStar(waypointStart, waypointEnd,1);
            //waypoints[i] = reducedMapAStar(waypointStart, waypointEnd,5);
            simpleWaypointDestinations[i] = env->goal_locations[i][0].first;
            paths[i].clear();
        }

        if(!waypoints[i].empty() && getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < TOO_CLOSE_ON_REDUCEDMAPWAYPOINTS )
        {
            paths[i].clear();
        }  
        if(paths[i].empty() /*&& !skipPathfinding && agentMapCalc <= 1500*/)
        {
        const auto pathStart{std::chrono::steady_clock::now()};
            if(!env->goal_locations[i].empty())
            {
                agentMapCalc++;
                while(!waypoints[i].empty() && (getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < CLOSENESS))
                {
                    waypoints[i].pop_front();
                }
                if(waypoints[i].size() == 1 && getManhattanDistance(env->curr_states[i].location, env->goal_locations[i].front().first) < CLOSENESS)
                {
                    waypoints[i].pop_front();
                }
                if(!waypoints[i].empty())
                {
                    //cout << "started planning for next waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstarWithProximity(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                waypoints[i].front(),
                                                reservation,
                                                8);
                }
                else
                {
                    //cout << "started planning for last waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstarWithProximity(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                env->goal_locations[i].front().first,
                                                reservation,
                                                0);
                }
            }
            int lastLocation = -1;
            int t = 1;
            for(auto p : path)
            {
                reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
                if(lastLocation != -1)
                {
                    reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
                }
                lastLocation = p.second.first;
                t++;
            }
            paths[i] = path;
            //const auto pathEnd{std::chrono::steady_clock::now()};
            //const std::chrono::duration<double> pathTime {pathEnd - pathStart};
            //cout << "Path planning for agent " << i << " took: " << pathTime.count() << endl;
        }
        if(!path.empty())
        {
            if (path.front().second.first != env->curr_states[i].location)
            {
                actions[i] = Action::FW; //forward action
                if(env->map[path.front().second.first] == 1)
                {
                    actions[i] = Action::W;
                    path.clear();
                }
            } 
            else if (path.front().second.second != env->curr_states[i].orientation)
            {
                int incr = path.front().second.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; //CCR--clockwise rotate
                } 
            }
        }
        else
        {
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
            actions[i] = Action::CR;
        }
        const auto checkForSkipEnd{std::chrono::steady_clock::now()};
        const std::chrono::duration<double> checkForSkipTime {checkForSkipEnd - checkForSkipStart};
    
        if(checkForSkipTime.count() >= 0.72 && !skipPathfinding)
        {
            skipPathfinding = true;
            cout << "SKIPPING PATHFINDING AT: " << checkForSkipTime.count() << endl;
        }
    }
    const auto endPathPlanning{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> pathPlanningSeconds {endPathPlanning - startPathPlanning};
    cout << "Path planning for " << env->num_of_agents << " agents: " << pathPlanningSeconds.count() << endl;


    const auto startConflictResolution{std::chrono::steady_clock::now()};
    const auto startNthConflict{std::chrono::steady_clock::now()};
    const auto endNthConflict{std::chrono::steady_clock::now()};
    int agentsWithConflicts = 0;
    std::set<int> agentsWithConflictsSet;
    const std::chrono::duration<double> nthConflictSeconds {endNthConflict - startNthConflict};
    //cout << "                1. conflict ran for: " << nthConflictSeconds.count() << endl;
    pair<bool, std::vector<bool>> possibleConflicts = findConflicts();
    int conflictCounter = 1;
    while(possibleConflicts.first)
    {
        const auto startConflictResolution{std::chrono::steady_clock::now()};
        conflictCounter++;
        for(int i = 0; i < env->num_of_agents; i++)
        {
            list<pair<int,pair<int,int>>> path = paths[i];
            if(path.empty() || possibleConflicts.second[i])
            {
                agentsWithConflicts++;
                agentsWithConflictsSet.insert(i);
                int lastLocation = -1;
                int t = 1;
                for(auto p : path)
                {
                    //remove tuple from set
                    std::tuple<int,int,int> tup = make_tuple(p.second.first, -1, selfTime + t);
                    reservation.erase(tup);
                    heatMap[p.second.first]--;
                    /*for (auto it = reservation.begin(); it != reservation.end();)
                    {
                    if (*it == tup)
                        it = reservation.erase(it);
                    else
                        ++it;
                    }*/

                    if(lastLocation != -1)
                    {
                        tup = make_tuple(lastLocation, p.second.first, selfTime + t);
                        reservation.erase(tup);
                        /*for (auto it = reservation.begin(); it != reservation.end();)
                        {
                        if (*it == tup)
                            it = reservation.erase(it);
                        else
                            ++it;
                        }*/
                    }
                    lastLocation = p.second.first;
                    t++;
                }
                path.clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                path.emplace_front(make_pair(selfTime, make_pair(env->curr_states[i].location, nextOrientation)));
                paths[i] = path;
                reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
                actions[i] = Action::CR;
            }
        }
        possibleConflicts = findConflicts();
        const auto endNthConflict{std::chrono::steady_clock::now()};
        const std::chrono::duration<double> nthConflictSeconds {endNthConflict - startNthConflict};
        //cout << "Found conflict! " << conflictCounter << ". conflict ran for: " << nthConflictSeconds.count() <<endl;
    }
    const auto endConflictResolution{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> conflictResolutionSeconds {endConflictResolution - startConflictResolution};
    cout << "Conflict resolution for " << conflictCounter << " deep conflicts with " << agentsWithConflictsSet.size() << " conflicting agents took: " << conflictResolutionSeconds.count() << endl;

    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(!paths[i].empty())
        {
            heatMap[paths[i].front().second.first]--;
            paths[i].pop_front();
        }
    }
    return;
}

void MAPFPlanner::planWithMultipleIterationConflictChecksOnReducedMapWaypointsImprovedConflictResolution(int time_limit, std::vector<Action> & actions)
{
    
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    reservation.clear();
    std::set<int> conflictAgents;

    // setup for the reservations
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        conflictAgents.insert(i);
        list<pair<int,pair<int,int>>> path = paths[i];
        int lastLocation = -1;
        int t = 1;
        for(auto p : path)
        {
            reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
            if(lastLocation != -1)
            {
                reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
            }
            lastLocation = p.second.first;
            t++;
        }
    }

     updateAgentMap();

    // updating paths for agents without any goal locations
    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(env->goal_locations[i].empty())
        {
            list<pair<int,pair<int,int>>> path;
            pair<int,int> tmpPair = make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
            path.emplace_front(make_pair(selfTime + 1, tmpPair));
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime + 1));
            paths[i] = path;
        }
    }

    // making new paths 
    const auto startPathPlanning{std::chrono::steady_clock::now()};
    for(int i = 0; i < env->num_of_agents; i++)
    {
        list<pair<int,pair<int,int>>> path = paths[i];
        if(simpleWaypointDestinations[i] != env->goal_locations[i][0].first)
        {
            //cout << "generating waypoints for agent: " << i << endl;
            int waypointStart = BFStoReducedMap(env->curr_states[i].location);
            int waypointEnd = BFStoReducedMap(env->goal_locations[i][0].first);
            waypoints[i] = reducedMapWaypointsAStar(waypointStart, waypointEnd,1);
            //waypoints[i] = reducedMapAStar(waypointStart, waypointEnd,5);
            simpleWaypointDestinations[i] = env->goal_locations[i][0].first;
            paths[i].clear();
        }

        if(!waypoints[i].empty() && getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < TOO_CLOSE_ON_REDUCEDMAPWAYPOINTS )
        {
            paths[i].clear();
        }  

        if(paths[i].empty())
        {
            if(!env->goal_locations[i].empty())
            {
                while(!waypoints[i].empty() && (getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < CLOSENESS))
                {
                    waypoints[i].pop_front();
                }
                if(waypoints[i].size() == 1 && getManhattanDistance(env->curr_states[i].location, env->goal_locations[i].front().first) < CLOSENESS)
                {
                    waypoints[i].pop_front();
                }
                if(!waypoints[i].empty())
                {
                    //cout << "started planning for next waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstarWithProximity(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                waypoints[i].front(),
                                                reservation,
                                                5);
                }
                else
                {
                    //cout << "started planning for last waypoint for agent: " << i << endl;
                    path = spaceTimeSingleAgentAstarWithProximity(env->curr_states[i].location,
                                                env->curr_states[i].orientation,
                                                env->goal_locations[i].front().first,
                                                reservation,
                                                0);
                }
            }
            int lastLocation = -1;
            int t = 1;
            for(auto p : path)
            {
                reservation.insert(make_tuple(p.second.first,-1,selfTime + t));
                if(lastLocation != -1)
                {
                    reservation.insert(make_tuple(lastLocation, p.second.first, selfTime + t));
                }
                lastLocation = p.second.first;
                t++;
            }
            paths[i] = path;
        }
        if(!path.empty())
        {
            if (path.front().second.first != env->curr_states[i].location)
            {
                actions[i] = Action::FW; //forward action
            } 
            else if (path.front().second.second != env->curr_states[i].orientation)
            {
                int incr = path.front().second.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; //C--counter clockwise rotate
                } 
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; //CCR--clockwise rotate
                } 
            }
        }
        else
        {
            reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
            actions[i] = Action::CR;
        }
    }
    const auto endPathPlanning{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> pathPlanningSeconds {endPathPlanning - startPathPlanning};
    cout << "Path planning for " << env->num_of_agents << " agents: " << pathPlanningSeconds.count() << endl;
    
    const auto startConflictResolution{std::chrono::steady_clock::now()};
    const auto startNthConflict{std::chrono::steady_clock::now()};
    std::tuple<bool, std::unordered_map<int,bool>, std::set<int>> possibleConflicts = findConflicts(conflictAgents);
    const auto endNthConflict{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> nthConflictSeconds {endNthConflict - startNthConflict};
    cout << "                1. conflict ran for: " << nthConflictSeconds.count() << endl;
    bool isConflict = std::get<0> (possibleConflicts);
    std::unordered_map<int,bool> isConflictAgent = std::get<1> (possibleConflicts);
    conflictAgents = std::get<2> (possibleConflicts);
    int conflictCounter = 1;
    while(isConflict)
    {
        const auto startNthConflict{std::chrono::steady_clock::now()};
        isConflict = std::get<0> (possibleConflicts);
        isConflictAgent = std::get<1> (possibleConflicts);
        conflictAgents = std::get<2> (possibleConflicts);
        conflictCounter++;
        for(int i : conflictAgents)
        {
            list<pair<int,pair<int,int>>> path = paths[i];
            if(path.empty() || isConflictAgent[i])
            {
                //std::vector<Set::iterator> iterators(reservation.size());
                //std::vector<std::tuple<int,int,int>> tuples;
                int lastLocation = -1;
                int t = 1;
                for(auto p : path)
                {
                    //remove tuple from set
                    std::tuple<int,int,int> tup = make_tuple(p.second.first, -1, selfTime + t);
                    reservation.erase(tup);
                    //tuples.push_back(make_tuple(p.second.first, -1, selfTime + t));
                    //iterators[i] = reservation.find(tup);
                    /*for (auto it = reservation.begin(); it != reservation.end();)
                    {
                    if (*it == tup)
                        it = reservation.erase(it);
                    else
                        ++it;
                    }*/

                    if(lastLocation != -1)
                    {
                        tup = make_tuple(lastLocation, p.second.first, selfTime + t);
                        reservation.erase(tup);
                        //tuples.push_back(make_tuple(p.second.first, p.second.first, selfTime + t));
                        //iterators[i] = reservation.find(tup);
                        /*for (auto it = reservation.begin(); it != reservation.end();)
                        {
                        if (*it == tup)
                            it = reservation.erase(it);
                        else
                            ++it;
                        }*/
                    }
                    lastLocation = p.second.first;
                    t++;
                }
                /*for(auto it : iterators)
                {
                    reservation.erase(it);
                }*/
                path.clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                path.emplace_front(make_pair(selfTime, make_pair(env->curr_states[i].location, nextOrientation)));
                paths[i] = path;
                reservation.insert(make_tuple(env->curr_states[i].location, -1, selfTime+1));
                actions[i] = Action::CR;
            }
        }
        possibleConflicts = findConflicts(conflictAgents);
        const auto endNthConflict{std::chrono::steady_clock::now()};
        const std::chrono::duration<double> nthConflictSeconds {endNthConflict - startNthConflict};
        cout << "Found conflict! " << conflictCounter << ". conflict ran for: " << nthConflictSeconds.count() <<endl;
    }
    const auto endConflictResolution{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> conflictResolutionSeconds {endConflictResolution - startConflictResolution};
    cout << "Conflict resolution for " << conflictCounter << " conflicts took: " << conflictResolutionSeconds.count() << endl;

    for(int i = 0; i < env->num_of_agents; i++)
    {
        if(!paths[i].empty())
        {
            heatMap[paths[i].front().second.first]--;
            paths[i].pop_front();
        }
    }
    return;
}


void MAPFPlanner::updateAgentMap()
{
    agentMap.clear();
    for(int i = 0; i < env->num_of_agents; i++)
    {
        agentMap[env->curr_states[i].location] = i;
    }
}

pair<bool, int> MAPFPlanner::isOtherAgentTakenPlace(unordered_map<int,int> &locationTakenByAgent, int location)
{
    if(auto it = locationTakenByAgent.find(location); it != locationTakenByAgent.end())
    {
        return make_pair(true,it->second);
    }
    return make_pair(false,-1);
}

list<pair<int,pair<int,int>>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
{
    
    NormalNode* currNode;
    int timeStep = env->curr_timestep;
    list<pair<int,pair<int,int>>> path;
    pair<int,int> tmpPair;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                tmpPair = make_pair(curr->location, curr->direction);
                path.emplace_front(make_pair(timeStep + curr->g-1, tmpPair));
                if (auto iterator = allNormalNodes.find(curr->location); iterator != allNormalNodes.end())
                {
                    currNode = iterator->second;
                    currNode->lockedTimeSteps.emplace(timeStep + curr->g);
                }
                curr = curr->parent;
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        list<pair<int,int>> neighbors = getNeighborsWithRestriction(curr->location, curr->direction, timeStep + curr->g);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}

list<pair<int,pair<int,int>>> MAPFPlanner::spaceTimeSingleAgentAstar(int start, int start_direct, int end, std::set<std::tuple<int,int,int>> &reservation)
{
    list<pair<int,pair<int,int>>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<pair<int, int>,AstarNode*> all_nodes;
    //unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    //all_nodes[start*4 + start_direct] = s;
    int limit = 140000;
    std::tuple<int,int,int> tup;
    std::tuple<int,int,int> tup2;
    
    
    while (!open_list.empty())
    {
        limit--;
        AstarNode* curr = open_list.top();
        open_list.pop();
        pair<int,int> tmpPair = make_pair(curr->location*4 + curr->direction, curr->g);
        if (all_nodes.find(tmpPair) != all_nodes.end())
                continue;
        all_nodes[tmpPair] = curr;

        if (curr->location == end || limit < 1)
        {
            while(curr->parent!=NULL) 
            {
                std::pair<int,int> tmpPair = make_pair(curr->location, curr->direction);
                path.emplace_front(make_pair(selfTime + curr->g-1, tmpPair));
                curr = curr->parent;
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, false);
        for (const pair<int,int>& neighbor: neighbors)
        {
            tup = make_tuple(neighbor.first, -1, selfTime + curr->g + 1);
            tup2 = make_tuple(neighbor.first, curr->location, selfTime + curr->g + 1);

            if(reservation.find(tup) != reservation.end())
            {
                continue;
            }

            if(reservation.find(tup2) != reservation.end())
            {
                continue;
            }
            
            pair<int,int> neighborKey = make_pair(neighbor.first * 4 + neighbor.second, curr->g + 1);


            if (all_nodes.find(neighborKey) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighborKey];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                //all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}

list<pair<int,pair<int,int>>> MAPFPlanner::spaceTimeSingleAgentAstarWithProximity(int start, int start_direct, int end, std::set<std::tuple<int,int,int>> &reservation, int proximity)
{
    list<pair<int,pair<int,int>>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<pair<int, int>,AstarNode*> all_nodes;
    vector<AstarNode*> allNodesVec;
    //unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    allNodesVec.push_back(s);
    //all_nodes[start*4 + start_direct] = s;
    int limit = 6000;//env->map.size() * 4;
    std::tuple<int,int,int> tup;
    std::tuple<int,int,int> tup2;
    
    
    while (!open_list.empty())
    {
        limit--;
        AstarNode* curr = open_list.top();
        open_list.pop();
        pair<int,int> tmpPair = make_pair(curr->location*4 + curr->direction, curr->g);
        if (all_nodes.find(tmpPair) != all_nodes.end())
                continue;
        all_nodes[tmpPair] = curr;

        if (getManhattanDistance(curr->location, end) <= proximity || limit < 1)
        {
            while(curr->parent!=NULL) 
            {
                std::pair<int,int> tmpPair = make_pair(curr->location, curr->direction);
                //heatMap[curr->location]++;
                heatMap[curr->location] *= 2;
                path.emplace_front(make_pair(selfTime + curr->g-1, tmpPair));
                curr = curr->parent;
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, true);
        for (const pair<int,int>& neighbor: neighbors)
        {
            tup = make_tuple(neighbor.first, -1, selfTime + curr->g + 1);
            tup2 = make_tuple(neighbor.first, curr->location, selfTime + curr->g + 1);

            if(reservation.find(tup) != reservation.end())
            {
                continue;
            }

            if(reservation.find(tup2) != reservation.end())
            {
                continue;
            }
            
            pair<int,int> neighborKey = make_pair(neighbor.first * 4 + neighbor.second, curr->g + 1);


            if (all_nodes.find(neighborKey) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighborKey];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                } 
                else if((curr->g + 1 == old->g && heatMap[curr->location] < heatMap[old->location] ))
                {
                    //cout << "changed best node because of heatmap!" <<endl;
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                allNodesVec.push_back(next_node);

                //all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    /*for (auto n: all_nodes)
    {
        delete n.second;
    }
    while(!open_list.empty())
    {
        AstarNode* curr =  open_list.top();
        open_list.pop();
        delete curr;
    }
    all_nodes.clear();*/
    for(auto i : allNodesVec)
    {
        delete i;
    }
    allNodesVec.clear();
    return path;
}

list<int> MAPFPlanner::reducedMapAStar(int start, int end, int step)
{
    list<int> waypoints;
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    unordered_map<int,int> parents;
    pair<int,int> s = make_pair(getManhattanDistance(start,end), start);
    open_list.push(s);
    all_nodes[start] = getManhattanDistance(start,end);
    parents[start] = -1;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            int distance = 0;
            while(currLocation!=-1) 
            {
                if(distance == 0)
                {
                    waypoints.emplace_front(currLocation);
                    distance = step;
                }
                distance--;
                currLocation = parents[currLocation];
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        list<int> neighbors = getPlainNeighborsOnReducedMap(currLocation);
        int newG = all_nodes[currLocation] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    return waypoints;
}

list<int> MAPFPlanner::reducedMapWaypointsAStar(int start, int end, int step)
{
    list<int> waypoints;
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    unordered_map<int,int> parents;
    pair<int,int> s = make_pair(getManhattanDistance(start,end), start);
    open_list.push(s);
    all_nodes[start] = getManhattanDistance(start,end);
    parents[start] = -1;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            int distance = 0;
            while(currLocation!=-1) 
            {
                if(distance == 0)
                {
                    waypoints.emplace_front(currLocation);
                    distance = step;
                }
                distance--;
                currLocation = parents[currLocation];
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        unordered_map<int,int> tmpMap = reducedMapWaypoints[currLocation];
        list<int> neighbors;
        for(auto kv : tmpMap)
        {
            neighbors.emplace_back(kv.first);
        }
        int currG = all_nodes[currLocation];
        for (const int& neighbor: neighbors)
        {
            int newG = currG + reducedMapWaypoints[currLocation][neighbor];
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    return waypoints;
}

int MAPFPlanner::BFStoReducedMap(int start) // returns the closest point on the reduced map
{
    int waypoint;
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        open_list.pop();

        if(reducedMap[curr.second] > 0)
        {
            waypoint = curr.second;
            break;
        }

        //cout << "GET NEIGHBOURS" << endl;
        list<int> neighbors = getPlainNeighbors(curr.second);
        int newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {

            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
    return waypoint;
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}

bool MAPFPlanner::validateMoveOnReducedMap(int loc1, int loc2)
{
    int loc1X = loc1 / env->cols;
    int loc1Y = loc1 % env->cols;

    if (loc1X < 0 || loc1X >= env->rows || loc1Y < 0 || loc1Y > env->cols || reducedMap[loc1] == 0)
    {
        return false;
    }

    int loc2X = loc2 / env->cols;
    int loc2Y = loc2 % env->cols;

    if (abs(loc1X-loc2X) + abs(loc1Y-loc2Y) > 1)
        return false;
    return true;
}

list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction, bool isRestricted)
{
    list<pair<int,int>> neighbors;
    
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    
    if(isRestricted) // ONE-WAY
    {
        NormalNode* currNode = getNormalNodeByLocation(location);
        NormalNode* forwardNode = getNormalNodeByLocation(forward);

        //forward
        if (forward>=0 && forward < env->map.size() && validateMove(forward,location) && env->map[forward] == 0 &&
            (currNode->directions.find(new_direction) != currNode->directions.end()) )
        {
            neighbors.emplace_back(make_pair(forward,new_direction));
        }

        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        if(currNode->directions.find(new_direction) != currNode->directions.end())
        {
            neighbors.emplace_back(make_pair(location,new_direction));
        }
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        if(currNode->directions.find(new_direction) != currNode->directions.end())
        {
            neighbors.emplace_back(make_pair(location,new_direction));
        }
        //neighbors.emplace_back(make_pair(location,direction)); //wait
        
    }
    else // NO RESTRICTION
    {
        if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.emplace_back(make_pair(location,new_direction));
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.emplace_back(make_pair(location,new_direction));
        //neighbors.emplace_back(make_pair(location,direction)); //wait
    }

    
    return neighbors;
}

list<pair<int,int>> MAPFPlanner::getNeighborsWithRestriction(int location,int direction, int g)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    NormalNode* currNode = getNormalNodeByLocation(location);
    NormalNode* forwardNode = getNormalNodeByLocation(forward);
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
    {
        if(env->curr_timestep == debuggingTimeStep && forwardNode->location == 788)
        {
             for(auto deb : forwardNode->lockedTimeSteps)
            {
                cout << deb << ", " ;
            }
            cout << endl;
            cout << (currNode->directions.find(new_direction) != currNode->directions.end()) << " && " << ((forwardNode->directions.find(new_direction) != forwardNode->directions.end()) || !(forwardNode->isOneWay)) << " && " << !(forwardNode->lockedTimeSteps.find(g) != forwardNode->lockedTimeSteps.end()) << " && " << !(forwardNode->lockedTimeSteps.find(g+1) != forwardNode->lockedTimeSteps.end()) << endl;
        }
        if((currNode->directions.find(new_direction) != currNode->directions.end()) &&
            ((forwardNode->directions.find(new_direction) != forwardNode->directions.end()) || !(forwardNode->isOneWay)) &&
            !(forwardNode->lockedTimeSteps.find(g) != forwardNode->lockedTimeSteps.end()) &&
            !(forwardNode->lockedTimeSteps.find(g+1) != forwardNode->lockedTimeSteps.end()))
        {
            
            neighbors.emplace_back(make_pair(forward,new_direction));
        }
    }
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    if ((currNode->directions.find(new_direction) != currNode->directions.end()) &&
         !(currNode->lockedTimeSteps.find(g) != currNode->lockedTimeSteps.end()) &&
         !(currNode->lockedTimeSteps.find(g+1) != currNode->lockedTimeSteps.end()))
    {
        neighbors.emplace_back(make_pair(location,new_direction));
    }
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    if ((currNode->directions.find(new_direction) != currNode->directions.end()) &&
         !(currNode->lockedTimeSteps.find(g) != currNode->lockedTimeSteps.end()) &&
         !(currNode->lockedTimeSteps.find(g+1) != currNode->lockedTimeSteps.end()))
    {
        neighbors.emplace_back(make_pair(location,new_direction));
    }

    if(!(currNode->lockedTimeSteps.find(g) != currNode->lockedTimeSteps.end()) &&
        !(currNode->lockedTimeSteps.find(g+1) != currNode->lockedTimeSteps.end()) )
    {
        neighbors.emplace_back(make_pair(location,direction)); //wait
    }
    if(env->curr_timestep == debuggingTimeStep)
    {
        cout << "Searching node: [" <<  location << ", " << direction << "] to the open list at time step: " << g << endl;
        for(auto deb : neighbors)
        {
            cout << "   Returned neighbours: [" <<  deb.first << ", " << deb.second << "] to the open list at time step: " << g << endl;
        }
    }
    return neighbors;
}

list<int> MAPFPlanner::getPlainNeighbors(int location)
{
    list<int> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    for(int i = 0; i < 4; i++)
    {
        int forward = candidates[i];
        if(forward >= 0 && forward < env->map.size() && validateMove(forward, location))
        {
            neighbors.emplace_back(forward);
        }
    }
    return neighbors;
}

list<int> MAPFPlanner::getPlainNeighborsOnReducedMap(int location)
{
    list<int> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    for(int i = 0; i < 4; i++)
    {
        int forward = candidates[i];
        if(forward >= 0 && forward < env->map.size() && validateMoveOnReducedMap(forward, location))
        {
            neighbors.emplace_back(forward);
        }
    }
    return neighbors;
}

pair<bool, std::vector<bool>> MAPFPlanner::findConflicts()
{
    bool found = false;
    std::vector<bool> conflict = std::vector<bool>(env->num_of_agents, false);
    std::vector<int> surroundingAgents;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        if(!paths[i].empty() && paths[i].front().second.first != env->curr_states[i].location) // this robot moves
        {
            int location = env->curr_states[i].location;
            int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
            int forward = candidates[env->curr_states[i].orientation];
            surroundingAgents.clear();
            if (auto iterator = agentMap.find(location+1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location+env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location-1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location-env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            for(int j = 0; j < surroundingAgents.size(); j++)
            {
                int agentJ = surroundingAgents[j];
                if(agentJ == i){ continue; }
                if( (env->curr_states[agentJ].location == forward && 
                    ((paths[agentJ].empty() || paths[agentJ].front().second.first == forward) ||
                    (!paths[agentJ].empty() && paths[agentJ].front().second.first == location))) ||
                    (!paths[agentJ].empty() && paths[agentJ].front().second.first == forward) )
                {
                    //string agentJString = paths[agentJ].empty() ? std::to_string(env->curr_states[agentJ].location)+" empty path" : std::to_string(paths[agentJ].front().second.first);
                    //cout << "CONFLICT FOUND! Agent between agent-location: (" << i << ", " << paths[i].front().second.first <<  ") <-> (" << agentJ << ", " << agentJString << ")" << endl;
                    conflict[i] = true;
                    found = true;
                }
            }
        }
    }
    return make_pair(found, conflict);
}

std::tuple<bool, std::unordered_map<int,bool>, std::set<int>> MAPFPlanner::findConflicts(std::set<int> agentsToCheck)
{
    bool found = false;
    std::unordered_map<int,bool> conflict;
    std::vector<int> surroundingAgents;
    std::set<int> agentsWithSurroundingAgents;
    /*cout << "Checking agents: ";
    for(int i : agentsToCheck)
    {
        cout << i << " ";
    }
    cout << endl;*/
    for(int i : agentsToCheck)
    {
        if(!paths[i].empty() && paths[i].front().second.first != env->curr_states[i].location) // this robot moves
        {
            int location = env->curr_states[i].location;
            int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
            int forward = candidates[env->curr_states[i].orientation];
            surroundingAgents.clear();
            if (auto iterator = agentMap.find(location+1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location+env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location-1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if (auto iterator = agentMap.find(location-env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
            if(!surroundingAgents.empty()) {agentsWithSurroundingAgents.insert(i);}
            for(int j = 0; j < surroundingAgents.size(); j++)
            {
                int agentJ = surroundingAgents[j];
                if(agentJ == i){ continue; }
                if( (env->curr_states[agentJ].location == forward && 
                    ((paths[agentJ].empty() || paths[agentJ].front().second.first == forward) ||
                    (!paths[agentJ].empty() && paths[agentJ].front().second.first == location))) ||
                    (!paths[agentJ].empty() && paths[agentJ].front().second.first == forward) )
                {
                    //string agentJString = paths[agentJ].empty() ? std::to_string(env->curr_states[agentJ].location)+" empty path" : std::to_string(env->curr_states[agentJ].location)+"->"+std::to_string(paths[agentJ].front().second.first) ;
                    //cout << "CONFLICT FOUND! Agent between agent-location: (" << i << ", " << env->curr_states[i].location << "->" << paths[i].front().second.first << ") <-> (" << agentJ << ", " << agentJString << ")" << endl;
                    conflict[i] = true;
                    found = true;
                    agentsWithSurroundingAgents.insert(agentJ);
                    break;
                }
            }
        }
    }
    //cout << "---------------------" << endl;
    return std::make_tuple(found, conflict, agentsWithSurroundingAgents);
}

void MAPFPlanner::clearPath(list<pair<int,pair<int,int>>> &path)
{
    NormalNode* currNode;
    std::set<int> tmpSet; 
    cout << "PATH RESET: " << endl;
    for(auto p : path)
    {
        currNode = getNormalNodeByLocation(p.second.first);
        tmpSet = currNode->lockedTimeSteps;
        /*
        cout << "   Item and NODE " << p.first << " - " << currNode->location << ": ";
        for(auto deb : currNode->lockedTimeSteps)
        {
            cout << deb << ", " ;
        }
        */

        for (auto it = tmpSet.begin(); it != tmpSet.end();)
        {
        if (*it == p.first+1)
            it = tmpSet.erase(it);
        else
            ++it;
        }
        currNode->lockedTimeSteps = tmpSet;
        /*
        cout << " -> ";
        for(auto deb : currNode->lockedTimeSteps)
        {
            cout << deb << ", " ;
        }
        cout << endl;
        */
    }
    path.clear();
    
}

MAPFPlanner::NormalNode* MAPFPlanner::getNormalNodeByLocation(int location)
{   
    NormalNode* returnNode;
    if (auto iterator = allNormalNodes.find(location); iterator != allNormalNodes.end())
    {
        returnNode = iterator->second;
    }
    return returnNode;
    
}

//returns the number of traversable Nodes from the 8 neighbors and the available directions to move to
pair<int, std::set<int>> MAPFPlanner::getNodeNeighborCountWithAvailableDirections(int location)
{
    pair<int, std::set<int>> nodeNeighborCountWithAvailableDirections;
    std::set<int> possibleDirections;
    int neighborCount = 0;
    int candidates[8] = 
    { 
        location + 1,               // right
        location + env->cols,       // down
        location - 1,               // left
        location - env->cols,       // up
        location + 1 + env->cols,   // right-down
        location + 1 - env->cols,   // right-up
        location - 1 + env->cols,   // left-down
        location - 1 - env->cols    // left-up
    };
    for(int i = 0 ; i < 4 ; i++){
        int neighborLoc = candidates[i];
        if (neighborLoc >=0 && neighborLoc < env->map.size() && validateMove(neighborLoc,location))
        {
            possibleDirections.emplace(i);
            neighborCount++;
        }
    }
    for(int i = 4; i < 8; i++)
    {
        int neighborLoc = candidates[i];
        if (neighborLoc >=0 && neighborLoc < env->map.size() && validateMove(neighborLoc,location))
        {
            neighborCount++;
        }
    }
    nodeNeighborCountWithAvailableDirections = make_pair(neighborCount, possibleDirections);
    return nodeNeighborCountWithAvailableDirections;
}

void MAPFPlanner::debug(int debugTimeStep, int debugLocation)
{
    if(env->curr_timestep == debugTimeStep)
    {
    cout << "-----DEBUG START------\nNode: " << debugLocation<<"\nPossible directions from node: ";  
        NormalNode* debugNode;
        if (auto debug = allNormalNodes.find(debugLocation); debug != allNormalNodes.end())
        {
            debugNode = debug->second;
            for(auto deb : debugNode->directions)
            {
                cout << deb << ", ";
            }
            cout << "\nAnd this node is locked in the following timesteps: ";
            for(auto deb : debugNode->lockedTimeSteps)
            {
                cout << deb << ", " ;
            }
            cout << endl;
        }
    cout << "-----DEBUG END------" << endl;
    }
}
