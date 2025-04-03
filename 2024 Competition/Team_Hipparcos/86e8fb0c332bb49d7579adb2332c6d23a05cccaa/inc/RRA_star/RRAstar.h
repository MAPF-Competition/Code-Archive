#ifndef RRASTAR_H
#define RRASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include "RRA_star/RRAstarNode.h"
#include "map_utils/MapUtils.h"
#include "SharedEnv.h"

class RRAstar {
public:
    RRAstar(SharedEnvironment* _env): env(_env) { }
    ~RRAstar();

    void initialize(int start, int starDir, int goal);
    void initialize(int start, int starDir, int areaId, const MapUtils* maputils);

    int abstractDist(int loc, int dir);
    void reset();
    bool isDistKnown(int loc, int dir) const;

private:
    SharedEnvironment* env;
    std::priority_queue<RRAstarNode*, std::vector<RRAstarNode*>, RRAstarNode::cmp> open;
    std::unordered_map<int, RRAstarNode*> closed;
    std::unordered_map<int, RRAstarNode*> allNodes;
    int start;

    bool resume(int loc, int dir);
    int getManhattanDist(int loc1, int loc2) const;
    std::list<std::pair<int,int>> getNeighbors(int location,int direction) const;
    std::list<std::pair<int,int>> getGoalNeighbors(int loc) const;
    bool validateMove(int loc1, int loc2) const;

};

#endif // RRASTAR_H