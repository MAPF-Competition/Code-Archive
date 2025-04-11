#ifndef RRASTAR_H
#define RRASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <list>
#include "RRA_star/RRAstarNode.h"
#include "SharedEnv.h"

class RRAstar {
public:
    RRAstar(SharedEnvironment* _env): env(_env) { }
    ~RRAstar();

    void initialize(int start, int starDir, int goal);
    int abstractDist(int loc, int dir);
    void reset();

    int getManhattanDist(int loc1, int loc2) const;

private:
    SharedEnvironment* env;
    std::priority_queue<RRAstarNode*, std::vector<RRAstarNode*>, RRAstarNode::cmp> open;
    std::unordered_map<int, RRAstarNode*> closed;
    std::unordered_map<int, RRAstarNode*> allNodes;
    int start;

    bool resume(int loc, int dir);
    std::list<std::pair<int,int>> getNeighbors(int location,int direction) const;
    std::list<std::pair<int,int>> getGoalNeighbors(int loc) const;
    bool validateMove(int loc1, int loc2) const;

};

#endif // RRASTAR_H