#ifndef AGENT2_H
#define AGENT2_H

#include "SharedEnv.h"
#include "RRA_star/RRAstar.h"
#include "map_utils/MapUtils.h"

class Agent2 {
public:
    const int id;
    int p {-1};
    int goalTime;
    std::vector<std::pair<int,int>> locations;

    Agent2(int _id, SharedEnvironment* _env, MapUtils* _maputils): id(_id), heuristic(_env), env(_env), maputils(_maputils) { }

    void setGoal();
    int getGoal() const { return goal; }
    int getLoc(int time) const;
    int getDir(int time) const;
    bool isNewGoal() const;
    void boostPriority() { p -= 1000; }
    void resetPriority() { p = goalDist; }

    std::vector<std::pair<int,int>> getNeighborsWithDist(int time);
    void calculateNeighborDists(int time);

private:
    SharedEnvironment* env;
    MapUtils* maputils;
    RRAstar heuristic;
    int goal {-1};
    int goalDist {-1};
    std::list<int>::const_iterator nextArea;
    std::list<int>::const_iterator endArea;

    bool validateMove(int loc1, int loc2) const;
    void initializeHeuristic(int areaFrom, int areaTo, int time);
    void setArea(int time);
};

#endif // AGENT2_H