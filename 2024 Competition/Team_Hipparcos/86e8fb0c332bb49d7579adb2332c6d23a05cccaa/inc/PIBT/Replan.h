#ifndef REPLAN_H
#define REPLAN_H

#include <vector>
#include <list>
#include "SharedEnv.h"
#include "PIBT/Agent2.h"

class Replan {
public:
    Replan(SharedEnvironment* _env, std::vector<std::vector<int>>& _reservations): 
        env(_env), reservations(_reservations), reservationsCopy(_reservations) {}
    ~Replan();

    bool replan(std::vector<Agent2*>& agents);

private:
    SharedEnvironment* env;
    std::vector<std::vector<int>>& reservations;
    std::vector<std::vector<int>> reservationsCopy;
    std::vector<Agent2*> agentsCopy;
    std::pair<bool,std::list<std::pair<int,int>>> singleAgentPlan(int id, int start, int startDir, int end, int endDir);
    int getManhattanDistance(int loc1, int loc2);
    std::list<std::pair<int,int>> getNeighbors(int loc, int dir, int time);
    bool validateMove(int loc, int loc2);
};

#endif // REPLAN_H
