#ifndef PIBT_H
#define PIBT_H

#include <vector>
#include <thread>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "PIBT/Agent2.h"
#include "map_utils/MapUtils.h"

class PIBT {
public:
    PIBT(SharedEnvironment* _env): env(_env), maputils(_env) { }
    ~PIBT();

    void initialize();
    void nextStep(int timeLimit, std::vector<Action>& actions);

private:
    const unsigned int threadCnt {std::thread::hardware_concurrency()};
    int agentsPerThread;
    int remainingAgents;
    SharedEnvironment* env;
    std::vector<Agent2*> agentsById;
    std::vector<Agent2*> agents;
    std::vector<std::vector<int>> reservations;
    MapUtils maputils;
    int time;

    bool getNextLoc(Agent2* const a, const Agent2* const b);
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent2* const a);
    Action getNextAction2(Agent2* const a);

    void calculateGoalDistances();
};

#endif // PIBT_H