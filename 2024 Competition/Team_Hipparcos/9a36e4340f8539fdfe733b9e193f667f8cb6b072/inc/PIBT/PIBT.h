#ifndef PIBT_H
#define PIBT_H

#include<vector>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "PIBT/Agent.h"

class PIBT {
public:
    PIBT(SharedEnvironment* _env): env(_env) { }
    ~PIBT();

    void initialize();
    void nextStep(int timeLimit, std::vector<Action>& actions);

private:
    SharedEnvironment* env;
    std::vector<Agent*> agents;
    std::vector<int> prevReservations;
    std::vector<int> nextReservations;
    bool manhattan;

    bool getNextLoc(Agent* const a, const Agent* const b);
    std::vector<std::pair<int,int>> getNeighbors(Agent* const a) const;
    bool validateMove(int loc1, int loc2) const;
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent* const a);

    void setGoalsParallel();
    void setGoals(const TimePoint& endTime);

    bool neighborTieBreak(const std::pair<int, int>& a, const std::pair<int, int>& b, const Agent* agent) const;
    int getTurnCount(int fromLoc, int fromDir, int toLoc) const;
};

#endif // PIBT_H