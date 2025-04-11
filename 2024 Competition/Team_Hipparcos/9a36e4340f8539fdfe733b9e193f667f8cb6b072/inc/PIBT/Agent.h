#ifndef AGENT_H
#define AGENT_H

#include "SharedEnv.h"
#include "RRA_star/RRAstar.h"

class Agent {
public:
    const int id;
    double p;
    int nextLoc {-1};

    Agent(int _id, double _p, SharedEnvironment* _env): id(_id), p(_p), heuristic(_env), env(_env) { }

    void setGoal(bool manhattan);
    int getDist(int loc, int dir, bool manhattan);
    int getLoc() const;
    int getDir() const;
    bool isNewGoal() const;

private:
    SharedEnvironment* env;
    RRAstar heuristic;
    int goal {-1};

};

#endif // AGENT_H