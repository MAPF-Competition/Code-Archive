#include <random>
#include <set>
#include <algorithm>
#include <chrono>
#include "PIBT/PIBT.h"
#include "PIBT/Replan.h"

void PIBT::initialize() {
    agentsPerThread = env->num_of_agents / threadCnt;
    remainingAgents = env->num_of_agents % threadCnt;

    maputils.eraseDeadEnds();

    int areaDist = (env->map.size() < 2048) ? env->cols + 1 : 5;
    maputils.divideIntoAreas(areaDist);
    maputils.calculateDistanceBetweenAreas();

    agents.reserve(env->num_of_agents);  
    agentsById.reserve(env->num_of_agents);    
  
    for (int i {0}; i < env->num_of_agents; ++i) {
        Agent2* agent = new Agent2(i, env, &maputils);
        agents.push_back(agent);
        agentsById.push_back(agent);
    }
}

void PIBT::nextStep(int timeLimit, std::vector<Action>& actions) {
    const TimePoint startTime  {std::chrono::steady_clock::now()};
    const TimePoint pibtEndTime {startTime + std::chrono::milliseconds(timeLimit / 2 - 10)};
    const TimePoint lnsEndTime {startTime + std::chrono::milliseconds(timeLimit - 20)};
    time = 0;
    reservations.push_back(std::vector<int>(env->map.size(), -1));

    for (auto& agent : agents) {
        agent->goalTime = -1;
        agent->locations.clear();
        agent->locations.push_back({env->curr_states[agent->id].location, env->curr_states[agent->id].orientation});
        reservations[time][agent->getLoc(time)] = agent->id;
        if (maputils.deadEndMap[agent->getLoc(time)] == MapUtils::DEADLOC) { agent->boostPriority(); }
    }

    calculateGoalDistances();

    std::sort(agents.begin(), agents.end(), [](const Agent2* a, const Agent2* b) {
        if (a->p == b->p) {
            return a->id < b->id;
        }
        return a->p < b->p;
    });

    while (time < 20 && std::chrono::steady_clock::now() < pibtEndTime) {     
        reservations.push_back(std::vector<int>(env->map.size(), -1));
   
        for (auto& agent : agents) {
            if(agent->locations[time].first == agent->getGoal() && agent->goalTime == -1) {
                agent->goalTime = time;
            }
            if (agent->locations.size() <= time + 1) {
                getNextLoc(agent, nullptr);
            }
        }

        std::vector<Action> actions(env->num_of_agents, Action::NA);
        std::vector<bool> visited(env->num_of_agents, false);
        for (auto& a : agents) {
            if (actions[a->id] == Action::NA) {
                actions[a->id] = getNextAction(actions, visited, a);
            }
        }

        ++time;
    }

    for (auto& agent : agents) {
        if(agent->goalTime == -1) {
            agent->goalTime = time;
        }
    }

    const TimePoint pibtTime  {std::chrono::steady_clock::now()};
    const auto pibtDuration {std::chrono::duration_cast<std::chrono::milliseconds>(pibtTime - startTime)};
    std::cout << "******** " << time << " PIBT iterations took " << pibtDuration.count() << " ms ********\n";

    std::random_device rd;
    std::mt19937 rng(rd());
    int failCnt {0};
    int iterations {0};

    while (std::chrono::steady_clock::now() < lnsEndTime) {
        std::vector<Agent2*> replanAgents;
        std::sample(agents.begin(), agents.end(), std::back_inserter(replanAgents), 10, rng);
        Replan replan(env, reservations);
        bool success = replan.replan(replanAgents);
        if (success) {
            failCnt = 0;
        } 
        else {
            ++failCnt;
        }

        ++iterations;
    }

    const TimePoint lnsTime  {std::chrono::steady_clock::now()};
    const auto lnsDuration = std::chrono::duration_cast<std::chrono::milliseconds>(lnsTime - pibtTime);
    std::cout << "******** " << iterations << " LNS iterations took " << lnsDuration.count() << " ms ********\n";

    actions = std::vector<Action>(env->num_of_agents, Action::NA);
    for (auto& a : agents) {
        actions[a->id] = getNextAction2(a);
    }

    reservations.clear();
}

bool PIBT::getNextLoc(Agent2* const a, const Agent2* const b) {
    a->locations.push_back({-1,-1});
    auto neighbors = a->getNeighborsWithDist(time);
    std::sort(neighbors.begin(), neighbors.end(), [this, &a, &b](const std::pair<int, int>& n1, const std::pair<int, int>& n2) {
        if (maputils.deadEndMap[n1.first] == MapUtils::DEADLOC && a->getGoal() != n1.first) { return false; }

        if (n1.second == n2.second) {
            return reservations[time][n1.first] == -1;
        }
        return n1.second < n2.second;
    });

    for (const auto& neighbor : neighbors) {
        if (reservations[time+1][neighbor.first] != -1) { continue; }

        if (b != nullptr && reservations[time][neighbor.first] == b->id) { continue; }

        a->locations[time+1].first = neighbor.first;
        reservations[time+1][neighbor.first] = a->id;

        int otherAgent {reservations[time][neighbor.first]};

        if (otherAgent != -1 && agentsById[otherAgent]->locations.size() <= time + 1) {
            if (getNextLoc(agentsById[otherAgent], a) == false) { continue; }
        }

        return true;
    }

    a->locations[time+1].first = a->getLoc(time);
    reservations[time+1][a->getLoc(time+1)] = a->id;
    return false;
}

Action PIBT::getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent2* const a) {
    if (a->getLoc(time+1) == -1) { assert(false); }
    Action action;
    int dir;
    int diff {a->getLoc(time+1) - a->getLoc(time)};
    
    if (diff == 0) {
        a->locations[time+1].second = a->locations[time].second;
        return Action::W;
    }
    else if (diff == 1) {
        dir = 0;
    }
    else if (diff == env->cols) {
        dir = 1;
    }
    else if (diff == -1) {
        dir = 2;
    }
    else if (diff == -env->cols) {
        dir = 3;
    }
    else {
        assert(false);
    }

    if (a->getDir(time) == dir) {
        int otherAgent {reservations[time][a->getLoc(time+1)]};
        assert(otherAgent != a->id);

        if (otherAgent != -1 && actions[otherAgent] == Action::NA && visited[otherAgent] == false) {
            visited[a->id] = true;
            actions[otherAgent] = getNextAction(actions, visited, agentsById[otherAgent]);
            action = reservations[time+1][a->getLoc(time+1)] == a->id ? Action::FW : Action::W;
        }
        else if (otherAgent != -1 && actions[otherAgent] == Action::NA && visited[otherAgent] == true) {
            action = Action::FW;
        }
        else if (otherAgent != -1 && actions[otherAgent] != Action::FW) {
            action = Action::W;
        }
        else {
            action = Action::FW;
        }
        a->locations[time+1].second = a->locations[time].second;
    }
    else {
        int incr {dir - a->getDir(time)};
        if (incr == 1 || incr == -3 || incr == 2 || incr == -2) {
            a->locations[time+1].second = (a->locations[time].second + 1) % 4;
            action = Action::CR;
        }
        else if (incr == -1 || incr == 3) {
            a->locations[time+1].second = (a->locations[time].second - 1 + 4) % 4;
            action = Action::CCR;
        }
    }

    if (action != Action::FW) {
        if (reservations[time+1][a->getLoc(time+1)] == a->id) {
            reservations[time+1][a->getLoc(time+1)] = -1;
        }
        reservations[time+1][a->getLoc(time)] = a->id;
        a->locations[time+1].first = a->locations[time].first;
    }

    return action;
}

Action PIBT::getNextAction2(Agent2* const a) {
    Action action;
    int dir;
    int diff {a->getLoc(0) - a->getLoc(1)};
    int diff2 {a->getDir(0) - a->getDir(1)};

    if (diff != 0) {
        action = Action::FW;
    }
    else if (diff2 == 0) {
        action = Action::W;
    }    
    else if (diff2 == -1 || diff2 == 3) {
        action = Action::CR;
    }
    else if (diff2 == 1 || diff2 == -3) {
        action = Action::CCR;
    }
    
    return action;
}

void PIBT::calculateGoalDistances() {
    std::vector<std::thread> threads;
    int remaining {remainingAgents};
    int from {0};
    int to;

    for (int i {0}; i < threadCnt; ++i) {
        to = from + agentsPerThread + (remaining-- > 0 ? 1 : 0);

        threads.emplace_back([this, from, to]() {
            for (int j {from}; j < to; ++j) {
                if (agents[j]->isNewGoal()) {
                    agents[j]->setGoal(); 
                }

                agents[j]->calculateNeighborDists(time);
            }
        });

        from = to;
    }

    for (auto t = threads.begin(); t != threads.end(); ++t) {
        t->join();
    }
}

PIBT::~PIBT() {
    for (auto a : agents) {
        delete a;
    }
}
