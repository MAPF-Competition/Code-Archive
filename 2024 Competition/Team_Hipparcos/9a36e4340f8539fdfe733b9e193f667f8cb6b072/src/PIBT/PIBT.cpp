#include <random>
#include <set>
#include <algorithm>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "PIBT/PIBT.h"

void PIBT::initialize() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    std::set<double> uniqueValues;

    while (uniqueValues.size() < env->num_of_agents) {
        double randomValue = dis(gen);
        uniqueValues.insert(randomValue);
    }

    prevReservations.resize(env->map.size(), -1);
    nextReservations.resize(env->map.size(), -1);

    agents.reserve(env->num_of_agents);    
    int i {0};
    for (const auto& p : uniqueValues) {
        Agent* agent = new Agent(i, 1 - p, env);
        agents.push_back(agent);
        ++i;
    }

    manhattan = env->map_name.find("sortation") != std::string::npos 
        || env->map_name.find("warehouse") != std::string::npos;
}

void PIBT::nextStep(int timeLimit, std::vector<Action>& actions) {
    TimePoint startTime  {std::chrono::steady_clock::now()};
    TimePoint endTime {startTime + std::chrono::milliseconds(timeLimit - 20)};

    int newGoalCnt {0};
    for (auto& agent : agents) {
        if (agent->isNewGoal()) { ++newGoalCnt; }
        prevReservations[agent->getLoc()] = agent->id;
    }

    if (!manhattan && (newGoalCnt > 99 || (newGoalCnt > 2 && env->map.size() > 9999))) {
        setGoalsParallel(); 
    } 
    else {
        setGoals(endTime);
    }

    for (auto& agent : agents) {
        if (std::chrono::steady_clock::now() > endTime) {
            std::cout << "-------- Out of time during nextLoc (t: " << env->curr_timestep << ") at agent: " << agent->id << '\n';
            break;
        }
        if (agent->nextLoc == -1) {
            getNextLoc(agent, nullptr);
        }
    }

    actions = std::vector<Action>(env->num_of_agents, Action::NA);
    std::vector<bool> visited(env->num_of_agents, false);
    for (int i{0}; i < env->num_of_agents; ++i) {
        if (actions[i] == Action::NA) {
            actions[i] = getNextAction(actions, visited, agents[i]);
        }
        agents[i]->nextLoc = -1;
    }

    std::fill(prevReservations.begin(), prevReservations.end(), -1);
    std::fill(nextReservations.begin(), nextReservations.end(), -1);
}

bool PIBT::getNextLoc(Agent* const a, const Agent* const b) {
    auto neighbors = getNeighbors(a);
    std::sort(neighbors.begin(), neighbors.end(), 
        [this, a](const std::pair<int, int>& x, const std::pair<int, int>& y) { return neighborTieBreak(x, y, a); });

    for (const auto& neighbor : neighbors) {
        if (nextReservations[neighbor.first] != -1) {
            continue;
        }

        if (b != nullptr && prevReservations[neighbor.first] == b->id) {
            continue;
        }

        a->nextLoc = neighbor.first;
        nextReservations[neighbor.first] = a->id;

        int otherAgent {prevReservations[neighbor.first]};

        if (otherAgent != -1 && agents[otherAgent]->nextLoc == -1) {
            if (getNextLoc(agents[otherAgent], a) == false) {
                a->nextLoc = -1;
                nextReservations[neighbor.first] = -1;
                continue;
            }
        }

        return true;
    }

    return false;
}

std::vector<std::pair<int,int>> PIBT::getNeighbors(Agent* const a) const {
    std::vector<std::pair<int,int>> neighbours;
    int loc {a->getLoc()};
    int dir {a->getDir()};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], a->getDist(candidates[i], i, manhattan)));
        }
    }

    if (!manhattan)
        neighbours.emplace_back(std::make_pair(loc, a->getDist(loc, dir, manhattan)));

    return neighbours;
}

bool PIBT::validateMove(int loc1, int loc2) const {
    int loc_x {loc1 / env->cols};
    int loc_y {loc1 % env->cols};
    if (env->map[loc1] == 1 || loc_x >= env->rows || loc_y >= env->cols || loc_x < 0 || loc_y < 0) {
        return false;
    }

    int loc2_x {loc2 / env->cols};
    int loc2_y {loc2 % env->cols};
    if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1) {
        return false;
    }

    return true;
}

Action PIBT::getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent* const a) {
    if (a->nextLoc == -1) { return Action::W; }
    Action action;
    int dir;
    int diff {a->nextLoc - a->getLoc()};
    
    if (diff == 0) {
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

    if (a->getDir() == dir) {
        int otherAgent {prevReservations[a->nextLoc]};
        assert(otherAgent != a->id);

        if (otherAgent != -1 && actions[otherAgent] == Action::NA && visited[otherAgent] == false) {
            visited[a->id] = true;
            actions[otherAgent] = getNextAction(actions, visited, agents[otherAgent]);
            action = nextReservations[a->nextLoc] == a->id ? Action::FW : Action::W;
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
    }
    else {
        int incr {dir - a->getDir()};
        if (incr == 1 || incr == -3 || incr == 2 || incr == -2) {
            action = Action::CR;
        }
        else if (incr == -1 || incr == 3) {
            action = Action::CCR;
        }
    }

    if (action != Action::FW) {
        if (nextReservations[a->nextLoc] == a->id) {
            nextReservations[a->nextLoc] = -1;
        }
        nextReservations[a->getLoc()] = a->id;
    }

    return action;
}

void PIBT::setGoalsParallel() {
    boost::asio::thread_pool pool(boost::thread::hardware_concurrency());

    for (auto& agent : agents) {
        if (agent->isNewGoal()) {
            boost::asio::post(pool, [agent]() { agent->setGoal(false); });
        }
    }

    pool.join();
}

void PIBT::setGoals(const TimePoint& endTime) {
    for (auto& agent : agents) {
        if (std::chrono::steady_clock::now() > endTime) {
            std::cout << "-------- Out of time during setGoal (t: " << env->curr_timestep << ") at agent: " << agent->id << '\n';
            break;
        }

        if (agent->isNewGoal()) {
            agent->setGoal(manhattan);
        }
    }
}

bool PIBT::neighborTieBreak(const std::pair<int, int>& a, const std::pair<int, int>& b, const Agent* agent) const {
    if (a.second == b.second) {
        if (prevReservations[a.first] == prevReservations[b.first]) {
            return getTurnCount(agent->getLoc(), agent->getDir(), a.first) < getTurnCount(agent->getLoc(), agent->getDir(), b.first);
        }
        return prevReservations[a.first] == -1;
    }
    return a.second < b.second;
}

int PIBT::getTurnCount(int fromLoc, int fromDir, int toLoc) const {
    int dir;
    int diff {toLoc - fromLoc};
    
    if (diff == 0) {
        return 99;
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

    int incr {abs(dir - fromDir)};
    if (incr == 1 || incr == 3) {
        return 1;
    }
    else {
        return incr;
    }
}


PIBT::~PIBT() {
    for (auto a : agents) {
        delete a;
    }
}