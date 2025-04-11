#include <chrono>   
#include "PIBT/Agent.h"

void Agent::setGoal(bool manhattan) {
    if (env->goal_locations[id].empty()) {
        goal = env->curr_states[id].location;
    }
    else {
        goal = env->goal_locations[id].front().first;
    }

    // auto start {std::chrono::steady_clock::now()};

    if (!manhattan) {
        heuristic.reset();
        heuristic.initialize(env->curr_states[id].location, env->curr_states[id].orientation, goal);
    }

    // auto end {std::chrono::steady_clock::now()};
    // auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    // if (duration.count() > 10) 
    // {
    //     std::cout << "******** Heuristic for agent "<< id << " took " << duration.count() << " ms (" << env->curr_states[id].location
    //         << ", " << goal << ")\n";
    // }
}

int Agent::getDist(int loc, int dir, bool manhattan) {
    // auto start {std::chrono::steady_clock::now()};

    if (manhattan) { return heuristic.getManhattanDist(loc, goal); }

    return heuristic.abstractDist(loc, dir);

    // auto end {std::chrono::steady_clock::now()};
    // auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    // if (duration.count() > 10) {
    //     std::cout << "******** Heuristic for agent "<< id << " took " << duration.count() << " ms (" << env->curr_states[id].location
    //         << ", " << goal << ")\n";
    // }

    // return result;
}

int Agent::getLoc() const {
    return env->curr_states[id].location;
}

int Agent::getDir() const {
    return env->curr_states[id].orientation;
}

bool Agent::isNewGoal() const {
    if (env->goal_locations[id].empty()) {
        return goal == -1;
    }

    return goal != env->goal_locations[id].front().first;
}