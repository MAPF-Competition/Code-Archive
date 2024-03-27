#pragma once
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Planner.h"
#include "SharedEnv.h"

/// @brief Time manager for a planning algorithm
/// @tparam PLANNER A planning algorithm: PlannerLazy, PlannerLong...
/// @details When we are given one second to compute the next array of actions, the planning algorithm may finish
/// earlier. In such case, we want to use this spare time to compute the next array of actions. 
/// However, if we finish a task, what are we doing with the idle agent? 
/// 
/// The Secretary is not responsible for the planning algorithm PLANNER to finish planning within the allowed time
template <class PLANNER>
class Secretary {
    const int maxSteps = 6000;  // Maximum number of steps to compute (> 5000)
    PLANNER &planner;           // Planning algorithm
    SharedEnvironment env, *shared_env; // I think that one of these is independent from the "real" environment
    std::vector<std::vector<Action>> actionsv; // actionsv[i] = i-th vector of actions. Size must be larger than the number of steps
    int stepsPlanned = 0;       // Counter of arrays of actions planned
    int stepsRequested = 0;     // Counter of the calls to Secretary::plan()
    int stepsDispatched = 0;    // Counter of the arrays of actions sent 
    int stepsMissed = 0;
    volatile int emptyGoals = 1; // Number of agents without a task. @todo Why 1?
    std::thread planner_thread;
    std::mutex sync; // To control access to stepsPlanned, stepsDispatched, actionsv, started
    std::condition_variable cv;
    int moves[4];
    bool started = false; // started == true if ...
    std::chrono::high_resolution_clock::time_point start_time;
    std::vector<std::pair<int,int>> achievedGoals; // achievedGoals[a] = last finished task of agent a
    double dispatchTime = 0.0;
    bool halt = false;
public:
    Secretary(PLANNER &_planner) : planner(_planner), actionsv(maxSteps) {}

    /// @brief Function called before planning
    /// @param _env Environment: map, agents, positions, tasks...
    /// @param parameters Parameters of the planner
    void init(SharedEnvironment* _env, const Parameters & parameters) {
        halt = parameters.halt;
        if (halt)
            return;
        shared_env = _env;
        env = *_env;
        moves[0] = 1;
        moves[1] = env.cols;
        moves[2] = -1;
        moves[3] = -env.cols;
        planner.init(&env, parameters); // We call the init function of the planner algorithm planner.init must end before timeout
        // We manualy initialize env to start planning, event if the tasks are unknown
        env.goal_locations.resize(env.num_of_agents);
        emptyGoals = env.num_of_agents;
        for (int i = 0; i < env.num_of_agents; i++)
            achievedGoals.push_back(std::make_pair(-1,-1)); // each agent has a dummy task at location -1 revealed at step -1

        planner_thread = std::thread([this](){this->run();}); // call the main function
    }

    /// @brief Planning function called at each step
    /// @param actions Array of actions that we fill and send for execution
    void plan(std::vector<Action> & actions) {
        if (halt) {
            actions = std::vector<Action>(env.num_of_agents, Action::W);
            return;
        }

        std::unique_lock<std::mutex> lock{sync};
        stepsRequested++;
        if (!started) { // code executed in the first call to this function. @todo change by stepsRequested == 1 ?
            start_time  = std::chrono::high_resolution_clock::now();
            env.curr_states = shared_env->curr_states;
            started = true;
        }
        dispatchTime = elapsed(); // time since the first call to Secretary::plan()
        // Update emptyGoals
        emptyGoals = 0;
        for (int a = 0; a < env.num_of_agents; a++) {
            // if shared_env has a new task and ... @todo what?
            if (env.goal_locations[a].size() < shared_env->goal_locations[a].size() && shared_env->goal_locations[a].front() != achievedGoals[a]) {
                env.goal_locations[a] = shared_env->goal_locations[a];
                // for(auto goal : env.goal_locations[i]) {
                //     std::cout << goal.first << "," << goal.second << " ";
                // }
                // std::cout << std::endl;
            }
            emptyGoals += env.goal_locations[a].empty();
        }
        std::cout << elapsed() << ": "  << "Empty: " << emptyGoals << std::endl;
        lock.unlock();
        cv.notify_one();

        // std::cout << "***" << shared_env->goal_locations.size() << std::endl;
        // for(int i = 0; i < shared_env->goal_locations.size(); i++)
        //     std::cout << shared_env->goal_locations.at(i).size() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(990)); // we let the thread work for 0.99 seconds

        lock.lock();
        if (stepsDispatched >= stepsPlanned) {
            actions = std::vector<Action>(env.num_of_agents, Action::W);
            stepsMissed++;
            std::cout << elapsed() << ": "  << "Sending WAIT!" << std::endl;
            return;
        }
        actions = actionsv[stepsDispatched++]; // get the computed steps
        std::cout << elapsed() << ": "  << "Sending movements." << std::endl;
        lock.unlock();
    }

protected:
    /// @brief Main function, which plans the actions of a step for XXX seconds
    void run() {
        std::cout<< "Planner thread started." << std::endl;
        int count = 0; // Number of steps computed
        while (true) { // run this loop forever (until we plan 6000 steps)
            count++;
            std::vector<Action> actions(env.num_of_agents, Action::W);
            double timeLeft; // time available for planning the next array of actions

            {
                std::unique_lock<std::mutex> lock{sync};
                cv.wait(lock, [this]() { return this->emptyGoals == 0; });
                timeLeft = 1.0 + dispatchTime - elapsed() + count + stepsMissed - stepsRequested; // @todo time for this step ??
                // std::cout<< stepsRequested << " " << elapsed() << " " << timeLeft << std::endl;
                std::cout << elapsed() << ": " << "Time available until step " << count << ": " << timeLeft << std::endl;
                lock.unlock();
            }

            planner.plan(actions, timeLeft);

            // Update env after planning
            std::lock_guard<std::mutex> lock{sync};
            env.curr_timestep++;
            for (int i = 0; i < env.num_of_agents; i++) {
                env.curr_states[i] = result_state(env.curr_states[i], actions[i]); // update states
                if (env.curr_states[i].location == env.goal_locations.at(i).front().first) {
                    // std::cout << elapsed() << ": "  << i << " achieved his goal!\n";
                    achievedGoals[i] = env.goal_locations.at(i).front();
                    env.goal_locations.at(i).erase(env.goal_locations.at(i).begin());
                    if (env.goal_locations.at(i).empty()) // @todo Isn't this always true?
                        emptyGoals = emptyGoals + 1;
                }
            }

            // for(int i = 0; i < 6; i++) {
            //     std::cout << i << " " << env.curr_states[i].location << " " << env.goal_locations.at(i).front().first << std::endl;
            // }

            actionsv[stepsPlanned] = actions;
            stepsPlanned++;
            // std::cout << "Planned steps: " << stepsPlanned << std::endl;
            if (stepsPlanned >= maxSteps)
                break;
        }
    }

    /// @brief Apply an action to a state ( = position = location + direction + time)
    /// @param prev Current state
    /// @param action An action: FW, CR, CCR or W
    /// @return The next state.
    /// @note This is copied from ActionModel.h
    State result_state(const State & prev, Action action) {
        int new_location = prev.location;
        int new_orientation = prev.orientation;
        if (action == Action::FW)       { new_location = new_location += moves[prev.orientation]; }
        else if (action == Action::CR)  { new_orientation = (prev.orientation + 1) % 4; }
        else if (action == Action::CCR) { new_orientation = (prev.orientation + 3) % 4; }
        return State(new_location, prev.timestep + 1, new_orientation);
    }

    /// @brief Count the time since the beginning
    /// @return Number of seconds since start_time was initialized
    double elapsed() {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();
    }
};
