#include <Planner/Default/default_planner_solver.hpp>

#include <Objects/Environment/environment.hpp>
#include <Objects/Basic/randomizer.hpp>

#include "planner.h"
#include "SharedEnv.h"
#include "const.h"
#include "flow.h"
#include "heuristics.h"
#include "pibt.h"

#include <thread>
#include <mutex>
#include <atomic>

std::vector<std::vector<pair<int, int>>>
DefaultPlannerSolver::get_goal_locations(const std::vector<uint32_t> &desired_dirs) const {
    std::vector<std::vector<pair<int, int>>> goal_locations(desired_dirs.size());
    for (uint32_t r = 0; r < desired_dirs.size(); r++) {
        Position p = get_graph().get_pos(get_robots_handler().get_robot(r).node);
        uint32_t dir = desired_dirs[r];
        if (dir == -1) {

        } else {
            p = Position(p.get_pos(), dir);
            if (p.move_forward().is_valid()) {
                p = p.move_forward();
            }
        }
        goal_locations[r].emplace_back(p.get_pos() - 1, 0);
    }
    return goal_locations;
}

std::vector<Action> DefaultPlannerSolver::get_actions(const std::vector<uint32_t> &desired_dirs) const {
    std::vector<Action> actions;
    auto goal_locations = get_goal_locations(desired_dirs);

    //default planner data
    std::vector<int> decision;
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DefaultPlanner::DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    DefaultPlanner::TrajLNS trajLNS;

    {
        assert(env->num_of_agents != 0);
        p.resize(env->num_of_agents);
        decision.resize(env->map.size(), -1);
        prev_states.resize(env->num_of_agents);
        next_states.resize(env->num_of_agents);
        decided.resize(env->num_of_agents, DefaultPlanner::DCR({-1, DefaultPlanner::DONE::DONE}));
        occupied.resize(env->map.size(), false);
        checked.resize(env->num_of_agents, false);
        ids.resize(env->num_of_agents);
        require_guide_path.resize(env->num_of_agents, false);
        for (int i = 0; i < ids.size(); i++) {
            ids[i] = i;
        }

        // initialise the heuristics tables containers
        //init_heuristics(env);
        std::mt19937 mt1;
        mt1.seed(0);
        srand(0);

        new(&trajLNS) DefaultPlanner::TrajLNS(env, DefaultPlanner::global_heuristictable,
                                              DefaultPlanner::global_neighbors);
        trajLNS.init_mem();

        //assign intial priority to each agent
        std::shuffle(ids.begin(), ids.end(), mt1);
        for (int i = 0; i < ids.size(); i++) {
            p[ids[i]] = ((double) (ids.size() - i)) / ((double) (ids.size() + 1));
        }
        p_copy = p;
    }

    TimePoint start_time = std::chrono::steady_clock::now();
    //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
    int pibt_time = DefaultPlanner::PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
    //TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - DefaultPlanner::TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);

    // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
    /*if (env->curr_timestep == 0) {
        dummy_goals.resize(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; i++) {
            dummy_goals.at(i) = env->curr_states.at(i).location;
        }
    }*/

    // data sturcture for record the previous decision of each agent
    prev_decision.clear();
    prev_decision.resize(get_map().get_size(), -1);

    // update the status of each agent and prepare for planning
    int count = 0;
    for (int i = 0; i < desired_actions.size(); i++) {
        //initialise the shortest distance heuristic table for the goal location of the agent
        //if ((std::chrono::steady_clock::now() < end_time)) {
        for (int j = 0; j < goal_locations[i].size(); j++) {
            int goal_loc = goal_locations[i][j].first;
            if (trajLNS.heuristics.at(goal_loc).empty()) {
                init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
                count++;
            }
        }
        //}


        // set the goal location of each agent
        if (goal_locations[i].empty()) {
            trajLNS.tasks[i] = dummy_goals.at(i);
            p[i] = p_copy[i];
        } else {
            trajLNS.tasks[i] = goal_locations[i].front().first;
        }

        // check if the agent need a guide path update, when the agent has no guide path or the guide path does not end at the goal location
        require_guide_path[i] = false;
        if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
            require_guide_path[i] = true;

        // check if the agent completed the action in the previous timestep
        // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
        assert(env->curr_states[i].location >= 0);
        prev_states[i] = env->curr_states[i];
        next_states[i] = State();
        prev_decision[env->curr_states[i].location] = i;
        if (decided[i].loc == -1) {
            decided[i].loc = env->curr_states[i].location;
            assert(decided[i].state == DefaultPlanner::DONE::DONE);
        }
        if (prev_states[i].location == decided[i].loc) {
            decided[i].state = DefaultPlanner::DONE::DONE;
        }
        if (decided[i].state == DefaultPlanner::DONE::NOT_DONE) {
            decision.at(decided[i].loc) = i;
            next_states[i] = State(decided[i].loc, -1, -1);
        }

        // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
        if (require_guide_path[i])
            p[i] = p_copy[i];
        else if (!goal_locations[i].empty())
            p[i] = p[i] + 1;

        // give priority bonus to the agent if the agent is in a deadend location
        if (!goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1) {
            p[i] = p[i] + 10;
        }
    }

    // compute the congestion minimised guide path for the agents that need guide path update
    for (int i = 0; i < env->num_of_agents; i++) {
        //if (std::chrono::steady_clock::now() > end_time)
        //    break;
        if (require_guide_path[i]) {
            if (!trajLNS.trajs[i].empty())
                remove_traj(trajLNS, i);
            update_traj(trajLNS, i);
        }
    }

    // iterate and recompute the guide path to optimise traffic flow
    //std::unordered_set<int> updated;
    //frank_wolfe(trajLNS, updated, get_now() + Milliseconds(100));

    // sort agents based on the current priority
    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
        return p.at(a) > p.at(b);
    });

    // compute the targeted next location for each agent using PIBT
    for (int i: ids) {
        if (decided[i].state == DefaultPlanner::DONE::NOT_DONE) {
            continue;
        }
        if (next_states[i].location == -1) {
            assert(prev_states[i].location >= 0 && prev_states[i].location < env->map.size());
            causalPIBT(i, -1, prev_states, next_states,
                       prev_decision, decision,
                       occupied, trajLNS);
        }
    }

    // post processing the targeted next location to turning or moving actions
    actions.resize(env->num_of_agents);
    for (int id: ids) {
        //clear the decision table based on which agent has next_states
        if (next_states.at(id).location != -1)
            decision.at(next_states.at(id).location) = -1;
        // if agent is newly assigned a targeted next location, record the decision as not done yet
        if (next_states.at(id).location >= 0) {
            decided.at(id) = DefaultPlanner::DCR({next_states.at(id).location, DefaultPlanner::DONE::NOT_DONE});
        }

        // post process the targeted next location to turning or moving actions
        actions.at(id) = DefaultPlanner::getAction(prev_states.at(id), decided.at(id).loc, env);
        checked.at(id) = false;
    }

    // recursively check if the FW action can be executed by checking whether all agents in the front of the agent can move forward
    // if any agent cannot move foward due to turning, all agents behind the turning agent will not move forward.
    for (int id = 0; id < env->num_of_agents; id++) {
        if (!checked.at(id) && actions.at(id) == Action::FW) {
            moveCheck(id, checked, decided, actions, prev_decision);
        }
    }

    //trajLNS.~TrajLNS();

    ASSERT(actions.size() == desired_actions.size(), "unmatch sizes");
    return actions;
}

double DefaultPlannerSolver::get_score(const std::vector<uint32_t> &desired_dirs) const {
    auto actions = get_actions(desired_dirs);
    uint32_t cnt_ok = 0;
    for (uint32_t r = 0; r < actions.size(); r++) {
        if (actions[r] == desired_actions[r]) {
            cnt_ok++;
        }
    }
    return cnt_ok * 100.0 / actions.size();
}

DefaultPlannerSolver::DefaultPlannerSolver(SharedEnvironment *env, const std::vector<Action> &desired_actions,
                                           const std::vector<uint32_t> &init_desired_dirs) :
        env(env), desired_actions(desired_actions), best_desired_dirs(init_desired_dirs) {

    Randomizer rnd;
    // TODO: random init
    /*best_desired_dirs.resize(desired_actions.size(), -1);
    for (uint32_t r = 0; r < best_desired_dirs.size(); r++) {
        best_desired_dirs[r] = rnd.get(-1, 3);
    }*/
    best_score = get_score(best_desired_dirs);
}

void DefaultPlannerSolver::solve(TimePoint end_time) {
    Randomizer rnd;
    Printer() << best_score;

    std::mutex mutex;
    std::atomic<uint32_t> step;

    auto do_work = [&]() {
        for (; get_now() < end_time; ++step) {
            std::vector<uint32_t> desired_dirs;
            // copy desired_dirs
            {
                std::unique_lock locker(mutex);
                desired_dirs = best_desired_dirs;
            }

            // random change
            for (uint32_t N = rnd.get(1, 5); N; N--) {
                uint32_t r = rnd.get(0, desired_actions.size() - 1);
                desired_dirs[r] = rnd.get(-1, 3);
            }

            double cur_score = get_score(desired_dirs);

            if (cur_score > best_score) {
                std::unique_lock locker(mutex);
                if (cur_score > best_score) {
                    Printer() << "->" << cur_score;
                    best_score = cur_score;
                    best_desired_dirs = desired_dirs;
                }
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    /*for (; get_now() < end_time; step++) {
        uint32_t r = rnd.get(0, desired_dirs.size() - 1);
        uint32_t old_d = desired_dirs[r];
        desired_dirs[r] = rnd.get(-1, 3);

        double new_score = get_score();

        //ASSERT(score == get_score(), "no determine");

        ASSERT(get_actions() == get_actions(), "no determine");

        if (new_score > cur_score) {
            cur_score = new_score;
            Printer() << "->" << new_score;
        } else {
            desired_dirs[r] = old_d;
            //ASSERT(old_score == get_score(), "invalid rollback");
        }
    }*/
    Printer() << '\n';
    Printer() << "DefaultPlannerSolver steps: " << step << '\n';
}

std::vector<uint32_t> DefaultPlannerSolver::get_targets(const std::vector<uint32_t> &desired_dirs) const {
    auto goal_locations = get_goal_locations(desired_dirs);
    std::vector<uint32_t> targets;
    for (uint32_t r = 0; r < goal_locations.size(); r++) {
        targets.push_back(goal_locations[r][0].first);
    }
    return targets;
}

namespace DefaultPlanner {
    extern std::vector<int> decision;
    extern std::vector<int> prev_decision;
    extern std::vector<double> p;
    extern std::vector<State> prev_states;
    extern std::vector<State> next_states;
    extern std::vector<int> ids;
    extern std::vector<double> p_copy;
    extern std::vector<bool> occupied;
    extern std::vector<DefaultPlanner::DCR> decided;
    extern std::vector<bool> checked;
    extern std::vector<bool> require_guide_path;
    extern std::vector<int> dummy_goals;
    extern DefaultPlanner::TrajLNS trajLNS;
}

std::vector<uint32_t> call_default_planner_solver(SharedEnvironment *env, const std::vector<Action> &desired_actions,
                                                  const std::vector<uint32_t> &init_desired_dirs) {
    FAILED_ASSERT("TODO");
    DefaultPlannerSolver solver(env, desired_actions, init_desired_dirs);
    /*solver.solve(get_now() + Milliseconds(SCHEDULER_TRICK_TIME));

    ASSERT(solver.get_actions(solver.best_desired_dirs) == solver.get_actions(solver.best_desired_dirs),
           "no determine");

    std::vector<Action> my_actions = solver.get_actions(solver.best_desired_dirs);

    std::vector<Action> default_actions;
    {
        DefaultPlanner::decision.clear();
        DefaultPlanner::prev_decision.clear();
        DefaultPlanner::p.clear();
        DefaultPlanner::prev_states.clear();
        DefaultPlanner::next_states.clear();
        DefaultPlanner::ids.clear();
        DefaultPlanner::p_copy.clear();
        DefaultPlanner::occupied.clear();
        DefaultPlanner::decided.clear();
        DefaultPlanner::checked.clear();
        DefaultPlanner::require_guide_path.clear();
        DefaultPlanner::dummy_goals.clear();
        DefaultPlanner::trajLNS.~TrajLNS();
    }
    {
        env->goal_locations = solver.get_goal_locations(solver.best_desired_dirs);
        DefaultPlanner::initialize(0, env);
        DefaultPlanner::plan(0, default_actions, env);
    }

    ASSERT(default_actions == my_actions, "pizda");*/
    {
        DefaultPlanner::decision.clear();
        DefaultPlanner::prev_decision.clear();
        DefaultPlanner::p.clear();
        DefaultPlanner::prev_states.clear();
        DefaultPlanner::next_states.clear();
        DefaultPlanner::ids.clear();
        DefaultPlanner::p_copy.clear();
        DefaultPlanner::occupied.clear();
        DefaultPlanner::decided.clear();
        DefaultPlanner::checked.clear();
        DefaultPlanner::require_guide_path.clear();
        DefaultPlanner::dummy_goals.clear();
        DefaultPlanner::trajLNS.~TrajLNS();
    }
    //4414

    return solver.get_targets(solver.best_desired_dirs);
}
