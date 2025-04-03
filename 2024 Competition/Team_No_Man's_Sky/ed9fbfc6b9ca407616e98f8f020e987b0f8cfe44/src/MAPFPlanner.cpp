#include <Entry.h>
#include <random>

//default planner includes
#include "const.h"
#include "heuristics.h"
#include "planner.h"

#include <Objects/Environment/environment.hpp>
#include <settings.hpp>

#include <thread>

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 *
 * This function call the planner initialize function with a time limit fore preprocessing.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit) {
#ifdef ENABLE_DEFAULT_PLANNER
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit, env);
#elif defined(ENABLE_SMART_PLANNER)
    smart_planner.initialize(preprocess_time_limit);
#else
    init_environment(*env);
#endif
}

namespace DefaultPlanner {
    extern std::unordered_set<int> free_agents;
    extern std::unordered_set<int> free_tasks;
};// namespace DefaultPlanner
using namespace DefaultPlanner;

std::vector<HeuristicTable> empty_heuristic_table;
std::vector<HeuristicTable> save_heuristic_table;

/**
 * Plans a path using default planner
 *
 * This function performs path planning within the timelimit given, and call the plan function in default planner.
 * The planned actions are output to the provided actions vector.
 *
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit, vector<Action> &actions) {
#ifdef ENABLE_DEFAULT_PLANNER
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::plan(limit, actions, env);
#elif defined(ENABLE_SMART_PLANNER)
    actions.clear();
    smart_planner.plan(time_limit, actions);
#else
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 50);

    update_environment(*env);
#ifdef ENABLE_GUIDANCE_PATH_PLANNER
    get_gpp().update(env->curr_timestep, end_time);
#endif
    auto [plan, desired_plan] = eplanner.plan(end_time);
    actions = std::move(plan);

#ifdef ENABLE_DEFAULT_SCHEDULER_TRICK
    if (get_map_type() == MapType::RANDOM) {
        DefaultPlanner::free_agents.clear();
        DefaultPlanner::free_tasks.clear();

        for (uint32_t r = 0; r < env->num_of_agents; r++) {
            uint32_t t = env->curr_task_schedule[r];
            if (t == -1) {
                continue;
            }
            env->task_pool.at(t).agent_assigned = r;
        }

        // добавить свободные задачи
        for (auto &[t, task]: env->task_pool) {
            if (task.agent_assigned == -1) {
                DefaultPlanner::free_tasks.insert(t);
            }
        }

        // добавить агентов без задач
        for (uint32_t r = 0; r < env->num_of_agents; r++) {
            uint32_t t = env->curr_task_schedule[r];
            if (t == -1) {
                DefaultPlanner::free_agents.insert(r);
            }
        }

        // добавить агентов с задачами
        {
            for (uint32_t r = 0; r < env->num_of_agents; r++) {
                uint32_t t = env->curr_task_schedule[r];
                if (t == -1) {
                    continue;
                }
                ASSERT(env->task_pool.count(t), "no contains");
                const auto &task = env->task_pool.at(t);
                // есть задача и она в процессе выполнения
                // не можем ее убрать
                if (task.idx_next_loc != 0) {
                    continue;
                }
                uint32_t node = get_robots_handler().get_robot(r).node;
                uint32_t pos = get_graph().get_pos(node).get_pos();
                uint32_t to_node = get_graph().get_to_node(node, actions[r]);
                uint32_t to_pos = get_graph().get_pos(to_node).get_pos();
                uint32_t target = task.locations[0] + 1;

                if (pos == target || to_pos == target) {
                    continue;
                }

                DefaultPlanner::free_agents.insert(r);
                DefaultPlanner::free_tasks.insert(t);
            }
        }
    }
#endif

#ifdef ENABLE_HM_SCHEDULER_TRICK
    if (get_map_type() == MapType::RANDOM) {
        PRINT(Printer() << "enable hm scheduler trick\n";);
        ETimer timer;

        // clear HM
        // global_heuristictable = empty_heuristic_table;

        auto get_dist = [&](uint32_t from, uint32_t to) {
            return save_heuristic_table[to].htable[from];
        };
        // init HM with task dist
        for (uint32_t t: free_tasks) {
            auto &task = env->task_pool.at(t);
            for (int i = 0; i + 1 < task.locations.size(); i++) {
                uint32_t from = task.locations[i];
                uint32_t to = task.locations[i + 1];
                HeuristicTable &ht = global_heuristictable[to];
                ht.htable[from] = get_dist(from, to);
            }
        }
        PRINT(Printer() << "build HM scheduler trick 1: " << timer << "\n";);
        // init HM with target dist
        {
            std::vector<uint32_t> vec_free_tasks;
            for (uint32_t t: free_tasks) {
                vec_free_tasks.push_back(t);
            }
            ASSERT(vec_free_tasks.size() == free_tasks.size(), "invalid vec");

            auto do_work = [&](uint32_t thr) {
                for (uint32_t k = thr; k < vec_free_tasks.size(); k += THREADS) {
                    uint32_t t = vec_free_tasks[k];
                    ASSERT(env->task_pool.count(t), "no contains task");
                    auto &task = env->task_pool.at(t);
                    ASSERT(!task.locations.empty(), "is empty");
                    ASSERT(task.idx_next_loc == 0, "assigned task");
                    uint32_t to = task.locations[0];
                    HeuristicTable &ht = global_heuristictable[to];
                    for (uint32_t r: free_agents) {
                        uint32_t from = env->curr_states[r].location;
                        ht.htable[from] = get_dist(from, to) * 5;
                    }
                }
            };
            std::vector<std::thread> threads(THREADS);
            for (uint32_t thr = 0; thr < THREADS; thr++) {
                threads[thr] = std::thread(do_work, thr);
            }
            for (uint32_t thr = 0; thr < THREADS; thr++) {
                threads[thr].join();
            }
            /*for (uint32_t t: free_tasks) {
                auto &task = env->task_pool.at(t);
                uint32_t to = task.locations[0];
                HeuristicTable &ht = global_heuristictable[to];
                for (uint32_t r: free_agents) {
                    uint32_t from = env->curr_states[r].location;
                    ht.htable[from] = get_dist(from, to) * 5;
                }
            }*/
        }
        static uint32_t total_time = 0;
        total_time += timer.get_ms();
        PRINT(Printer() << "build HM scheduler trick total: " << timer << ", mean: " << total_time * 1.0 / (1 + env->curr_timestep) << "ms\n";);
    }
#endif

#endif
}
