#include "Entry.h"
#include "Tasks.h"
#include "heuristics.h"
#include "utils.h"

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit) {
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule) {
    static ETimer total_timer;
    ETimer timer;
    PRINT(
            Printer() << "\n";
            Printer() << "Timestep: " << env->curr_timestep << '\n';);

    //if (get_map_type() != MapType::RANDOM) {
    //    return;
    //}

    //call the task scheduler to assign tasks to agents
    scheduler->plan(time_limit, proposed_schedule);

    PRINT(
            uint32_t cnt = 0;
            for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
                cnt += proposed_schedule[r] != -1;
            } Printer()
            << "Scheduler robots init: " << cnt << "/" << proposed_schedule.size() << " ("
            << cnt * 100.0 / proposed_schedule.size() << "%)\n";);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);

    //call the planner to compute the actions
    planner->plan(time_limit, plan);

    PRINT(
            static std::array<uint64_t, 5> total_counts{};
            std::array<uint32_t, 5> counts{};
            for (uint32_t r = 0; r < plan.size(); r++) {
                counts[plan[r]]++;
                total_counts[plan[r]]++;
            };
            Printer() << "Actions:\n";
            Printer() << "F: " << counts[0] << '\n';
            Printer() << "R: " << counts[1] << '\n';
            Printer() << "C: " << counts[2] << '\n';
            Printer() << "W: " << counts[3] << '\n';
            Printer() << "N: " << counts[4] << '\n';

            Printer() << "Total action:\n";
            Printer() << "F: " << total_counts[0] << '\n';
            Printer() << "R: " << total_counts[1] << '\n';
            Printer() << "C: " << total_counts[2] << '\n';
            Printer() << "W: " << total_counts[3] << '\n';
            Printer() << "N: " << total_counts[4] << '\n';);

#ifdef ENABLE_SCHEDULER_TRICK
    PRINT(
            static double sum_plans_equal = 0;
            uint32_t cnt_ok = 0;
            static std::array<std::array<uint32_t, 5>, 5> data;
            get_myplan().resize(plan.size(), Action::W);
            for (uint32_t r = 0; r < plan.size(); r++) {
                cnt_ok += plan[r] == get_myplan()[r];
                ASSERT(static_cast<uint32_t>(plan[r]) < 4, "invalid plan");
                ASSERT(static_cast<uint32_t>(get_myplan()[r]) < 4, "invalid plan");
                data[get_myplan()[r]][plan[r]]++;
            };
            Printer() << "Plans equal: " << cnt_ok * 100.0 / plan.size() << "%\n";
            sum_plans_equal += cnt_ok * 100.0 / plan.size();
            Printer() << "Total plans equal: " << sum_plans_equal / (1 + env->curr_timestep) << "%\n";
            for (uint32_t a = 0; a < 4; a++) {
                for (uint32_t b = 0; b < 4; b++) {
                    char ca = a == 0 ? 'F' : (a == 1 ? 'R' : (a == 2 ? 'C' : 'W'));
                    char cb = b == 0 ? 'F' : (b == 1 ? 'R' : (b == 2 ? 'C' : 'W'));
                    Printer() << ca << cb << ": " << data[a][b] << '\n';
                }
            });
#endif

    PRINT(
            Printer() << "Entry time: " << timer << '\n';
            Printer() << "Total time: " << total_timer << '\n';);
}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> &proposed_schedule) {
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++) {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        ASSERT(env->task_pool.count(t_id), "no contains task");
        auto &task = env->task_pool[t_id];
        int i_loc = task.idx_next_loc;
        ASSERT(i_loc < env->task_pool[t_id].locations.size(), "invalid i_loc");
        env->goal_locations[i].push_back(
                {task.locations.at(i_loc), task.t_revealed});
    }
}
