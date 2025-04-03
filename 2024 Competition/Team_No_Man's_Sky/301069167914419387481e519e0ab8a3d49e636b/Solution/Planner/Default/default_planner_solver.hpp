#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>

#include <atomic>

struct DefaultPlannerSolver {

    // desired_actions[r]
    // те операции, что нам выдал PIBTS
    // мы хотим получить решение близко к этому
    std::vector<Action> desired_actions;

    // desired_dirs[r] = 0-3 направления задачи, -1 если она прям здесь
    std::vector<uint32_t> best_desired_dirs;

    std::atomic<double> best_score;

    SharedEnvironment *env = nullptr;

    [[nodiscard]] std::vector<Action> get_actions(const std::vector<uint32_t> &desired_dirs) const;

    [[nodiscard]] double get_score(const std::vector<uint32_t> &desired_dirs) const;

    DefaultPlannerSolver(SharedEnvironment *env, const std::vector<Action> &desired_actions,
                         const std::vector<uint32_t> &init_desired_dirs);

    void solve(TimePoint end_time);

    [[nodiscard]] std::vector<std::vector<pair<int, int>>>
    get_goal_locations(const std::vector<uint32_t> &desired_dirs) const;

    [[nodiscard]] std::vector<uint32_t> get_targets(const std::vector<uint32_t> &desired_dirs) const;

};

std::vector<uint32_t> call_default_planner_solver(SharedEnvironment *env, const std::vector<Action> &desired_actions,
                                                  const std::vector<uint32_t> &init_desired_dirs);

