#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <settings.hpp>

#include <SharedEnv.h>

#include <cstdint>
#include <vector>

struct SchedulerSolver {

    double cur_score = 0;

    // desires[r] = task id
    std::vector<uint32_t> desires;

    // task_to_robot[task] = robot id
    std::vector<uint32_t> task_to_robot;

    std::vector<uint32_t> free_robots;

    std::vector<uint32_t> free_tasks;

    // dp[r] = отсортированный вектор (dist, task_id)
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp;

    std::vector<int> timestep_updated;

    SharedEnvironment *env = nullptr;

    // task_metric[t]
    std::vector<uint32_t> task_metric;

    // task_target[t] = цель задачи (pos)
    std::vector<uint32_t> task_target;

    // phantom_agent_dist[r] = расстояние до выполнения задачи фантомного робота r
    // если это значение 0, то это обычный робот, иначе фантомный
    std::vector<uint32_t> phantom_agent_dist;

    double temp = 1;

    void rebuild_dp(uint32_t r);

    [[nodiscard]] bool compare(double cur_score, double old_score, Randomizer &rnd) const;

    template<typename rollback_t>
    bool consider(double old_score, Randomizer &rnd, rollback_t &&rollback) {
        if (compare(cur_score, old_score, rnd)) {
            return true;
        } else {
            rollback();
            ASSERT(std::abs(old_score - cur_score) / std::max(std::abs(old_score), std::abs(cur_score)) < 1e-6,
                   "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
                           std::to_string(old_score - cur_score));
            return false;
        }
    }

    [[nodiscard]] uint64_t get_dist(uint32_t r, uint32_t t) const;

    //void set(uint32_t r, uint32_t t);

    void remove(uint32_t r);

    void add(uint32_t r, uint32_t t);

    bool try_peek_task(Randomizer &rnd);

    void validate();

public:
    SchedulerSolver() = default;

    explicit SchedulerSolver(SharedEnvironment *env);

    void update();

    void rebuild_dp(TimePoint end_time);

    void triv_solve(TimePoint end_time);

    void solve(TimePoint end_time);

    [[nodiscard]] std::vector<int> get_schedule(TimePoint end_time) const;

    [[nodiscard]] double get_score() const;
};
