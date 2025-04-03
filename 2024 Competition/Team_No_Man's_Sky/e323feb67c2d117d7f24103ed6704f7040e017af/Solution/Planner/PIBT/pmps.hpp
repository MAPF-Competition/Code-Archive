#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

// Parallel Multiverse PIBT Solver
struct PMPS {

    struct World {
        // множество роботов
        std::vector<Robot> robots;

        double cur_score = 0;

        // used_edge[edge][depth] = robot id
        std::vector<std::array<uint32_t, DEPTH>> used_edge;

        // used_pos[pos][depth] = robot id
        std::vector<std::array<uint32_t, DEPTH>> used_pos;

        // desires[r] = запланированное действие робота
        std::vector<uint32_t> desires;

        // инструмент для рекурсии
        std::vector<uint32_t> visited;
        uint32_t visited_counter = 1;

        Randomizer rnd;

        // сохраненный скор перед попыткой изменения
        double old_score = 0;

        double temp = 1;

        // когда заканчивать выполнение
        TimePoint end_time;

        // препроцессинг
        // smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired)
        std::vector<std::vector<int32_t>> smart_dist_dp;

        // препроцессинг
        // robot_desires[r] = { desired }
        std::vector<std::vector<uint32_t>> robot_desires;

        // важность робота
        std::vector<double> robot_power;

        // грамотный порядок роботов для PIBT
        std::vector<uint32_t> order;

        World(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed);

        bool consider();

        [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

        [[nodiscard]] bool is_free_path(uint32_t r) const;

        [[nodiscard]] EPath get_path(uint32_t r, uint32_t desired) const;

        [[nodiscard]] uint32_t get_used(uint32_t r) const;

        [[nodiscard]] int32_t get_smart_dist_IMPL(uint32_t r, uint32_t desired) const;

        [[nodiscard]] int32_t get_smart_dist(uint32_t r, uint32_t desired) const;

        void update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const;

        void add_path(uint32_t r);

        void remove_path(uint32_t r);

        // return 0, if failed
        // return 1, if success+accepted
        // return 2, if success+not accepted
        uint32_t try_build(uint32_t r, uint32_t &counter, uint32_t depth);

        bool try_build(uint32_t r);

        // return 0, if failed
        // return 1, if success+accepted
        // return 2, if success+not accepted
        uint32_t build(uint32_t r, uint32_t depth, uint32_t &counter);

        bool build(uint32_t r);

        void simulate_pibt();

        [[nodiscard]] double get_triv_score() const;
    };

private:
    // множество роботов
    const std::vector<Robot> &robots;

    World main_world;

public:
    explicit PMPS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed);

    void solve(uint64_t random_seed);

    [[nodiscard]] std::vector<Action> get_actions() const;

    [[nodiscard]] std::vector<uint32_t> get_desires() const;

    [[nodiscard]] std::vector<int64_t> get_changes() const;

    [[nodiscard]] double get_score() const;
};
