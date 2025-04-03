#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

#include <atomic>
#include <map>
#include <unordered_map>
#include <unordered_set>

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
// Solver mode
class PIBTS {

    std::atomic<uint32_t> version = 0;

    double cur_score = 0;

    const std::vector<Robot> &robots;

    TimePoint end_time;

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    // desires[r] = запланированное действие робота
    std::vector<uint32_t> desires;

    struct State {
        uint32_t version = 0;

        double cur_score = 0;

        // used_edge[depth][edge] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_edge;

        // used_pos[depth][pos] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_pos;

        // desires[r] = запланированное действие робота
        std::unordered_map<uint32_t, uint32_t> desires;
    };

    // neighbours[r] = { neighbours }
    std::vector<std::vector<uint32_t>> neighbours;

    std::vector<uint32_t> order;

    std::vector<uint32_t> weight;
    uint32_t max_weight = 0;

    std::vector<uint32_t> visited;
    uint32_t visited_counter = 1;

    Randomizer rnd;

    double old_score = 0;

    double temp = 1;

    double visited_bound = 0.8;

    // smart_dist_dp[r][desired]
    std::vector<std::vector<int32_t>> smart_dist_dp;

    [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    [[nodiscard]] bool is_free_path(uint32_t r, const State &state) const;

    [[nodiscard]] EPath get_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r, const State &state) const;

    [[nodiscard]] int32_t get_smart_dist_IMPL(uint32_t r, uint32_t desired) const;

    [[nodiscard]] int32_t get_smart_dist(uint32_t r, uint32_t desired) const;

    void update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const;

    void add_path(uint32_t r);

    void add_path(uint32_t r, State &state) const;

    void remove_path(uint32_t r);

    void remove_path(uint32_t r, State &state) const;

    void flush_state(const State &state);

    bool try_build_state(uint32_t r, State &state, uint32_t &counter, Randomizer &rnd) const;

    bool try_build_state(uint32_t r, Randomizer &rnd);

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

    bool build_state(uint32_t r, uint32_t depth, uint32_t &counter, State &state);

    bool build_state(uint32_t r);

    uint32_t parallel_build(uint32_t r, uint32_t depth, uint32_t &counter, State &state, Randomizer& rnd) const;

    bool parallel_build(uint32_t r, Randomizer& rnd);

    void do_work(uint32_t thr);

public:
    explicit PIBTS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed);

    void simulate_pibt();

    [[nodiscard]] double get_score() const;

    [[nodiscard]] std::vector<Action> get_actions() const;

    [[nodiscard]] std::vector<uint32_t> get_desires() const;

    [[nodiscard]] std::vector<int64_t> get_changes() const;

    [[nodiscard]] double get_kek();
};
