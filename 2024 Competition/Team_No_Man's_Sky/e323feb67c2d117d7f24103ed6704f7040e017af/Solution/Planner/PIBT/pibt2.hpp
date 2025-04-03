#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

//#include <optional>
//#include <unordered_map>

// -i ./example_problems/random.domain/random_32_32_20_300.json -o test.json -s 10000 -t 10000 -p 100000000
// score: 28611
// timer: 14.1829s

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
class PIBT2 {
    // [r][edge] = weight
    //const std::vector<std::unordered_map<uint32_t, uint32_t>> &weights;

    const std::vector<Robot> &robots;

    TimePoint end_time;

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    // действие, которое он хочет
    std::vector<uint32_t> desires;

    /*struct State {
        // used_edge[depth][edge] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_edge;

        // used_pos[depth][pos] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_pos;

        // действие, которое он хочет
        std::unordered_map<uint32_t, uint32_t> desires;
    };*/

    [[nodiscard]] bool validate_path(uint32_t r) const;

    //[[nodiscard]] bool validate_path(uint32_t r, const State &state) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    //[[nodiscard]] bool is_free_path(uint32_t r, const State &state) const;

    [[nodiscard]] const EPath &get_path(uint32_t r) const;

    //[[nodiscard]] EPath get_path(uint32_t r, const State &state) const;

    //[[nodiscard]] uint32_t get_path_weight(uint32_t r) const;

    //[[nodiscard]] uint32_t get_path_weight(uint32_t r, const State &state) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    //[[nodiscard]] uint32_t get_used(uint32_t r, const State &state) const;

    void add_path(uint32_t r);

    //void add_path(uint32_t r, State &state) const;

    void remove_path(uint32_t r);

    //void remove_path(uint32_t r, State &state) const;

    bool build(uint32_t r, uint32_t depth, uint32_t &counter);

    //std::optional<State> build2_IMPL(uint32_t r) const;

    //bool build2(uint32_t r);

public:
    PIBT2(const std::vector<Robot> &robots//, const std::vector<std::unordered_map<uint32_t, uint32_t>> &weights
    );

    std::vector<Action> solve(const std::vector<uint32_t> &order, const TimePoint end_time);
};
