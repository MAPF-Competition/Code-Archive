#pragma once

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/graph.hpp>

#include <vector>
#include <cstdint>

// Dynamic Heuristic Matrix for Robots
class DHMR {
    // [r][node] = dist to target
    std::vector<std::vector<uint32_t>> matrix;

    // pos_to_robot[pos] = robot id or -1
    std::vector<uint32_t> pos_to_robot;

    // [r][node]
    std::vector<std::vector<uint32_t>> visited_sphere;

    // [r][node]
    std::vector<std::vector<uint32_t>> visited;

    // [r][node]
    //std::vector<std::vector<uint32_t>> visited_matrix;

    void build(uint32_t r, uint32_t timestep);

public:

    DHMR() = default;

    explicit DHMR(const Graph &graph);

    void update(uint32_t timestep, TimePoint end_time);

    [[nodiscard]] uint32_t get(uint32_t r, uint32_t node) const;
};

DHMR &get_dhmr();
