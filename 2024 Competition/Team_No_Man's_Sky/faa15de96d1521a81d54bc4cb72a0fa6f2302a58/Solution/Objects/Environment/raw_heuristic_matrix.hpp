#pragma once

#include <Objects/Environment/graph.hpp>

class RawHeuristicMatrix {
    // matrix[target (map pos)][source (graph node)] = dist source -> target
    std::vector<std::vector<uint16_t>> matrix;

    void build(uint32_t source, const Graph &graph);

public:
    RawHeuristicMatrix() = default;

    explicit RawHeuristicMatrix(const Graph &graph);

    // source and dest is graph node
    // if dest is NONE(zero), then returns INVALID_DIST
    // [[nodiscard]] uint32_t get(uint32_t source, uint32_t target) const;

    // source is graph node
    // target is map pos
    // if dest is NONE(zero), then returns INVALID_DIST
    [[nodiscard]] uint32_t get(uint32_t source, uint32_t target) const;
};

RawHeuristicMatrix &get_rhm();
