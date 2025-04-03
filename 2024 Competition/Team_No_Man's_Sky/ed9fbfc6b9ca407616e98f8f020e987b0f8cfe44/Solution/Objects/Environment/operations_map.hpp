#pragma once

#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/operations.hpp>

#include <tuple>

class OperationsMap {

    // map_nodes[node][operation] = nodes path
    std::vector<std::vector<EPath>> map_nodes;

    // map_edges[node][operation] = edges path
    std::vector<std::vector<EPath>> map_edges;

    // map_poses[node][operation] = poses path
    std::vector<std::vector<EPath>> map_poses;

    static std::tuple<EPath, EPath, EPath> get_paths(uint32_t node, const Operation &operation);

    void build(uint32_t source, const std::vector<Operation> &operations);

public:
    OperationsMap() = default;

    OperationsMap(const Graph &graph, const std::vector<Operation> &operations);

    [[nodiscard]] const EPath &get_nodes_path(uint32_t node, uint32_t operation) const;

    [[nodiscard]] const EPath &get_edges_path(uint32_t node, uint32_t operation) const;

    [[nodiscard]] const EPath &get_poses_path(uint32_t node, uint32_t operation) const;
};

OperationsMap &get_omap();
