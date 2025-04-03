#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Environment/graph_guidance.hpp>
#include <Objects/Environment/map.hpp>

#include <array>
#include <cstdint>
#include <vector>

// Contains information about graph.
// About edges, transform position to node and back.
// Undirected edges.
// 0 = NONE
class Graph {

    // pos_to_node[pos][dir] = graph node
    std::vector<std::array<uint32_t, 4>> pos_to_node;

    // node_to_pos[node] = position
    std::vector<Position> node_to_pos;

    // to_node[node][action] = to node
    std::vector<std::array<uint32_t, 4>> to_node;

    // to_edge[node][action] = to edge
    std::vector<std::array<uint32_t, 4>> to_edge;

    // to_weight[node][action] = weight
    std::vector<std::array<uint32_t, 4>> weight;

    // pos_to_zip[pos] = zip
    std::vector<uint32_t> pos_to_zip;

    // zip_to_pos[zip] = pos
    std::vector<uint32_t> zip_to_pos;

    uint32_t edges_size = 0;

public:
    Graph() = default;

    explicit Graph(const Map &map, const GraphGuidance &gg);

    [[nodiscard]] uint32_t get_zipes_size() const;

    [[nodiscard]] uint32_t get_nodes_size() const;

    [[nodiscard]] uint32_t get_edges_size() const;

    [[nodiscard]] uint32_t get_zip(uint32_t pos) const;

    [[nodiscard]] uint32_t get_pos_from_zip(uint32_t zip) const;

    // graph node -> Position
    [[nodiscard]] Position get_pos(uint32_t node) const;

    // Position -> graph node
    [[nodiscard]] uint32_t get_node(const Position &pos) const;

    // (pos, dir) -> graph node
    [[nodiscard]] uint32_t get_node(uint32_t pos, uint32_t dir) const;

    // graph node + action -> graph node
    [[nodiscard]] uint32_t get_to_node(uint32_t node, uint32_t action) const;

    // graph node + action -> graph edge
    [[nodiscard]] uint32_t get_to_edge(uint32_t node, uint32_t action) const;

    // graph node + action -> edge weight from GraphGuidance
    [[nodiscard]] uint32_t get_weight(uint32_t node, uint32_t action) const;
};

Graph &get_graph();
