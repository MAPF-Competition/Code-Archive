#include <Objects/Environment/operations_map.hpp>

#include <Objects/Basic/assert.hpp>

std::tuple<EPath, EPath, EPath> OperationsMap::get_paths(uint32_t node, const Operation &operation) {
    if (!node) {
        return {EPath{}, EPath{}, EPath{}};
    }
    EPath nodes_path{};
    EPath edges_path{};
    EPath poses_path{};

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = operation[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node) {
            return {EPath{}, EPath{}, EPath{}};
        }

        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        nodes_path[depth] = to_node;
        edges_path[depth] = to_edge;
        poses_path[depth] = get_graph().get_zip(to_pos);

        node = to_node;
    }
    return {nodes_path, edges_path, poses_path};
}

void OperationsMap::build(uint32_t source, const std::vector<Operation> &operations) {
    auto &nodes = map_nodes[source];
    auto &edges = map_edges[source];
    auto &poses = map_poses[source];

    for (auto operation: operations) {
        auto [nodes_path, edges_path, poses_path] = get_paths(source, operation);
        nodes.push_back(nodes_path);
        edges.push_back(edges_path);
        poses.push_back(poses_path);
    }
}

OperationsMap::OperationsMap(const Graph &graph, const std::vector<Operation> &operations) {
    map_nodes.resize(graph.get_nodes_size());
    map_edges.resize(graph.get_nodes_size());
    map_poses.resize(graph.get_nodes_size());

    for (uint32_t node = 0; node < graph.get_nodes_size(); node++) {
        build(node, operations);
    }
}

const EPath &OperationsMap::get_nodes_path(uint32_t node, uint32_t operation) const {
    ASSERT(node < map_nodes.size(), "invalid node");
    ASSERT(operation < map_nodes[node].size(), "invalid operation");
    return map_nodes[node][operation];
}

const EPath &OperationsMap::get_edges_path(uint32_t node, uint32_t operation) const {
    ASSERT(node < map_edges.size(), "invalid node");
    ASSERT(operation < map_edges[node].size(), "invalid operation");
    return map_edges[node][operation];
}

const EPath &OperationsMap::get_poses_path(uint32_t node, uint32_t operation) const {
    ASSERT(node < map_poses.size(), "invalid node");
    ASSERT(operation < map_poses[node].size(), "invalid operation");
    return map_poses[node][operation];
}

OperationsMap &get_omap() {
    static OperationsMap omap;
    return omap;
}
