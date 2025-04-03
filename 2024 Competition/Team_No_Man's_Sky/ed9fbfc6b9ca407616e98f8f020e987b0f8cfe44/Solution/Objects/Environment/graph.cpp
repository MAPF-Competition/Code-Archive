#include <Objects/Environment/graph.hpp>

#include <Objects/Basic/assert.hpp>

Graph::Graph(const Map &map, const GraphGuidance &gg) {
    pos_to_zip.resize(map.get_size());
    pos_to_node.resize(map.get_size());
    node_to_pos.resize(1);
    zip_to_pos.resize(1);
    for (uint32_t pos = 1; pos < map.get_size(); pos++) {
        if (!map.is_free(pos)) {
            continue;
        }
        pos_to_zip[pos] = zip_to_pos.size();
        zip_to_pos.push_back(pos);
        for (uint32_t dir = 0; dir < 4; dir++) {
            Position p(pos, dir);
            ASSERT(p.is_valid(), "p is invalid");
            pos_to_node[pos][dir] = node_to_pos.size();
            node_to_pos.push_back(p);
        }
    }

    to_node.resize(node_to_pos.size());
    to_edge.resize(node_to_pos.size());
    weight.resize(node_to_pos.size());

    std::map<std::pair<uint32_t, uint32_t>, uint32_t> edges;
    for (uint32_t node = 1; node < node_to_pos.size(); node++) {
        for (uint32_t action = 0; action < 4; action++) {
            Position p = node_to_pos[node];
            Position to = p.simulate_action(static_cast<Action>(action));
            if (!to.is_valid()) {
                continue;
            }

            to_node[node][action] = get_node(to);
            weight[node][action] = gg.get(p.get_pos(), p.get_dir(), action);

            uint32_t a = p.get_pos();
            uint32_t b = to.get_pos();

            if (a == b) {
                continue;
            }

            if (a > b) {
                std::swap(a, b);
            }
            if (!edges.count({a, b})) {
                edges[{a, b}] = edges.size() + 1;
            }

            to_edge[node][action] = edges[{a, b}];
        }
    }
    edges_size = edges.size() + 1;
}

uint32_t Graph::get_zipes_size() const {
    return zip_to_pos.size();
}

uint32_t Graph::get_nodes_size() const {
    return node_to_pos.size();
}

uint32_t Graph::get_edges_size() const {
    return edges_size;
}

uint32_t Graph::get_zip(uint32_t pos) const {
    return pos_to_zip[pos];
}

uint32_t Graph::get_pos_from_zip(uint32_t zip) const {
    return zip_to_pos[zip];
}

Position Graph::get_pos(uint32_t node) const {
    ASSERT(0 < node && node < node_to_pos.size(), "invalid node: " + std::to_string(node));
    return node_to_pos[node];
}

uint32_t Graph::get_node(const Position &pos) const {
    ASSERT(pos.is_valid(), "invalid position");
    ASSERT(0 <= pos.get_pos() && pos.get_pos() < pos_to_node.size(), "invalid pos: " + std::to_string(pos.get_x()) + " " + std::to_string(pos.get_y()) + " " + std::to_string(pos.get_dir()));
    ASSERT(0 <= pos.get_dir() && pos.get_dir() < 4, "invalid dir");
    return pos_to_node[pos.get_pos()][pos.get_dir()];
}

uint32_t Graph::get_node(uint32_t pos, uint32_t dir) const {
    ASSERT(0 <= pos && pos < pos_to_node.size(), "invalid pos");
    ASSERT(0 <= dir && dir < 4, "invalid dir");
    return pos_to_node[pos][dir];
}

uint32_t Graph::get_to_node(uint32_t node, uint32_t action) const {
    ASSERT(0 < node && node < to_node.size(), "invalid node");
    ASSERT(action < 4, "invalid action");
    return to_node[node][action];
}

uint32_t Graph::get_to_edge(uint32_t node, uint32_t action) const {
    ASSERT(0 < node && node < to_edge.size(), "invalid node");
    ASSERT(action < 4, "invalid action");
    return to_edge[node][action];
}

uint32_t Graph::get_weight(uint32_t node, uint32_t action) const {
    ASSERT(0 < node && node < to_edge.size(), "invalid node");
    ASSERT(action < 4, "invalid action");
    return weight[node][action];
}

Graph &get_graph() {
    static Graph graph;
    return graph;
}
