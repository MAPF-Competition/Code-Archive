#include <Objects/Environment/heuristic_matrix.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Containers/linear_heap.hpp>
#include <settings.hpp>

#include <thread>

void HeuristicMatrix::build(uint32_t source, const Graph &graph) {
    auto &dists = matrix[source];
    dists.assign(graph.get_nodes_size(), -1);

    // (dist, node)
    std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, std::greater<>> heap;

    std::vector<bool> visited(graph.get_nodes_size());

    for (uint32_t dir = 0; dir < 4; dir++) {
        heap.push({0, graph.get_node(Position(source, dir))});
    }

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        if (visited[node]) {
            continue;
        }
        visited[node] = true;
        dists[node] = dist;

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = 0;
            if (action == 0) {
                to = get_graph().get_to_node(node, 1);// R
                to = get_graph().get_to_node(to, 1);  // R
                to = get_graph().get_to_node(to, 0);  // F
                if (to) {
                    to = get_graph().get_to_node(to, 1);// R
                    to = get_graph().get_to_node(to, 1);// R
                }
            } else if (action == 1) {
                to = get_graph().get_to_node(node, 2);
            } else if (action == 2) {
                to = get_graph().get_to_node(node, 1);
            }

            if (to && !visited[to]) {
                uint64_t to_dist = dist + graph.get_weight(to, action);
                ASSERT(to_dist == static_cast<uint16_t>(to_dist), "dist overflow");
                if (dists[to] > to_dist) {
                    dists[to] = to_dist;
                    heap.push({to_dist, to});
                }
            }
        }
    }
}

HeuristicMatrix::HeuristicMatrix(const Graph &graph) {
    ETimer timer;
    matrix.resize(get_map().get_size());

    auto do_work = [&](uint32_t thr) {
        for (uint32_t pos = thr + 1; pos < matrix.size(); pos += THREADS) {
            if (Position(pos, 0).is_valid()) {
                build(pos, graph);
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    PRINT(Printer() << "HM init: " << timer << '\n';);
}

uint32_t HeuristicMatrix::get(uint32_t source, uint32_t target) const {
    if (!target) {
        return INVALID_DIST;
    }
    ASSERT(0 < target && target < get_map().get_size(), "invalid dest");
    /*ASSERT(Position(target, 0).is_valid(), "invalid");
    ASSERT(Position(target, 1).is_valid(), "invalid");
    ASSERT(Position(target, 2).is_valid(), "invalid");
    ASSERT(Position(target, 3).is_valid(), "invalid");*/

    ASSERT(target < matrix.size(), "invalid target");
    ASSERT(source < matrix[target].size(), "invalid source");

    return matrix[target][source];
}

HeuristicMatrix &get_hm() {
    static HeuristicMatrix hm;
    return hm;
}
