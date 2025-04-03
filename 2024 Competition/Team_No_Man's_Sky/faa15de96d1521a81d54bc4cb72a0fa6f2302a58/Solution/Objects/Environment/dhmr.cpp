#include <Objects/Environment/dhmr.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/operations_map.hpp>
#include <Objects/Environment/robot_handler.hpp>

#include <set>
#include <thread>
#include <unordered_set>

DHMR::DHMR(const Graph &graph) : matrix(MAX_AGENTS_NUM, std::vector<uint32_t>(graph.get_nodes_size())),
                                 visited_sphere(MAX_AGENTS_NUM, std::vector<uint32_t>(graph.get_nodes_size())),
                                 visited(MAX_AGENTS_NUM, std::vector<uint32_t>(graph.get_nodes_size()))
//visited_matrix(MAX_AGENTS_NUM, std::vector<uint32_t>(graph.get_nodes_size()))
{
}

void DHMR::build(uint32_t r, uint32_t timestep) {
    //Printer() << "DHMR::build(" << r << ")\n"; std:cout.flush();

    constexpr uint32_t DEPTH = 10;

    const uint32_t target = get_robots_handler().get_robot(r).target;

    if (!target) {
        return;
    }

    // (depth, node)
    std::vector<std::tuple<uint32_t, uint32_t>> poses;

    //std::unordered_set<uint32_t> visited_sphere;

    // с помощью Дейкстры обойдем шар узлов графа радиуса DEPTH
    {
        // (metric, node, depth)
        std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>,
                            std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>>
                heap;

        heap.push({0, get_robots_handler().get_robot(r).node, 0});

        while (!heap.empty()) {
            auto [metric, node, depth] = heap.top();
            heap.pop();

            ASSERT(node, "invalid node");

            if (visited_sphere[r][node] == timestep) {
                continue;
            }
            visited_sphere[r][node] = timestep;

            poses.emplace_back(depth, node);

            if (depth >= DEPTH) {
                ASSERT(depth == DEPTH, "invalid depth");
                continue;
            }

            // F, C, R
            for (uint32_t action = 0; action < 3; action++) {
                uint32_t to = get_graph().get_to_node(node, action);

                if (to && visited_sphere[r][to] != timestep) {
                    heap.push({metric + get_graph().get_weight(node, action), to, depth + (action == 0)});
                }
            }
        }

        //std::sort(poses.begin(), poses.end(), std::greater<>());
    }

    // (metric, node)
    std::priority_queue<std::tuple<uint32_t, uint32_t>,
                        std::vector<std::tuple<uint32_t, uint32_t>>, std::greater<>>
            heap;

    auto target_is_in_sphere = [&]() {
        for (auto [depth, node]: poses) {
            if (get_graph().get_pos(node).get_pos() == target) {
                return true;
            }
        }
        return false;
    };

    bool is_in_sphere = false;
    if (target_is_in_sphere()) {
        is_in_sphere = true;
        // если таргет уже в сфере, то просто его же и добавим
        for (auto [depth, node]: poses) {
            if (get_graph().get_pos(node).get_pos() == target) {
                heap.push({0, node});
            }
        }
    } else {
        // иначе добавим всех, кто на максимальной глубине
        for (auto [depth, node]: poses) {
            ASSERT(depth <= DEPTH, "invalid depth");
            if (depth == DEPTH) {
                heap.push({get_hm().get(node, target), node});
            }
        }
    }
    /*for (auto [depth, node]: poses) {
        ASSERT(depth <= DEPTH, "invalid depth");
        if (depth == DEPTH) {
            heap.push({get_hm().get(node, target), node});
        }
    }*/

    ASSERT(!heap.empty(), "heap is empty");

    //std::unordered_set<uint32_t> visited;

    auto simulate_dijkstra = [&]() {
        while (!heap.empty()) {
            auto [metric, node] = heap.top();
            heap.pop();

            ASSERT(visited_sphere[r][node] == timestep, "outside the sphere");

            if (visited[r][node] == timestep) {
                continue;
            }
            visited[r][node] = timestep;
            matrix[r][node] = metric;

            /*uint32_t d = get_hm().get(node, target);
            if (metric != d) {
                //Printer() << "bad: " << metric << " != " << d << ", is_in_sphere: " << is_in_sphere << '\n';
                //Position target_pos(target, 0);
                //ASSERT(metric == d, "invalid metric");
            }*/

            // F, C, R
            for (uint32_t action = 0; action < 3; action++) {
                uint32_t to = 0;
                //get_graph().get_to_node(node, action);
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

                if (!to) {
                    continue;
                }

                ASSERT(node == get_graph().get_to_node(to, action), "invalid to");

                if (to && visited_sphere[r][to] == timestep && visited[r][to] != timestep) {
                    uint32_t to_metric = metric;
                    to_metric += get_graph().get_weight(to, action);// edge weight

                    if (action == 0) {
                        uint32_t to_pos = get_graph().get_pos(to).get_pos();
                        uint32_t to_r = pos_to_robot[to_pos];

                        // другой робот
                        if (to_r != -1 && to_r != r) {
                            uint32_t our_dir = get_graph().get_pos(to).get_dir();
                            uint32_t him_dir = get_graph().get_pos(get_robots_handler().get_robot(to_r).node).get_dir();

                            if (our_dir != him_dir) {
                                // to_metric += 0; // 70129
                                // to_metric += 1; // 70529
                                // to_metric += 3; // 71332
                                to_metric += 5;// 71427
                            }
                        }
                    }

                    heap.push({to_metric, to});
                }
            }
        }
    };

    simulate_dijkstra();

    /*if (visited.size() != visited_sphere.size()) {
        for (auto [depth, node]: poses) {
            heap.push({get_hm().get(node, target), node});
        }
        simulate_dijkstra();
    }*/

    /*if (visited.size() != visited_sphere.size()) {
        Printer() << "failed to build sphere:\n";
        int cnt = 0;
        std::cout << "{";
        for (uint32_t node: visited_sphere) {
            if (!visited.count(node)) {
                cnt++;
                Printer() << node << ", ";
            }
        }
        std::cout << "}" << std::endl;
        std::cout << "cnt: " << cnt << std::endl;
    }*/
    /*ASSERT(visited.size() == visited_sphere.size(),
           "failed to build sphere: " + std::to_string(visited.size()) + " != " +
           std::to_string(visited_sphere.size()));*/
}

void DHMR::update(uint32_t timestep, TimePoint end_time) {
#ifdef ENABLE_DHMR
    ++timestep;
    ETimer timer;
    const auto &robots = get_robots_handler().get_robots();

    pos_to_robot.assign(get_map().get_size(), -1);
    for (uint32_t r = 0; r < robots.size(); r++) {
        uint32_t pos = get_graph().get_pos(robots[r].node).get_pos();
        ASSERT(pos_to_robot[pos] == -1, "invalid pos to robot");
        pos_to_robot[pos] = r;
    }

    {
        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                build(r, timestep);
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr].join();
        }
    }

    PRINT(Printer() << "DHMR::update: " << timer << '\n';);

#endif
}

uint32_t DHMR::get(uint32_t r, uint32_t node) const {
    ASSERT(r < matrix.size(), "invalid robot");
    ASSERT(matrix[r].size() == get_graph().get_nodes_size(), "invalid matrix");
    ASSERT(0 <= node && node < matrix[r].size(), "invalid desired");
    return matrix[r][node];
}

DHMR &get_dhmr() {
    static DHMR dhmr;
    return dhmr;
}
