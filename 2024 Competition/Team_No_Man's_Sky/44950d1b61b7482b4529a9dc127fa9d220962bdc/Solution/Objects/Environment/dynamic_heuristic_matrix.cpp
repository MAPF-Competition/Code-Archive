#include <Objects/Environment/dynamic_heuristic_matrix.hpp>

#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <settings.hpp>

#include <atomic>
#include <thread>

void DynamicHeuristicMatrix::rebuild(uint32_t source, uint32_t timestep) {
    ASSERT(source < matrix.size(), "invalid source");
    auto &dists = matrix[source];
    dists.assign(get_graph().get_nodes_size(), -1);

    // (dist, node)
    std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, std::greater<>> heap;

    std::vector<bool> visited(dists.size());

    for (uint32_t dir = 0; dir < 4; dir++) {
        ASSERT(Position(source, dir).is_valid(), "invalid");
        uint32_t node = get_graph().get_node(Position(source, dir));
        heap.push({0, node});
        dists[node] = 0;
    }

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        ASSERT(0 < node && node < get_graph().get_nodes_size(), "invalid node");
        ASSERT(node < visited.size(), "invalid node");

        if (visited[node]) {
            continue;
        }
        visited[node] = true;
        ASSERT(dists[node] == dist, "invalid dist");

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && !visited[to]) {
                uint32_t to_dist = dist;
                to_dist += get_graph().get_weight(node, action) + weights[node][action];
                if (dists[to] > to_dist) {
                    dists[to] = to_dist;
                    heap.push({to_dist, to});
                }
            }
        }
    }

    // queue[dist - queue_dist] = { node }
    /*std::deque<std::vector<uint32_t>> queue(1);
    uint32_t queue_dist = 0;

    std::vector<bool> visited(dists.size());

    auto update_queue = [&]() {
        while (!queue.empty() && queue.front().empty()) {
            queue.pop_front();
            queue_dist++;
        }
    };

    auto push_queue = [&](uint32_t dist, uint32_t node) {
        ASSERT(queue_dist <= dist, "invalid dist");
        uint32_t index = dist - queue_dist;
        if (index >= queue.size()) {
            queue.resize(index + 1);
        }
        queue[index].push_back(node);
    };

    for (uint32_t dir = 0; dir < 4; dir++) {
        ASSERT(Position(source, dir).is_valid(), "invalid");
        uint32_t node = get_graph().get_node(Position(source, dir));
        queue[0].push_back(node);
        dists[node] = 0;
    }

    while (true) {
        update_queue();
        if (queue.empty()) {
            break;
        }
        ASSERT(!queue.front().empty(), "empty");
        uint32_t node = queue.front().back();
        queue.front().pop_back();

        ASSERT(0 < node && node < get_graph().get_nodes_size(), "invalid node");
        ASSERT(node < visited.size(), "invalid node");

        if (visited[node]) {
            continue;
        }
        visited[node] = true;
        ASSERT(dists[node] == queue_dist, "invalid dist");

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && !visited[to]) {
                uint32_t to_dist = queue_dist;
                to_dist += std::max(0,
                                    static_cast<int32_t>(get_graph().get_weight(node, action)) + weights[node][action]);
                if (dists[to] > to_dist) {
                    dists[to] = to_dist;
                    push_queue(to_dist, to);
                }
            }
        }
    }*/
}

DynamicHeuristicMatrix::DynamicHeuristicMatrix(const Map &map, const Graph &graph) {
    matrix.resize(map.get_size());
    timestep_updated.resize(map.get_size());
    weights.resize(graph.get_nodes_size());
}

void DynamicHeuristicMatrix::update(SharedEnvironment &env, TimePoint end_time) {
#ifdef ENABLE_DHM
    Timer timer;

    auto read_power = [&]() {
        std::ifstream input("Tmp/args" + std::to_string(get_unique_id()));
        int x;
        input >> x;
        return x;
    };

    static double workload = env.num_of_agents * 1.0 / get_map().get_count_free();
    static double power =
#ifdef ENABLE_GG_SOLVER
            read_power();
#else
            std::max(1.0, workload * 14 * 2 - 2);
#endif

    // set weights
    {
        //robot_paths.resize(get_robots_handler().size());

        for (auto &weight: weights) {
            for (uint32_t action = 0; action < 4; action++) {
                weight[action] = 0;
            }
        }

        const auto &robots = get_robots_handler().get_robots();
        for (uint32_t r = 0; r < robots.size(); r++) {
            const auto &robot = robots[r];
            ASSERT(0 < robot.node && robot.node < get_graph().get_nodes_size(), "invalid node");
            for (uint32_t dir = 0; dir < 4; dir++) {
                Position p(get_graph().get_pos(robot.node));
                p = Position(p.get_x(), p.get_y(), dir);
                p = p.move_forward();
                if (p.is_valid()) {
                    p = p.rotate().rotate();
                    uint32_t node = get_graph().get_node(p);
                    weights[node][0] += power;
                }
                weights[robot.node][dir] += power;
            }
            /*for (uint32_t action = 0; action < 4; action++) {
                uint32_t to = get_graph().get_to_node(robot.node, action);
                weights[robot.node][action] += power;
            }*/

            /*for (int32_t i = ROBOT_PATH_DEPTH - 1; i > 0; i--) {
                std::swap(robot_paths[r][i], robot_paths[r][i - 1]);
            }
            robot_paths[r][0] = robot.node;

            for (uint32_t i = 0; i + 1 < ROBOT_PATH_DEPTH; i++) {
                uint32_t from = robot_paths[r][i + 1];
                uint32_t to = robot_paths[r][i];
                if (!from || !to) {
                    break;
                }

                for (uint32_t action = 0; action < 4; action++) {
                    if (get_graph().get_to_node(from, action) == to) {
                        //weights[from][action] -= 1;
                    }
                }
            }*/
        }
    }

    // (score, target pos)
    /*std::vector<std::pair<double, uint32_t>> pool;
    for (auto &[t, task]: env.task_pool) {
        uint32_t target = task.get_next_loc() + 1;
        ASSERT(0 < target && target < get_map().get_size(), "invalid target");
        pool.emplace_back(timestep_updated[target], target);
    }
    std::sort(pool.begin(), pool.end());
    pool.erase(std::unique(pool.begin(), pool.end()), pool.end());*/

    std::vector<std::pair<double, uint32_t>> pool;
    {
        // mp[pos] = score
        std::unordered_map<uint32_t, double> mp;
        const auto &robots = get_robots_handler().get_robots();
        for (uint32_t r = 0; r < robots.size(); r++) {
            if (robots[r].target) {
                mp[robots[r].target]++;
            }
        }
        for (auto [pos, score]: mp) {
            score = (env.curr_timestep - static_cast<double>(timestep_updated[pos])) - score * 10;
            pool.emplace_back(score, pos);
        }
        std::sort(pool.begin(), pool.end());
    }

    std::atomic<uint32_t> total_rebuild = 0;

    auto do_work = [&](uint32_t thr) {
        for (uint32_t index = thr;
             index < pool.size() && get_now() < end_time && index < DHM_REBUILD_COUNT; index += THREADS) {
            auto [_, target] = pool[index];

            timestep_updated[target] = env.curr_timestep;
            rebuild(target, env.curr_timestep);
            ++total_rebuild;
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    PRINT(
            Printer() << "total rebuild: " << total_rebuild << '/' << pool.size() << '\n';
            Printer() << "rebuild time: " << timer << '\n';)

#endif
}

uint32_t DynamicHeuristicMatrix::get(uint32_t source, uint32_t target) const {
    if (!target) {
        return INVALID_DIST;
    }

#ifdef ENABLE_DHM
    ASSERT(target < matrix.size(), "invalid target");

    if (!matrix[target].empty()) {
        source = get_graph().get_to_node(get_graph().get_to_node(source, 1), 1);
        // get_graph().get_node(get_graph().get_pos(source).rotate().rotate());
        ASSERT(source < matrix[target].size(), "invalid source");
        ASSERT(matrix[target][source] != -1, "invalid matrix");
        return matrix[target][source];
    } else {
        return get_hm().get(source, target);
    }
#else
    return get_hm().get(source, target);
#endif
}

DynamicHeuristicMatrix &get_dhm() {
    static DynamicHeuristicMatrix dhm;
    return dhm;
}
