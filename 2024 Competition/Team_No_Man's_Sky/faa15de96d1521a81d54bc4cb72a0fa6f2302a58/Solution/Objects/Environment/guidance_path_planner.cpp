#include <Objects/Environment/guidance_path_planner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>

#include <set>
#include <thread>
#include <unordered_set>

void GuidancePathPlanner::build(uint32_t r) {
    const uint32_t source = get_robots_handler().get_robot(r).node;
    const uint32_t target = get_robots_handler().get_robot(r).target;

    if (!target) {
        return;
    }

    constexpr uint32_t HM_MULT = 1;

    // (metric, node, depth, prev)
    std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>,
                        std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>>, std::greater<>>
            heap;

    heap.emplace(get_hm().get(source, target) * HM_MULT, source, 1, -1);

    // parent[to] = from
    std::unordered_map<uint32_t, uint32_t> parent;

    std::unordered_set<uint32_t> visited;

    while (!heap.empty()) {
        auto [metric, node, depth, prev] = heap.top();
        heap.pop();

        ASSERT(node, "invalid node");

        if (visited.count(node)) {
            continue;
        }
        visited.insert(node);

        parent[node] = prev;

        if (get_graph().get_pos(node).get_pos() == target || depth >= GPP_DEPTH) {
            auto &path = guidance_paths[r];
            path.clear();
            do {
                path.push_back(node);
                node = parent[node];
            } while (node != -1);
            ASSERT(path.size() <= GPP_DEPTH, "kek: " + std::to_string(path.size()) + "!=" + std::to_string(GPP_DEPTH));
            std::reverse(path.begin(), path.end());

            for (uint32_t node: path) {
                uint32_t pos = get_graph().get_pos(node).get_pos();
                weight[pos]++;
            }

            while (path.size() < GPP_DEPTH) {
                uint32_t x = path.back();
                path.push_back(x);
            }
            ASSERT(path.size() == GPP_DEPTH, "invalid path: " + std::to_string(path.size()) + "!=" + std::to_string(GPP_DEPTH));
            return;
        }

        metric -= get_hm().get(node, target) * HM_MULT;

        // F, C, R
        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);

            if (to && !visited.count(to)) {
                uint32_t to_metric = metric + get_hm().get(node, target) * HM_MULT;
                // get_graph().get_weight(node, action)
                uint32_t to_pos = get_graph().get_pos(to).get_pos() * HM_MULT;
                to_metric += get_graph().get_weight(node, action);
                to_metric += weight[to_pos];
                /*if (action == 0 && pos_to_robot[to_pos] != -1) {
                    to_metric += 1;// penalty for the agent
                }*/
                heap.push({to_metric, to, depth + 1, node});
            }
        }
    }

    FAILED_ASSERT("unable to what?");
}

void GuidancePathPlanner::update(uint32_t timestep, TimePoint end_time) {
#ifdef ENABLE_GUIDANCE_PATH_PLANNER
    ++timestep;
    ETimer timer;
    const auto &robots = get_robots_handler().get_robots();

    guidance_paths.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        guidance_paths[r].clear();
        /*if (!guidance_paths[r].empty()) {
            guidance_paths[r].erase(guidance_paths[r].begin());
        }*/
    }

    weight.assign(get_map().get_size(), 0);
    /*pos_to_robot.assign(get_map().get_size(), -1);
    for (uint32_t r = 0; r < robots.size(); r++) {
        uint32_t pos = get_graph().get_pos(robots[r].node).get_pos();
        ASSERT(pos_to_robot[pos] == -1, "invalid pos to robot");
        pos_to_robot[pos] = r;
    }*/

    {
        for (uint32_t r = 0; r < robots.size(); r++) {
            build(r);
        }
        /*auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                build(r);
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr].join();
        }*/
    }

    PRINT(Printer() << "GuidancePathPlanner::update: " << timer << '\n';);

#endif
}

const std::vector<uint32_t> &GuidancePathPlanner::get(uint32_t r) const {
    return guidance_paths[r];
}

GuidancePathPlanner &get_gpp() {
    FAILED_ASSERT("bad work");
    static GuidancePathPlanner gpp;
    return gpp;
}
