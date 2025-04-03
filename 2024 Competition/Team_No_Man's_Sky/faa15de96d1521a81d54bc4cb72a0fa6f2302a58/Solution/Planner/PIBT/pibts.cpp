#include <Planner/PIBT/pibts.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <settings.hpp>

#include <thread>

//#define PRINT_RECURSIVE

bool PIBTS::consider() {
    // оптимизация: если мы собираемся ухудшать скор, который лучше чем best, то сохранить best
    // this can be really slow
    //if (cur_score > best_score + 1e-6) {
    //    best_score = cur_score;
    //    best_desires = desires;
    //}

    return old_score - 1e-6 <= cur_score
// old_score > cur_score
#ifdef ENABLE_PIBTS_ANNEALING
           || rnd.get_d() < std::exp(-((old_score - cur_score) / old_score) / temp)
#endif
            ;
}

bool PIBTS::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;
}

bool PIBTS::is_free_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desires[r]), "invalid path");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_pos[poses_path[depth]][depth] != -1) {
            ASSERT(used_pos[poses_path[depth]][depth] != r, "invalid used_node");
            return false;
        }
    }
    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_edge[edges_path[depth]][depth] != -1) {
            ASSERT(used_edge[edges_path[depth]][depth] != r, "invalid used_edge");
            return false;
        }
    }
    return true;
}

EPath PIBTS::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    return get_omap().get_nodes_path(robots[r].node, desired);
}

uint32_t PIBTS::get_used(uint32_t r) const {
    uint32_t answer = -1;

    auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        if (used_pos[to_pos][depth] != -1) {
            if (answer == -1) {
                answer = used_pos[to_pos][depth];
            } else if (answer != used_pos[to_pos][depth]) {
                return -2;
            }
        }
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (used_edge[to_edge][depth] != -1) {
            if (answer == -1) {
                answer = used_edge[to_edge][depth];
            } else if (answer != used_edge[to_edge][depth]) {
                return -2;
            }
        }
    }
    return answer;
}

std::vector<uint32_t> PIBTS::get_multi_used(uint32_t r) const {
    std::vector<uint32_t> answer;

    auto add = [&](uint32_t r) {
        if (std::find(answer.begin(), answer.end(), r) == answer.end()) {
            answer.push_back(r);
        }
    };

    auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        if (used_pos[to_pos][depth] != -1) {
            add(used_pos[to_pos][depth]);
        }
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (used_edge[to_edge][depth] != -1) {
            add(used_edge[to_edge][depth]);
        }
    }
    return answer;
}

int64_t PIBTS::get_smart_dist_IMPL(uint32_t r, uint32_t desired) const {
    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);

    const uint32_t target = robots[r].target;

    int64_t dist =
#ifdef ENABLE_DHMR
            get_dhmr().get(r, path.back());
#else
            get_hm().get(path.back(), target);
#endif

    if (op.back() == Action::W) {
        uint32_t node = path[path.size() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int64_t>(
#ifdef ENABLE_DHMR
                                          get_dhmr().get(r, to)
#else
                                          get_hm().get(to, target)
#endif
                                                  ));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int64_t>(
#ifdef ENABLE_DHMR
                                          get_dhmr().get(r, to)
#else
                                          get_hm().get(to, target)
#endif
                                                  ));
        }

        if (op[op.size() - 2] == Action::W) {
            uint32_t to = node;
            to = get_graph().get_to_node(to, 1);
            to = get_graph().get_to_node(to, 1);
            dist = std::min(dist, static_cast<int64_t>(
#ifdef ENABLE_DHMR
                                          get_dhmr().get(r, to)
#else
                                          get_hm().get(to, target)
#endif
                                                  ));
        }
    }

    // [KEK]: если мы проходим по таргету, то мы должны это делать как можно раньше. 7200 -> 7297
    for (uint32_t d = 0; d < DEPTH; d++) {
        if (get_graph().get_pos(path[d]).get_pos() == target) {
            dist = d;
            dist = -dist;
        }
    }

    dist = dist * 50 - desired;

    // стой и никому не мешай
    if (robots[r].is_disable()) {
        dist = desired;
    }
    return dist;
}

int64_t PIBTS::get_smart_dist(uint32_t r, uint32_t desired) const {
    return smart_dist_dp[r][desired];
}

void PIBTS::update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const {
    int32_t diff = get_smart_dist(r, 0) - get_smart_dist(r, desired);
    cur_score += sign * diff * robot_power[r];
}

void PIBTS::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == -1, "already used");
        used_pos[to_pos][depth] = r;
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == -1, "already used");
            used_edge[to_edge][depth] = r;
        }
    }

    update_score(r, desires[r], cur_score, +1);
}

void PIBTS::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == r, "invalid node");
        used_pos[to_pos][depth] = -1;
    }
    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == r, "invalid edge");
            used_edge[to_edge][depth] = -1;
        }
    }
    update_score(r, desires[r], cur_score, -1);
}

uint32_t PIBTS::try_echo_slam(std::vector<uint32_t> &rids, uint32_t &counter, uint32_t depth) {
    if (rids.empty()) {
        if (consider()) {
            return 1;// accepted
        } else {
            return 2;// not accepted
        }
    }

    if (counter > 1000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    uint32_t r = rids[0];
    rids.erase(rids.begin());

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    FAILED_ASSERT("robot_desires[r][0]");
    for (uint32_t desired: robot_desires[r][0]) {
        desires[r] = desired;
        std::vector<uint32_t> to_rids = get_multi_used(r);
        if (to_rids.empty()) {
            add_path(r);
            uint32_t res = try_echo_slam(rids, ++counter, depth + 1);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                desires[r] = old_desired;
                return res;
            }
            remove_path(r);
        } else {
            bool ok = true;
            for (uint32_t to_r: to_rids) {
                ASSERT(to_r != r, "invalid to_r");
                ASSERT(std::find(rids.begin(), rids.end(), to_r) == rids.end(), "already in rids");
                if (visited[to_r] == visited_counter) {
                    ok = false;
                }
            }

            if (!ok) {
                continue;
            }

            if (rids.size() + to_rids.size() > 2) {
                continue;
            }

            if (rnd.get_d() < 0.2) {
                continue;
            }

            for (uint32_t to_r: to_rids) {
                rids.push_back(to_r);
                remove_path(to_r);
            }
            add_path(r);

            uint32_t res = try_echo_slam(rids, ++counter, depth + 1);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                for (uint32_t to_r: to_rids) {
                    add_path(to_r);
                }
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            for (uint32_t to_r: to_rids) {
                add_path(to_r);
            }

            while (!to_rids.empty()) {
                to_rids.pop_back();
                rids.pop_back();
            }
        }
    }

    desires[r] = old_desired;
    visited[r] = 0;
    rids.insert(rids.begin(), r);
    return 0;
}

bool PIBTS::try_echo_slam(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    std::vector<uint32_t> rids = {r};
    uint32_t res = try_echo_slam(rids, counter, 0);
#ifdef PRINT_RECURSIVE
    if (res == 0) {
        log << "invalid  try_echo_slam, count: " << counter << "\n";
    }
#endif
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
#ifdef PRINT_RECURSIVE
    if (std::abs(old_score - cur_score) > 1e-6) {
        log << '\n';
    }
#endif
    return true;
}

uint32_t PIBTS::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter > 1000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r][current_depth]) {
        ASSERT(get_operation_depth(desired) <= current_depth, "invalid robot_desires");
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
#ifdef PRINT_RECURSIVE
                if (std::abs(old_score - cur_score) > 1e-6) {
                    log << "accept try_build: " << old_score << "->" << cur_score << ", depth: " << depth
                        << ", count: " << counter << ", temp: " << temp << ", perc: "
                        << std::exp(-((old_score - cur_score) / old_score) / temp) * 100;
                    //log << " (" << r << ", " << get_operations()[desired] << ", "
                    //          << (get_smart_dist(r, 0) - get_smart_dist(r, desired)) << ")";
                }
#endif
                return 1;// accepted
            } else {
#ifdef PRINT_RECURSIVE
                log << "failed try_build: " << old_score << "->" << cur_score << ", depth: " << depth
                    << ", count: " << counter << ", temp: " << temp << ", perc: "
                    << std::exp(-((old_score - cur_score) / old_score) / temp) * 100 << "\n";
#endif
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2 && visited[to_r] != visited_counter) {
            if (rnd.get_d() < 0.2) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(visited[to_r] != visited_counter, "already visited");

            remove_path(to_r);
            add_path(r);

            uint32_t res = try_build(to_r, ++counter, depth + 1);
            if (res == 1) {
#ifdef PRINT_RECURSIVE
                /*if(std::abs(old_score - cur_score) > 1e-6) {
                    log << " (" << r << ", " << get_operations()[desired] << ", "
                              << (get_smart_dist(r, 0) - get_smart_dist(r, desired)) << ")";
                }*/
#endif
                return res;
            } else if (res == 2) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    visited[r] = 0;
    return 0;
}

bool PIBTS::try_build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = try_build(r, counter, 0);
#ifdef PRINT_RECURSIVE
    if (res == 0) {
        log << "invalid  try_build, count: " << counter << "\n";
    }
#endif
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
#ifdef PRINT_RECURSIVE
    if (std::abs(old_score - cur_score) > 1e-6) {
        log << '\n';
    }
#endif
    return true;
}

/*uint32_t
PIBTS::try_rebuild_neighbors(uint32_t id, const std::vector<uint32_t> &rids, uint32_t &counter, uint32_t depth) {
    if (id == rids.size()) {
        // все поставили
        if (consider()) {
            return 1;// accepted
        } else {
            return 2;// not accepted
        }
    }

    if (counter == -1 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    uint32_t r = rids[id];
    // хотим поставить путь для r
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);

            uint32_t res = try_rebuild_neighbors(id + 1, rids, ++counter, depth + 1);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                desires[r] = old_desired;
                return res;
            } else {
                remove_path(r);
            }
        }
    }
    desires[r] = old_desired;
    return 0;
}

bool PIBTS::try_rebuild_neighbors(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;

    auto rids = neighbors[r];
    rids.push_back(r);

    std::shuffle(rids.begin(), rids.end(), rnd.generator);
    uint32_t size = rnd.get(1, 4);
    if (rids.size() > size) {
        rids.resize(size);
    }

    // удалим им все пути
    for (uint32_t r: rids) {
        remove_path(r);
    }

    uint32_t counter = 0;
    uint32_t res = try_rebuild_neighbors(0, rids, counter, 0);
    if (res == 1) {
        return true;
    }
    for (uint32_t r: rids) {
        add_path(r);
    }
    //ASSERT(std::abs(old_score - cur_score) < 1e-6,
    //       "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
    //       std::to_string(std::abs(old_score - cur_score)));
    return false;
}*/

uint32_t PIBTS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 ||//
        //counter > 30'000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r][current_depth]) {
        ASSERT(get_operation_depth(desired) <= current_depth, "invalid robot_desires");
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
#ifdef PRINT_RECURSIVE
                if (std::abs(old_score - cur_score) > 1e-6) {
                    log << "accept build: " << old_score << "->" << cur_score << ", depth: " << depth
                        << ", count: "
                        << counter;
                    //log << " (" << r << ", " << get_operations()[desired] << ", "
                    //          << (get_smart_dist(r, 0) - get_smart_dist(r, desired)) << ")";
                }
#endif
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2// && visited[to_r] != visited_counter
        ) {
            if (counter > 3000 && depth >= 6) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            //ASSERT(visited[to_r] != visited_counter, "already visited");

            // TODO: у to_r может быть приоритет ниже чем у r
            // но to_r уже построен, потому что был какой-то x, который имел приоритет больше чем r
            // и этот x построил to_r
            if (desires[to_r] != 0
#ifdef ENABLE_PIBTS_TRICK
                && rnd.get_d() < 0.8
#endif
            ) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            uint32_t res = build(to_r, depth + 1, ++counter);
            if (res == 1) {
#ifdef PRINT_RECURSIVE
                if (std::abs(old_score - cur_score) > 1e-6) {
                    //log << " (" << r << ", " << get_operations()[desired] << ", "
                    //          << (get_smart_dist(r, 0) - get_smart_dist(r, desired)) << ")";
                }
#endif
                return res;
            } else if (res == 2) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    visited[r] = 0;
    desires[r] = old_desired;
    return 0;
}

bool PIBTS::build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = build(r, 0, counter);
    if (res == 0 || res == 2) {
        add_path(r);
        //ASSERT(std::abs(old_score - cur_score) < 1e-6,
        //       "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
        //       std::to_string(std::abs(old_score - cur_score)));
        return false;
    }
#ifdef PRINT_RECURSIVE
    if (std::abs(old_score - cur_score) > 1e-6) {
        log << '\n';
    }
#endif
    return true;
}

void PIBTS::reset(uint32_t r, std::vector<uint32_t> &destroyed) {
    if (visited[r] == visited_counter) {
        return;
    }
    visited[r] = visited_counter;
    remove_path(r);
    desires[r] = 0;
    destroyed.push_back(r);

    std::vector<uint32_t> rids = get_multi_used(r);
    for (uint32_t to_r: rids) {
        reset(to_r, destroyed);
    }

    add_path(r);
}

PIBTS::PIBTS(const std::vector<Robot> &robots, TimePoint end_time)
    : robots(robots), end_time(end_time) {

    ETimer timer;

    visited.resize(robots.size());
    desires.resize(robots.size());
    best_desires.resize(robots.size());

    {
        ETimer timer;
        std::array<uint32_t, DEPTH> value{};
        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            value[depth] = -1;
        }
        used_pos.resize(get_graph().get_zipes_size(), value);
        used_edge.resize(get_graph().get_edges_size(), value);

        PRINT(Printer() << "init used: " << timer << '\n';);
    }

    // build order and power
    {
        ETimer timer;
        order.resize(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return robots[lhs].priority < robots[rhs].priority;
        });

        std::vector<int32_t> weight(robots.size());

        /*uint32_t prev = 0;
        uint32_t w = 0;
        for (uint32_t r: order) {
            if (prev != robots[r].priority) {
                w++;
            }
            weight[r] = w;
            prev = robots[r].priority;
        }
        int32_t max_weight = w + 1;*/

        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
        int32_t max_weight = robots.size() + 1;

        robot_power.resize(robots.size());
        const double workload = robots.size() * 1.0 / get_map().get_count_free();
        for (uint32_t r = 0; r < robots.size(); r++) {
            double power = (max_weight - weight[r]) * 1.0 / max_weight;
            if (robots[r].is_disable()) {
                power = 0;
            }
            if (get_test_type() == TestType::GAME) {
                power = power * power;
            } else if (get_test_type() == TestType::RANDOM_4) {
                power = power * power;
            } else if (get_test_type() == TestType::RANDOM_5) {
                power = power * power;
            } else if (get_test_type() == TestType::WAREHOUSE) {
                power = 1;// std::sqrt(power)
            } else if (get_test_type() == TestType::SORTATION) {
                power = 1;// std::sqrt(power)
            }
            robot_power[r] = power;
        }

        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return std::tie(robot_power[lhs], lhs) > std::tie(robot_power[rhs], rhs);
        });

        PRINT(Printer() << "init order and power: " << timer << '\n';);
    }

    // init neighbors
    /*{
        ETimer timer;
        neighbors.resize(robots.size());

        // used_edge[edge][depth] = robot id
        std::vector<std::array<std::vector<uint32_t>, DEPTH>> used_edge(get_graph().get_edges_size());

        // used_pos[pos][depth] = robot id
        std::vector<std::array<std::vector<uint32_t>, DEPTH>> used_pos(get_graph().get_zipes_size());

        ETimer timer;
        for (uint32_t r = 0; r < robots.size(); r++) {
            for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                if (!validate_path(r, desired)) {
                    continue;
                }
                const uint32_t source = robots[r].node;

                auto add = [&](std::vector<uint32_t> &vec) {
                    uint32_t i = 0;
                    for (; i < vec.size() && r < vec[i]; i++) {}

                    if (i < vec.size() && vec[i] == r) {
                        // already contains
                    } else {
                        vec.insert(vec.begin() + i, r);
                    }
                };

                {
                    auto &poses_path = get_omap().get_poses_path(source, desired);
                    for (uint32_t depth = 0; depth < DEPTH; depth++) {
                        uint32_t to_pos = poses_path[depth];
                        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
                        add(used_pos[to_pos][depth]);
                    }
                }
                {
                    auto &edges_path = get_omap().get_edges_path(source, desired);
                    for (uint32_t depth = 0; depth < DEPTH; depth++) {
                        uint32_t to_edge = edges_path[depth];
                        ASSERT(to_edge < used_edge.size(), "invalid to_edge");
                        if (to_edge) {
                            add(used_edge[to_edge][depth]);
                        }
                    }
                }
            }
        }
        Printer() << "build used: " << timer << '\n';
        timer.reset();

        for (uint32_t edge = 1; edge < used_edge.size(); edge++) {
            for (uint32_t depth = 0; depth < DEPTH; depth++) {
                for (uint32_t r: used_edge[edge][depth]) {
                    for (uint32_t r2: used_edge[edge][depth]) {
                        if (r != r2 &&
                            std::find(neighbors[r].begin(), neighbors[r].end(), r2) == neighbors[r].end()) {
                            neighbors[r].push_back(r2);
                        }
                    }
                }
            }
        }
        std::cout << "build edges: " << timer << '\n';
        timer.reset();

        for (uint32_t pos = 1; pos < used_pos.size(); pos++) {
            for (uint32_t depth = 0; depth < DEPTH; depth++) {
                for (uint32_t r: used_pos[pos][depth]) {
                    for (uint32_t r2: used_pos[pos][depth]) {
                        if (r != r2 &&
                            std::find(neighbors[r].begin(), neighbors[r].end(), r2) == neighbors[r].end()) {
                            neighbors[r].push_back(r2);
                        }
                    }
                }
            }
        }
        std::cout << "build poses: " << timer << '\n';

    //warehouse_large_10000
    //build used: 15.2807ms
    //build edges: 3.43853ms
    //build poses: 5.34468ms
    //init neighbors: 30.1128ms
    Printer() << "init neighbors: " << timer << '\n';
    }*/

    // init smart_dist_dp
    {
        ETimer timer;
        smart_dist_dp.resize(robots.size(), std::vector<int64_t>(get_operations().size()));

        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    smart_dist_dp[r][desired] += get_smart_dist_IMPL(r, desired);
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
        /*for (uint32_t r = 0; r < robots.size(); r++) {
            for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                if (!validate_path(r, desired)) {
                    continue;
                }
                smart_dist_dp[r][desired] += get_smart_dist_IMPL(r, desired);
            }
        }*/
        PRINT(Printer() << "init smart_dist_dp: " << timer << '\n';);
    }

    {
        ETimer timer;
        for (uint32_t r = 0; r < robots.size(); r++) {
            desires[r] = 0;
            add_path(r);
        }
        PRINT(Printer() << "add paths: " << timer << '\n';);
    }

    // build robot_desires
    {
        ETimer timer;

        robot_desires.resize(robots.size());

        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                // (priority, desired)
                std::vector<std::pair<int64_t, uint32_t>> steps;
                for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    int64_t priority = get_smart_dist(r, desired);
                    steps.emplace_back(priority, desired);
                }
                std::sort(steps.begin(), steps.end());
                robot_desires[r].resize(DEPTH + 1);
                for (auto [priority, desired]: steps) {
                    for (uint32_t d = std::max(static_cast<uint32_t>(3), get_operation_depth(desired)); d <= DEPTH; d++) {
                        robot_desires[r][d].push_back(desired);
                    }
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

        /*for (uint32_t r = 0; r < robots.size(); r++) {
            // (priority, desired)
            std::vector<std::pair<int64_t, uint32_t>> steps;
            for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
                if (!validate_path(r, desired)) {
                    continue;
                }
                int64_t priority = get_smart_dist(r, desired);
                steps.emplace_back(priority, desired);
            }
            std::stable_sort(steps.begin(), steps.end());
            for (auto [priority, desired]: steps) {
                robot_desires[r].push_back(desired);
            }
        }*/
        PRINT(Printer() << "init robot_desires: " << timer << '\n';);
    }

    PRINT(Printer() << "init PIBTS: " << timer << '\n';);
}

void PIBTS::solve(uint64_t seed) {
    rnd = Randomizer(seed);

    if (get_test_type() == TestType::RANDOM_5) {
        current_depth = 3;
    } else {
        current_depth = rnd.get(3, 5);
    }
    temp = 0;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0) {
            continue;
        }
        build(r);
    }

    best_desires = desires;
    best_score = cur_score;

    temp = 0.001;

    if constexpr (true) {
        for (step = 0; get_now() < end_time && step < PIBTS_STEPS; step++) {
            if (step && step % 128 == 0 && get_test_type() != TestType::RANDOM_5) {
                current_depth = rnd.get(3, 5);
            }
            uint32_t r = rnd.get(0, robots.size() - 1);
            try_build(r);
            temp *= 0.999;
        }
        if (best_score + 1e-6 < cur_score) {
            //Printer() << "improve: " << best_score << " -> " << cur_score << '\n';
            best_desires = desires;
            best_score = cur_score;
        }
    } else {
        FAILED_ASSERT("deprecated");
        while (get_now() < end_time) {
            //Printer() << "earthquake: " << cur_score << " -> ";
            int N = rnd.get(10, 20);
            ++visited_counter;
            std::vector<uint32_t> destroyed;
            while (N--) {
                uint32_t r = rnd.get(0, robots.size() - 1);
                reset(r, destroyed);
            }
            std::shuffle(destroyed.begin(), destroyed.end(), rnd.generator);
            //Printer() << cur_score << " (" << destroyed.size() << ") -> ";
            for (uint32_t step = 0; step < PIBTS_STEPS && get_now() < end_time; step++) {
                uint32_t r = rnd.get_d() < 0.5 ? rnd.get(destroyed) : rnd.get(0, robots.size() - 1);
                try_build(r);
                temp *= 0.999;
                this->step++;
            }
            //Printer() << cur_score << "\n";
            if (best_score + 1e-6 < cur_score) {
                //Printer() << "improve: " << best_score << " -> " << cur_score << '\n';
                best_desires = desires;
                best_score = cur_score;
            }
        }
    }
}

std::vector<Action> PIBTS::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[best_desires[r]][0];
        if (best_desires[r] == 0) {
            auto dist = std::min({get_dhm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target),
                                  get_dhm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target),
                                  get_dhm().get(get_graph().get_to_node(robots[r].node, 3), robots[r].target)});
            if (dist == get_dhm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target)) {
                answer[r] = Action::CR;
            } else if (dist == get_dhm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target)) {
                answer[r] = Action::CCR;
            } else {
                answer[r] = Action::W;
            }
        }
    }
    return answer;
}

std::vector<uint32_t> PIBTS::get_desires() const {
    return best_desires;
}

std::vector<int64_t> PIBTS::get_changes() const {
    std::vector<int64_t> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_smart_dist(r, 0) - get_smart_dist(r, best_desires[r]);
    }
    return answer;
}

double PIBTS::get_score() const {
    return best_score;
}
