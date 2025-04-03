#include <Planner/PIBT/pmps.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <settings.hpp>

#include <thread>
#include <atomic>

using World = PMPS::World;

//#define PRINT_RECURSIVE

bool World::consider() {
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

bool World::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;
}

bool World::is_free_path(uint32_t r) const {
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

EPath World::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    return get_omap().get_nodes_path(robots[r].node, desired);
}

uint32_t World::get_used(uint32_t r) const {
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

int32_t World::get_smart_dist_IMPL(uint32_t r, uint32_t desired) const {
    int32_t dist = get_dhm().get(get_omap().get_nodes_path(robots[r].node, desired).back(), robots[r].target);

    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);
    if (op.back() == Action::W) {
        uint32_t node = path[path.size() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(to, robots[r].target)));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(to, robots[r].target)));
        }

        if (op[op.size() - 2] == Action::W) {
            node = get_graph().get_to_node(node, 1);
            node = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(node, robots[r].target)));
        }
    }

    static std::vector<int32_t> add_weights = {
            -20, //WWW
            15, //FFF
            13, //FFW
            11, //FWW
            5, //FRF
            5, //FCF
            2, //RFW
            2, //CFW
            4, //RFF
            4, //CFF
            1, //RRF
            3, //WFW
            7, //FWF
            6, //WFF
            1, //WWF
            2, //RWF
            2, //CWF
    };
    dist = dist * 10 - add_weights[desired];
    return dist;
}

int32_t World::get_smart_dist(uint32_t r, uint32_t desired) const {
    return smart_dist_dp[r][desired];
}

void World::update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const {
    int32_t diff = get_smart_dist(r, 0) - get_smart_dist(r, desired);
    cur_score += sign * diff * robot_power[r];
}

void World::add_path(uint32_t r) {
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

void World::remove_path(uint32_t r) {
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

uint32_t World::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter > 1000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
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
            if (rnd.get_d() < 0.1) {
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

bool World::try_build(uint32_t r) {
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
        ASSERT(std::abs(old_score - cur_score) < 1e-6,
               "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
               std::to_string(std::abs(old_score - cur_score)));
        return false;
    }
#ifdef PRINT_RECURSIVE
    if (std::abs(old_score - cur_score) > 1e-6) {
        log << '\n';
    }
#endif
    return true;
}

uint32_t World::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 ||//
        //counter > 30'000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
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
    return false;
}

bool World::build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = build(r, 0, counter);
    if (res == 0 || res == 2) {
        add_path(r);
        ASSERT(std::abs(old_score - cur_score) < 1e-6,
               "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
               std::to_string(std::abs(old_score - cur_score)));
        return false;
    }
#ifdef PRINT_RECURSIVE
    if (std::abs(old_score - cur_score) > 1e-6) {
        log << '\n';
    }
#endif
    return true;
}

World::World(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed)
        : robots(robots), end_time(end_time), rnd(seed) {

    visited.resize(robots.size());
    desires.resize(robots.size());

    {
        std::array<uint32_t, DEPTH> value{};
        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            value[depth] = -1;
        }
        used_pos.resize(get_graph().get_zipes_size(), value);
        used_edge.resize(get_graph().get_edges_size(), value);
    }

    // build order and power
    {
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
        max_weight = w + 1;*/

        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
        int32_t max_weight = robots.size() + 1;

        robot_power.resize(robots.size());
        //const double workload = robots.size() * 1.0 / get_map().get_count_free();
        for (uint32_t r = 0; r < robots.size(); r++) {
            double power = (max_weight - weight[r]) * 1.0 / max_weight;
            power = power * power;
            ASSERT(0 < power && power <= 1, "invalid power");
            robot_power[r] = power;
        }
    }

    smart_dist_dp.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        smart_dist_dp[r].resize(get_operations().size());
        for (uint32_t desired = 0; desired < smart_dist_dp[r].size(); desired++) {
            if (validate_path(r, desired)) {
                smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired);
            }
        }
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        desires[r] = 0;
        add_path(r);
    }

    robot_desires.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
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
    }
}

void World::simulate_pibt() {
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
}

double World::get_triv_score() const {
    double score = 0;
    for (uint32_t r = 0; r < robots.size(); r++) {
        update_score(r, desires[r], score, +1);
    }
    ASSERT(std::abs(score - cur_score) < 1e-5,
           "invalid score: " + std::to_string(score) + " != " + std::to_string(cur_score) + ", diff: " +
           std::to_string(std::abs(score - cur_score)));
    return score;
}

PMPS::PMPS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed)
        : robots(robots), main_world(robots, end_time, seed) {
}

void PMPS::solve(uint64_t random_seed) {
    constexpr uint32_t THR = 4;

    // свободен ли main_world?
    std::atomic<bool> locker = true;

    Randomizer rnd(random_seed);
    main_world.rnd = Randomizer(rnd.get());
    main_world.simulate_pibt();

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        while (get_now() < main_world.end_time) {

            while (true) {
                bool expected = true;
                if (locker.compare_exchange_strong(expected, false)) {
                    break;
                }
            }

            ASSERT(locker == false, "invalid locker");
            World world = main_world;
            ASSERT(locker == false, "invalid locker");
            locker = true;

            double start_score = world.cur_score;

            world.rnd = Randomizer(seed);
            seed = ((seed * 745215) ^ 71263554125) + 13;

            TimePoint end_time = get_now() + Milliseconds(10);

            world.temp = 0.001;
            uint32_t step = 0;
            for (; /*step < PIBTS_STEPS && */get_now() < end_time && get_now() < world.end_time; step++) {
                uint32_t r = world.rnd.get(0, robots.size() - 1);
                world.try_build(r);
                world.temp *= 0.99;
            }

            if (world.cur_score > main_world.cur_score + 1e-3) {
                while (true) {
                    bool expected = true;
                    if (locker.compare_exchange_strong(expected, false)) {
                        break;
                    }
                }

                ASSERT(locker == false, "invalid locker");
                if (world.cur_score > main_world.cur_score + 1e-3) {
                    Printer() << "improve(" << thr << "): " << start_score << "->" << world.cur_score << ", steps: " << step << '\n';

                    ASSERT(locker == false, "invalid locker");
                    main_world = world;
                    ASSERT(locker == false, "invalid locker");
                } else {
                    //Printer() << "failed:(" << thr << "): " << start_score << "->" << world.cur_score << '\n';
                }
                ASSERT(locker == false, "invalid locker");
                locker = true;
            }
        }
    };

    std::vector<std::thread> threads(THR);
    for (uint32_t thr = 0; thr < THR; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THR; thr++) {
        threads[thr].join();
    }
}

std::vector<Action> PMPS::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[main_world.desires[r]][0];
        if (main_world.desires[r] == 0) {
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

std::vector<uint32_t> PMPS::get_desires() const {
    return main_world.desires;
}

std::vector<int64_t> PMPS::get_changes() const {
    std::vector<int64_t> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = main_world.get_smart_dist(r, 0) - main_world.get_smart_dist(r, main_world.desires[r]);
    }
    return answer;
}

double PMPS::get_score() const {
    return main_world.cur_score;
}
