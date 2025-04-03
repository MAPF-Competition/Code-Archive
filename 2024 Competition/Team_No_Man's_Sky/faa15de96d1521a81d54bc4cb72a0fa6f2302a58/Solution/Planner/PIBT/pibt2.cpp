#include <Planner/PIBT/pibt2.hpp>

#include <Objects/Basic/assert.hpp>

bool PIBT2::validate_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desires[r])[0] > 0;
    /*auto &path = get_operations()[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node || !to_edge) {
            return false;
        }

        node = to_node;
    }
    return true;*/
}

/*bool PIBT2::validate_path(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;

    auto &path = get_operations()[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node || !to_edge) {
            return false;
        }

        node = to_node;
    }
    return true;
}*/

bool PIBT2::is_free_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r), "invalid path");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_pos[depth][poses_path[depth]] != -1) {
            ASSERT(used_pos[depth][poses_path[depth]] != r, "invalid used_node");
            return false;
        }
        if (used_edge[depth][edges_path[depth]] != -1) {
            ASSERT(used_edge[depth][edges_path[depth]] != r, "invalid used_edge");
            return false;
        }
    }

    /*uint32_t node = robots[r].node;
    auto &path = get_operations()[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        if (used_edge[depth][to_edge] != -1) {
            ASSERT(used_edge[depth][to_edge] != r, "invalid used_edge");
            return false;
        }

        if (used_pos[depth][to_pos] != -1) {
            ASSERT(used_pos[depth][to_pos] != r, "invalid used_node");
            return false;
        }

        node = to_node;
    }*/
    return true;
}

/*bool PIBT2::is_free_path(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r), "invalid path");

    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        if (used_edge_value != -1) {
            ASSERT(used_edge_value != r, "invalid used_edge");
            return false;
        }

        if (used_pos_value != -1) {
            ASSERT(used_pos_value != r, "invalid used_node");
            return false;
        }

        node = to_node;
    }
    return true;
}*/

const EPath &PIBT2::get_path(uint32_t r) const {
    uint32_t node = robots[r].node;
    return get_omap().get_nodes_path(node, desires[r]);
    /*auto &path = get_operations()[desires[r]];
    std::array<uint32_t, DEPTH> result{};
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);

        result[depth] = to_node;
        node = to_node;
    }
    return result;*/
}

/*EPath PIBT2::get_path(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
    std::array<uint32_t, DEPTH> result{};
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);

        result[depth] = to_node;
        node = to_node;
    }
    return result;
}

uint32_t PIBT2::get_path_weight(uint32_t r) const {
    uint32_t node = robots[r].node;
    auto &path = get_operations()[desires[r]];
    uint32_t result = 0;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        ASSERT(r < weights.size(), "invalid weights size");
        if (weights[r].count(to_edge)) {
            result += weights[r].at(to_edge);
        }

        node = to_node;
    }
    return result;
}

uint32_t PIBT2::get_path_weight(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
    uint32_t result = 0;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        ASSERT(r < weights.size(), "invalid weights size");
        if (weights[r].count(to_edge)) {
            result += weights[r].at(to_edge);
        }

        node = to_node;
    }
    return result;
}*/

uint32_t PIBT2::get_used(uint32_t r) const {
    uint32_t node = robots[r].node;
    uint32_t answer = -1;

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        if (used_edge[depth][to_edge] != -1) {
            if (answer == -1) {
                answer = used_edge[depth][to_edge];
            } else if (answer != used_edge[depth][to_edge]) {
                answer = -2;
                break;
            }
        }

        if (used_pos[depth][to_pos] != -1) {
            if (answer == -1) {
                answer = used_pos[depth][to_pos];
            } else if (answer != used_pos[depth][to_pos]) {
                answer = -2;
                break;
            }
        }
    }
    /*for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        if (used_edge[depth][to_edge] != -1) {
            if (answer == -1) {
                answer = used_edge[depth][to_edge];
            } else if (answer != used_edge[depth][to_edge]) {
                answer = -2;
                break;
            }
        }

        if (used_pos[depth][to_pos] != -1) {
            if (answer == -1) {
                answer = used_pos[depth][to_pos];
            } else if (answer != used_pos[depth][to_pos]) {
                answer = -2;
                break;
            }
        }
        node = to_node;
    }*/
    return answer;
}

/*uint32_t PIBT2::get_used(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
    std::set<uint32_t> answer;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        if (used_edge_value != -1) {
            answer.insert(used_edge_value);
        }

        if (used_pos_value != -1) {
            answer.insert(used_pos_value);
        }

        node = to_node;
    }
    if (answer.size() > 1) {
        return -2;
    }
    if (answer.empty()) {
        return -1;
    }
    return *answer.begin();
}*/

void PIBT2::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired: " + std::to_string(desires[r]) + " " + std::to_string(get_operations().size()));

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;
    }
    /*auto &path = get_operations()[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;

        node = to_node;
    }*/
}

/*void PIBT2::add_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired: " + std::to_string(desired) + " " + std::to_string(get_operations().size()));

    uint32_t node = robots[r].node;
    ASSERT(node != 0, "invalid start node");
    auto &path = get_operations()[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == -1, "already used");
        state.used_edge[depth][to_edge] = r;

        ASSERT(used_pos_value == -1, "already used");
        state.used_pos[depth][to_pos] = r;

        node = to_node;
    }
}*/

void PIBT2::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == r, "invalid edge");
        used_edge[depth][to_edge] = -1;

        ASSERT(used_pos[depth][to_pos] == r, "invalid node");
        used_pos[depth][to_pos] = -1;
    }
    /*uint32_t node = robots[r].node;
    auto &path = get_operations()[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == r, "invalid edge");
        used_edge[depth][to_edge] = -1;

        ASSERT(used_pos[depth][to_pos] == r, "invalid node");
        used_pos[depth][to_pos] = -1;

        node = to_node;
    }*/
}

/*void PIBT2::remove_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");

    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == r, "invalid edge");
        state.used_edge[depth][to_edge] = -1;

        ASSERT(used_pos_value == r, "invalid node");
        state.used_pos[depth][to_pos] = -1;

        node = to_node;
    }
}*/

bool PIBT2::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 256 == 0 && get_now() > end_time)) {
        counter = -1;
        return false;
    }

    uint32_t old_desired = desires[r];

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
        desires[r] = desired;
        if (validate_path(r) && get_used(r) != -2) {
            const auto& path = get_path(r);

            int64_t priority = get_hm().get(path.back(), get_robots_handler().get_robot(r).target);
            // priority += get_path_weight(r);// ENABLE PATH WEIGHT
            steps.emplace_back(priority, desired);
        }
    }

    std::stable_sort(steps.begin(), steps.end());
    //std::reverse(steps.begin(), steps.end());

    for (auto [priority, desired]: steps) {
        desires[r] = desired;
        if (is_free_path(r)) {
            // отлично! там никого нет
            add_path(r);
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 3'000 && depth > 6) {
                continue;
            } else if (counter > 10'000 && depth > 3) {
                continue;
            } else if (counter > 30'000) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (desires[to_r] != 0) {
                continue;
            }
            remove_path(to_r);
            add_path(r);

            if (build(to_r, depth + 1, ++counter)) {
                return true;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    return false;
}

/*std::optional<PIBT2::State> PIBT2::build2_IMPL(uint32_t r) const {
    struct comparator {
        bool operator()(const std::tuple<double, uint32_t, State> &lhs, const std::tuple<double, uint32_t, State> &rhs) const {
            return std::get<0>(lhs) < std::get<0>(rhs);
        }
    };

    std::multiset<std::tuple<double, uint32_t, State>, comparator> Q;
    Q.insert({0, r, {}});

    while (!Q.empty()) {
        auto [score, r, state] = *Q.begin();
        Q.erase(Q.begin());

        if (r == -1) {
            return state;
        }

        // (priority, desired)
        std::vector<std::pair<int64_t, uint32_t>> steps;
        for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
            auto cur_state = state;

            cur_state.desires[r] = desired;
            if (!validate_path(r, cur_state) || get_used(r, cur_state) == -2) {
                continue;
            }

            auto path = get_path(r, cur_state);

            int64_t priority = get_hm().get(path.back(), get_robots_handler().get_robot(r).target) * 10;
            priority += get_path_weight(r, cur_state);

            steps.emplace_back(priority, desired);
        }

        std::stable_sort(steps.begin(), steps.end());
        std::reverse(steps.begin(), steps.end());

        for (int32_t k = 0; k < steps.size(); k++) {
            auto [priority, desired] = steps[k];
            auto cur_state = state;

            cur_state.desires[r] = desired;
            if (!validate_path(r, cur_state) || get_used(r, cur_state) == -2) {
                ASSERT(false, "kek");
                continue;
            }

            auto path = get_path(r, cur_state);

            //int64_t priority = get_hm().get_to_pos(path.back(), get_robots_handler().get_robot(r).target) * 10;
            //priority += get_path_weight(r, cur_state);

            if (is_free_path(r, cur_state)) {
                // отлично! там никого нет
                add_path(r, cur_state);
                Q.insert({score - k * k, -1, cur_state});
            } else {
                // о нет! там кто-то есть

                uint32_t to_r = get_used(r, cur_state);
                ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
                if ((cur_state.desires.count(to_r) ? cur_state.desires.at(to_r) : desires[to_r]) != 0) {
                    continue;
                }
                remove_path(to_r, cur_state);
                add_path(r, cur_state);

                Q.insert({score - k * k, to_r, cur_state});
            }
        }
    }

    return std::nullopt;
}

bool PIBT2::build2(uint32_t r) {
    auto result = build2_IMPL(r);
    if (!result) {
        return false;
    }
    auto state = *result;

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        for (auto [pos, value]: state.used_pos[depth]) {
            used_pos[depth][pos] = value;
        }
        for (auto [edge, value]: state.used_edge[depth]) {
            used_edge[depth][edge] = value;
        }
    }
    for (auto [r, desired]: state.desires) {
        desires[r] = desired;
    }
    return true;
}*/

PIBT2::PIBT2(const std::vector<Robot> &robots//, const std::vector<std::unordered_map<uint32_t, uint32_t>> &weights
             )
    : robots(robots)//, weights(weights)
{

    desires.resize(robots.size());

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        used_pos[depth].assign(get_map().get_size(), -1);
        used_edge[depth].assign(get_graph().get_edges_size(), -1);
    }

    for (uint32_t r = 0; r < this->robots.size(); r++) {
        add_path(r);
    }
}

std::vector<Action> PIBT2::solve(const std::vector<uint32_t> &order, const TimePoint end_time) {
    this->end_time = end_time;

    for (uint32_t r: order) {
        if (get_now() > end_time) {
            break;
        }
        if (desires[r] == 0) {
            remove_path(r);
            uint32_t counter = 0;
            if (!build(r, 0, counter)) {

                add_path(r);
            }
        }
    }

    std::vector<Action> answer(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[desires[r]][0];
    }

    return answer;
}
