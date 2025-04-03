#include <Objects/Environment/workload_map.hpp>

#include <Objects/Environment/robot_handler.hpp>
#include <Objects/Environment/graph.hpp>

#include <set>

void WorkloadMap::build(uint32_t from) {
    // (metric, x, y)
    std::priority_queue<std::tuple<uint64_t, int, int>,
            std::vector<std::tuple<uint64_t, int, int>>, std::greater<>> heap;

    const uint32_t source_x = from / workload[0].size();
    const uint32_t source_y = from % workload[0].size();

    heap.push({0, source_x, source_y});

    //const uint32_t from = source_x * workload[0].size() + source_y;
    dp[from].assign(dp.size(), -1);

    const std::array<std::pair<int, int>, 4> actions = {std::pair<int, int>{0, 1},
                                                        {1, 0},
                                                        {0, -1},
                                                        {-1, 0}};

    while (!heap.empty()) {
        auto [metric, x, y] = heap.top();
        heap.pop();

        ASSERT(cnt_free[x][y] > 0, "no free");

        const uint32_t to = x * workload[0].size() + y;

        if (dp[from][to] != -1) {
            continue;
        }
        dp[from][to] = metric;

        for (auto [dx, dy]: actions) {
            if (0 <= x + dx && x + dx < workload.size() && //
                0 <= y + dy && y + dy < workload[0].size() && //
                cnt_free[x + dx][y + dy] != 0 &&//
                dp[from][(x + dx) * workload[0].size() + y + dy] == -1
                    ) {

                heap.push({metric + workload[x + dx][y + dy] * 10 / cnt_free[x + dx][y + dy] + X_LEN + Y_LEN, x + dx, y + dy});
            }
        }
    }
}

WorkloadMap::WorkloadMap(const Map &map, const Graph &graph) :
        workload(map.get_rows() / X_LEN + (map.get_rows() % X_LEN != 0),
                 std::vector<uint32_t>(map.get_cols() / Y_LEN + (map.get_cols() % Y_LEN != 0))),

        cnt_free(map.get_rows() / X_LEN + (map.get_rows() % X_LEN != 0),
                 std::vector<uint32_t>(map.get_cols() / Y_LEN + (map.get_cols() % Y_LEN != 0), 0)) {

    for (uint32_t x = 0; x < map.get_rows(); x++) {
        for (uint32_t y = 0; y < map.get_cols(); y++) {
            if (map.is_free(Position(x, y, 0).get_pos())) {
                cnt_free[x / X_LEN][y / Y_LEN]++;
            }
        }
    }

    /*for (uint32_t x = 0; x < workload.size(); x++) {
        for (uint32_t y = 0; y < workload[x].size(); y++) {
            if (!is_free[x][y]) {
                Printer() << "is not free: " << x << ' ' << y << '\n';
            }
        }
    }
    std::exit(100);*/
}

void WorkloadMap::update(SharedEnvironment &env, TimePoint end_time) {
    workload.assign(env.rows / X_LEN + (env.rows % X_LEN != 0),
                    std::vector<uint32_t>(env.cols / Y_LEN + (env.cols % Y_LEN != 0), 0));
    const auto &robots = get_robots_handler().get_robots();
    for (auto &robot: robots) {
        Position p = get_graph().get_pos(robot.node);
        uint32_t x = p.get_x() / X_LEN;
        uint32_t y = p.get_y() / Y_LEN;
        ASSERT(x < workload.size(), "invalid x");
        ASSERT(y < workload[x].size(), "invalid y");
        ASSERT(cnt_free[x][y] > 0, "invalid free");
        workload[x][y]++;
    }

    // TODO: multithread rebuild
    dp.resize(workload.size() * workload[0].size());
    for (uint32_t from = 0; from < dp.size(); from++) {
        build(from);
    }
}

uint64_t WorkloadMap::get(uint32_t source_node, uint32_t target_pos) const {

    uint32_t target_x = Position(target_pos, 0).get_x() / X_LEN;
    uint32_t target_y = Position(target_pos, 0).get_y() / Y_LEN;

    Position p = get_graph().get_pos(source_node);
    uint32_t source_x = p.get_x() / X_LEN;
    uint32_t source_y = p.get_y() / Y_LEN;

    return dp[source_x * workload[0].size() + source_y][target_x * workload[0].size() + target_y];

    // (metric, x, y)
    std::priority_queue<std::tuple<uint64_t, int, int>,
            std::vector<std::tuple<uint64_t, int, int>>, std::greater<>> heap;
    {
        Position p = get_graph().get_pos(source_node);
        heap.push({0, p.get_x() / X_LEN, p.get_y() / Y_LEN});
    }

    std::set<std::pair<int, int>> visited;

    const std::array<std::pair<int, int>, 4> actions = {std::pair<int, int>{0, 1},
                                                        {1, 0},
                                                        {0, -1},
                                                        {-1, 0}};

    while (!heap.empty()) {
        auto [metric, x, y] = heap.top();
        heap.pop();

        ASSERT(cnt_free[x][y] > 0, "no free");

        if (visited.count({x, y})) {
            continue;
        }
        visited.insert({x, y});

        if (x == target_x && y == target_y) {
            Position p = get_graph().get_pos(source_node);
            uint32_t source_x = p.get_x() / X_LEN;
            uint32_t source_y = p.get_y() / Y_LEN;

            ASSERT(metric == dp[source_x * workload[0].size() + source_y][x * workload[0].size() + y], "invalid dp");

            return metric;
        }

        for (auto [dx, dy]: actions) {
            if (0 <= x + dx && x + dx < workload.size() && //
                0 <= y + dy && y + dy < workload[0].size() && //
                cnt_free[x + dx][y + dy] != 0 &&//
                !visited.count({x + dx, y + dy})
                    ) {

                heap.push({metric + workload[x][y] * 10 / cnt_free[x][y] + X_LEN + Y_LEN, x + dx, y + dy});
            }
        }
    }

    FAILED_ASSERT("invalid target");
    return 1e10;
}

WorkloadMap &get_wmap() {
    static WorkloadMap wmap;
    return wmap;
}
