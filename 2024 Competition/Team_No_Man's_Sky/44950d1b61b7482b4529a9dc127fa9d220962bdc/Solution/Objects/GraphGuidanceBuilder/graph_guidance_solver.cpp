#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Basic/position.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <thread>

Meta operator+(Meta lhs, const Meta &rhs) {
    ASSERT(lhs.size() == rhs.size(), "unmatch sizes");
    for (uint32_t pos = 0; pos < lhs.size(); pos++) {
        for (uint32_t dir = 0; dir < 5; dir++) {
            for (uint32_t act = 0; act < 5; act++) {
                lhs[pos][dir][act] += rhs[pos][dir][act];
            }
        }
    }
    return lhs;
}

void GraphGuidanceSolver::change_path(GraphGuidance &gg, Randomizer &rnd) const {
    uint32_t pos = rnd.get(1, gg.get_size() - 1);
    uint32_t len = rnd.get(1, 30);

    int diff = rnd.get(-2, 2);
    while (len--) {
        uint32_t dir = rnd.get(0, 3);
        uint32_t act = rnd.get(0, 3);

        int val = gg.get(pos, dir, act);
        val += diff;
        val = std::max(1, val);
        val = std::min(50, val);
        gg.set(pos, dir, act, val);

        int x = (pos - 1) / gg.get_cols();
        int y = (pos - 1) % gg.get_rows();
        ASSERT(0 <= x && x < gg.get_rows(), "invalid x");
        ASSERT(0 <= y && y < gg.get_cols(), "invalid y");

        {
            int dir = rnd.get(0, 3);
            if (dir == 0) {
                y++;
            } else if (dir == 1) {
                x++;
            } else if (dir == 2) {
                y--;
            } else if (dir == 3) {
                x--;
            }

            if (0 <= x && x < gg.get_rows() && 0 <= y && y < gg.get_cols()) {
                pos = x * gg.get_cols() + y + 1;
            }
        }
    }
}

void GraphGuidanceSolver::big_change(GraphGuidance &gg, Randomizer &rnd) const {
    int left = -1;
    int right = 1;
    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t act = 0; act < 4; act++) {
            for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                int val = gg.get(pos, dir, act);
                val += rnd.get(left, right);
                val = std::max(1, val);
                val = std::min(1000, val);
                gg.set(pos, dir, act, val);
            }
        }
    }
}

void GraphGuidanceSolver::smart_change(GraphGuidance &gg, const Meta &meta, Randomizer &rnd) const {
    // (priority, pos, dir, action)
    std::vector<std::tuple<double, uint32_t, uint32_t, uint32_t>> pool;
    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t act = 0; act < 4; act++) {
            for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                double priority = 1e9;
                for (uint32_t other_dir = 0; other_dir < 4; other_dir++) {
                    priority = std::min(priority,
                                        std::abs(static_cast<double>(meta[pos][dir][act]) - meta[pos][other_dir][act]));
                }
                pool.emplace_back(priority, pos, dir, act);
            }
        }
    }
    std::sort(pool.begin(), pool.end(), std::greater<>());

    uint32_t len = rnd.get(1, 30);
    for (uint32_t i = 0; i < len && i < pool.size(); i++) {
        if (rnd.get_d() < 0.3) {
            continue;
        }
        auto [priority, pos, dir, act] = pool[i];
        int val = gg.get(pos, dir, act);
        val += rnd.get(-2, 2);
        val = std::max(1, val);
        val = std::min(1000, val);
        gg.set(pos, dir, act, val);
    }
}

void GraphGuidanceSolver::change_opw(std::vector<int> &opw, Randomizer &rnd) const {
    double p = rnd.get_d();
    if (p < 0.1) {
        // swap
        uint32_t a = rnd.get(0, opw.size() - 1);
        uint32_t b = rnd.get(0, opw.size() - 1);
        std::swap(opw[a], opw[b]);
    } else if (p < 0.3) {
        // +-
        uint32_t a = rnd.get(0, opw.size() - 1);
        int d = rnd.get(-10, 10);
        opw[a] += d;
    } /*else if (p < 0.6) {
        // reverse
        uint32_t a = rnd.get(0, opw.size() - 1);
        uint32_t b = rnd.get(0, opw.size() - 1);
        if (a > b) {
            std::swap(a, b);
        }
        while (a < b) {
            std::swap(opw[a], opw[b]);
            a++;
            b--;
        }
    } else if (p < 0.8) {
        // shuffle
        std::shuffle(opw.begin(), opw.end(), rnd.generator);
    } */else {
        // segment +-
        uint32_t a = rnd.get(0, opw.size() - 1);
        uint32_t b = rnd.get(0, opw.size() - 1);
        if (a > b) {
            std::swap(a, b);
        }
        for (uint32_t i = a; i <= b; i++) {
            opw[i] += rnd.get(-10, 10);
        }
    }
}

void GraphGuidanceSolver::simulate_solver(uint32_t thr) {
    Randomizer rnd(228 * thr + 984213);
    while (true) {
        GraphGuidance gg;
        std::vector<int> opw;
        Meta meta;
        {
            std::unique_lock locker(mutex);
            gg = best_gg;
            opw = best_opw;
            meta = best_meta;
        }

        //double p = rnd.get_d();
        //if (p < 0.3) {
        //    smart_change(gg, meta, rnd);
        //}else
        /*if (p < 0.2) {
            big_change(gg, rnd);
        } else {
            change_path(gg, rnd);
        }

        if (rnd.get_d() < 0.5) {
            change_opw(opw, rnd);
        }*/

        change_opw(opw, rnd);

        double score = 0;
        std::tie(score, meta) = get_score(gg, opw, thr);

        /*double again_score = get_score(gg, dhm_power, thr);
        ASSERT(std::abs(cur_score - again_score) < 1e-9,
               "invalid scores: " + std::to_string(cur_score) + " != " + std::to_string(again_score) + ", thr: " +
               std::to_string(thr));*/

        {
            std::unique_lock locker(mutex);
            if (score > best_score || rnd.get_d() < 5.0 / (best_score - score + 1)) {
                Printer() << "improve(" << thr << "): " << best_score << " -> " << score << '\n';
                Printer().get().flush();

                best_score = score;
                best_gg = gg;
                best_opw = opw;
                best_meta = meta;

                {
                    std::ofstream output("best_gg");
                    output << best_gg;
                }
                {
                    std::ofstream output("best_opw");
                    output << best_opw.size() << '\n';
                    for (auto x: best_opw) {
                        output << x << '\n';
                    }
                }
                {
                    std::ofstream output("best_meta");
                    output << gg.get_rows() << ' ' << gg.get_cols() << '\n';
                    for (uint32_t dir = 0; dir < 5; dir++) {
                        for (uint32_t act = 0; act < 5; act++) {
                            for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                                output << meta[pos][dir][act] << ' ';
                            }
                            output << '\n';
                        }
                        output << '\n';
                    }
                }
            }
        }
    }
}

GraphGuidanceSolver::GraphGuidanceSolver(const GraphGuidance &gg, const vector<int> &opw)
        : best_gg(gg),
          best_opw(opw) {
    std::tie(best_score, best_meta) = get_score(gg, opw, 0);
}

void GraphGuidanceSolver::solve() {
    std::vector<std::thread> threads(32);
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr] = std::thread([&](uint32_t thr) {
            simulate_solver(thr);
        }, thr);
    }
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr].join();
    }
}

std::pair<double, Meta>
GraphGuidanceSolver::get_score(const GraphGuidance &gg, const vector<int> &opw, uint32_t thr) {
    ETimer timer;

    {
        std::ofstream output("Tmp/gg" + std::to_string(thr));
        output << gg;
    }
    {
        std::ofstream output("Tmp/opw" + std::to_string(thr));
        output << opw.size() << '\n';
        for (auto x: opw) {
            output << x << '\n';
        }
    }

    constexpr uint32_t steps = 500;

    auto call = [&](const std::string &test) {
        int ret = std::system(
                ("./lifelong -i ./example_problems/" + test + " -o Tmp/test" +
                 std::to_string(thr) + ".json -s " + std::to_string(steps) + " -t 100000000 -p 100000000 --unique_id " +
                 std::to_string(thr) +
                 " > Tmp/output" +
                 std::to_string(thr)).c_str());
        ASSERT(ret == 0, "invalid return code: " + std::to_string(ret));

        uint32_t finished_tasks = 0;
        {
            using json = nlohmann::basic_json<nlohmann::ordered_map>;
            json data;
            std::ifstream input("Tmp/test" + std::to_string(thr) + ".json");
            try {
                data = json::parse(input);
            } catch (const json::parse_error &error) {
                FAILED_ASSERT(error.what());
            }
            finished_tasks = data["numTaskFinished"];
        }

        std::ifstream input("Tmp/meta" + std::to_string(thr));
        // [pos][dir][action]
        Meta meta(gg.get_size());

        uint32_t rows, cols;
        input >> rows >> cols;

        ASSERT(gg.get_rows() == rows && gg.get_cols() == cols, "invalid rows/cols");
        for (uint32_t dir = 0; dir < 5; dir++) {
            for (uint32_t act = 0; act < 5; act++) {
                for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                    input >> meta[pos][dir][act];
                }
            }
        }

        auto calc = [&](uint32_t dir, uint32_t act) {
            uint64_t s = 0;
            for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                s += static_cast<uint64_t>(meta[pos][dir][act]) * meta[pos][dir][act];
            }
            return (static_cast<double>(s) / (gg.get_size() - 1)) / steps;
        };

        double s = 0;
        for (uint32_t dir = 0; dir < 4; dir++) {
            s += calc(dir, 1);
            s += calc(dir, 2);
            s += calc(dir, 3) * 2;
        }
        double score = finished_tasks - s * 5;
        return std::tuple{score, meta, finished_tasks};
    };

    auto [score1, meta1, finished_tasks1] = call("random.domain/random_32_32_20_400_2.json");
    auto [score2, meta2, finished_tasks2] = call("random.domain/random_32_32_20_500_2.json");

    double score = score1 + score2;
    Meta meta = meta1 + meta2;
    uint32_t finished_tasks = finished_tasks1 + finished_tasks2;

    {
        std::unique_lock locker(mutex);
        Printer() << "finish(" << thr << "): " << score << ", " << finished_tasks << ", " << timer << '\n';
        Printer().get().flush();
    }
    return {score, meta};
}
