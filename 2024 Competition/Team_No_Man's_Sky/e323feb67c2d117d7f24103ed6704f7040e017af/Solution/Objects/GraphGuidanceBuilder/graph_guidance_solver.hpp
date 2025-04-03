#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Objects/Environment/graph_guidance.hpp>

#include <mutex>

// [pos][dir][action]
using Meta = std::vector<std::array<std::array<uint32_t, 5>, 5>>;

Meta operator+(Meta lhs, const Meta &rhs);

class GraphGuidanceSolver {

    std::mutex mutex;

    GraphGuidance best_gg;

    std::vector<int> best_opw;

    Meta best_meta;

    double best_score = 0;

    void change_path(GraphGuidance &gg, Randomizer &rnd) const;

    void big_change(GraphGuidance &gg, Randomizer &rnd) const;

    void smart_change(GraphGuidance &gg, const Meta &meta, Randomizer &rnd) const;

    void change_opw(std::vector<int> &opw, Randomizer &rnd) const;

    void simulate_solver(uint32_t thr);

public:
    GraphGuidanceSolver(const GraphGuidance &gg, const vector<int> &opw);

    void solve();

    [[nodiscard]] std::pair<double, Meta> get_score(const GraphGuidance &gg, const vector<int> &opw, uint32_t thr);
};
