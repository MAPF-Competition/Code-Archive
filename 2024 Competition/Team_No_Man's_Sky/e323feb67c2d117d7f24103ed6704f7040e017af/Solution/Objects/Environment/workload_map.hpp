#pragma once

#include <SharedEnv.h>

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/map.hpp>
#include <Objects/Environment/graph.hpp>

#include <vector>
#include <cstdint>

class WorkloadMap {
    constexpr static inline uint32_t X_LEN = 4;
    constexpr static inline uint32_t Y_LEN = 4;

    // workload[bx][by] = number of agents in this block
    std::vector<std::vector<uint32_t>> workload;

    // cnt_free[bx][by]
    std::vector<std::vector<uint32_t>> cnt_free;

    // dp[from][to]
    std::vector<std::vector<uint64_t>> dp;

    void build(uint32_t from);

public:

    WorkloadMap() = default;

    WorkloadMap(const Map &map, const Graph &graph);

    void update(SharedEnvironment &env, TimePoint end_time);

    [[nodiscard]] uint64_t get(uint32_t source_node, uint32_t target_pos) const;
};

WorkloadMap &get_wmap();
