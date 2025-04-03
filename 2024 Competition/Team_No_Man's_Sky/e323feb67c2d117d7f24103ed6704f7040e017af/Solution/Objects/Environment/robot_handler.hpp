#pragma once

#include <SharedEnv.h>
#include <cstdint>
#include <vector>

struct Robot {
    uint32_t node = 0;    // start node from graph
    uint32_t target = 0;  // target pos from map
    uint32_t priority = -1; // -1 if disabled agent

    [[nodiscard]] bool is_disable() const;
};

class RobotsHandler {
    std::vector<Robot> robots;

public:
    RobotsHandler() = default;

    explicit RobotsHandler(uint32_t agents_num);

    void update(const SharedEnvironment &env);

    [[nodiscard]] const Robot &get_robot(uint32_t r) const;

    [[nodiscard]] const std::vector<Robot> &get_robots() const;

    [[nodiscard]] uint32_t size() const;
};

RobotsHandler &get_robots_handler();
