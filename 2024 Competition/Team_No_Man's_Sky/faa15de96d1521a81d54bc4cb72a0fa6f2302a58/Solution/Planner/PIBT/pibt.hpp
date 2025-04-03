#pragma once

#include <Objects/Basic/position.hpp>

// Priority Inheritance with BackTracking
// Each robot is assigned a direction where the robot would like to go
class PIBT {
    struct Robot {
        uint32_t node = 0;

        uint32_t pos = 0;

        // куда мы хотим
        // -1 -- не определено
        // иначе это направление для forward
        int desired = -1;
    };

    std::vector<Robot> robots;

    std::unordered_map<uint32_t, uint32_t> pos_to_robot;

    bool build(uint32_t r, int banned_desired, uint32_t depth);

public:
    PIBT();

    std::vector<Action> solve(const std::vector<uint32_t> &order, const std::chrono::steady_clock::time_point end_time);
};
