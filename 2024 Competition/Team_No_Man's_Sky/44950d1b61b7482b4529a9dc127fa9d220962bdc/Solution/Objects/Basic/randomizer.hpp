#pragma once

#include <Objects/Basic/assert.hpp>
#include <random>

struct Randomizer {
    std::mt19937_64 generator;

public:
    Randomizer();

    explicit Randomizer(uint64_t seed);

    // uint64_t
    uint64_t get();

    // int64_t [left, right]
    int64_t get(int64_t left, int64_t right);

    // double [0, 1]
    double get_d();

    // double [left, right]
    double get_d(double left, double right);

    template<typename Container_t>
    auto &get(Container_t &container) {
        ASSERT(!container.empty(), "container is empty");
        return *std::next(container.begin(), get(0, container.size() - 1));
    }

    template<typename Container_t>
    const auto &get(const Container_t &container) {
        ASSERT(!container.empty(), "container is empty");
        return *std::next(container.begin(), get(0, container.size() - 1));
    }
};
