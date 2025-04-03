#pragma once

#include <cstdint>
#include <vector>

class DSU {
    std::vector<uint32_t> parent;
    std::vector<uint32_t> sz;

public:
    explicit DSU(uint32_t size);

    uint32_t get(uint32_t x);

    uint32_t get_size(uint32_t x);

    void uni(uint32_t a, uint32_t b);
};
