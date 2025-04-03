#include <Objects/Containers/dsu.hpp>

#include <Objects/Basic/assert.hpp>
#include <numeric>

DSU::DSU(uint32_t size) {
    parent.resize(size);
    std::iota(parent.begin(), parent.end(), 0);
    sz.resize(size, 1);
}

uint32_t DSU::get(uint32_t x) {
    ASSERT(x < parent.size(), "invalid x");
    if (parent[x] == x) {
        return x;
    } else {
        return parent[x] = get(parent[x]);
    }
}

uint32_t DSU::get_size(uint32_t x) {
    return sz[get(x)];
}

void DSU::uni(uint32_t a, uint32_t b) {
    a = get(a);
    b = get(b);
    if (a != b) {
        if (sz[a] > sz[b]) {
            std::swap(a, b);
        }
        parent[a] = b;
        sz[b] += sz[a];
    }
}
