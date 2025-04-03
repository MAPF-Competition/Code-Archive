#pragma once

#include <Objects/Basic/assert.hpp>
#include <cstdint>
#include <vector>

template<typename T>
class LinearHeap {
    std::vector<T> data;
    uint32_t top_id = 0;

public:
    [[nodiscard]] bool empty() const {
        return top_id == data.size();
    }
    void push(T &&value) {
        uint32_t i = data.size();
        data.push_back(std::move(value));
        for (; i > top_id && data[i] < data[i - 1]; --i) {
            std::swap(data[i - 1], data[i]);
        }
    }

    [[nodiscard]] const T &top() const {
        ASSERT(top_id < data.size(), "invalid top");
        return data[top_id];
    }

    void pop() {
        ASSERT(top_id < data.size(), "invalid pop");
        ++top_id;
    }
};
