#pragma once

#include <cstdint>

// TODO
template<typename T>
class PersVector {
    struct Node;
    using PNode = Node *;

    struct Node {
        PNode left = nullptr;
        PNode right = nullptr;
        T value{};
    };

    uint32_t m_size = 0;
    PNode root = nullptr;

    PNode build(uint32_t left, uint32_t right) {
        auto ptr = new Node();
        if (left != right) {
            uint32_t mid = (left + right) / 2;
            ptr->left = build(left, mid);
            ptr->right = build(mid + 1, right);
        }
        return ptr;
    }

    PNode set(PNode ptr, uint32_t left, uint32_t right, uint32_t index, const T &value) {
        if (left == right) {
            ptr->value = value;
        } else {
            uint32_t mid = (left + right) / 2;
            // TODO
        }
        return ptr;
    }

public:
    PersVector() = default;

    explicit PersVector(uint32_t size) : m_size(size), root(build(0, size - 1)) {
    }

    const T &operator[](uint32_t index) const {
        uint32_t left = 0, right = size() - 1;
        PNode ptr = root;
        while (left < right) {
            uint32_t mid = (left + right) / 2;
            if (index <= mid) {
                ptr = ptr->left;
                right = mid;
            } else {
                ptr = ptr->right;
                left = mid + 1;
            }
        }
        return ptr->value;
    }

    [[nodiscard]] PersVector set(uint32_t index, const T &value) const {
        PersVector res;
        res.m_size = size();
        res.root = set(root, 0, size() - 1, index, value);
        return res;
    }

    [[nodiscard]] uint32_t size() const {
        return size;
    }
};