#pragma once

#include <ActionModel.h>

#include <array>
#include <cstdint>
#include <vector>

static constexpr inline uint32_t DEPTH = 5;

using Operation = std::array<Action, DEPTH>;

using EPath = std::array<uint32_t, DEPTH>;

class OperationsGenerator {
    std::vector<Operation> pool;

    void generate(Operation &op, uint32_t i);

public:
    std::vector<Operation> get();
};

std::vector<Operation> &get_operations();

uint32_t get_operation_depth(uint32_t index);

std::vector<uint32_t> &get_operations_ids(uint32_t d);

void init_operations();

std::ostream &operator<<(std::ostream &output, const Operation &op);

std::istream &operator>>(std::istream &input, Operation &op);
