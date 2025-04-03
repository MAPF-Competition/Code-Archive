#pragma once

#include "ActionModel.h"

class Position {
    uint16_t x = 0, y = 0;
    uint16_t dir = 0;// 0:east, 1:south, 2:west, 3:north

public:
    Position() = default;

    Position(uint32_t pos, uint32_t dir);

    Position(uint32_t x, uint32_t y, uint32_t dir);

    [[nodiscard]] uint32_t get_pos() const;

    [[nodiscard]] uint32_t get_x() const;

    [[nodiscard]] uint32_t get_y() const;

    [[nodiscard]] uint32_t get_dir() const;

    // корректная позиция + проходимо
    [[nodiscard]] bool is_valid() const;

    // двигает вперед учитывая направление
    [[nodiscard]] Position move_forward() const;

    // поворачивает по часовой стрелке
    [[nodiscard]] Position rotate() const;

    // поворачивает против часовой стрелке
    [[nodiscard]] Position counter_rotate() const;

    // применяет action на эту точку
    [[nodiscard]] Position simulate_action(const Action &action) const;

    friend bool operator==(const Position &lhs, const Position &rhs);

    friend bool operator!=(const Position &lhs, const Position &rhs);

    friend bool operator<(const Position &lhs, const Position &rhs);

    friend std::ostream &operator<<(std::ostream &output, const Position &pos);
};
