#pragma once

#include <Objects/Environment/map.hpp>
#include <Objects/Environment/info.hpp>

#include <settings.hpp>

#include <vector>

#include <SharedEnv.h>

class GuidanceMap {
    // map[x][y]:
    // @ препятствие
    // > рекомендуем вправо
    // < рекомендуем влево
    // ^ рекомендуем вверх
    // v рекомендуем вниз
    // . нет рекомендации
    std::vector<std::string> desired;

    void set_random(const Map &map);

    void set_city(const Map &map);

    void set_game(const Map &map);

    void set_warehouse(const Map &map);

    void set_sortation(const Map &map);

    void overlay(const std::vector<std::string> &image, uint32_t x, uint32_t y);

public:
    GuidanceMap() = default;

    GuidanceMap(MapType type, const Map &map);

    [[nodiscard]] uint32_t get_rows() const;

    [[nodiscard]] uint32_t get_cols() const;

    [[nodiscard]] char get(uint32_t x, uint32_t y) const;

    friend std::ostream &operator<<(std::ostream &output, const GuidanceMap &map);

    friend std::istream &operator>>(std::istream &input, GuidanceMap &map);
};

GuidanceMap &get_guidance_map();
