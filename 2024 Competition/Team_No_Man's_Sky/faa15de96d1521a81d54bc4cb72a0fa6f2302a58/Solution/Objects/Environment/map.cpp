#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/map.hpp>
#include <cstddef>
#include <vector>

Map::Map(const SharedEnvironment &env) : rows(env.rows), cols(env.cols) {
    ASSERT(get_size() == env.map.size() + 1, "size mismatch");
    map.resize(env.map.size() + 1);
    map[0] = false;
    for (uint32_t pos = 0; pos < env.map.size(); pos++) {
        map[pos + 1] = env.map[pos] == 0;
        if (map[pos + 1]) {
            cnt_free++;
        }
    }
}

Map::Map(const std::vector<bool> &mp, size_t cols, size_t rows) : rows(rows), cols(cols), map(mp) {
    map.resize(rows * cols);
    ASSERT(false, "incorrect logic");
}


uint32_t Map::get_rows() const {
    return rows;
}

uint32_t Map::get_cols() const {
    return cols;
}

uint32_t Map::get_size() const {
    return rows * cols + 1;
}

uint32_t Map::get_count_free() const {
    return cnt_free;
}

bool Map::is_free(uint32_t pos) const {
    ASSERT(pos < map.size(), "invalid pos");
    return map[pos];
}

Map &get_map() {
    static Map map;
    return map;
}
