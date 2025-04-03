#include <Objects/Environment/info.hpp>

#include <settings.hpp>

MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

TestType &get_test_type() {
    static TestType type = TestType::NONE;
    return type;
}

static std::array<TestInfo, static_cast<uint32_t>(TestType::NONE) + 1> test_info = {

        TestInfo{5000 /*TODO*/, MAX_AGENTS_NUM},// CITY_1
        TestInfo{5000 /*TODO*/, MAX_AGENTS_NUM},// CITY_2
        TestInfo{5000, 2500},                   // GAME
        TestInfo{600, 100},                     // RANDOM_1
        TestInfo{600, 200},                     // RANDOM_2
        TestInfo{800, 400},                     // RANDOM_3
        TestInfo{1000, 400},                    // RANDOM_4
        TestInfo{2000, 800},                    // RANDOM_5
        TestInfo{5000, 10000},                  // SORTATION
        TestInfo{5000, 10000},                  // WAREHOUSE
        TestInfo{5000, 10000},                  // NONE
};

TestInfo get_test_info() {
    return test_info[static_cast<uint32_t>(get_test_type())];
}
