#include <Tools/tools.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>
#include <Objects/Environment/map.hpp>

#include <../inc/nlohmann/json.hpp>

using json = nlohmann::json;

void build_meta_info(const std::string &from, const std::string &to) {
    json data;
    std::ifstream input(from);
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        ASSERT(false, "failed to load: " + from + ", " + error.what());
    }

    // [pos][dir][action]
    std::vector<std::array<std::array<uint32_t, 5>, 5>> dp(get_map().get_size());

    std::vector<Position> starts;

    for (auto start: data["start"]) {
        std::string dir = start[2];
        uint32_t dir_val =
                dir == "E" ? 0 : (dir == "S" ? 1 : (dir == "W" ? 2 : (dir == "N" ? 3 : (ASSERT(false, "failed"), -1))));
        Position pos(start[0], start[1], dir_val);
        starts.push_back(pos);
    }

    uint32_t id = 0;
    for (const auto &path_line: data["actualPaths"]) {
        std::string path = path_line;
        Position pos = starts[id];
        for (int i = 0; i < path.size(); i += 2) {
            char c = path[i];
            if (c != 'F' && c != 'R' && c != 'C' && c != 'W') {
                //std::cout << "invalid action: " << c << std::endl;
            }
            Action act = c == 'F' ? Action::FW : (c == 'R' ? Action::CR : (c == 'C' ? Action::CCR : Action::W));

            dp[pos.get_pos()][pos.get_dir()][static_cast<uint32_t>(act)]++;
            dp[pos.get_pos()][4][static_cast<uint32_t>(act)]++;
            dp[pos.get_pos()][pos.get_dir()][4]++;
            dp[pos.get_pos()][4][4]++;

            pos = pos.simulate_action(act);
        }
        id++;
    }

    std::ofstream output(to);
    output << get_map().get_rows() << ' ' << get_map().get_cols() << std::endl;
    for (uint32_t dir = 0; dir < 5; dir++) {
        for (uint32_t act = 0; act < 5; act++) {
            for (uint32_t pos = 0; pos < get_map().get_size(); pos++) {
                if (pos != 0) {
                    output << ' ';
                }
                output << dp[pos][dir][act];
            }
            output << '\n';
        }
        output << '\n';
    }
}
