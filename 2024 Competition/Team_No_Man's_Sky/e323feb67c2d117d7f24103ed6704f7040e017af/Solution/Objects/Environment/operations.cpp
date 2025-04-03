#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

#include <fstream>

bool verify_lol(const Operation &op) {
    // не нужно, чтобы на конце операции были повороты
    for (int i = op.size() - 1; i >= 0 && op[i] != Action::FW; i--) {
        if (op[i] == Action::CR || op[i] == Action::CCR) {
            return false;
        }
    }
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size() && op[j] != Action::FW; j++) {
            if (op[i] == Action::CR && op[j] == Action::CCR) {
                return false;
            }
            if (op[i] == Action::CCR && op[j] == Action::CR) {
                return false;
            }
        }
    }
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size() && op[j] != Action::FW; j++) {
            for (int k = j + 1; k < op.size() && op[k] != Action::FW; k++) {
                if ((op[i] == Action::CR || op[i] == Action::CCR) && op[i] == op[j] && op[j] == op[k]) {
                    return false;
                }
            }
        }
    }
    return true;
}

void OperationsGenerator::generate(Operation &op, uint32_t i) {
    if (i == DEPTH) {
        if (verify_lol(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 0; action < 4; action++) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1);
        }
    }
}

std::vector<Operation> OperationsGenerator::get() {
    //Operation op;
    //generate(op, 0);

    // read pool
    {
        //std::ifstream input("Tmp/actions" + std::to_string(get_unique_id()) + ".txt");
        std::stringstream input(
                // random 100: 2618
                // random 400: 6397
                // random 700: 5663
                // random 800: 4346
                //"17 WWWW CWFW RWFW WWFW WFFW FWFW WFWW RRFW CFFW RFFW CFWW RFWW FCFW FRFW FWWW FFWW FFFW"

                // random 100: 2669
                // random 400:
                // random 700: 5798
                // random 800: 3871
                //"45 WWWW WWWF RRWF RRFW CWWF RWWF RWFW RRFF CWFW RFWW CFWW RWFF CWFF RFRF CFCF RFWF CFWF WFWW WWFW WWFF WFRF WFCF CFFW RFFW FRWF FCWF FWWW WFWF RFFF CFFF FRFW FCFW WFFW FWWF FCFF FRFF WFFF FWFW FWFF FFWW FFCF FFRF FFWF FFFW FFFF"

                // good
                "129 WWWWW RRWWF RRWFW RWWWF CWWWF RRFWW RRWFF RRFRF RRFCF RWWFW CWWFW RRFWF WWWWF RWFWW RWWFF CWFWW CWWFF RWFRF RWFCF CWFRF CWFCF RRFFW WWWFW RFRWF RFCWF RFWWW RWFWF CFRWF CFCWF CFWWW CWFWF RRFFF WWFWW WWWFF RFRFW RFCFW RFWWF RWFFW CFRFW CFCFW CFWWF CWFFW WWFRF WWFCF WFRWF WFCWF WFWWW WWFWF RFRFF RFCFF RFWFW RWFFF CFRFF CFCFF CFWFW CWFFF FRWWF FCWWF FWWWW WFRFW WFCFW WFWWF WWFFW RFFWW RFWFF CFFWW CFWFF FRWFW FCWFW RFFRF RFFCF CFFRF CFFCF FWWWF WFRFF WFCFF WFWFW WWFFF RFFWF CFFWF FRFWW FRWFF FCFWW FCWFF FRFRF FRFCF FCFRF FCFCF FWWFW WFFWW WFWFF RFFFW CFFFW WFFRF WFFCF FRFWF FCFWF FWFWW FWWFF WFFWF FWFRF FWFCF RFFFF CFFFF FRFFW FCFFW FFRWF FFCWF FFWWW FWFWF WFFFW FRFFF FCFFF FFRFW FFCFW FFWWF FWFFW WFFFF FFRFF FFCFF FFWFW FWFFF FFFWW FFWFF FFFRF FFFCF FFFWF FFFFW FFFFF"

        );
        uint32_t num;
        input >> num;
        for (uint32_t i = 0; i < num; i++) {
            std::string line;
            input >> line;
            Operation op;
            ASSERT(line.size() == op.size(), "does not match sizes: >" + line + "<, " + std::to_string(op.size()));
            std::stringstream ss(line);
            ss >> op;
            pool.push_back(op);
        }
    }

    /*auto kek = [&](Operation op) {
        double s = 0;
        for (uint32_t d = 0; d < op.size(); d++) {
            s += (op[d] == Action::FW) * (op.size() - d) * 3;
            s -= (op[d] == Action::CR) * (op.size() - d);
            s -= (op[d] == Action::CCR) * (op.size() - d);
        }
        return s;
    };
    std::stable_sort(pool.begin(), pool.end(), [&](auto lhs, auto rhs) {
        return kek(lhs) < kek(rhs);
    });

    // add WWW
    {
        Operation op;
        for (uint32_t i = 0; i < op.size(); i++) {
            op[i] = Action::W;
        }
        auto it = std::find(pool.begin(), pool.end(), op);
        if (it != pool.end()) {
            pool.erase(it);
        }
        pool.insert(pool.begin(), op);
    }*/

    std::vector<Operation> result = pool;

    /*std::set<std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>>> visited;
    for (auto operation: pool) {
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
        Position p;
        std::set<std::pair<uint32_t, uint32_t>> visited_poses;
        visited_poses.insert({p.get_x(), p.get_y()});
        bool ok = true;
        for (uint32_t d = 0; d < DEPTH; d++) {
            p = p.simulate_action(operation[d]);
            positions[d] = {p.get_x(), p.get_y()};
            if (operation[d] == Action::FW && visited_poses.count(positions[d])) {
                ok = false;
            }
            visited_poses.insert(positions[d]);
        }
        std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>> kek = {p.get_dir(), positions};
        if (!visited.count(kek) && ok) {
            visited.insert(kek);
            result.push_back(operation);
        }
    }*/

    PRINT(
            Printer() << "Operations:\n"
                      << result.size() << ' ';
            for (auto operation
                 : result) {
                Printer() << operation << ' ';
            };
            Printer() << '\n';);
    //_exit(100);
    return result;
}

std::vector<Operation> &get_operations() {
    static std::vector<Operation> operations;
    return operations;
}

static inline std::vector<uint32_t> operation_depth;

uint32_t get_operation_depth(uint32_t index) {
    ASSERT(0 <= index && index < operation_depth.size(), "invalid index");
    ASSERT(operation_depth.size() == get_operations().size(), "unmatch sizes");
    return operation_depth[index];
}

std::vector<uint32_t> &get_operations_ids(uint32_t d) {
    static std::array<std::vector<uint32_t>, DEPTH + 1> data;
    ASSERT(3 <= d && d <= 5, "invalid d");
    return data[d];
}

void init_operations() {
    get_operations() = OperationsGenerator().get();

    auto get_operation_depth = [&](const Operation &op) {
        uint32_t d = op.size();
        for (; d > 0 && op[d - 1] == Action::W; d--) {
        }
        return d;
    };

    operation_depth.resize(get_operations().size());
    for (uint32_t i = 0; i < get_operations().size(); i++) {
        uint32_t d = get_operation_depth(get_operations()[i]);
        if (d <= 3) {
            operation_depth[i] = 3;
            get_operations_ids(3).push_back(i);
        } else if (d == 4) {
            operation_depth[i] = 4;
            get_operations_ids(4).push_back(i);
        } else if (d == 5) {
            operation_depth[i] = 5;
            get_operations_ids(5).push_back(i);
        } else {
            FAILED_ASSERT("unexpected depth");
        }
    }
}

std::ostream &operator<<(std::ostream &output, const Operation &op) {
    for (auto op: op) {
        if (op == Action::W) {
            output << 'W';
        } else if (op == Action::FW) {
            output << 'F';
        } else if (op == Action::CCR) {
            output << 'C';
        } else if (op == Action::CR) {
            output << 'R';
        } else if (op == Action::NA) {
            output << 'N';
        }
    }
    return output;
}

std::istream &operator>>(std::istream &input, Operation &op) {
    for (auto &op: op) {
        char c;
        input >> c;
        if (c == 'W') {
            op = Action::W;
        } else if (c == 'F') {
            op = Action::FW;
        } else if (c == 'C') {
            op = Action::CCR;
        } else if (c == 'R') {
            op = Action::CR;
        } else if (c == 'N') {
            op = Action::NA;
        } else {
            ASSERT(false, "invalid action");
        }
    }
    return input;
}
