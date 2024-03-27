#include <MAPFPlanner.h>
#include "nlohmann/json.hpp"

#include <limits.h>
#include <unistd.h>

std::string getexepath() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string path = std::string(result, (count > 0) ? count : 0);
    return path.substr(0, path.find_last_of("/"));
}

void MAPFPlanner::initialize(int preprocess_time_limit) {
    if (env->map_name == "Paris_1_256.map") {
        if (env->num_of_agents < 2000) {                // Instance #1: city_1500
            param.algorithm = Algorithm::Lazy;
            param.values["barriers"] = 0;        // 0
            param.values["path_bound"] = 140;    // 100
            param.values["congestion"] = 120;    // 60
            param.values["must_have_moved"] = 2; // 5
            // param.halt = true;
        } else {                                        // Instance #2: city_3000
            param.algorithm = Algorithm::Lazy;
            param.values["barriers"] = 1;        // best 2, prev 6, new 2
            param.values["path_bound"] = 70;     // best 40, prev -1, new -1
            param.values["congestion"] = 120;    // best 64, prev 90, new 25
            param.values["must_have_moved"] = 0; // best 2, prev 8, new 15
            // param.halt = true;
        }
    } else if (env->map_name == "random-32-32-20.map") {
        if (env->num_of_agents < 150) {                 // Instance #4 (random_100)
            param.algorithm = Algorithm::Long;          // Not confident on these parameters
            param.values["barriers"] = 0;               // 0
            param.values["congestion"] = 30;            // 50
            param.values["must_have_moved"] = 2;        // 2
            param.values["exp"] = 1.75;                  // 1.3
            param.values["sub"] = 2;                    // 0
            param.values["conf1"] = 3.0;                // 2
            param.values["conf2"] = 12.0;                // 4
            param.values["confadd"] = 0;              // 1.5
            param.values["confexp"] = 1.25;              // 2
            param.values["probexp"] = 1.0;              // ?
            param.values["slack"] = 0;                  // -0
            param.values["confdistexp"] = 0;            // 1?
            // param.halt = true;
        } else if (env->num_of_agents < 250) {          // Instance #3 (random_200)
            param.algorithm = Algorithm::Long;
            param.values["barriers"] = 0;        // 0
            param.values["congestion"] = 64;     // 64
            param.values["must_have_moved"] = 2; // 6
            param.values["exp"] = 1.75;          // 1.75
            param.values["sub"] = 1;             // 1
            param.values["conf1"] =  8.0;        // 12
            param.values["conf2"] = 12.0;        // 4
            param.values["confadd"] = 0;         // 0
            param.values["confexp"] = 1.0;       // 1
            param.values["probexp"] = 1.0;       // 1
            param.values["slack"] = 0;           // 1
            param.values["confdistexp"] = 0.25;  // .5
            // param.halt = true;
        } else if (env->num_of_agents < 450) {          // Instance #5 (random_400)
            param.algorithm = Algorithm::Lazy;
            param.values["barriers"] = 0;        // best 5, prev 0, new 0
            param.values["path_bound"] = 50;     // best 35, prev 45, new 45
            param.values["congestion"] = 15;     // best 40, prev 60, new 10
            param.values["must_have_moved"] = 0; // best 3, prev 10, new 5
            // param.halt = true;
        } else if (env->num_of_agents < 650) {          // Instance #7 (random_600)
            param.algorithm = Algorithm::Comb;
            param.values["barriers"] = 0;
            param.values["path_bound"] = 4;              // 4
            param.values["valuemul"] = 1.0;              // 1.0
            param.values["maxqueue"] = 12;               // 12
            param.values["confexp"] = 1.125;             // 1.125
            param.values["fwbonus"] = 0.3;               // .1
            param.values["achievebonus"] = 0.25;          // .3
            param.values["elite"] = .55;                  // .5
            param.values["greedy"] = 1;                  // 0
            param.values["nonelite"] = 768.0;
            // param.halt = true;
        } else {                                        // Instance #8 (random_800)
            param.algorithm = Algorithm::SAT; 
            param.values["barriers"] = 0;
            param.values["enqueue_bound"] = 100;
            param.values["agents_bound"] = 0.85;
            param.values["shots"] = 1;
            // param.halt = true;
        }
    } else if (env->map_name == "brc202d.map") {        // Instance #9 (game_6500)
        param.algorithm = Algorithm::Game;
        param.values["barriers"] = 1;
        param.values["time_bound"] = 0.97;
        param.values["enqueue_bound"] = 6;
        // param.halt = true;
    } else if (env->map_name == "sortation_large.map") { // Instance #6 (sortation_10K)
        param.algorithm = Algorithm::Lazy;
        param.values["barriers"] = 1;        // 5
        param.values["path_bound"] = -1;     // -1
        param.values["congestion"] = 60;     // 60
        param.values["must_have_moved"] = 2; // 2
        // param.halt = true;
    } else if (env->map_name == "warehouse_large.map") { // Instance #10 (warehouse_10K)
        param.algorithm = Algorithm::Lazy;
        param.values["barriers"] = 1;        // best 1, prev 3, new 1
        param.values["path_bound"] = 25;     // best -1, prev -1, new -1
        param.values["congestion"] = 120;    // best 35, prev 60, new 40
        param.values["must_have_moved"] = 0; // best 3, prev 10, new 0
        // param.halt = true;
    }

    import_parameters(param);

    if (param.algorithm == Algorithm::Long) { m_algo_long.init(env, param); }
    if (param.algorithm == Algorithm::Lazy) { s_algo_lazy.init(env, param); }
    if (param.algorithm == Algorithm::SAT)  { m_algo_sat.init(env, param);  }
    if (param.algorithm == Algorithm::Game) { m_algo_game.init(env, param); }
    if (param.algorithm == Algorithm::Comb) { m_algo_comb.init(env, param); }
}

void MAPFPlanner::plan(int time_limit, vector<Action> & actions) {
    cout << "### Timestep " << env->curr_timestep << " ###" << endl;
    if (param.algorithm == Algorithm::Long)  { m_algo_long.plan(actions);  }
    if (param.algorithm == Algorithm::Lazy)  { s_algo_lazy.plan(actions);  }
    if (param.algorithm == Algorithm::SAT)   { m_algo_sat.plan(actions);   }
    if (param.algorithm == Algorithm::Game)  { m_algo_game.plan(actions);  }
    if (param.algorithm == Algorithm::Comb)  { m_algo_comb.plan(actions);  }
}

void MAPFPlanner::import_parameters(Parameters & parameters) const {
    // std::string fn = "build/parameters.json";
    std::string fn = getexepath() + "/parameters.json";
    std::ifstream f(fn);
    if (!f.is_open()) {
        std::cout << "There is no parameters file " << fn << endl;
        return;
    }
    nlohmann::json data = nlohmann::json::parse(f);
    if (data.contains("planner")) {
        if (data["planner"] == "Long")  parameters.algorithm = Algorithm::Long;
        if (data["planner"] == "Lazy")  parameters.algorithm = Algorithm::Lazy;
        if (data["planner"] == "SAT")   parameters.algorithm = Algorithm::SAT;
        if (data["planner"] == "Game")  parameters.algorithm = Algorithm::Game;
        if (data["planner"] == "Comb")  parameters.algorithm = Algorithm::Comb;
    }

    for (auto& [key, value] : data.items()) {
        if (key != "planner") {
            parameters.values[(std::string)key] = static_cast<double>(value);
            // cout << key << "=" << parameters.values.at(key) << endl;
        }
    }
}
