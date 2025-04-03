#include <fstream>

#include <Objects/Environment/graph_guidance.hpp>
#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>

int main() {
    Printer().get() = std::ofstream("gg_log");

    /*{
        std::ofstream output("best_gg");
        GraphGuidance gg(32, 32);
        for (uint32_t pos = 0; pos < 32 * 32 + 1; pos++) {
            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    gg.set(pos, dir, action, 2);
                }
            }
        }
        output << gg;
        output.flush();
        return 0;
    }*/
    //std::string params = "-i ./example_problems/random.domain/random_32_32_20_300.json -o test.json -s 2000 -t 100000000 -p 100000000";
    /*std::string params;
    for (int i = 1; i < argc; i++) {
        params += argv[i];
        params += ' ';
    }
    std::cout << "params: " << params << '\n';*/

    GraphGuidance gg;
    {
        std::ifstream input("best_gg");
        input >> gg;
    }

    std::vector<int> opw;
    {
        std::ifstream input("best_opw");
        uint32_t n = 0;
        input >> n;
        opw.resize(n);
        for (int &x: opw) {
            input >> x;
        }
    }

    GraphGuidanceSolver ggs(gg, opw);
    ggs.solve();
}

// если брать веса ~10
// finish(3): 3973.11, 4156, 99.7245s
// если взять GuidanceMap, где все веса 2, 4, 6, но в основном 2
// finish(0): 5740.84, 5821, 27.6075s (1 поток, поэтому быстрее)
// finish(7): 5830.81, 5911, 52.9073s
//finish(21): 5992.92, 6066, 48.8274s

// с 3800 без моего гуиданса
//finish(28): 5213.59, 5292, 54.191s
//finish(18): 5305.98, 5386, 53.7566s

//finish(0): 3739.84, 3875, 56.037s