#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

using json = nlohmann::json;

uint32_t call(const std::string &test, int steps_num, uint32_t test_id) {
    std::cout << "call(" + std::to_string(test_id) + "): " << std::flush;
    ETimer timer;

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    //std::system("mkdir Tmp");
    int ret_code = std::system(
            ("./cmake-build-release-remote-host/lifelong -i " + test + " -o Tmp/test" + std::to_string(test_id) +
             ".json -s " + std::to_string(steps_num) + " -t 110 -p 1000000000 --unique_id " + std::to_string(test_id) +
             " > Tmp/log" + std::to_string(test_id) + ".txt").c_str());

    ASSERT(ret_code == 0, "invalid ret code");

    json data;
    std::ifstream input("Tmp/test" + std::to_string(test_id) + ".json");
    uint32_t value = 0;
    try {
        data = json::parse(input);

        if (data["numEntryTimeouts"] != 0) {
            std::cout << "\nENTRY TIMEOUT" << std::endl;
        }
        if (data["numPlannerErrors"] != 0) {
            std::cout << "\nPLANNER ERROR" << std::endl;
        }
        if (data["numScheduleErrors"] != 0) {
            std::cout << "\nSCHEDULER ERROR" << std::endl;
        }
        value = data["numTaskFinished"];
    } catch (const json::parse_error &error) {
        std::cerr << "Message: " << error.what() << std::endl;
    }
    std::cout << value << ", " << timer << std::endl;
    return value;
}

std::vector<std::tuple<std::string, int>> tests = {
        /*{"Data2023/city.domain/MR23-I-01.json", 1500},
        {"Data2023/city.domain/MR23-I-02.json", 3500},
        {"Data2023/random.domain/MR23-I-03.json", 500},
        {"Data2023/random.domain/MR23-I-04.json", 500},
        {"Data2023/random.domain/MR23-I-05.json", 1000},
        {"Data2023/warehouse.domain/MR23-I-06.json", 5000},
        {"Data2023/random.domain/MR23-I-07.json", 1000},
        {"Data2023/random.domain/MR23-I-08.json", 2000},
        {"Data2023/game.domain/MR23-I-09.json", 5000},
        {"Data2023/warehouse.domain/MR23-I-10.json", 5000},*/

        /*{"example_problems/random.domain/random_32_32_20_100.json", 600},
        {"example_problems/random.domain/random_32_32_20_200.json", 600},
        {"example_problems/random.domain/random_32_32_20_400.json", 800},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000},
        {"example_problems/random.domain/random_32_32_20_800.json", 2000},*/

/*
без DHM
было
call(0): 2344, 81.1565s
call(1): 4283, 81.3892s
call(2): 5295, 81.6076s
call(3): 4630, 81.7764s
call(4): 5251, 82.0631s
call(5): 4317, 82.2454s
call(6): 4035, 82.4149s
call(7): 2839, 82.5492s
total: 32994
стало
call(0): 2389, 81.0131s
call(1): 4358, 81.2507s
call(2): 5605, 81.4233s
call(3): 5678, 81.5824s
call(4): 5963, 81.8965s
call(5): 5242, 81.9681s
call(6): 4612, 82.1581s
call(7): 3230, 82.3145s
total: 37077
call(0): 2372, 81.0982s
call(1): 4384, 81.3203s
call(2): 5737, 81.4955s
call(3): 5958, 81.6592s
call(4): 5404, 81.9323s
call(5): 5218, 82.1242s
call(6): 4761, 82.2865s
call(7): 3169, 82.4019s
total: 37003
call(0): 2375, 80.9836s
call(1): 4401, 81.1999s
call(2): 5589, 81.3695s
call(3): 6248, 81.5102s
call(4): 5819, 81.7269s
call(5): 5337, 81.9073s
call(6): 4666, 82.0644s
call(7): 3200, 82.1649s
total: 37635

call(0): 2511, 80.9182s
call(1): 4690, 81.1642s
call(2): 5895, 81.3342s
call(3): 6270, 81.5159s
call(4): 5645, 81.7643s
call(5): 5126, 81.9115s
call(6): 4644, 82.0414s
call(7): 3241, 82.1724s
total: 38022

call(0): 2507, 80.9987s
call(1): 4577, 81.1965s
call(2): 5853, 81.4203s
call(3): 6547, 81.6333s
call(4): 5515, 81.8766s
call(5): 4673, 82.106s
call(6): 4629, 82.2622s
call(7): 3227, 82.4486s
total: 37528

PIBTS_STEPS=5000
call(0): 2370, 6.529s
call(1): 4372, 10.7662s
call(2): 5579, 20.747s
call(3): 5669, 46.6419s
call(4): 5850, 61.9223s
call(5): 5126, 124.2s
call(6): 4496, 205.95s
call(7): 3411, 428.444s
total: 36873

DHM
call(0): 2388, 81.1487s
call(1): 4221, 81.3912s
call(2): 5418, 81.6306s
call(3): 6251, 81.7905s
call(4): 5314, 82.0746s
call(5): 4870, 82.2144s
call(6): 4099, 82.3886s
call(7): 3014, 82.4356s
total: 35575
*/
        {"example_problems/random.domain/random_32_32_20_100.json", 1000},
        {"example_problems/random.domain/random_32_32_20_200.json", 1000},
        {"example_problems/random.domain/random_32_32_20_300.json", 1000},
        {"example_problems/random.domain/random_32_32_20_400.json", 1000},
        {"example_problems/random.domain/random_32_32_20_500.json", 1000},
        {"example_problems/random.domain/random_32_32_20_600.json", 1000},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000},
        {"example_problems/random.domain/random_32_32_20_800.json", 1000},
};

uint32_t call() {
    uint32_t res = 0;
    for (uint32_t i = 0; i < tests.size(); i++) {
        res += call(std::get<0>(tests[i]), std::get<1>(tests[i]), i);
    }
    return res;
}

int main() {
    uint32_t total = call();
    std::cout << "total: " << total << std::endl;
}

/*
MAPFPLANNER
10k
call(0): 21740, 303.916s
call(1): 34562, 299.623s
call(2): 31075, 292.259s
call(3): 26337, 286.827s
call(4): 24899, 280.435s
call(5): 23453, 273.153s
total: 162066

1k
call(0): 2169, 30.7614s
call(1): 3447, 30.0941s
call(2): 3622, 29.5492s
call(3): 3317, 28.8547s
call(4): 2797, 28.4491s
call(5): 2358, 27.6499s
total: 17710
*/

/*
PIBT2

10k
call(0): 22747, 21.7433s
call(1): 32159, 26.0835s
call(2): 28078, 44.2924s
call(3): 24978, 68.7657s
call(4): 22338, 108.811s
call(5): 20170, 147.452s
total: 150470

1k
call(0): 2264, 1.77612s
call(1): 3309, 2.44252s
call(2): 3017, 3.35352s
call(3): 2701, 4.90992s
call(4): 2577, 5.88574s
call(5): 2237, 8.23335s
total: 16105
*/

/*
PIBTS:
call(0): 2409, 30.8636s
call(1): 3974, 30.8771s
call(2): 4073, 31.1857s
call(3): 3624, 31.2429s
call(4): 3212, 31.5072s
call(5): 2592, 31.7745s
total: 19884

call(0): 23963, 306.801s
call(1): 38591, 309.525s
call(2): 33340, 313.238s
call(3): 23042, 315.141s
call(4): 19438, 318.031s
call(5): 18218, 322.4s
total: 156592

500 solver steps
call(0): 2420, 6.16338s
call(1): 3962, 5.70953s
call(2): 4118, 12.6022s
call(3): 3592, 27.3051s
call(4): 3081, 59.3801s
call(5): 2594, 92.1525s
total: 19767

PIBT2 + 500 solver steps
call(0): 2404, 6.25056s
call(1): 3864, 10.0511s
call(2): 4106, 33.6654s
call(3): 3570, 46.5616s
call(4): 3100, 75.3258s
call(5): 2643, 133.274s
total: 19687

PIBT2 + 500 solver steps + SHEDULER_TRICK
call(0): 2448, 14.288s
call(1): 3875, 21.5622s
call(2): 4245, 38.3575s
call(3): 3639, 111.126s
call(4): 3218, 133.659s
call(5): 2794, 208.623s
total: 20219

PIBT2 + 500 solver steps + SHEDULER_TRICK + DHM
call(0): 2449, 39.5195s
call(1): 4133, 38.4003s
call(2): 4814, 42.247s
call(3): 4317, 59.6616s
call(4): 3565, 87.4388s
call(5): 3042, 133.383s
total: 22320

32 cores
call(0): 2458, 11.3463s
call(1): 4172, 17.9982s
call(2): 4775, 31.7107s
call(3): 4306, 109.144s
call(4): 3624, 201.867s
call(5): 3014, 297.687s
total: 22349

call(0): 2480, 15.5587s
call(1): 4212, 23.0197s
call(2): 4705, 34.664s
call(3): 4099, 72.9627s
call(4): 3329, 110.47s
call(5): 2885, 144.093s
total: 21710

improve
call(0): 2505, 13.3046s
call(1): 4359, 22.1051s
call(2): 5117, 34.5822s
call(3): 4293, 61.9291s
call(4): 3472, 117.993s
call(5): 3030, 155.069s
total: 22776

call(0): 2500, 13.713s
call(1): 4156, 22.5615s
call(2): 5031, 36.1933s
call(3): 5235, 47.8588s
call(4): 4342, 70.1563s
call(5): 3452, 115.241s
total: 24716


call(0): 2517, 15.1908s
call(1): 4185, 22.887s
call(2): 5081, 35.1256s
call(3): 5272, 43.8822s
call(4): 4321, 62.9133s
call(5): 3485, 97.3295s
total: 24861

PIBTS_STEPS = 500
call(0): 2520, 17.0641s
call(1): 4171, 24.8905s
call(2): 5060, 32.5171s
call(3): 5292, 35.2482s
call(4): 4399, 56.4403s
call(5): 3408, 94.0458s
total: 24850

PIBTS_STEPS = 0
call(0): 2334, 9.62642s
call(1): 3660, 19.5243s
call(2): 4123, 36.5378s
call(3): 3822, 34.6181s
call(4): 3384, 53.9861s
call(5): 2627, 83.8888s
total: 19950

32 threads PIBTS
call(0): 2505, 130.993s
call(1): 4179, 131.167s
call(2): 5139, 131.314s
call(3): 5488, 131.506s
call(4): 4254, 131.796s
call(5): 3368, 132.011s
total: 24933
*/
