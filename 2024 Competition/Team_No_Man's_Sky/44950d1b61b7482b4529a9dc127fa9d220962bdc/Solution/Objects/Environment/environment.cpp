#include <Objects/Environment/environment.hpp>

#include <Scheduler/journey_graph.hpp>
#include <Scheduler/scheduler.hpp>
#include <Tools/tools.hpp>
#include <settings.hpp>

#include "heuristics.h"
#include "planner.h"

#include <thread>

namespace DefaultPlanner {
    extern DefaultPlanner::TrajLNS trajLNS;
}

using namespace DefaultPlanner;

void init_default_heuristic(SharedEnvironment &env) {
#if defined(ENABLE_DEFAULT_PLANNER) || defined(ENABLE_DEFAULT_SCHEDULER)
    ETimer timer;
    init_heuristics(&env);

    auto do_work = [&](uint32_t thr) {
        for (uint32_t pos = thr + 1; pos < get_map().get_size(); pos += THREADS) {
            if (get_map().is_free(pos)) {
                HeuristicTable &ht = global_heuristictable[pos - 1];
                init_heuristic(ht, &env, pos - 1);

#ifdef ENABLE_DEFAULT_PLANNER

                Neighbors *ns = &trajLNS.neighbors;

                std::vector<int> neighbors;
                int cost, diff;
                while (!ht.open.empty()) {
                    HNode curr = ht.open.front();
                    ht.open.pop_front();

                    getNeighborLocs(ns, neighbors, curr.location);

                    for (int next: neighbors) {
                        cost = curr.value + 1;
                        diff = curr.location - next;

                        //assert(next >= 0 && next < get_map().get_size() - 1);
                        //set current cost for reversed direction

                        if (cost >= ht.htable[next])
                            continue;

                        ht.open.emplace_back(next, 0, cost);
                        ht.htable[next] = cost;
                    }
                }
#else// ENABLE_DEFAULT_SCHEDULER
                for (uint32_t dir = 0; dir < 4; dir++) {
                    uint32_t source = get_graph().get_node(Position(pos, dir));
                    for (uint32_t target = 1; target < get_map().get_size(); target++) {
                        if (get_map().is_free(target)) {
                            ht.htable[target - 1] = std::min(ht.htable[target - 1], static_cast<int>(get_hm().get(source, target)));
                        }
                    }
                }
#endif
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    PRINT(Printer() << "init_default_heuristic: " << timer << '\n';);
#endif
}

void init_d2path() {
#ifdef ENABLE_DEFAULT_PLANNER
    ETimer timer;
    auto do_work = [&](uint32_t thr) {
        for (uint32_t r = thr; r < trajLNS.env->num_of_agents; r += THREADS) {
            init_dist_2_path(trajLNS.traj_dists[r], trajLNS.env, trajLNS.trajs[r]);
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    PRINT(Printer() << "init_d2path: " << timer << '\n';);
#endif
}

void init_environment(SharedEnvironment &env) {
    static bool already_init = false;
    if (already_init) {
        return;
    }
    already_init = true;

    // update map type
    {
        if (env.map_name == "warehouse_large.map") {
            get_map_type() = MapType::WAREHOUSE;
        } else if (env.map_name == "sortation_large.map") {
            get_map_type() = MapType::SORTATION;
        } else if (env.map_name == "brc202d.map") {
            get_map_type() = MapType::GAME;
        } else if (env.map_name == "Paris_1_256.map") {
            get_map_type() = MapType::CITY;
        } else if (env.map_name == "random-32-32-20.map") {
            get_map_type() = MapType::RANDOM;
        } else {
            get_map_type() = MapType::NONE;
            FAILED_ASSERT("undefined map");
        }
    }

    //if(get_map_type() != MapType::RANDOM){
    //    return;
    //}

    get_map() = Map(env);
#ifdef ENABLE_GG_SOLVER
    // read GraphGuidance
    {
        std::ifstream input("Tmp/gg" + std::to_string(get_unique_id()));
        input >> get_gg();
    }
    // read operations weights
    /*{
        std::ifstream input("Tmp/opw" + std::to_string(get_unique_id()));
        uint32_t k = 0;
        input >> k;
        get_operations_weights().resize(k);
        for (int &x: get_operations_weights()) {
            input >> x;
        }
    }*/
#else
    Printer().get() = std::ofstream("printer.txt");
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());
    // warehouse bad guidance map
    if (get_map_type() == MapType::RANDOM
        // TODO: GuidanceMap для warehouse довольно плох
        //|| get_map_type() == MapType::WAREHOUSE
    ) {
        //Printer() << "GuidanceMap\n";
        get_gg() = GraphGuidance(get_guidance_map());
    } else {
        //Printer() << "without GuidanceMap\n";
        get_gg() = GraphGuidance(env);
    }

    // init operations weights
    /*{
        std::stringstream input("45 -7 38 -18 11 72 -34 72 -4 52 31 38 17 -32 7 45 24 48 -35 49 19 -75 0 25 -54 -26 2 -9 -71 -22 -2 38 29 18 -72 16 46 0 9 1 21 -7 15 33 19 -1");
                //("17 -200 20 20 10 60 70 30 10 40 40 20 20 50 50 110 130 150");
        uint32_t k = 0;
        input >> k;
        get_operations_weights().resize(k);
        for (int &x: get_operations_weights()) {
            input >> x;
        }
    }*/
#endif
    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    //get_rhm() = RawHeuristicMatrix(get_graph());
    get_dhm() = DynamicHeuristicMatrix(get_map(), get_graph());
    get_dhmr() = DHMR(get_graph());
    //get_wmap() = WorkloadMap(get_map(), get_graph());
    //get_jg() = JG::JourneyGraph(&env);
    init_operations();
    get_omap() = OperationsMap(get_graph(), get_operations());

    //ASSERT(get_operations_weights().size() == get_operations().size(), "unmatch sizes: " + std::to_string(get_operations_weights().size()) + "!=" + std::to_string(get_operations().size()));

    init_default_heuristic(env);

    // generate random agents
    /*Randomizer rnd(74124);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 6500; i++){
        uint32_t pos = 0;
        while(true){
            pos = rnd.get(1, get_map().get_size() - 1);
            if(get_map().is_free(pos) && !S.count(pos)){
                break;
            }
        }
        S.insert(pos);
    }
    for(uint32_t pos : S){
        pos--;
        output << pos << '\n';
    }
    output.flush();
    std::exit(0);*/
}

void update_environment(SharedEnvironment &env) {
    double task_reveal = env.task_pool.size() * 1.0 / env.num_of_agents;
    // true everywhere
    //ASSERT(std::abs(task_reveal - 1.5) < 1e-3, "invalid task reveal");

    // update test type
    {
        if (get_map_type() == MapType::RANDOM) {
            if (env.num_of_agents == 100) {
                get_test_type() = TestType::RANDOM_1;
            } else if (env.num_of_agents == 200) {
                get_test_type() = TestType::RANDOM_2;
            } else if (env.num_of_agents == 400) {
                get_test_type() = TestType::RANDOM_3;
            } else if (env.num_of_agents == 700) {
                get_test_type() = TestType::RANDOM_4;
            } else if (env.num_of_agents == 800) {
                get_test_type() = TestType::RANDOM_5;
            }
        } else if (get_map_type() == MapType::CITY) {
            // FAILED_ASSERT("TODO");
            // CITY-02:
            // 3000 failed
            // 3500 ok
            // ASSERT(env.num_of_agents < 3500, "invalid num of agents");
        } else if (get_map_type() == MapType::GAME) {
            get_test_type() = TestType::GAME;
        } else if (get_map_type() == MapType::WAREHOUSE) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::WAREHOUSE;
            }
        } else if (get_map_type() == MapType::SORTATION) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::SORTATION;
            }
        } else {
            FAILED_ASSERT("invalid map");
        }
    }

    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == -1) {
        get_robots_handler() = RobotsHandler(env.num_of_agents);
        init_d2path();
    }
    get_robots_handler().update(env);
    if (prev_timestep_updated == env.curr_timestep) {
        // for planner
        //get_dhmr().update(env.curr_timestep, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
        return;
    }
    prev_timestep_updated = env.curr_timestep;
    //get_dhm().update(env, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
    //get_dhmr().update(env.curr_timestep, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
    //get_wmap().update(env, get_now());
}

std::vector<Action> &get_myplan() {
    static std::vector<Action> plan;
    return plan;
}
