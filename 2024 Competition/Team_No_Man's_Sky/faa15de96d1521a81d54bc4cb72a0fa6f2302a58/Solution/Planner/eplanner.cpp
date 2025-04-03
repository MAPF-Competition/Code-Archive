#include <Planner/eplanner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <Planner/PIBT/pibts.hpp>
#include <Planner/PIBT/pmps.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

std::pair<std::vector<Action>, std::vector<uint32_t>> EPlanner::plan(TimePoint end_time) {
    ETimer timer;

    std::vector<Action> plan(env->num_of_agents, Action::W);

#ifdef ENABLE_PIBTS
    {
        // (score, actions)
        using ItemType = std::tuple<double, std::vector<Action>
                                    // time, desires, changes, step, log_str
                                    ,
                                    int, std::vector<uint32_t>, std::vector<int64_t>, uint32_t>;

        constexpr uint32_t THR = THREADS;
        std::vector<std::vector<ItemType>> results_pack(THR);

        PIBTS main_pibt_solver(get_robots_handler().get_robots(), end_time);

        auto do_work = [&](uint32_t thr, uint64_t seed) {
            ETimer timer;
            PIBTS pibt = main_pibt_solver;
            pibt.solve(seed);
            auto time = timer.get_ms();
            results_pack[thr].emplace_back(pibt.get_score(), pibt.get_actions(), time, pibt.get_desires(),
                                           pibt.get_changes(), pibt.step);
        };

        static Randomizer rnd;
        std::vector<std::thread> threads(THR);
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr] = std::thread(do_work, thr, rnd.get());
        }
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr].join();
        }

        std::vector<ItemType> results;
        for (uint32_t thr = 0; thr < THR; thr++) {
            for (auto &item: results_pack[thr]) {
                results.emplace_back(std::move(item));
            }
        }

        double best_score = -1e300;
        std::vector<uint32_t> best_desires;
        std::sort(results.begin(), results.end(), [&](const auto &lhs, const auto &rhs) {
            return std::get<0>(lhs) > std::get<0>(rhs);
        });

        PRINT(Printer() << "RESULTS(" << results.size() << "): ";);

        for (const auto &[score, actions, time, desires, changes, steps]: results) {
            PRINT(Printer() << "(" << score << ", " << time << ", " << steps << ") ";);
            if (best_score < score) {
                best_score = score;
                best_desires = desires;
                plan = actions;
            }
        }

        PRINT(Printer() << "\nbest: " << best_score << '\n';);


        PRINT(
                /*static std::vector operation_matrix(get_operations().size(), std::vector<int64_t>(get_operations().size()));
                static std::vector<int> prev_desired(env->num_of_agents, 0);
                static std::vector<uint32_t> total_desires(get_operations().size());
                static std::vector<int64_t> total_changes(get_operations().size());
                for (uint32_t r = 0; r < env->num_of_agents; r++) {
                    if (results.empty()) {
                        break;
                    }
                    uint32_t desired = std::get<3>(results[0])[r];
                    int64_t change = std::get<4>(results[0])[r];
                    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
                    total_desires[desired]++;
                    total_changes[desired] += change;

                    operation_matrix[prev_desired[r]][desired]++;
                    prev_desired[r] = desired;
                } Printer()
                << "Desires:\n";
                std::vector<std::tuple<int64_t, uint32_t>> kek;
                for (uint32_t d = 0; d < total_desires.size(); d++) {
                    Printer() << d << ' ' << get_operations()[d] << ' ' << total_desires[d] << ' ' << total_changes[d] << '\n';
                    if (d != 0) {
                        kek.emplace_back(total_desires[d]//total_changes[d]
                                         ,
                                         d);
                    }
                } std::sort(kek.begin(), kek.end());
                for (auto [score, d]
                     : kek) {
                    Printer() << get_operations()[d] << ' ';
                } Printer()
                << '\n';*/

                /*if (env->curr_timestep == 999) {
                    std::vector<std::tuple<uint64_t, uint32_t, uint32_t>> pool;
                    Printer() << "Operation matrix:\n";
                    for (uint32_t d = 0; d < get_operations().size(); d++) {
                        for (uint32_t k = 0; k < get_operations().size(); k++) {
                            Printer() << operation_matrix[d][k] << ' ';
                            pool.emplace_back(operation_matrix[d][k], d, k);
                        }
                        Printer() << '\n';
                    }

                    std::sort(pool.begin(), pool.end());
                    for (auto [count, d, k]: pool) {
                        Printer() << get_operations()[d] << "->" << get_operations()[k] << ": " << count << '\n';
                    }
                }*/
        );

        PRINT(Printer() << "Planner: " << timer << '\n';);

        return {plan, best_desires};
    }
#endif

#ifdef ENABLE_PMPS

    {
        constexpr uint32_t THR = 8;
        // (score, actions)
        using ItemType = std::tuple<double, std::vector<Action>>;
        std::vector<ItemType> results(THR);

        auto do_work = [&](uint32_t thr, uint64_t seed) {
            PMPS solver(get_robots_handler().get_robots(), end_time, env->curr_timestep);
            solver.solve(seed);
            results[thr] = {solver.get_score(), solver.get_actions()};
        };

        static Randomizer rnd(228);
        std::vector<std::thread> threads(THR);
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr] = std::thread(do_work, thr, rnd.get());
        }
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr].join();
        }
        std::sort(results.begin(), results.end(), [&](const auto &lhs, const auto &rhs) {
            return std::get<0>(lhs) > std::get<0>(rhs);
        });
        plan = std::get<1>(results[0]);

        PRINT(Printer() << "RESULTS(" << results.size() << "): ";);
        for (const auto &[score, actions]: results) {
            PRINT(Printer() << score << ' ';);
        }
        PRINT(Printer() << '\n';);


        PRINT(Printer() << "Planner: " << timer << '\n';);
        return {plan, {}};
    }

#endif
    FAILED_ASSERT("kek");
    throw std::exception();
}
