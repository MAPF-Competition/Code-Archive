//
// Created by take_ on 24-12-27.
//

#ifndef SORTINGSYSTEM_H
#define SORTINGSYSTEM_H

#include "BasicSystem.h"
#include "SortingGraph.h"
#include "SharedEnv.h"
#include "ActionModel.h"
#include "pibt.h"


class SortingSystem :
        public BasicSystem
{
public:
    int c; // param for induct assignment

    SortingSystem(const SortingGrid& G, MAPFSolver& solver);
    ~SortingSystem();

    void simulate(int simulation_time);
    void simulate_LoRR(int simulation_time, SharedEnvironment* env, vector<Action> & _actions);

private:
    const SortingGrid& G;

    // record usage of induct stations
    boost::unordered_map<int, int> drives_in_induct_stations; // induct location + #drives that intends to go to this induct station

    void initialize();
    void initialize_LoRR();

    // assign tasks
    void initialize_start_locations(); // 把起点加入path
    void random_initialize_LoRR_start_goal_locations(SharedEnvironment* env); // 把起点加入path
    void load_LoRR_start_goal_locations(SharedEnvironment* env); // 读取LoRR agent starts and goals
    void initialize_goal_locations();
    void update_goal_locations();

    [[nodiscard]] int assign_induct_station(int curr) const;
    [[nodiscard]] int assign_eject_station() const;
};


#endif // SORTINGSYSTEM_H
