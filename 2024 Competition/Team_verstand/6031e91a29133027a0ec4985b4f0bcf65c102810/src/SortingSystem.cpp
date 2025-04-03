//
// Created by take_ on 24-12-27.
//
#include "SortingSystem.h"
#include <stdlib.h>
#include "PBS.h"
#include <boost/tokenizer.hpp>
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"


SortingSystem::SortingSystem(const SortingGrid& G, MAPFSolver& solver): BasicSystem(G, solver), c(8), G(G) {}


SortingSystem::~SortingSystem() {}


void SortingSystem::initialize_start_locations()
{
    int N = G.size();
    std::vector<bool> used(N, false);

    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
    for (int k = 0; k < num_of_drives;)
    {
        int loc = rand() % N;
        if (G.types[loc] != "Obstacle" && !used[loc])
        {
            int orientation = -1;
            if (consider_rotation)
            {
                orientation = rand() % 4;
            }
            starts[k] = State(loc, 0, orientation);
            paths[k].emplace_back(starts[k]);
            used[loc] = true;
            finished_tasks[k].emplace_back(loc, 0);
            k++;
        }
    }
}

void SortingSystem::random_initialize_LoRR_start_goal_locations(SharedEnvironment* env)
{
    int N = G.size();
    // cout << "N: " << N << endl;
    std::vector<bool> used(N, false);

    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
    for (int k = 0; k < num_of_drives;)
    {
        int loc = rand() % N;
        if (G.types[loc] != "Obstacle" && !used[loc])
        {
            int orientation = -1;
            if (consider_rotation)
            {
                orientation = rand() % 4;
            }
            starts[k] = State(loc, 0, orientation);
            paths[k].emplace_back(starts[k]);
            used[loc] = true;
            finished_tasks[k].emplace_back(loc, 0);
            k++;
        }
    }

    for (int k = 0; k < num_of_drives;)
    {
        int loc = rand() % N;
        if (G.types[loc] != "Obstacle" && !used[loc])
        {
            int goal = loc;
            goal_locations[k].emplace_back(goal, 0);
            used[loc] = true;
            k++;
        }
    }
}

void SortingSystem::load_LoRR_start_goal_locations(SharedEnvironment* env)
{
    // load LoRR start and goal location
    for (int k = 0; k < num_of_drives;k++)
    {
        starts[k] = env->curr_states[k];
        starts[k].timestep = 0;
        // 转换方向. 因为RHCR对方向的定义和静思啊系统不一样
        if(starts[k].orientation == 3)
        {
            starts[k].orientation = 1;
        }
        else if(starts[k].orientation == 1)
        {
            starts[k].orientation = 3;
        }

        paths[k].emplace_back(starts[k]);
    }

    for (int k = 0; k < num_of_drives;k++)
    {
        goal_locations[k] = env->goal_locations[k];
        goal_locations[k][0].second = 0;
    }
}

void SortingSystem::initialize_goal_locations()
{
    if (hold_endpoints || useDummyPaths)
        return;
    // Choose random goal locations
    // a close induct location can be a goal location, or
    // any eject locations can be goal locations
    // Goal locations are not necessarily unique
    for (int k = 0; k < num_of_drives; k++)
    {
        int goal;
        if (k % 2 == 0) // to induction
        {
            goal = assign_induct_station(starts[k].location);
            drives_in_induct_stations[goal]++;
        }
        else // to ejection
        {
            goal = assign_eject_station();
        }
        goal_locations[k].emplace_back(goal, 0);
    }
}


void SortingSystem::update_goal_locations()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        pair<int, int> curr(paths[k][timestep].location, timestep); // current location

        pair<int, int> goal; // The last goal location
        if (goal_locations[k].empty())
        {
            goal = curr;
        }
        else
        {
            goal = goal_locations[k].back();
        }
        int min_timesteps = G.get_Manhattan_distance(curr.first, goal.first); // cannot use h values, because graph edges may have weights  // G.heuristics.at(goal)[curr];
        min_timesteps = max(min_timesteps, goal.second);
        while (min_timesteps <= simulation_window)
            // The agent might finish its tasks during the next planning horizon
        {
            // assign a new task
            int next;
            if (G.types[goal.first] == "Induct")
            {
                next = assign_eject_station();
            }
            else if (G.types[goal.first] == "Eject")
            {
                next = assign_induct_station(curr.first);
                drives_in_induct_stations[next]++; // the drive will go to the next induct station
            }
            else
            {
                std::cout << "ERROR in update_goal_function()" << std::endl;
                std::cout << "The fiducial type should not be " << G.types[curr.first] << std::endl;
                exit(-1);
            }
            goal_locations[k].emplace_back(next, 0);
            min_timesteps += G.get_Manhattan_distance(next, goal.first); // G.heuristics.at(next)[goal];
            min_timesteps = max(min_timesteps, goal.second);
            goal = make_pair(next, 0);
        }
    }
}


int SortingSystem::assign_induct_station(int curr) const
{
    int assigned_loc;
    double min_cost = DBL_MAX;
    for (auto induct : drives_in_induct_stations)
    {
        double cost = G.heuristics.at(induct.first)[curr] + c * induct.second;
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = induct.first;
        }
    }
    return assigned_loc;
}


int SortingSystem::assign_eject_station() const
{
    int n = rand() % G.ejects.size();
    boost::unordered_map<std::string, std::list<int> >::const_iterator it = G.ejects.begin();
    std::advance(it, n);
    int p = rand() % it->second.size();
    auto it2 = it->second.begin();
    std::advance(it2, p);
    return *it2;
}

void SortingSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();

    // for (; timestep < simulation_time; timestep += simulation_window)
    {
        std::cout << "Timestep " << timestep << std::endl;

        update_start_locations();
        update_goal_locations();

        cout << "before plan: " << paths[0].size() << endl;

        solve();

        // 计算出了总长94的路径, 排除了前5段的冲突(planning_window), 只需要返回第一段(simulation_window)
        cout << "after plan: " << paths[0].size() << endl;

        // move drives
        /*
        auto new_finished_tasks = move();
        std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;
         */

        // update tasks
        /*// LORR不需要
        for (auto task : new_finished_tasks)
        {
            int id, loc, t;
            std::tie(id, loc, t) = task;
            finished_tasks[id].emplace_back(loc, t);
            num_of_tasks++;
            if (G.types[loc] == "Induct")
            {
                drives_in_induct_stations[loc]--; // the drive will leave the current induct station
            }
        }
         */

        /*
        if (congested())
        {
            cout << "***** Too many traffic jams ***" << endl;
            break;
        }
         */
    }

    update_start_locations();
    std::cout << std::endl << "Done!" << std::endl;
    save_results();
}

void SortingSystem::simulate_LoRR(int simulation_time, SharedEnvironment* env, vector<Action> & _actions)
{
    // cout << 269 << endl;
    std::cout << "*** Simulating LoRR " << seed << " ***" << std::endl;
    // this->simulation_time = simulation_time;
    initialize_LoRR();

    // random_initialize_LoRR_start_goal_locations(env);
    load_LoRR_start_goal_locations(env);

    // for (; timestep < simulation_time; timestep += simulation_window)
    {
        std::cout << "Timestep " << timestep << std::endl;

        // update_start_locations();
        // update_goal_locations();

        cout << "95 goal: " << env->goal_locations[95][0].first << endl;

        cout << "95 before " << paths[95].size()
        << ": " << paths[95][0].location << " " << paths[95][0].orientation << endl;

        solve();

        // 计算出了总长94的路径, 排除了前5段的冲突(planning_window), 只需要返回第一段(simulation_window)
        cout << " 95 after "<< paths[95].size() << ": "
        << paths[95][0].location << " " << paths[95][0].orientation << " "
        << paths[95][1].location << " " << paths[95][1].orientation << " "
        << paths[95][2].location << " " << paths[95][2].orientation  << endl;

        _actions.resize(env->num_of_agents);
        for(int i=0;i<env->num_of_agents;i++)
        {
            if(paths[i].size() > 1)
            {
                _actions[i] = DefaultPlanner::getAction(paths[i][0], paths[i][1]);
            }
            else
            {
                _actions[i] = Action::W;
            }
        }

        // move drives
        /*
        auto new_finished_tasks = move();
        std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;
         */

        // update tasks
        /*// LORR不需要
        for (auto task : new_finished_tasks)
        {
            int id, loc, t;
            std::tie(id, loc, t) = task;
            finished_tasks[id].emplace_back(loc, t);
            num_of_tasks++;
            if (G.types[loc] == "Induct")
            {
                drives_in_induct_stations[loc]--; // the drive will leave the current induct station
            }
        }
         */

        /*
        if (congested())
        {
            cout << "***** Too many traffic jams ***" << endl;
            break;
        }
         */
    }

    // update_start_locations();
    std::cout << "Simulation Done!" << std::endl;
    // save_results();
}

void SortingSystem::initialize()
{
    initialize_solvers();

    starts.resize(num_of_drives);
    goal_locations.resize(num_of_drives);
    paths.resize(num_of_drives);
    finished_tasks.resize(num_of_drives);

    for (const auto induct : G.inducts)
    {
        drives_in_induct_stations[induct.second] = 0;
    }

    bool succ = load_records(); // continue simulating from the records
    if (!succ)
    {
        timestep = 0;
        succ = load_locations();
        if (!succ)
        {
            cout << "Randomly generating initial locations" << endl;
            initialize_start_locations();
            initialize_goal_locations();
        }
    }

    // initialize induct station counter
    for (int k = 0; k < num_of_drives; k++)
    {
        // goals
        int goal = goal_locations[k].back().first;
        if (G.types[goal] == "Induct")
        {
            drives_in_induct_stations[goal]++;
        }
        else if (G.types[goal] != "Eject")
        {
            std::cout << "ERROR in the type of goal locations" << std::endl;
            std::cout << "The fiducial type of the goal of agent " << k << " is " << G.types[goal] << std::endl;
            exit(-1);
        }
    }
}

void SortingSystem::initialize_LoRR()
{
    initialize_solvers();

    starts.resize(num_of_drives);
    goal_locations.resize(num_of_drives);
    paths.resize(num_of_drives);
    finished_tasks.resize(num_of_drives);

    timestep = 0;
    // cout << "Randomly generating LoRR initial locations" << endl;
    // initialize_goal_locations();

    cout << "Initialization Done" << endl;
}