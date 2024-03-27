#include "ActionModel.h"
#include "Grid.h"
#include "SharedEnv.h"
#include "ash/preprocess.hpp"
#include "ash/push_search.hpp"

#include <iostream>
#include <iomanip>
#include <map>

void print_state(std::ostream& out, const SharedEnvironment& env)
{
    std::map<int,int> cell_to_agent;
    for (int agid = 0; agid < env.num_of_agents; ++agid)
    {
        cell_to_agent[env.curr_states[agid].location] = agid;
    }

    for (int i = 0; i < env.map.size(); ++i)
    {
        if (i>0 && i%env.cols==0)
        {
            out << '\n';
        }

        if (cell_to_agent.count(i))
        {
            switch (env.curr_states[cell_to_agent[i]].orientation)
            {
                case 0: // EAST:
                    out << '>';
                    break;
                case 1: // SOUTH:
                    out << 'v';
                    break;
                case 2: // WEST:
                    out << '<';
                    break;
                case 3: // NORTH:
                    out << '^';
                    break;
                default:
                    break;
            }
        }
        else
        {
            out << (env.map[i]? '@' : ' ');
        }
    }
}

int main(int argc, char* argv[])
{
    using ash::Orientation;
    Orientation EAST = Orientation::EAST;
    Orientation NORTH = Orientation::NORTH;
    Orientation SOUTH = Orientation::SOUTH;
    Orientation WEST = Orientation::WEST;

    std::string input_map("../example_problems/warehouse.domain/maps/warehouse_small.map");

    Grid map(input_map);
    SharedEnvironment env;
    env.rows = map.rows;
    env.cols = map.cols;
    env.map = map.map;

    ActionModelWithRotate action_model(map);

    env.curr_states = std::vector<State>{
        State(ash::ravel(env, 13, 4), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 14, 4), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 15, 4), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 13, 5), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 14, 5), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 15, 5), -1, static_cast<int>(NORTH)),
        State(ash::ravel(env, 14, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 15, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 8, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 9, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 10, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 6), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 7), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 8), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 9), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 10), -1, static_cast<int>(EAST)),
        State(ash::ravel(env, 11, 11), -1, static_cast<int>(EAST)),
    };
    auto init_states_copy = env.curr_states;
    env.num_of_agents = env.curr_states.size();

    ash::Preprocessor preprocessor;
    preprocessor.init(env, true);
    std::cout << "Preprocessor init time: " << preprocessor.get_elapsed() << " s.\n"
              << "Memory (MB): " << preprocessor.get_memory_usage() << std::endl;

    ash::PushSearch search;
    search.init(env, ash::StrongHeuristic(preprocessor));
    search.sync();

    std::cout << std::boolalpha
              << search.is_valid_shift(0, {EAST, SOUTH, WEST, NORTH})
              << std::endl;

    std::cout << std::boolalpha
              << search.shift_delay(0, {EAST, SOUTH, WEST, NORTH})
              << std::endl;

    std::cout << std::boolalpha
              << search.is_valid_shift(0, {EAST})
              << std::endl;

    std::cout << std::boolalpha
              << search.is_valid_shift(0, {WEST})
              << std::endl;

    std::cout << std::boolalpha
              << search.is_valid_shift(0, {WEST, WEST})
              << std::endl;

    std::cout << std::boolalpha
              << search.is_valid_shift(8, {NORTH})
              << std::endl;

    std::cout << std::boolalpha
              << search.is_valid_shift(
                      8, {EAST, EAST, EAST, SOUTH, SOUTH, SOUTH, SOUTH, EAST})
              << std::endl;

    print_state(std::cout, env);
    std::cout << std::endl;

    std::cout << std::boolalpha
              << search.commit_shift(0, {EAST, SOUTH, WEST, NORTH})
              << std::endl;

    std::cout << std::boolalpha
              << search.commit_shift(
                      8, {EAST, EAST, EAST, SOUTH, SOUTH, SOUTH, SOUTH, EAST})
              << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        auto action_vector = search.step();
        env.curr_states = action_model.result_states(
                env.curr_states, action_vector);
        print_state(std::cout, env);
        std::cout << "\n---" << std::endl;
    }

    env.curr_states = init_states_copy;
    search.sync();
    print_state(std::cout, env);
    std::cout << "\n---" << std::endl;
    std::cout << std::boolalpha
              << search.is_valid_shift(0, {EAST, EAST, EAST})
              << std::endl;
    std::cout << search.commit_shift(0, {EAST, EAST, EAST}) << std::endl;
    std::cout << std::boolalpha
              << search.is_valid_shift(3, {NORTH})
              << std::endl;
    std::cout << search.commit_shift(3, {NORTH}) << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        auto action_vector = search.step();
        env.curr_states = action_model.result_states(
                env.curr_states, action_vector);
        print_state(std::cout, env);
        std::cout << "\n---" << std::endl;
    }
}

