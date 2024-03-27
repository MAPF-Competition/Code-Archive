#include "Grid.h"
#include "SharedEnv.h"
#include "ash/preprocess.hpp"
#include "ash/push_search.hpp"

#include <iostream>
#include <iomanip>

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
#if 1
    env.goal_locations = {
        {{ash::ravel(env, 15, 10), 0}},
        {{ash::ravel(env, 16, 10), 0}},
        {{ash::ravel(env, 17, 10), 0}},
        {{ash::ravel(env, 18, 10), 0}},
        {{ash::ravel(env, 19, 10), 0}},
        {{ash::ravel(env, 20, 10), 0}},
        {{ash::ravel(env, 21, 10), 0}},
        {{ash::ravel(env, 22, 10), 0}},
        {{ash::ravel(env, 23, 10), 0}},
        {{ash::ravel(env, 24, 10), 0}},
        {{ash::ravel(env, 25, 10), 0}},
        {{ash::ravel(env, 26, 10), 0}},
        {{ash::ravel(env, 27, 10), 0}},
        {{ash::ravel(env, 28, 10), 0}},
        {{ash::ravel(env, 29, 10), 0}},
        {{ash::ravel(env, 30, 10), 0}},
        {{ash::ravel(env, 31, 10), 0}}
    };
#else
    env.goal_locations = {
        {{ash::ravel(env, 14, 4), 0}},
        {{ash::ravel(env, 14, 5), 0}},
        {{ash::ravel(env, 17, 10), 0}},
        {{ash::ravel(env, 13, 4), 0}},
        {{ash::ravel(env, 13, 5), 0}},
        {{ash::ravel(env, 20, 10), 0}},
        {{ash::ravel(env, 21, 10), 0}},
        {{ash::ravel(env, 22, 10), 0}},
        {{ash::ravel(env, 23, 10), 0}},
        {{ash::ravel(env, 24, 10), 0}},
        {{ash::ravel(env, 25, 10), 0}},
        {{ash::ravel(env, 26, 10), 0}},
        {{ash::ravel(env, 27, 10), 0}},
        {{ash::ravel(env, 28, 10), 0}},
        {{ash::ravel(env, 29, 10), 0}},
        {{ash::ravel(env, 30, 10), 0}},
        {{ash::ravel(env, 31, 10), 0}}
    };
#endif
    env.num_of_agents = env.curr_states.size();

    ash::Preprocessor preprocessor;
    preprocessor.init(env, true);
    std::cout << "Preprocessor init time: " << preprocessor.get_elapsed() << " s.\n"
              << "Memory (MB): " << preprocessor.get_memory_usage() << std::endl;

    ash::PushSearch search;
    search.init(env, ash::StrongHeuristic(preprocessor));
    search.sync();

    print_state(std::cout, env);
    std::cout << std::endl;

    for (int agid = 0; agid < env.num_of_agents; ++agid)
    {
        if (search.is_scheduled(agid))
        {
            std::cout << "Agent " << agid << " is already scheduled." << std::endl;
            continue;
        }
        bool found = search.search(agid);

        if (!found)
        {
            std::cout << "Couldn't schedule agent " << agid << '.' << std::endl;
            continue;
        }

        int delay = search.commit_shift();
        auto path = search.get_path();
        std::cout << "Agent " << agid << ": path = [";
        bool first = true;
        for (auto dir : path)
        {
            if (!first)
            {
                std::cout << ", ";
            }
            std::cout << ash::to_string(dir);
            first = false;
        }
        std::cout << "], cost = " << search.get_cost()
                  << ", elapsed = " << search.get_elapsed() << " s, delay = "
                  << delay << " timestep(s)" << std::endl;
        std::cout << std::boolalpha << search.is_valid_shift(agid, path) << std::endl;
    }
    for (int i = 0; i < 3; ++i)
    {
        auto action_vector = search.step();
        env.curr_states = action_model.result_states(
                env.curr_states, action_vector);
        print_state(std::cout, env);
        std::cout << "\n---" << std::endl;
    }



    //for (auto&&[agid,plan] : push_search.get_plans())
    //{
        //std::cout << "Agent " << agid << "'s plan: " << ash::to_string(plan) << std::endl;
    //}

    //std::cout << rtable.shift_agents(pusher_loc, path) << std::endl;

    //print_map(std::cout, env, rtable);
    //std::cout << std::endl;

}

