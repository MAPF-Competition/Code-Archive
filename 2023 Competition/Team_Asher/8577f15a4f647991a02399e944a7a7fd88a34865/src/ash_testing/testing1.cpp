#include "Grid.h"
#include "SharedEnv.h"
#include "ash/preprocess.hpp"
#include "ash/astar.hpp"

#include <iostream>
#include <iomanip>
#include <map>
#include <random>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

po::variables_map vm;

void print_map(std::ostream& out, const SharedEnvironment& env,
        const ash::AgentPosition& start, int goal,
        const ash::Plan<ash::TimedAgentPosition>& plan)
{
    std::map<int,ash::Orientation> locations_in_plan;
    for (const auto& action_state : plan)
    {
        const auto& state = action_state.state;
        locations_in_plan[state.position.location] = state.position.orientation;
    }
    for (int i = 0; i < env.map.size(); ++i)
    {
        if (i>0 && i%env.cols==0)
        {
            out << '\n';
        }
        if (i == start.location)
        {
            out << 'S';
        }
        else if (i == goal)
        {
            out << 'G';
        }
        else if (locations_in_plan.find(i) != locations_in_plan.end())
        {
            switch (locations_in_plan[i])
            {
                case ash::Orientation::EAST:
                    out << '>';
                    break;
                case ash::Orientation::SOUTH:
                    out << 'v';
                    break;
                case ash::Orientation::WEST:
                    out << '<';
                    break;
                case ash::Orientation::NORTH:
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
    po::options_description desc("Allowed options");

    desc.add_options()("help", "produce help message")
        ("mt", "Multithreaded map preprocessing")
        ("inputFile", po::value<std::string>()->required(),
         "input map file")
        ("seed", po::value<int>()->default_value(42),
         "rng seed")
        ("windowLength", po::value<int>()->default_value(-1),
         "search window length")
        ("heuristic", po::value<std::string>()->default_value("strong"),
         "heuristic used fo A*");

    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    po::notify(vm);

    auto input_map = vm["inputFile"].as<std::string>();
    int seed = vm["seed"].as<int>();
    int win_length = vm["windowLength"].as<int>();
    bool multithreading = vm.count("mt");

    Grid map(input_map);
    SharedEnvironment env;
    env.rows = map.rows;
    env.cols = map.cols;
    env.map = std::move(map.map);
    std::cout << std::endl;

    ash::Preprocessor preprocessor;

    ash::Heuristic heuristic;
    auto heurname = vm["heuristic"].as<std::string>();
    if (heurname == "strong")
    {
        heuristic = ash::StrongHeuristic(preprocessor);
    }
    else if (heurname == "weak")
    {
        heuristic = ash::WeakHeuristic(env);
    }
    else
    {
        std::cerr << "unknown heuristic: "  << heurname << '\n';
        return -1;
    }

    std::cout << "Using " << heurname << " heuristic" << std::endl;
    std::cout << "Multithreading: " << std::boolalpha << multithreading << std::endl;

    preprocessor.init(env, multithreading);
    std::cout << "Preprocessor init time: " << preprocessor.get_elapsed() << " s.\n"
              << "Memory (MB): " << preprocessor.get_memory_usage() << std::endl;



    std::mt19937 rng(seed);
    const auto& free_locations = preprocessor.get_free_locations();
    std::uniform_int_distribution<int> unif_loc(
            0, free_locations.size()-1);
    std::uniform_int_distribution<int> unif_orientation(0, 3);

    int start_location = free_locations[unif_loc(rng)];
    int goal = free_locations[unif_loc(rng)];
    while (start_location == goal)
    {
        goal = free_locations[unif_loc(rng)];
    }
    
    ash::Orientation start_orientation =
        ash::ALL_ORIENTATIONS[unif_orientation(rng)];

    ash::TimedAgentPosition start = {
        ash::AgentPosition{start_location, start_orientation}, 0};

    ash::Astar<ash::TimedAgentPosition> astar;
    astar.init(env, heuristic)
         .set_start(start)
         .set_goal(goal)
         .set_window_length(win_length);

    bool success = astar.search();

    if (!success)
    {
        std::cout << "Unsuccessful search" << std::endl;
    }
    else
    {
        std::cout << "Elapsed (s): "
                  << astar.get_elapsed_time() << '\n'
                  << "Plan cost: "
                  << astar.get_plan_cost() << '\n'
                  << "#Expanded nodes: "
                  << astar.get_number_of_expanded_nodes() << '\n'
                  << "#Generated nodes: "
                  << astar.get_number_of_generated_nodes() << '\n'
                  << "Effective branching factor: "
                  << astar.get_effective_branching_factor() << std::endl;
        auto plan = astar.get_plan();
        if (env.cols < 200)
        {
            print_map(std::cout, env, start.position, goal, plan);
            std::cout << std::endl;
        }
        std::cout << ash::to_string(plan) << std::endl;
    }
}

