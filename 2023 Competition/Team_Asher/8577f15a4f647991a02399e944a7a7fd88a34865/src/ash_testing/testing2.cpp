#include "Grid.h"
#include "SharedEnv.h"

#include "ash/heuristic.hpp"
#include "ash/preprocess.hpp"
#include "ash/astar.hpp"

#include <iostream>
#include <map>

constexpr int win_length = 11;

void print_map(std::ostream& out, const SharedEnvironment& env,
        const ash::TimedAgentPosition agents[2])
{
    for (int i = 0; i < env.map.size(); ++i)
    {
        if (i>0 && i%env.cols==0)
        {
            out << '\n';
        }
        if (env.map[i])
        {
            out << '@';
        }
        else
        {
            bool agent_in_location = false;
            for (int j = 0; j < 2; ++j)
            {
                if (agents[j].position.location == i)
                {
                    agent_in_location = true;
                    switch (agents[j].position.orientation)
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
            }
            if (!agent_in_location)
            {
                out << ' ';
            }
        }
    }
}

void solve_and_print_stats(std::ostream& out, ash::Astar<ash::TimedAgentPosition>& astar)
{
    bool success = astar.search();
    if (!success)
    {
        std::cout << "Search unsucessful" << std::endl;
    }
    else
    {
        std::cout << "Elapsed (s): "
                  << astar.get_elapsed_time() << '\n'
                  << "Plan estimated cost: "
                  << astar.get_plan_cost() << '\n'
                  << "#Expanded nodes: "
                  << astar.get_number_of_expanded_nodes() << '\n'
                  << "#Generated nodes: "
                  << astar.get_number_of_generated_nodes() << '\n'
                  << "Effective branching factor: "
                  << astar.get_effective_branching_factor()
                  << std::endl;
    }
}

auto build_initial_plan(const ash::TimedAgentPosition& state, int length)
{
    ash::Plan<ash::TimedAgentPosition> plan{{NA, state}};
    for (int t = 1; t <= length; ++t)
    {
        plan.push_back({W, plan.back().state});
        plan.back().state.time += 1;
    }
    return plan;
}

int main(int argc, char* argv[])
{
    SharedEnvironment env;
    env.rows = 11;
    env.cols = 3;
    env.map = std::vector<int>{1,1,1,1,0,1,1,0,1,1,0,1,1,0,1,0,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,1,1};

    ash::Preprocessor preprocessor;
    preprocessor.init(env, true);

    ash::ConstraintSet constraint_set;

    ash::Astar<ash::TimedAgentPosition> astar;
    astar.init(env, ash::StrongHeuristic(preprocessor))
         .set_constraint_set(constraint_set);

    ash::TimedAgentPosition agent_positions[2] = {
        {{28, ash::Orientation::EAST},0},
        //{{10, ash::Orientation::EAST},0}
        {{4, ash::Orientation::EAST},0}
    };
    int agent_goals[2] = {15, 28};
    //int agent_goals[2] = {4, 28};
    ash::Plan<ash::TimedAgentPosition> plans[2] = {
        build_initial_plan(agent_positions[0], win_length),
        build_initial_plan(agent_positions[1], win_length)
    };

    constraint_set.reserve(plans[1]);
    astar.set_start(agent_positions[0])
         .set_goal(agent_goals[0])
         .set_window_length(win_length);
    solve_and_print_stats(std::cout, astar);
    plans[0] = astar.get_plan();

    constraint_set.clear();
    constraint_set.reserve(plans[0]);

    astar.set_start(agent_positions[1])
         .set_goal(agent_goals[1]); 
    solve_and_print_stats(std::cout, astar);
    plans[1] = astar.get_plan();

    constraint_set.clear();
    constraint_set.reserve(plans[0]);
    constraint_set.reserve(plans[1]);
    std::cout << constraint_set << std::endl;

    for (int t = 0; t <= win_length; ++t)
    {
        if (t > 0)
        {
            std::cout << "\n\n---\n" << std::endl;
        }
        agent_positions[0] = plans[0][t].state;
        agent_positions[1] = plans[1][t].state;
        print_map(std::cout, env, agent_positions);
    }
    std::cout << std::endl;

    std::cout << ash::to_string(plans[0]) << std::endl;
    std::cout << ash::to_string(plans[1]) << std::endl;
}

