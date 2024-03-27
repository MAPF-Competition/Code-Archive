#include "ash/astar_planner.hpp"
#include "ash/heuristic.hpp"

#include <algorithm>
#include <chrono>

#include <map>

constexpr int ASTAR_TIMEOUT_MS = 40;


ash::AstarPlanner::AstarPlanner(int win_length) : win_length(win_length)
{
}

void ash::AstarPlanner::initialize(const SharedEnvironment& env,
        int preprocess_time_limit)
{
    this->env = &env;
    preprocessor.init(env, true);
    astar.init(env, StrongHeuristic(preprocessor))
         .set_constraint_set(constraint_set)
         .set_timeout(ASTAR_TIMEOUT_MS*1e-3)
         .set_window_length(win_length);
    std::cout << "[ash::AstarPlanner::initialize] Preprocessing finished in "
              << preprocessor.get_elapsed()
              << " s. Memory usage (MB): " << preprocessor.get_memory_usage()
              << std::endl;
}

void ash::AstarPlanner::plan(int time_limit, std::vector<Action>& planned_actions)
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef typename Clock::duration Duration;

    auto start = std::chrono::high_resolution_clock::now();
    auto time_limit_d = std::chrono::duration_cast<Duration>(
            std::chrono::seconds(time_limit) -
            std::chrono::milliseconds(ASTAR_TIMEOUT_MS+10));

    step_plans();
    update_priorities();

    int num_of_successes = 0;
    for (; queue_pointer < env->num_of_agents; ++queue_pointer)
    {
        num_of_successes += update_agent_schedule(agent_queue[queue_pointer]);
        if (Clock::now()-start >= time_limit_d)
        {
            break;
        }
    }

    if (queue_pointer == env->num_of_agents)
    {
        queue_pointer = 0;
    }

    planned_actions.resize(env->num_of_agents);
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        planned_actions[agid] = plans[agid][1].action;
    }

    //std::map<int,std::vector<int>> collisions;
    //for (int agid = 0; agid < env->num_of_agents; ++agid)
    //{
        //int next_cell = get_neighbor(
                //*env, get_agent_state(agid), 
                //planned_actions[agid]).position.location;
        //collisions[next_cell].push_back(agid);
    //}
    //for (const auto&[cell, agents] : collisions)
    //{
        //if (agents.size() > 1)
        //{
            //std::cout << "collision in vertex " << cell << ", agents =";
            //for (int agid : agents)
            //{
                //std::cout << " " << agid;
            //}
            //std::cout << std::endl;
        //}
    //}

    double elapsed_s = std::chrono::duration<double>(
            Clock::now()-start).count();
    std::cout << "[ash::AstarPlanner::plan] Successful plans: " 
              << num_of_successes << ". Elapsed: " << elapsed_s
              << " s." << std::endl;
}

ash::TimedAgentPosition ash::AstarPlanner::get_agent_state(int agid) const
{
    return TimedAgentPosition{
        AgentPosition{
            env->curr_states[agid].location,
            static_cast<Orientation>(env->curr_states[agid].orientation)
        },
        env->curr_states[agid].timestep
    };
}

std::pair<int,int> ash::AstarPlanner::get_agent_goal(int agid) const
{
    if (env->goal_locations[agid].empty())
    {
        return {env->curr_states[agid].location, 0};
    }
    return env->goal_locations[agid].front();

}

void ash::AstarPlanner::reset_plan(int agid, const TimedAgentPosition& start)
{
    plans[agid].resize(win_length+1);
    plans[agid].front() = {NA, start};
    for (int t = 1; t <= win_length; ++t)
    {
        plans[agid][t] = {W, plans[agid][t-1].state};
        plans[agid][t].state.time += 1;
    }
}

void ash::AstarPlanner::build_initial_plans()
{
    plans.resize(env->num_of_agents);
    constraint_set.clear();
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        reset_plan(agid, get_agent_state(agid));
        constraint_set.reserve(plans[agid]);
    }
}

void ash::AstarPlanner::step_plans()
{
    if (plans.empty())
    {
        build_initial_plans();
        return;
    }

    constraint_set.clear();
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        plans[agid].erase(plans[agid].begin());
        plans[agid].push_back({W, plans[agid].back().state});
        plans[agid].back().state.time += 1;
        auto current_state = get_agent_state(agid);
        if (current_state != plans[agid].front().state)
        {
            std::cout << "WARNING! Agent " << agid
                      << "'s current state is not expected\n"
                      << "EXPECTED: " << plans[agid][0].state << '\n'
                      << "CURRENT: " << current_state << '\n';
            reset_plan(agid, current_state);
        }
        constraint_set.reserve(plans[agid]);
    }
}

void ash::AstarPlanner::update_priorities()
{
    if (agent_queue.empty())
    {
        agent_queue.resize(env->num_of_agents);
        std::iota(agent_queue.begin(), agent_queue.end(), 0);
        return;
    }
    std::vector<int> split1, split2;
    split1.reserve(env->num_of_agents);
    split2.reserve(env->num_of_agents);
    for (int i = 0; i < agent_queue.size(); ++i)
    {
        int agid = agent_queue[i];
        int goal_reveal_time = get_agent_goal(agid).second;
        if (goal_reveal_time == env->curr_timestep)
        {
            queue_pointer -= i<queue_pointer;
            split2.push_back(i);
        }
        else
        {
            split1.push_back(i);
        }
    }
    agent_queue = std::move(split1);
    agent_queue.insert(agent_queue.end(), split2.rbegin(), split2.rend());
    std::cout << "[ash::AstarPlanner::update_priorities] "
              << split2.size() << " task(s) achieved during this timestep."
              << std::endl;
}


bool ash::AstarPlanner::update_agent_schedule(int agid)
{
    constraint_set.release(plans[agid]);
    bool success = astar.set_start(plans[agid].front().state)
                        .set_goal(get_agent_goal(agid).first)
                        .search();
    if (success)
    {
        plans[agid] = astar.get_plan();
    }
    constraint_set.reserve(plans[agid]);
    return success;
}
