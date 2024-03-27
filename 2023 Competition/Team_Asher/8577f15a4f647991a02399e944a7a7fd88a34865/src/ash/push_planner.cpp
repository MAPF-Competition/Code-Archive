#include "ash/heuristic.hpp"
#include "ash/push_planner.hpp"

#include <algorithm>
#include <chrono>


ash::PushPlanner::PushPlanner()
{
}

void ash::PushPlanner::initialize(const SharedEnvironment& env,
        int preprocess_time_limit)
{
    this->env = &env;
    preprocessor.init(env, true);
    search.init(env, StrongHeuristic(preprocessor));
    std::cout << "[ash::PushPlanner::initialize] Preprocessing finished in "
              << preprocessor.get_elapsed()
              << " s. Memory usage (MB): " << preprocessor.get_memory_usage()
              << std::endl;
}

void ash::PushPlanner::plan(int time_limit, std::vector<Action>& planned_actions)
{
    typedef std::chrono::high_resolution_clock Clock;

    auto start = std::chrono::high_resolution_clock::now();

    update_priorities();
    search.sync();

    int num_of_scheduled_agents = 0;

    //blocked.assign(blocked.size(), false);
    for (int agid : agent_queue)
    {
        if (search.is_scheduled(agid))
        {
            ++num_of_scheduled_agents;
            continue;
        }
        else if (search.search(agid))
        {
            if (!search.is_valid_shift(agid, search.get_path()))
            {
                std::cout << "Something went horribly wrong" << std::endl;
            }
            search.commit_shift();
            ++num_of_scheduled_agents;
        }
        else
        {
            //blocked[agid] = true;
        }
    }

    planned_actions = search.step();

    auto elapsed = Clock::now() - start;
    double elapsed_s = std::chrono::duration<double>(elapsed).count();
    std::cout << "[ash::PushPlanner::plan] Elapsed: " << elapsed_s
              << " s. " << num_of_scheduled_agents << " agents scheduled."
              << std::endl;
}

ash::TimedAgentPosition ash::PushPlanner::get_agent_state(int agent_id) const
{
    return TimedAgentPosition{
        AgentPosition{
            env->curr_states[agent_id].location,
            static_cast<Orientation>(env->curr_states[agent_id].orientation)
        },
        env->curr_states[agent_id].timestep
    };
}

std::pair<int,int> ash::PushPlanner::get_agent_goal(int agent_id) const
{
    if (env->goal_locations[agent_id].empty())
    {
        return {env->curr_states[agent_id].location, 0};
    }
    return env->goal_locations[agent_id].front();

}

//void ash::PushPlanner::build_initial_plans()
//{
    //plans.resize(env->num_of_agents);
    ////hard_constraints.clear();
    //for (int i = 0; i < env->num_of_agents; ++i)
    //{
        //auto initial_state = get_agent_state(i);
        //plans[i] = Plan{{NA, initial_state}};
        ////hard_constraints.reserve(plans[i], i);
    //}
//}

//void ash::PushPlanner::step_plans()
//{
    //if (plans.empty())
    //{
        //build_initial_plans();
        //return;
    //}

    ////hard_constraints.clear();
    //for (int i = 0; i < env->num_of_agents; ++i)
    //{ 
        //step_plan(plans[i]);
        //auto current_state = get_agent_state(i);
        //if (current_state != plans[i][0].second)
        //{
            //std::cout << "WARNING! Agent " << i
                      //<< "'s current state is not expected\n"
                      //<< "EXPECTED: " << plans[i][0].second << '\n'
                      //<< "CURRENT: " << current_state << '\n';
            //plans[i] = Plan{{NA, current_state}};
        //}
        ////hard_constraints.reserve(plans[i], i);
    //}
//}

void ash::PushPlanner::update_priorities()
{
    if (agent_queue.empty())
    {
        agent_queue.resize(env->num_of_agents);
        //blocked.assign(env->num_of_agents, false);
        std::iota(agent_queue.begin(), agent_queue.end(), 0);
        return;
    }
    std::vector<int> split1, split2;
    split1.reserve(env->num_of_agents);
    split2.reserve(env->num_of_agents);
    int tasks_done = 0;
    for (int i = 0; i < agent_queue.size(); ++i)
    {
        int agid = agent_queue[i];
        int goal_reveal_time = get_agent_goal(agid).second;
        bool task_completed = goal_reveal_time==env->curr_timestep;
        tasks_done += task_completed;
        if (task_completed)
        //if (blocked[agid] || task_completed)
        {
            split2.push_back(agid);
        }
        else
        {
            split1.push_back(agid);
        }
    }
    agent_queue = std::move(split1);
    agent_queue.insert(agent_queue.end(), split2.rbegin(), split2.rend());
    /*
    std::cout << "[ash::PushPlanner::update_priorities] "
              << split2.size() << " agents were assigned a lower priority. "
              << tasks_done << " task(s) completed in the last timestep."
              << std::endl;
    */
    std::cout << "[ash::PushPlanner::update_priorities] "
              << tasks_done << " task(s) completed in the last timestep."
              << std::endl;
}

