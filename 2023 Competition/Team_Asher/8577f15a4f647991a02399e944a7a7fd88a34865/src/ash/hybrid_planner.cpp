#include "ash/heuristic.hpp"
#include "ash/hybrid_planner.hpp"

#include <algorithm>
#include <chrono>


constexpr int ASTAR_TIMELIMIT_MS = 30;

ash::HybridPlanner::HybridPlanner(int win_length, int max_steps_to_replan) :
    win_length(win_length), max_steps_to_replan(max_steps_to_replan)
{
}

void ash::HybridPlanner::initialize(const SharedEnvironment& env,
        int preprocess_time_limit)
{
    this->env = &env;
    preprocessor.init(env, true);
    push_search.init(env, StrongHeuristic(preprocessor));
    astar.init(env, StrongHeuristic(preprocessor))
         .set_timeout(ASTAR_TIMELIMIT_MS*1e-3)
         .set_constraint_set(constraint_set);
    std::cout << "[ash::HybridPlanner::initialize] Preprocessing finished in "
              << preprocessor.get_elapsed()
              << " s. Memory usage (MB): " << preprocessor.get_memory_usage()
              << std::endl;
}

void ash::HybridPlanner::plan(int time_limit_s, std::vector<Action>& planned_actions)
{
    start = Clock::now();
    time_limit = std::chrono::duration_cast<Duration>(
            std::chrono::seconds(time_limit_s) -
            std::chrono::milliseconds(ASTAR_TIMELIMIT_MS + 20));

    if (need_to_reschedule())
    {
        update_priorities();
        build_initial_plans();
    }
    else
    {
        step_window();
    }

    while (!timeout() && improve_solution())
    {
        improve_solution();
    }
    //improve_solution();

    planned_actions.resize(env->num_of_agents);
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        planned_actions[agid] = plans[agid][win_cursor+1].action;
    }

    std::cout << "[ash::HybridPlanner::plan] Elapsed: " << get_elapsed_seconds()
              << " s. " << std::endl;
}

ash::TimedAgentPosition ash::HybridPlanner::get_agent_state(int agent_id) const
{
    return TimedAgentPosition{
        AgentPosition{
            env->curr_states[agent_id].location,
            static_cast<Orientation>(env->curr_states[agent_id].orientation)
        },
        env->curr_states[agent_id].timestep
    };
}

std::pair<int,int> ash::HybridPlanner::get_agent_goal(int agent_id) const
{
    if (env->goal_locations[agent_id].empty())
    {
        return {env->curr_states[agent_id].location, 0};
    }
    return env->goal_locations[agent_id].front();

}

bool ash::HybridPlanner::need_to_reschedule() const
{
    if (plans.empty())
    {
        std::cout << "[ash::HybrydPlanner::need_to_reschedule]"
                     " Initial timestep" << std::endl;
        return true;
    }

    if (win_cursor >= max_steps_to_replan)
    {
        std::cout << "[ash::HybridPlanner::need_to_reschedule]"
                     " Exceeded max #steps to replan" << std::endl;
        return true;
    }

    if (win_cursor >= effective_win_length)
    {
        std::cout << "[ash::HybridPlanner::need_to_reschedule]"
                  << " Exceeded plan length" << std::endl;
        return true;
    }
    
    //int tasks_solved = 0;
    //for (int agid = 0; agid < env->num_of_agents; ++agid)
    //{
        //int task_reveal_time = get_agent_goal(agid).second;
        //tasks_solved += task_reveal_time == env->curr_timestep;
    //}
    //if (tasks_solved > 0)
    //{
        //std::cout << "[ash::HybridPlanner::need_to_reschedule] Rescheduled: "
                  //<< tasks_solved << " task(s) solved" << std::endl;
        //return true;
    //}
    
    
    return false;
}

void ash::HybridPlanner::reset_plan(int agid, const TimedAgentPosition& state)
{
    plans[agid].resize(win_length+1);
    plans[agid][win_cursor] = {NA, state};
    for (int t = win_cursor; t <= win_length; ++t)
    {
        plans[agid][t] = {W, plans[agid][t-1].state};
        plans[agid][t].state.time += 1;
    }
}

void ash::HybridPlanner::build_initial_plans()
{
    push_search.sync();
    plans.resize(env->num_of_agents);
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        plans[agid].clear();
        plans[agid].push_back({NA,get_agent_state(agid)});
    }
    constraint_set.clear();
    for (int t = 1; t<=win_length && !timeout(); ++t)
    {
        for (const auto& agent : agent_queue)
        {
            int agid = agent.agid;
            if (push_search.is_scheduled(agid) || push_search.has_reached_goal(agid))
            {
                continue;
            }
            if (push_search.search(agid))
            {
                if (!push_search.is_valid_shift(agid, push_search.get_path()))
                {
                    std::cout << "[ash::HybridPlanner::build_initial_plans] "
                                 "Something went horribly wrong" << std::endl;
                    continue;
                }
                push_search.commit_shift();
            }
        }
        auto actions = push_search.step();
        for (int agid = 0; agid < env->num_of_agents; ++agid)
        {
            Action action = actions[agid];
            auto next_state = get_neighbor(*env, plans[agid].back().state, action);
            plans[agid].push_back({action, next_state});
        }
    }

    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        constraint_set.reserve(plans[agid]);
    }

    effective_win_length = plans[0].size() - 1;
    win_cursor = 0;
    agent_cursor = 0;
    astar_updates = 0;

    std::cout << "[ash::HybridPlanner::build_initial_plans] Elapsed: "
              << get_elapsed_seconds() << " s. Built initial plan."
              << " Window length: " << effective_win_length << '/' << win_length
              << std::endl;
}

bool ash::HybridPlanner::improve_solution()
{
    int successful_astar_searches_ts = 0;
    int astar_updates_ts = 0;

    if (agent_cursor == env->num_of_agents)
    {
        agent_cursor = 0;
        astar_updates = 0;
    }

    for (; agent_cursor<env->num_of_agents && !timeout(); ++agent_cursor)
    {
        int agid = agent_queue[agent_cursor].agid;
        constraint_set.release(plans[agid], win_cursor);
        astar.set_window_length(effective_win_length - win_cursor)
             .set_start(plans[agid][win_cursor].state)
             .set_goal(get_agent_goal(agid).first);
        bool success = astar.search();
        if (success)
        {
            ++successful_astar_searches_ts;
            auto plan = astar.get_plan();
            bool updated = false;
            for (int t = win_cursor+1; t <= effective_win_length; ++t)
            {
                if (plans[agid][t].action != plan[t-win_cursor].action)
                {
                    updated = true;
                }
                plans[agid][t] = plan[t-win_cursor];
            }
            astar_updates_ts += updated;
        }
        constraint_set.reserve(plans[agid], win_cursor);
    }

    astar_updates += astar_updates_ts;

    std::cout << "[ash::HybridPlanner::improve_solution] Elapsed: "
              << get_elapsed_seconds() << " s. "
              << " Successful searches: {this timestep = "
              << successful_astar_searches_ts
              << ", this cycle = "
              << agent_cursor
              << "}. Updates: {this timestep = "
              << astar_updates_ts
              << ", this cycle = "
              << astar_updates << "}" << std::endl;

    return agent_cursor<env->num_of_agents || astar_updates>0;
}

void ash::HybridPlanner::step_window()
{
    ++win_cursor;
    //constraint_set.clear();
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        auto current_state = get_agent_state(agid);
        if (current_state != plans[agid][win_cursor].state)
        {
            std::cout << "[ash::HybridPlanner::step_window] WARNING! Agent "
                      << agid << "'s current state is not expected. "
                         "EXPECTED: " << plans[agid][win_cursor].state << ". "
                      << "CURRENT: " << current_state << '\n';
            reset_plan(agid, current_state);
        }
        //constraint_set.reserve(plans[agid]);
    }
}

void ash::HybridPlanner::update_priorities()
{
    agent_queue.resize(env->num_of_agents);
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        auto agent_state = get_agent_state(agid);
        int loc = agent_state.position.location;
        int task_age = agent_state.time - get_agent_goal(agid).second;
        agent_queue[agid].priority = task_age;        
        if (preprocessor.is_deadend(loc))
        {
            agent_queue[agid].priority += 5'000 +
                100*preprocessor.get_depth(loc);
        }
        //else if (preprocessor.get_degree(loc) == 2)
        //{
            //agent_queue[agid].priority += 5'000;
        //} 
        agent_queue[agid].agid = agid;
    }
    std::sort(agent_queue.begin(), agent_queue.end(), std::greater<AgentInfo>());
}

