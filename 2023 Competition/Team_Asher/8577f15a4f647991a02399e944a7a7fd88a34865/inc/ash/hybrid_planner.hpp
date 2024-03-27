#pragma once

#include "ash/astar.hpp"
#include "ash/preprocess.hpp"
#include "ash/push_search.hpp"
#include "ash/utils.hpp"

namespace ash
{


class HybridPlanner
{
public:

    HybridPlanner(int win_length = 50, int max_steps_to_replan = 5);

    void initialize(const SharedEnvironment& env, int preprocess_time_limit);

    void plan(int time_limit, std::vector<Action>& planned_actions);


private:

    // [type definitions]

    typedef std::chrono::high_resolution_clock Clock;
    typedef Clock::duration Duration;
    typedef Clock::time_point TimePoint;

    struct AgentInfo
    {
        int agid;
        int priority;

        bool operator>(const AgentInfo& other) const
        {
            if (priority == other.priority)
            {
                return agid > other.agid;
            }
            return priority > other.priority;
        }
    };

    // [helper methods]
    
    Duration get_elapsed() const
    {
        return Clock::now() - start;
    }

    double get_elapsed_seconds() const
    {
        return std::chrono::duration<double>(get_elapsed()).count();
    }

    bool timeout() const
    {
        return get_elapsed() >= time_limit;
    }

    TimedAgentPosition get_agent_state(int agent_id) const;

    std::pair<int,int> get_agent_goal(int agent_id) const;

    bool need_to_reschedule() const;

    void reset_plan(int agid, const TimedAgentPosition& state);

    void build_initial_plans();

    bool improve_solution();

    void step_window();

    void update_priorities();

    // [attributes]
     
    TimePoint start;
    Duration time_limit;
    
    const SharedEnvironment* env;
    Preprocessor preprocessor;
    PushSearch push_search;
    Astar<TimedAgentPosition> astar;
    ConstraintSet constraint_set;
    int win_length;
    int max_steps_to_replan;

    std::vector<AgentInfo> agent_queue;
    std::vector<Plan<TimedAgentPosition>> plans;
    int effective_win_length;
    int win_cursor;
    int agent_cursor;
    int astar_updates;

        
};

} // namespace ash
