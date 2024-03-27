#pragma once

#include "ash/constraint_set.hpp"
#include "ash/heuristic.hpp"
#include "ash/utils.hpp"

#include <array>
#include <chrono>
#include <queue>
#include <unordered_map>

namespace ash
{
	
template<class S, class Ret>
using EnableIfTimed =
	std::enable_if_t<std::is_same_v<S, TimedAgentPosition>, Ret>;

template<class S>
class Astar
{
    public:
        static_assert(std::is_same_v<S,int> ||
                      std::is_same_v<S,AgentPosition> ||
                      std::is_same_v<S,TimedAgentPosition>);

        // [constructors]
        
        Astar();

        // [setters]
        
        Astar& init(const SharedEnvironment& env, Heuristic heur)
        {
            this->env = &env;
            this->heur = HeuristicAdapter<S>(std::move(heur));
            return *this;
        }

        Astar& set_constraint_set(const ConstraintSet& cs)
        {
            this->constraint_set = &cs;
            return *this;
        }
        
        Astar& set_timeout(double timeout)
        {
            std::int64_t timeout_ns = timeout*1e9;
            this->timeout = std::chrono::duration_cast<Duration>(
                    std::chrono::nanoseconds(timeout_ns));
            return *this;
        }

        Astar& set_start(const S& start)
        {
            this->start = start;
            return *this;
        }

        Astar& set_goal(int goal)
        {
            this->goal = goal;
            return *this;
        }
        
        
        template<class Ret = Astar&>
        EnableIfTimed<S, Ret> set_window_length(int win_length)
        {
			this->win_length = win_length;
			return *this;
		}

        // [getters]
         
        double get_elapsed_time() const
        {
            return elapsed;
        }

        int get_plan_cost() const
        {
            return plan_cost;
        }

        int get_number_of_expanded_nodes() const
        {
            return number_of_expanded_nodes;
        }

        int get_number_of_generated_nodes() const
        {
            return number_of_generated_nodes;
        }

        bool success() const
        {
            return plan_cost < INF;
        }

        bool has_timed_out() const
        {
            return timed_out;
        }

        double get_effective_branching_factor() const;

        Plan<S> get_plan() const;

        // [other]

        bool search();

    protected:

        // [type definitions]
    

        typedef std::chrono::high_resolution_clock Clock;
        typedef typename Clock::duration Duration;
        
        struct ActionStateCost
        {
            ActionType<S> action;
            S state;
            int cost;
        };
 
        typedef std::array<
            ActionStateCost,
            ActionTypeTrait<S>::NUMBER_OF_ACTIONS
        > Neighbors;
        
        struct OpenNode
        {
            S state;
            int f;
            int h;
        };

        struct VisitedNode
        {
            S previous_state;
            ActionType<S> previous_action;
            int g;
            bool closed;
        };

        struct OpenCompare
        {
            bool operator()(const OpenNode& lhs,
                            const OpenNode& rhs) const noexcept
            {
                return lhs.f>rhs.f || (lhs.f==rhs.f && lhs.h>rhs.h);
            }
        };

        typedef std::unordered_map<S,VisitedNode> VisitedSet;
        typedef std::priority_queue<OpenNode,std::vector<OpenNode>,
                OpenCompare> OpenSet;

        // [auxiliary methods]
         
        bool is_goal(const S& state) const;

        int transition_cost(const S& state, const S& neighbor) const;

        int get_neighbors(Neighbors& neighbors, const S& tpos) const;
        
        void setup();

        void expand_windowless();

        template<class Ret = void>
        EnableIfTimed<S, Ret> expand_with_window();

        void generate_neighbors(const S& state, int state_g);

        // [Astar parameters]
        const SharedEnvironment* env;
        const ConstraintSet* constraint_set;
        HeuristicAdapter<S> heur;
        Duration timeout;
        int win_length;

        // [algorithm data structures]
        VisitedSet visited;
        OpenSet open;
        S start;
        int goal;

        // [plan info]
        S plan_end;
        int plan_cost;
        bool early_cutoff;
        bool timed_out;

        // [statistics]
        double elapsed;
        int number_of_expanded_nodes;
        int number_of_generated_nodes;
};


} // ash namespace
