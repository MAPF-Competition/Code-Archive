#pragma once
#include "ash/heuristic.hpp"
#include "ash/preprocess.hpp"

namespace ash
{

class PushSearch
{
    public:

        PushSearch();

        void init(const SharedEnvironment& env, Heuristic heuristic);

        void sync();

        bool is_valid_shift(int agid, const std::vector<Orientation>& path) const;

        int shift_delay(int agid, const std::vector<Orientation>& path) const;
        
        int shift_delay() const;

        std::vector<int> get_affected_agents(int agid,
                const std::vector<Orientation>& path) const;

        std::vector<int> get_affected_agents() const;

        int commit_shift(int agid, const std::vector<Orientation>& path);

        int commit_shift();

        std::vector<Action> step();

        bool search(int agid);

        bool successful_search() const
        {
            return opt_cost < std::numeric_limits<int>::max();
        }

        const std::vector<Orientation>& get_path() const
        {
            return opt_path;
        }

        std::vector<Action> get_action_vector() const;

        int get_cost() const
        {
            return opt_cost;
        }

        bool is_scheduled(int agid) const
        {
            return agent_table[agid].is_scheduled();
        }

        bool has_reached_goal(int agid) const
        {
            return agent_table[agid].has_reached_goal;
        }

        double get_elapsed() const
        {
            return elapsed;
        }

    private:

        // [Type definitions]
 
        struct CellInfo
        {
            // < 0 if no agent here
            int current_agent;

            // < 0 if not reserved
            int reserved_to;

            bool is_occupied() const
            {
                return current_agent >= 0;
            }

            bool is_reserved() const
            {
                return reserved_to >= 0;
            }
        };

        struct AgentInfo
        {
            AgentPosition current_state;
            AgentPosition target_state;

            // if scheduled, this is the number of moves until the agent
            // adopts its reserved state
            int remaining_moves;

            bool has_reached_goal;

            bool is_scheduled() const
            {
                return remaining_moves > 0;
            }
        };

        // [Private methods]
        
        int get_goal(int agid) const
        {
            if (env->goal_locations[agid].empty())
            {
                return -1;
            }
            return env->goal_locations[agid].front().first;
        }
        
        bool is_pushable(int loc) const;

        bool is_valid_end_of_shift(int pusher_id, int loc) const;

        int availability(int loc) const;

        Action get_next_action(int agid) const;

        int transition_cost(int u, int v, Orientation dir) const;

        bool search_impl();

        std::pair<std::vector<Orientation>,int> search_impl(
                int push_loc, Orientation push_dir);


        // [Attributes]
        
        // Constant data structures
        const SharedEnvironment* env;
        Heuristic heuristic;

        // bookkeeping
        std::vector<AgentInfo> agent_table;
        std::vector<CellInfo> cell_table;

        // results of last search
        int pusher;
        std::vector<Orientation> opt_path;
        int opt_cost;

        // statistics
        double elapsed;
};


} // namespace ash
