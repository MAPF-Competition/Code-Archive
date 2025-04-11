#pragma once
#include "TrajLNS.h"
#include "SharedEnv.h"
#include <omp.h>
#include "util/HeuristicTable.h"
#include "flow.h"

namespace MyPlanner {
class Dist2PathHeuristicTable {
public:
    TrajLNS & lns;
    SharedEnvironment * env;
    std::shared_ptr<std::vector<float> > & map_weights;
    std::vector<omp_lock_t> agent_locks;
    int thread;

    Dist2PathHeuristicTable(
            TrajLNS & _lns, 
            std::shared_ptr<std::vector<float> > & _map_weights,
            int _thread=1
        ): 
        lns(_lns), 
        env(_lns.env), 
        map_weights(_map_weights),
        thread(_thread)
         {
        
        if (thread!=1) {
            agent_locks.resize(lns.env->num_of_agents);
            for (int agent_idx=0;agent_idx<lns.env->num_of_agents;++agent_idx) {
                omp_init_lock(&agent_locks[agent_idx]);
            }
        }

    }

    // TODO: enable TrafficFlow
    float get(int agent_idx, int loc, int orient=-1) {
        int min_heuristic;

        if (thread!=1)
            omp_set_lock(&agent_locks[agent_idx]);

        // TODO(rivers): the following are for traffic flow. enable them later.

        // if (!lns.traj_dists.empty() && !lns.traj_dists[agent_idx].empty())
        //     min_heuristic = get_dist_2_path(lns.traj_dists[agent_idx], lns.env, loc, &(lns.neighbors));
        // else

        if (orient!=-1)
            min_heuristic = UTIL::static_heuristic_table->get(loc, orient, lns.tasks.at(agent_idx));
        else
            min_heuristic = UTIL::static_heuristic_table_no_rot->get(loc, lns.tasks.at(agent_idx));


        if (thread!=1)
            omp_unset_lock(&agent_locks[agent_idx]);
        
        return min_heuristic;
    }

    ~ Dist2PathHeuristicTable() {
        if (thread!=1) {
            for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
                omp_destroy_lock(&agent_locks[agent_idx]);
            }
        }
    }

};
} // namespace MyPlanner