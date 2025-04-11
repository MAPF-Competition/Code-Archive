#include "planner.h"
#include "init.h"
#include "SharedEnv.h"
#include "flow.h"
#include "const.h"
#include "PIBT/causal_pibt.h"
#include "util/HeuristicTable.h"

namespace MyPlanner{

std::vector<int> ids;
std::vector<double> p_copy;
std::vector<bool> require_guide_path;
std::vector<int> dummy_goals;
TrajLNS trajLNS;
std::mt19937 mt1;
std::shared_ptr<Dist2PathHeuristicTable> dist2path_heuristics;

std::shared_ptr<PIBT::CausalPIBT> causal_pibt;

void initialize(int preprocess_time_limit, SharedEnvironment* env){
    // cout<<"plan initiallize limit "<< preprocess_time_limit<<endl;
    assert(env->num_of_agents != 0);

    ids.resize(env->num_of_agents);
    p_copy.resize(env->num_of_agents);
    require_guide_path.resize(env->num_of_agents);
    dummy_goals.resize(env->num_of_agents);

    init_heuristics(env);
    mt1.seed(0);
    srand(0);

    new (&trajLNS) TrajLNS(env, global_neighbors);
    trajLNS.init_mem();

    const int n_threads=1;
    dist2path_heuristics = std::make_shared<Dist2PathHeuristicTable>(trajLNS, UTIL::static_heuristic_table->map_weights, n_threads);

    for (int i = 0; i < ids.size();i++){
        ids[i] = i;
    }
    std::shuffle(ids.begin(), ids.end(), mt1);
    
    for (int i = 0; i < ids.size();i++){
        p_copy[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
    }

    causal_pibt = std::make_shared<PIBT::CausalPIBT>(env, dist2path_heuristics, p_copy, mt1);
    causal_pibt->initialize(preprocess_time_limit, env);

    return;
};

void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){
    TimePoint start_time = std::chrono::steady_clock::now();
    //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;

    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
    //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
    TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
    // cout << "plan limit " << time_limit <<endl;

    if (env->curr_timestep == 0){
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
        {
            dummy_goals.at(i) = env->curr_states.at(i).location; // use the curr_states as the dummy goals
        }
    }

    std::vector<int> dummy_goal_agent_ids;
    for(int i=0; i<env->num_of_agents; i++)
    {
        assert(env->curr_states[i].location >=0);
        
        if (env->goal_locations[i].empty()){
            trajLNS.tasks[i] = dummy_goals.at(i);
            // TODO(rivers): setting to lowest priorities makes sense!
            dummy_goal_agent_ids.push_back(i);
        }
        else{
            // TODO(rivers): here and entry we need to consider the following goals if possible.
            trajLNS.tasks[i] = env->goal_locations[i].front().first;
        }

        require_guide_path[i] = false;
        // if there is no guiding path or the goal is changed
        if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                require_guide_path[i] = true;
    }

    // TODO(rivers): the following are for traffic flow. enable them later.

    // // task change
    // for (int i = 0; i < env->num_of_agents;i++){
    //     if (std::chrono::steady_clock::now() >end_time)
    //         break;
    //     if (require_guide_path[i]){
    //         if (!trajLNS.trajs[i].empty())
    //             remove_traj(trajLNS, i);
    //         update_traj(trajLNS, i);
    //     }
    // }


    // std::unordered_set<int> updated;
    // // TODO(rivers): we can make this part parallel as well.
    // frank_wolfe(trajLNS, updated,end_time);

    causal_pibt->update(
        env->goal_locations,
        dummy_goal_agent_ids,
        require_guide_path
    );

    actions = causal_pibt->plan();

    // auto multi_step_actions=causal_pibt->multi_step(
    //     5,
    //     env->goal_locations,
    //     dummy_goal_agent_ids,
    //     require_guide_path
    // );

    // actions.resize(env->num_of_agents);
    // for (int i=0;i<env->num_of_agents;++i){
    //     actions[i]=multi_step_actions[i][0];
    // }

    return;

};


} // namespace MyPlanner