#include "planner.h"
#include "init.h"
#include "SharedEnv.h"
#include "flow.h"
#include "const.h"
#include "PIBT/causal_pibt.h"
#include "util/HeuristicTable.h"
#include "LNS/plns.h"
#include "util/TimeLimiter.h"

namespace MyPlanner{

std::vector<int> ids;
std::vector<double> p_copy;
std::vector<bool> require_guide_path;
std::vector<int> dummy_goals;
TrajLNS trajLNS;
std::mt19937 mt1;
std::shared_ptr<Dist2PathHeuristicTable> dist2path_heuristics;

std::shared_ptr<PIBT::CausalPIBT> causal_pibt;
std::shared_ptr<LNS::PLNSSolver> plns;

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

    int window_size=15;

    plns = std::make_shared<LNS::PLNSSolver>(
        env->rows,
        env->cols,
        env->map,
        dist2path_heuristics,
        env->num_of_agents,
        window_size,
        -1,
        -1,
        true
    );

    return;
};

void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){

    if (configs["planner"]=="dummy") {
        actions=std::vector<Action>(env->num_of_agents,Action::W);
        return;
    }

    std::cout<<"time_limt: "<<time_limit<<std::endl;

    UTIL::TimeLimiter time_limiter(time_limit/1000.0-0.05);

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

    // env->goal_locations may have empty goal locations because there is no assigned task!
    auto env_goal_locations=env->goal_locations;
    std::vector<int> dummy_goal_agent_ids;
    for(int i=0; i<env->num_of_agents; i++)
    {
        assert(env->curr_states[i].location >=0);
        
        if (env->goal_locations[i].empty()){
            trajLNS.tasks[i] = dummy_goals.at(i);
            // TODO(rivers): setting to lowest priorities makes sense!
            dummy_goal_agent_ids.push_back(i);
            env_goal_locations[i].emplace_back(dummy_goals.at(i),env->curr_timestep);
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

    if (configs["planner"]=="causal_pibt") {
        causal_pibt->update(
            env->curr_states,
            env_goal_locations,
            dummy_goal_agent_ids,
            require_guide_path
        );

        actions = causal_pibt->plan();

    } else if (configs["planner"]=="causal_pibt+plns") {

        UTIL::g_timer.record_p("causal_pibt_gen_init_sol_s");
        auto multi_step_actions=causal_pibt->multi_step(
            plns->window_size_for_PATH,
            env_goal_locations,
            dummy_goal_agent_ids,
            require_guide_path
        );
        UTIL::g_timer.record_d("causal_pibt_gen_init_sol_s","causal_pibt_gen_init_sol");

        // generate initial paths
        std::vector<int> goal_locations;
        std::vector<std::vector<std::pair<int,int> > > init_paths(env->num_of_agents);

        for (int i=0; i<env->num_of_agents; ++i) {
            goal_locations.push_back(env_goal_locations[i].front().first);

            int location = env->curr_states[i].location;
            int orientation = env->curr_states[i].orientation;

            std::vector<std::pair<int,int> > & init_path=init_paths[i];
            init_path.emplace_back(location, orientation);

            for (int j=0;j<plns->window_size_for_PATH;++j) {
                int action = multi_step_actions[i][j];
                auto pair = plns->move(location, orientation, action);
                location = pair.first;
                orientation = pair.second;
                init_path.emplace_back(location, orientation);
            }
        }

        // check: this is time consuming... don't use it unless debugging
        // UTIL::g_timer.record_p("plns_check_init_sol_s");
        // for (int t=0;t<=plns->window_size_for_PATH;++t) {
        //     // vertex collison free
        //     for (int i=0;i<env->num_of_agents;++i) {
        //         for (int j=i+1;j<env->num_of_agents;++j) {
        //             if (init_paths[i][t].first == init_paths[j][t].first) {
        //                 std::cerr<<"vertex collision at time "<<t<<" for agent "<<i<<" and "<<j<<std::endl;
        //                 exit(-1);
        //             }
        //         }
        //     }
        //     // edge collision free
        //     if (t>0)
        //     for (int i=0;i<env->num_of_agents;++i) {
        //         for (int j=i+1;j<env->num_of_agents;++j) {
        //             if (init_paths[i][t].first == init_paths[j][t-1].first && init_paths[i][t-1].first == init_paths[j][t].first) {
        //                 std::cerr<<"edge collision at time "<<t<<" for agent "<<i<<" and "<<j<<std::endl;
        //                 exit(-1);
        //             }
        //         }
        //     }
        // }
        // UTIL::g_timer.record_d("plns_check_init_sol_s","plns_check_init_sol");

        // std::cout<<"init_paths: "<<init_paths.size()<<std::endl;
        // std::cout<<"init_paths[0]: "<<init_paths[0].size()<<std::endl;
        // std::cout<<"init_paths[99]"<<init_paths[99].size()<<std::endl;

        // std::cout<<"remaining time: "<<time_limiter.get_remaining_time()<<std::endl;

        if (!time_limiter.timeout()) {
            auto _actions = plns->solve(goal_locations, init_paths, time_limiter.get_remaining_time());
            
            actions.resize(env->num_of_agents);
            for (int i=0;i<env->num_of_agents;++i){
                actions[i]=(Action)(_actions[i]);
            }
        } else {
            actions.resize(env->num_of_agents);
            for (int i=0;i<env->num_of_agents;++i){
                actions[i]=multi_step_actions[i][0];
            }
        }

    } else {
        std::cerr<<"error: unknown planner"<<std::endl;
        exit(-1);
    }

    // simulate the actions internally
    for (int i=0;i<env->num_of_agents;++i){
        int location = env->curr_states[i].location;
        int orientation = env->curr_states[i].orientation;
        int action = actions[i];
        auto pair = causal_pibt->move(location, orientation, action);
        int new_location = pair.first;

        // TODO: assume the current goal location is arrived 
        if (!env->goal_locations[i].empty() && new_location==env->goal_locations[i].front().first){
            int task_id = env->curr_task_schedule[i];
            if (task_id==-1) continue;
            auto & task = env->task_pool[task_id];
            if (task.idx_next_loc==task.locations.size()-1) {
                ++num_completed_task;
            }
        }
    }
    
    // std::cout<<"compelte "<<num_completed_task<<std::endl;

    // std::cerr<<"actions: "<<actions.size()<<std::endl;

    // actions.resize(env->num_of_agents);
    // for (int i=0;i<env->num_of_agents;++i){
    //     actions[i]=multi_step_actions[i][0];
    // }

    return;

};


} // namespace MyPlanner