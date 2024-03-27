#include "PIBT/PIBTSolver.h"
#include "PIBT/util.h"
#include <algorithm>


namespace PIBT
{

void PIBTSolver::initialize(const SharedEnvironment & env) {
    paths.resize(env.num_of_agents);
    agents.resize(env.num_of_agents);
    execution_order.resize(env.num_of_agents);

    for (int i=0;i<agents.size();++i) {
        execution_order[i]=i;
        occupied_now[agents[i].curr_state.location]=i;
    }
}

void PIBTSolver::init_plan(const SharedEnvironment & env) {
    if (timestep==0) {
        for (int i=0;i<agents.size();++i) {
            paths[i].push_back(env.curr_states[i]);
        }
    }

    for (int i=0;i<env.num_of_agents;++i) {
        auto & agent=agents[i];
        agent.curr_state=env.curr_states[i];
        agent.next_state=State();
        assert(env.goal_locations[i].size()>0);
        int goal_location=env.goal_locations[i][0].first;
        if (goal_location!=agent.goal_location){
            agent.goal_location=goal_location;
            agent.elapsed=0;
            agent.tie_breaker=getRandomFloat(0,1,MT);
        }
    }
    for (int i=0;i<agents.size();++i) {
        occupied_now[agents[i].curr_state.location]=i;
        agents[i].oriented=false;
    }
    occupied_next.clear();
    agent_groups.clear();

}

void PIBTSolver::plan(const SharedEnvironment & env) {
    init_plan(env);
    plan_for_single_step(env);
    // we just need to write planned step to our path.
    for (int group_idx=0;group_idx<agent_groups.size();++group_idx) {
        const auto & agent_group=agent_groups[group_idx];
        bool all_oriented=true;
        for (auto agent_idx:agent_group) {
            if (!agents[agent_idx].oriented) {
                all_oriented=false;
                break;
            }
        }

        // cerr<<"group "<<group_idx<<", all_oriented="<<all_oriented<<": ";

        for (auto agent_idx:agent_group) {
            // cerr<<" "<<agent_idx;
            const auto & agent=agents[agent_idx];
            // if (agent_idx==8){
            //     cerr<<agent.curr_state<<" "<<agent.next_state<<endl;
            // }

            if (all_oriented && agent.curr_state.location+action_model.moves[agent.curr_state.orientation]==agent.next_state.location) {
                paths[agent_idx].push_back(agent.next_state);
            } else {
                // if (agent_idx==18){
                //     cerr<<agent.curr_state<<" "<<agent.next_state<<endl;
                // }
                auto candidates=action_model.get_state_neighbors(agent.curr_state,true);
                int best_d=3;
                State next_state;
                for (auto candidate:candidates) {
                    int d=get_rotation_dist(candidate,agent.next_state.orientation);
                    // if (agent_idx==18){
                    //     cerr<<candidate<<" "<<d<<endl;
                    // }
                    if (d<best_d){
                        best_d=d;
                        next_state=candidate;
                    }
                }
                paths[agent_idx].push_back(next_state);
            }
        }
        // cerr<<endl;
    }

//    exit(-1);

    total_feasible_timestep+=1;

}

bool PIBTSolver::pibt(int idx, int parent_idx, list<int> & agent_group) {
    agent_group.push_back(idx);

    auto & agent=agents[idx];

    // if (idx==8)
    // cerr<<"fall: "<<agent.oriented<<endl;

    auto candidates = action_model.get_loc_neighbors(agent.curr_state);
    
    // randomize
    std::shuffle(candidates.begin(),candidates.end(),*MT);

    // sort
    std::sort(candidates.begin(),candidates.end(),std::bind(&PIBTSolver::better_than,this,idx,std::placeholders::_1,std::placeholders::_2));


    // if (idx==8){
    // cerr<<"hi:"<<agent.curr_state<<" ";
    // for (auto candidate:candidates) {
    //     cerr<<candidate<<" ";
    // }
    // cerr<<endl;
    // }

    for (const auto & candidate: candidates) {
        // avoid node conflicts
        auto iter=occupied_next.find(candidate.location);
        if (iter!=occupied_next.end()) continue;
        
        // avoid edge conflicts
        if (parent_idx!=-1 && candidate.location==agents[parent_idx].curr_state.location) continue;

        // reserve
        occupied_next[candidate.location]=idx;
        // make it an decided agent
        agent.next_state=candidate;


        // if (idx==8) {
        //     cerr<<idx<<" "<<candidate.location<<endl;
        // }

        iter=occupied_now.find(candidate.location);
        if (iter!=occupied_now.end()) {
            auto & occupying_agent=agents[iter->second];
            // if it is an undecided agent
            if (occupying_agent.next_state.location==-1) {
                // try to push the occupying agent away
                // if fail need to select another choice.
                if (!pibt(iter->second,idx,agent_group)) continue;
            }
        }

        if (candidate.orientation==agent.curr_state.orientation){
            agent.oriented=true;
        }

        // if (idx==8)
        // cerr<<"agent.o:"<<agent.oriented<<" "<<candidate<<" "<<agent.curr_state<<endl;

        return true;
    }

    // the last thing I can do is to orient toward my own best orientation.
    // int best_orient=get_best_orientation(idx,agent.curr_state);

    // int best_rotate_dist=4;
    // for (const auto & candidate: candidates) {
    // skip forward
    //     if (candidate.location!=agent.curr_state.location) continue;
    //     int rotate_dist=get_rotation_dist(agent.curr_state,best_orient);
    //     if (rotate_dist<best_rotate_dist) {
    //         best_rotate_dist=rotate_dist;
    //         agent.next_state=candidate;
    //     }
    // }

    occupied_next[agent.curr_state.location]=idx;
    agent.next_state=action_model.result_state(agent.curr_state,Action::W);
    agent.oriented=true;

    return false;
}

void PIBTSolver::plan_for_single_step(const SharedEnvironment & env) {
    // this is an adaption of pibt algorithm on the action model with rotation.
    std::sort(this->execution_order.begin(),this->execution_order.end(),std::bind(&PIBTSolver::prior_to,this,std::placeholders::_1,std::placeholders::_2));

    // plan according to the order
    for (int i=0;i<agents.size();++i) {
        int idx=execution_order[i];
        auto & agent=agents[idx];
        if (agent.next_state.location==-1){
            agent_groups.push_back(list<int>());
            auto & agent_group=agent_groups.back();
            pibt(idx,-1,agent_group);
        }

        bool all_oriented=true;
        for (auto agent_idx:agent_groups.back()) {
            if (!agents[agent_idx].oriented) {
                all_oriented=false;
                break;
            }
        }
        if (!all_oriented) {
            // all will remain at the oritinal positions, so we need to add some locks on the resources.
            // TODO: but should we need to release some resources as well? however, maybe some priortity is not longer maintained?
            for (auto agent_idx:agent_groups.back()) {
                occupied_next[agents[agent_idx].curr_state.location]=agent_idx;
            }
        }
    }
}

void PIBTSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions) {
    // check empty
    assert(actions.empty());

    for (int i=0;i<agents.size();++i) {
        // we will get action indexed at timestep+1
        if (paths[i].size()<=timestep+1){
            cerr<<"wierd error for agent "<<i<<". path length: "<<paths[i].size()<<", "<<"timestep+1: "<<timestep+1<<endl;
            assert(false);
        }
        actions.push_back(get_action_from_states(paths[i][timestep],paths[i][timestep+1]));
    }

    // TODO(hj) we probably still want to check the validness. so we need construct model or implement is_valid by ourselves.
    // check if not valid, this should not happen in general if the algorithm is correct? but maybe there exist deadlocks.
    // TODO(hj) detect deadlock?
    if (!action_model.is_valid(env.curr_states,actions)){
        cerr<<"planed actions are not valid in timestep "<<timestep+1<<"!"<<endl;
#ifdef DEV
        exit(-1);
#else
        actions.resize(agents.size(), Action::W);
#endif
    } else {
        // NOTE(hj): only successfully executing a planned step will increase this internal timestep, which is different from the real timestep used in the simulation system.
        timestep+=1;
    }

    // TODO(hj): when we need to replan?
    need_replan=false;

    // 1. exceed simulation window
    if (timestep==total_feasible_timestep){
        need_replan=true;
    }
    
    // 2. goal changes: there different ways to check this. let's just keep the old goal and compare.
    for (int i=0;i<agents.size();++i) {
        if (paths[i][timestep].location==agents[i].goal_location) {
            // arrive goal locations
            need_replan=true;
            break;
        }
    }
    
    // need_replan=true;
    
}


} // namespace PIBT
