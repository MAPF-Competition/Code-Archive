#pragma once
#include "common.h"
#include "SharedEnv.h"
#include "Types.h"
#include "TrajLNS.h"
#include <random>
#include "unordered_set"
#include "Dist2PathHeuristicTable.h"
#include "utils.h"

namespace MyPlanner {

namespace PIBT {

bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, std::shared_ptr<Dist2PathHeuristicTable> & heuristics
	  );


Action getAction(State& prev, State& next);

Action getAction(State& prev, int next_loc, SharedEnvironment* env);

bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);


class CausalPIBT {

public:
    CausalPIBT(
        SharedEnvironment * env, 
        std::shared_ptr<Dist2PathHeuristicTable> & heuristics,
        std::vector<double> & p_copy,
        std::mt19937 & mt1): 
        p(p_copy), p_copy(p_copy), heuristics(heuristics), mt1(mt1), env(env) {
        
    }

    std::vector<int> decision;
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> & p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    // TrajLNS & trajLNS;
    std::shared_ptr<Dist2PathHeuristicTable> & heuristics;
    std::mt19937 & mt1;
    SharedEnvironment * env;

    void initialize(int preprocesing_time_limit, SharedEnvironment* env) {
        // cout<<"plan initiallize limit "<< preprocess_time_limit<<endl;
        assert(env->num_of_agents != 0);
        p.resize(env->num_of_agents);
        decision.resize(env->map.size(), -1);
        prev_states.resize(env->num_of_agents);
        next_states.resize(env->num_of_agents);
        decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
        occupied.resize(env->map.size(),false);
        checked.resize(env->num_of_agents,false);
        ids.resize(env->num_of_agents);
        for (int i = 0; i < ids.size();i++){
            ids[i] = i;
        }

        std::shuffle(ids.begin(), ids.end(), mt1);
        for (int i = 0; i < ids.size();i++){
            p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
        }
        p_copy = p; // the default priority value
        return;
    }

    void update(
        std::vector<State> & curr_states,
        std::vector<std::vector<std::pair<int, int> > > & goal_locations,
        std::vector<int> & dummy_goal_agent_ids,
        std::vector<bool> & require_guide_path
    ) {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        for (auto i: dummy_goal_agent_ids) {
            p[i] = p_copy[i];
        }

        for (int i=0;i<env->num_of_agents;i++){
            prev_states[i] = curr_states[i];
            next_states[i] = State();
            prev_decision[curr_states[i].location] = i; 
            if (decided[i].loc == -1){
                decided[i].loc = curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            if (require_guide_path[i])
                p[i] = p_copy[i];
            else if (!goal_locations[i].empty())
                p[i] = p[i]+1;

            if (!goal_locations.empty() 
                &&
                global_neighbors[curr_states[i].location].size() == 1
            ) {
                //deadend agent will be given priority bonus
                p[i] = p[i] + 10;
            }
        }
    }

    std::vector<Action> plan() {

        std::vector<Action> actions;

        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        //pibt
        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, heuristics);
            }
        }
        
        actions.resize(env->num_of_agents);
        for (int id : ids){
            //clear the decision table based on which agent has next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;

            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }

        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,actions,prev_decision);
            }
        }

        prev_states = next_states;

        return actions;

    }

    std::shared_ptr<CausalPIBT> clone() {
        return std::make_shared<CausalPIBT>(*this);
    }

    // [n_agents, window_size]
    std::vector<std::vector<Action> > multi_step(
        int _window_size,
        std::vector<std::vector<std::pair<int, int> > > & _goal_locations,
        std::vector<int> & _dummy_goal_agent_ids,
        std::vector<bool> & _require_guide_path
    ) {

        auto causal_pibt = clone();
        auto goal_locations = _goal_locations;
        auto dummy_goal_agent_ids = _dummy_goal_agent_ids;
        auto require_guide_path = _require_guide_path;
        
        std::vector<std::vector<Action> > actions(env->num_of_agents);

        std::vector<State> curr_states = env->curr_states;

        for (int i=0;i<_window_size;++i) {
            
            causal_pibt->update(curr_states,goal_locations,dummy_goal_agent_ids,require_guide_path);
            auto _actions=causal_pibt->plan();

            // update curr_states
            for (int j=0;j<env->num_of_agents;++j) {
                int curr_location = curr_states[j].location;
                int curr_orientation = curr_states[j].orientation;
                int action = _actions[j];
                auto pair = move(curr_location, curr_orientation, action);
                curr_states[j].location = pair.first;
                curr_states[j].orientation = pair.second;
            }

            dummy_goal_agent_ids.clear();
            for (int j=0;j<env->num_of_agents;++j) {
                actions[j].push_back(_actions[j]);
                
                if (require_guide_path[j]){
                    require_guide_path[j] = false;
                }

                if (causal_pibt->prev_states[j].location == goal_locations[j][0].first){
                    // arrive at goal
                    // TODO(rivers): need to better handle here, e.g. for more than one errands. 
                    // NOTE(rivers): we need to allow multiple errands to be seen in the Entry.cpp.
                    dummy_goal_agent_ids.push_back(j);
                    goal_locations[j].erase(goal_locations[j].begin());
                    require_guide_path[j] = true;
                }
            }
        }

        return actions;

    }

    std::pair<int,int> move(int loc, int orient, int action) {
        // no check
        if (action==0) {
            if (orient==0) {
                loc+=1;
            } else if (orient==1) {
                loc+=env->cols;
            } else if (orient==2) {
                loc-=1;
            } else if (orient==3) {
                loc-=env->cols;
            }
        } else if (action==1) {
            orient=(orient+1)%4;
        } else if (action==2) {
            orient=(orient+3)%4;
        }
        return {loc, orient};
    }


};


} // PIBT

} // MyPlanner