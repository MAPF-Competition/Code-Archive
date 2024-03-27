#pragma once
#include "SharedEnv.h"
#include "ActionModel.h"
#include "RHCR/interface/CompetitionGraph.h"
#include "States.h"
#include "RHCR/interface/CompetitionActionModel.h"
#include <random>

namespace PIBT {


struct Agent {
public:
    State curr_state;
    State next_state;
    int goal_location;
    float elapsed;
    float tie_breaker;
    bool oriented;

    Agent():goal_location(-1),elapsed(-1),tie_breaker(-1),oriented(false){};
};



class PIBTSolver{
public:
    RHCR::CompetitionGraph& graph;
    std::vector<Path> paths;
    std::mt19937* const MT;   // seed for randomness
    std::vector<int> execution_order;
    vector<Agent> agents;
    
    // each list is a group of interacting agents, the key is the agent with the highest priority.
    vector<list<int>> agent_groups;

    // map from location to agent index
    unordered_map<int,int> occupied_now;
    unordered_map<int,int> occupied_next;
    CompetitionActionModelWithRotate action_model;
    bool need_replan = true;
    int total_feasible_timestep = 0;
    int timestep = 0;

    
    // TODO: to make seed configurable
    PIBTSolver(RHCR::CompetitionGraph & graph,SharedEnvironment * env,uint random_seed=0):graph(graph),action_model(env),MT(new std::mt19937(random_seed)){};
    ~PIBTSolver(){
        delete MT;
    };

    void initialize(const SharedEnvironment & env);
    void plan(const SharedEnvironment & env);
    void get_step_actions(const SharedEnvironment & env, vector<Action> & actions);

    void plan_for_single_step(const SharedEnvironment & env);
    void init_plan(const SharedEnvironment & env);

    bool pibt(int idx, int parent_idx, list<int> & agent_group);

    // for agent priority
    bool prior_to(const int i, const int j) const {
        const Agent & a=agents[i];
        const Agent & b=agents[j];

        if (a.elapsed != b.elapsed) return a.elapsed > b.elapsed;
        return a.tie_breaker > b.tie_breaker;
    }

    // for next state preference
    bool better_than(const int idx, const State & s1, const State & s2) {
        // if (idx==8)
        // cerr<<"compare starts"<<endl;
        const auto & agent = agents[idx];

        double d1=DBL_MAX,d2=DBL_MAX;
        auto iter=graph.heuristics.find(s1.location);
        if (iter!=graph.heuristics.end()){
            d1=iter->second[agent.goal_location];
        }
        iter=graph.heuristics.find(s2.location);
        if (iter!=graph.heuristics.end()){
            d2=iter->second[agent.goal_location];
        }

        // if (idx==8)
        // cerr<<"emm:"<<s1.location<<" "<<d1<<" "<<s2.location<<" "<<d2<<endl;

        // the smaller, the better
        // if (d1!=d2) return d1<d2;

        // we need to think about the orientation
        // int best_orient=get_best_orientation(idx,agent.curr_state);

        // if (idx==8)
        // cerr<<"dsd"<<best_orient<<endl;
        // int rot_d1=get_rotation_dist(agent.curr_state,s1.orientation);
        // int rot_d2=get_rotation_dist(agent.curr_state,s2.orientation);

        // int a1=get_action_from_states(agent.curr_state,s1);
        // int a2=get_action_from_states(agent.curr_state,s2);

        // if (idx==8)
        // cerr<<agent.curr_state.orientation<<" "<<"a1:"<<a1<<" rd1:"<<rot_d1<<" a2:"<<a2<<" rd2:"<<rot_d2<<endl;

        // if (rot_d1!=rot_d2) return rot_d1<rot_d2;

        // d1+=rot_d1;
        // d2+=rot_d2;

        // if (idx==8){
        //     cerr<<s1<<" "<<d1<<" "<<s2<<" "<<d2<<endl;
        // }

        if (d1!=d2) return d1<d2;

        return s1.orientation<s2.orientation;

        // tie break: not occupied in the previous step is better.
        // if (occupied_now.find(s1.location)!=occupied_now.end() && occupied_now.find(s2.location)==occupied_now.end()) {
        //     return false;
        // }

        // if (occupied_now.find(s1.location)==occupied_now.end() && occupied_now.find(s2.location)!=occupied_now.end()) {
        //     return true;
        // }

        return false;

    }

    int get_rotation_dist(const State & s,int best_orientation) {

        int d1=(s.orientation+4-best_orientation)%4;
        int d2=(best_orientation+4-s.orientation)%4;

        return d1<d2?d1:d2;
    }

    int get_best_orientation(const int idx, const State & s) {
        const auto & agent = agents[idx];

        // get location neighbors
        int curr_location=s.location;
        int x=s.location%action_model.env->cols;
        int y=s.location/action_model.env->cols;

        bool conditions[4];
        conditions[0]=(x==action_model.env->cols-1);
        conditions[1]=(y==action_model.env->rows-1);
        conditions[2]=(x==0);
        conditions[3]=(y==0);

        // find the next location we want to go, and thus, the orientation.
        int best_orientation=0;
        double best_dist=DBL_MAX;
        for (int i=0;i<4;++i){
            if (!conditions[i]){
                int next_location=curr_location+action_model.moves[i];
                auto iter=graph.heuristics.find(next_location);
                if (iter!=graph.heuristics.end()){
                    double dist=graph.heuristics[next_location][agent.goal_location];
                    // if (idx==8) {
                    //     cerr<<i<<" "<<dist<<endl;
                    // }
                    if (dist<best_dist) {
                        best_dist=dist;
                        best_orientation=i;
                    }
                }
            }
        }
        return best_orientation;
    }

    Action get_action_from_states(const State & state, const State & next_state){
        assert(state.timestep+1==next_state.timestep);
        
        if (state.location==next_state.location){
            // either turn or wait
            if (state.orientation==next_state.orientation) {
                return Action::W;
            } else if ((state.orientation-next_state.orientation+4)%4==3) {
                return Action::CR;
            } else if ((state.orientation-next_state.orientation+4)%4==1) {
                return Action::CCR;
            } else {
                assert(false);
                return Action::W;
            }
            }
        else {
            return Action::FW;
        }
    }

};

}