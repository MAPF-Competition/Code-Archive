#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"


namespace DefaultPlanner{

    //default planner data
    std::vector<int> decision;              // store the location of each agent at the next timestep
    std::vector<int> prev_decision;         // store the location of each agent at the previous timestep
    std::vector<double> p;                  // store the priority of each agent
    std::vector<State> prev_states;         // store the previous state of each agent
    std::vector<State> next_states;         // store the next state of each agent
    std::vector<int> ids;                   // store the id of each agent
    std::vector<double> p_copy;             // store the copy of the (initial) priority of each agent
    std::vector<bool> occupied;             // store the occupied status of each location
    std::vector<DCR> decided;               // store the decided action of each agent
    std::vector<bool> checked;              // store the checked status of each agent's next action
    std::vector<bool> require_guide_path;   // store the request for guide path of each agent
    std::vector<int> dummy_goals;           // store the dummy goal of each agent
    TrajLNS trajLNS;                            
    std::mt19937 mt1;

        
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
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
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();
            // initialize the priority of each agent randomly so that no tie will happen
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
            return;
    };

    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;

        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        // cout << "plan limit " << time_limit <<endl;
        // initialize the dummy goals as the starting postion of each agent at the first timestep
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }



        // cout<<"---timestep,"<< env->curr_timestep<<endl;
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            if ( ( std::chrono::steady_clock::now() < end_time) ){
                // initialise the heuristic table for the goal locations of current agent(i)
                for(int j=0; j<env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.at(goal_loc).empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                        count++;
                    }
                }
            }
            // set next (dummy) task
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }
            // request for guide path if the agent has no guide path or a new task is assigned
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                    require_guide_path[i] = true;
            
            // set the initial prev_states,next_states of each agent and 
            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 
            if (decided[i].loc == -1){  // if 
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc){ // 
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){ // 
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }
            // increase the priority of the agent each timestep when approaching the goal location, and reset the priority if request for guide path
            if(require_guide_path[i])
                p[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            //deadend agent will be given priority bonus so that it can move out of it soon
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }
            
        }
        // update traj and distance table for the agents that require guide path
        for (int i = 0; i < env->num_of_agents;i++){
            if (std::chrono::steady_clock::now() >end_time)
                break;
            if (require_guide_path[i]){
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                update_traj(trajLNS, i);
            }
        }


        std::unordered_set<int> updated;
        // Path Refinement and Replanning
        frank_wolfe(trajLNS, updated,end_time);

        // sort agents in decreasing order based on their priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        // use PIBT to resolve conflicts found in the previous timestep
        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        
        // set the actions(output of the function) for the agents
        actions.resize(env->num_of_agents);
        for (int id : ids){
            //clear the decision table
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;
            // set the next location of the agent 
            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }
        // make agent turn to the correct direction before moving forward
        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,actions,prev_decision);
            }
        }



        prev_states = next_states;
        return;

    };
}