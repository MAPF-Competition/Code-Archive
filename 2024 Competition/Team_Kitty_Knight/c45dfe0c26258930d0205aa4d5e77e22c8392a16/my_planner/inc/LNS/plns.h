#pragma once
#include <random>
#include <memory>
#include "LNS/Instance.h"
#include "LNS/Parallel/GlobalManager.h"
#include "util/Timer.h"
#include "util/TimeLimiter.h"
#include <iostream>
#include "Dist2PathHeuristicTable.h"

namespace LNS {

class PLNSSolver {
public:
    int num_of_agents;
    int window_size_for_PATH;
    
    std::shared_ptr<LNS::Parallel::GlobalManager> global_manager;
    std::shared_ptr<std::vector<float> > map_weights;
    std::shared_ptr<SharedEnvironment> env;
    std::shared_ptr<LNS::Instance> instance;
    std::shared_ptr<std::vector<LNS::Parallel::AgentInfo> > agent_infos;

    PLNSSolver(
        int rows,
        int cols,
        std::vector<int> & map,
        std::shared_ptr<MyPlanner::Dist2PathHeuristicTable> HT,
        int num_of_agents,
        int window_size_for_PATH,
        int num_threads,
        int max_iterations,
        bool verbose=true
    ): 
        num_of_agents(num_of_agents), 
        window_size_for_PATH(window_size_for_PATH),
        map_weights(HT->map_weights) {

        // auto grid=std::make_shared<Grid>(map_path);
        env=std::make_shared<SharedEnvironment>();
        env->map=map;
        env->rows=rows;
        env->cols=cols;  
        env->num_of_agents=num_of_agents;

        // build fake start states and goals for now
        env->curr_states.resize(num_of_agents);
        env->goal_locations.resize(num_of_agents,vector<pair<int,int>>(1));
        

        
        instance=std::make_shared<LNS::Instance>(*env);

        agent_infos=std::make_shared<std::vector<LNS::Parallel::AgentInfo> >();

        // TODO: agent_infos also maintains goal_location, etc.
        // a bad design! refactor it! the only thing we want is the disabled vector.
        for (int i=0; i<num_of_agents; ++i) {
            agent_infos->emplace_back();
            auto & agent_info=agent_infos->back();
            agent_info.id=i;
            // agent_info.disabled=false;
        }


        // TODO: make configurable
        bool async=true;
        // agent_infos
        int neighbor_size=8;
        LNS::Parallel::destroy_heuristic destroy_strategy=LNS::Parallel::destroy_heuristic::RANDOMWALK;
        bool ALNS=true;
        double decay_factor=0.01;
        double reaction_factor=0.1;
        std::string init_algo_name="LaCAM2";
        std::string replan_algo_name="PP";
        bool sipp=false;
        int window_size_for_CT=window_size_for_PATH;
        int window_size_for_CAT=window_size_for_PATH;
        int execution_window=1;
        // TODO: support disabled agents
        bool has_disabled_agents=false;
        bool fix_ng_bug=true;
        int screen=0;

        global_manager=std::make_shared<LNS::Parallel::GlobalManager>(
            async,
            *instance, HT, map_weights, agent_infos,
            neighbor_size, destroy_strategy,
            ALNS, decay_factor, reaction_factor,
            init_algo_name, replan_algo_name, sipp,
            window_size_for_CT, window_size_for_CAT, window_size_for_PATH, execution_window,
            has_disabled_agents,
            fix_ng_bug,
            screen,
            verbose,
            num_threads,
            max_iterations
        );


    }

    // starts and goals should 1 integer encoding the location of the agent
    // row*#cols + col
    // init_paths should be length of num_of_agents*(window_size_for_PATH+1), the 1 is for the start locations
    std::vector<int> solve(
        std::vector<int> & goal_locations,
        std::vector<std::vector<std::pair<int,int> > > & init_paths,
        double time_limit
    ) { 
        UTIL::TimeLimiter time_limiter(time_limit);

        std::vector<State> starts;
        std::vector<State> goals;

        for (int i=0; i<num_of_agents; ++i) {
            starts.emplace_back(init_paths[i][0].first, -1, init_paths[i][0].second);
            goals.emplace_back(goal_locations[i], -1, -1);
        }

        global_manager->reset();
        instance->set_starts_and_goals(starts, goals);

        // copy init paths
        for (int i=0; i<num_of_agents; ++i) {
            global_manager->agents[i].path.clear();
            for (int j=0; j<=window_size_for_PATH; ++j) {
                int location=init_paths[i][j].first;
                int orientation=init_paths[i][j].second;
                global_manager->agents[i].path.nodes.emplace_back(location,orientation);
            }
            // compute init costs
            global_manager->agents[i].path.path_cost=global_manager->agents[i].getEstimatedPathLength(
                global_manager->agents[i].path,goal_locations[i],global_manager->HT
            );
        }

        global_manager->run(time_limiter);

        // float total_delays=0;
        // for (int i=0;i<num_of_agents;++i) {
        //     total_delays+=(HT->get(i, start_locations[i], start_orientations[i])-global_manager->agents[i].getEstimatedPathLength(global_manager->agents[i].path,goal_locations[i],HT));
        // }

        // // TODO: this might be conflict!
        // for (int i=0;i<num_of_agents;++i) {
        //     if (global_manager->agents[i].path.nodes.size()<window_size_for_PATH+1) {
        //         global_manager->agents[i].path.nodes.resize(window_size_for_PATH+1,global_manager->agents[i].path.nodes.back());
        //     }
        // }

        // TODO
        // get actions from plan
        std::vector<int> actions(num_of_agents);
        for (int i=0; i<num_of_agents; ++i) {
            // we get the next location at index 1
            int next_location = global_manager->agents[i].path.nodes[1].location;
            int next_orientation = global_manager->agents[i].path.nodes[1].orientation;
            int location = global_manager->agents[i].path.nodes[0].location;
            int orientation = global_manager->agents[i].path.nodes[0].orientation;
            int y=location/env->cols;
            int x=location%env->cols;
            int ny=next_location/env->cols;
            int nx=next_location%env->cols;
            
            actions[i]=get_action(y,x,orientation,ny,nx,next_orientation);
        }

        return actions;
    }

    std::string playground(){
        return "hello, test!";
    }

    int get_action(int y, int x, int o, int ny, int nx, int no) {
        if (y!=ny || x!=nx) {
            return 0; // F
        } else {
            if ((o+1)%4==no) {
                return 1; // CR
            } else if ((o+3)%4==no) {
                return 2; // CCR
            } else {
                return 3; // W
            }
        }

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

}; // namespace LNS