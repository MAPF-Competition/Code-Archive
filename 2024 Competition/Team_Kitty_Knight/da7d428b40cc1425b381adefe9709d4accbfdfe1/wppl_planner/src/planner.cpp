#include <planner.h>
#include <random>
#include "util/HeuristicTable.h"
#include "util/Analyzer.h"
#include "boost/format.hpp"
#include "util/MyCommon.h"
#include "util/TimeLimiter.h"

namespace MyPlanner {

void WPPLPlanner::load_configs() {
    // load configs
	string config_path="wppl_planner/configs/"+env->map_name.substr(0,env->map_name.find_last_of("."))+".json";
    char * _config_path=getenv("CONFIG_PATH");
    if (_config_path!=NULL) {
        config_path=std::string(_config_path);
        std::cout<<"load config from "<<config_path<<std::endl;
    }
    std::ifstream f(config_path);
    try
    {
        config = nlohmann::json::parse(f);

        char * env_weight_path=getenv("MAP_WEIGHT_PATH");
        if (env_weight_path!=NULL) {
            config["map_weights_path"]=env_weight_path;
            std::cout<<"load weight from "<<env_weight_path<<std::endl;
        }

        ONLYDEV(std::cerr<<config<<std::endl;)
        config["scheduler_name"]=read_conditional_value(config,"scheduler_name",env->num_of_agents);
        config["lifelong_solver_name"]=read_conditional_value(config,"lifelong_solver_name",env->num_of_agents);
        config["map_weights_path"]=read_conditional_value(config,"map_weights_path",env->num_of_agents);

        if (config.contains("max_task_completed")) {
            config["max_task_completed"]=read_conditional_value(config,"max_task_completed",env->num_of_agents);
        }

        if (config.contains("max_execution_steps")) {
            config["max_execution_steps"]=read_conditional_value(config,"max_execution_steps",env->num_of_agents);
        }

        config["LaCAM2"]["order_strategy"]=read_conditional_value(config["LaCAM2"],"order_strategy",env->num_of_agents);
        config["LNS"]["LaCAM2"]["order_strategy"]=read_conditional_value(config["LNS"]["LaCAM2"],"order_strategy",env->num_of_agents);
        config["disable_corner_target_agents"]=read_conditional_value(config,"disable_corner_target_agents",env->num_of_agents);
        config["max_agents_in_use"]=read_conditional_value(config,"max_agents_in_use",env->num_of_agents);

        config["LNS"]["fix_ng_bug"]=read_conditional_value(config["LNS"],"fix_ng_bug",env->num_of_agents);
        config["LNS"]["window_size_for_CT"]=read_conditional_value(config["LNS"],"window_size_for_CT",env->num_of_agents);
        config["LNS"]["window_size_for_CAT"]=read_conditional_value(config["LNS"],"window_size_for_CAT",env->num_of_agents);
        config["LNS"]["window_size_for_PATH"]=read_conditional_value(config["LNS"],"window_size_for_PATH",env->num_of_agents);
        config["LNS"]["LaCAM2"]["planning_window"]=read_conditional_value(config["LNS"]["LaCAM2"],"planning_window",env->num_of_agents);


        string s=config.dump();
        std::replace(s.begin(),s.end(),',','|');
        config["details"]=s;
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cout << "Failed to load " << config_path << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }
}

std::string WPPLPlanner::load_map_weights(string weights_path) {
    // : make weights float
    // we have at least 5 weights for a location: right,down,left,up,stay
    map_weights=std::make_shared<std::vector<float> >(env->rows*env->cols*5,1);
    std::string suffix = "all_one";

    if (weights_path!=""){
        std::ifstream f(weights_path);
        try
        {
            nlohmann::json _weights = nlohmann::json::parse(f);
            if (_weights.size()!=map_weights->size()) {
                std::cerr<<"map weights size mismatch"<<std::endl;
                exit(-1);
            }

            for (int i=0;i<map_weights->size();++i){
                (*map_weights)[i]=_weights[i].get<float>();
            }
            
        }
        catch (nlohmann::json::parse_error error)
        {
            std::cerr << "Failed to load " << weights_path << std::endl;
            std::cerr << "Message: " << error.what() << std::endl;
            exit(1);
        }

        boost::filesystem::path _weights_path(weights_path);
        suffix=_weights_path.stem().string();
    }
    return suffix;
}

void WPPLPlanner::initialize(int preprocess_time_limit) {
    ONLYDEV(cout << "planner initialization begins" << endl;)
    load_configs();

    this->max_task_completed=read_param_json<int>(config,"max_task_completed",1000000);
    if (this->max_task_completed<0) {
        this->max_task_completed=1000000;
    }
    this->num_task_completed=0;

    ONLYDEV(
        analyzer.timestamp();
        analyzer.init_from_config(config);
        analyzer.set_dump_path(config["analysis_output"].get<string>());
    )

    max_execution_steps = read_param_json<int>(config,"max_execution_steps",1000000);
    if (max_execution_steps<0) {
        max_execution_steps=1000000;
    }

    ONLYDEV(std::cout<<"max execution steps: "<<max_execution_steps<<std::endl;)

    std::string weights_path=read_param_json<std::string>(config,"map_weights_path");
    std::string suffix=load_map_weights(weights_path);

    lifelong_solver_name=config["lifelong_solver_name"];

    // TODO: memory management is a disaster here...
    if (lifelong_solver_name=="LaCAM2") {
        if (!read_param_json<bool>(config["LaCAM2"],"consider_rotation")) {
            std::cerr<<"In LaCAM2, must consider rotation when compiled with NO_ROT unset"<<std::endl;
            exit(-1);
        }

        heuristics =std::make_shared<HeuristicTable>(env,map_weights,read_param_json<bool>(config["LaCAM2"],"use_orient_in_heuristic"));
        heuristics->preprocess(suffix);
        int max_agents_in_use=read_param_json<int>(config,"max_agents_in_use",-1);
        if (max_agents_in_use==-1) {
            max_agents_in_use=env->num_of_agents;
        }
        bool disable_corner_target_agents=read_param_json<bool>(config,"disable_corner_target_agents",false);
        int max_task_completed=read_param_json<int>(config,"max_task_completed",1000000);
        lacam2_solver = std::make_shared<LaCAM2::LaCAM2Solver>(heuristics,env,map_weights,max_agents_in_use,disable_corner_target_agents,max_task_completed,config["LaCAM2"]);
        lacam2_solver->initialize(*env);
        ONLYDEV(cout<<"LaCAMSolver2 initialized"<<endl;)
    } else if (lifelong_solver_name=="LNS") {
        if (read_param_json<bool>(config["LNS"]["LaCAM2"],"consider_rotation")) {
            std::cerr<<"In LNS, must not consider rotation when compiled with NO_ROT unset"<<std::endl;
            exit(-1);
        }
        heuristics =std::make_shared<HeuristicTable>(env,map_weights,true);
        heuristics->preprocess(suffix);
        //heuristics->preprocess();
        int max_agents_in_use=read_param_json<int>(config,"max_agents_in_use",-1);
        if (max_agents_in_use==-1) {
            max_agents_in_use=env->num_of_agents;
        }
        bool disable_corner_target_agents=read_param_json<bool>(config,"disable_corner_target_agents",false);
        int max_task_completed=read_param_json<int>(config,"max_task_completed",1000000);
        lacam2_solver = std::make_shared<LaCAM2::LaCAM2Solver>(heuristics,env,map_weights,max_agents_in_use,disable_corner_target_agents,max_task_completed,config["LNS"]["LaCAM2"]);
        lacam2_solver->initialize(*env);
        lns_solver = std::make_shared<LNS::LNSSolver>(heuristics,env,map_weights,config["LNS"],lacam2_solver,max_task_completed);
        lns_solver->initialize(*env);
        cout<<"LNSSolver initialized"<<endl;
    } else if (lifelong_solver_name=="DUMMY") {
        cout<<"using dummy solver"<<endl;

        // bad not used in planner, but its data structure is used in scheduler
        heuristics =std::make_shared<HeuristicTable>(env,map_weights,true);
        heuristics->preprocess();
        int max_agents_in_use=read_param_json<int>(config,"max_agents_in_use",-1);
        if (max_agents_in_use==-1) {
            max_agents_in_use=env->num_of_agents;
        }
        bool disable_corner_target_agents=read_param_json<bool>(config,"disable_corner_target_agents",false);
        int max_task_completed=read_param_json<int>(config,"max_task_completed",1000000);
        lacam2_solver = std::make_shared<LaCAM2::LaCAM2Solver>(heuristics,env,map_weights,max_agents_in_use,disable_corner_target_agents,max_task_completed,config["LNS"]["LaCAM2"]);
        lacam2_solver->initialize(*env);
        
    } else if (lifelong_solver_name=="Shadocks") {
        cout<<"using shadocks solver"<<endl;

        // bad not used in planner, but its data structure is used in scheduler
        heuristics =std::make_shared<HeuristicTable>(env,map_weights,true);
        heuristics->preprocess();
        int max_agents_in_use=read_param_json<int>(config,"max_agents_in_use",-1);
        if (max_agents_in_use==-1) {
            max_agents_in_use=env->num_of_agents;
        }
        bool disable_corner_target_agents=read_param_json<bool>(config,"disable_corner_target_agents",false);
        int max_task_completed=read_param_json<int>(config,"max_task_completed",1000000);
        lacam2_solver = std::make_shared<LaCAM2::LaCAM2Solver>(heuristics,env,map_weights,max_agents_in_use,disable_corner_target_agents,max_task_completed,config["LNS"]["LaCAM2"]);
        lacam2_solver->initialize(*env);

        shadocks_planner = std::make_shared<Shadocks::MAPFPlanner>(env);
        shadocks_planner->initialize(preprocess_time_limit);
    }
    else {
        cerr<<"unknown lifelong solver name"<<lifelong_solver_name<<endl;
        exit(-1);
    }

    ONLYDEV(cout << "planner initialization ends" << endl;)
}


// plan using simple A* that ignores the time dimension
void WPPLPlanner::plan(int time_limit,vector<Action> & actions) 
{

    TimeLimiter time_limiter(time_limit/1000.0-0.05);

    // NOTE we need to return within time_limit, but we can exploit this time duration as much as possible

    // check if we need to restart a plan task (thread)
    // if so, we need to stop the current one and then restart
    // we also need to clean the current action plan if restart

    // TODO if time_limit approachs, just return a valid move, e.g. all actions are wait.
    
    ONLYDEV(
        g_timer.record_p("_step_s");
    )

    if (env->curr_timestep>=max_execution_steps-1 || num_task_completed>=max_task_completed) {
        actions.clear();
        for (int i=0;i<env->num_of_agents;++i) {
            actions.push_back(Action::W);
        }
        return;
    }

    // TODO: may be bad but fine now
    for (int i=0;i<env->num_of_agents;++i) {
        if (env->goal_locations[i].empty()) {
            env->goal_locations[i].emplace_back(env->curr_states[i].location,env->curr_timestep);
        }
    }

    if (lifelong_solver_name=="LaCAM2") {
        ONLYDEV(cout<<"using LaCAM2"<<endl;)
        ONLYDEV(g_timer.record_p("mapf_lacam2_plan_s");)
        lacam2_solver->plan(*env);
        ONLYDEV(g_timer.record_d("mapf_lacam2_plan_s","mapf_lacam2_plan");)
        ONLYDEV(g_timer.record_p("mapf_lacam2_get_step_s");)
        lacam2_solver->get_step_actions(*env,actions);
        ONLYDEV(g_timer.record_d("mapf_lacam2_get_step_s","mapf_lacam2_get_step");)
    } else if (lifelong_solver_name=="LNS") {
        cout<<"using LNS"<<endl;
        lns_solver->observe(*env);
        lns_solver->plan(*env, time_limiter); 
        lns_solver->get_step_actions(*env,actions);
    } else if (lifelong_solver_name=="DUMMY") {
        cout<<"using DUMMY"<<endl;
    } else if (lifelong_solver_name=="Shadocks") {
        cout<<"using Shadocks"<<endl;
        shadocks_planner->plan(time_limit-0.01*1000, actions);
    } 
    else {
        cerr<<"unknown lifelong solver name"<<lifelong_solver_name<<endl;
        exit(-1);
    }

    // simulate the actions internally
    std::vector<State> next_states=action_model.result_states(env->curr_states,actions);
    for (int i=0;i<env->num_of_agents;++i){
        int new_location = next_states[i].location;
        int task_id = env->curr_task_schedule[i];
        if (task_id==-1) continue;
        auto & task = env->task_pool[task_id];
        if (task.idx_next_loc==task.locations.size()-1 && task.locations.back()==new_location) { 
            ++num_task_completed;
        }
    }

    ONLYDEV(
        double step_time=g_timer.record_d("_step_s","_step_e","_step");
        if (step_time>max_step_time) {
            max_step_time=step_time;
        }

        std::cerr<<"max_step_time: "<<max_step_time<<endl;
        g_timer.remove_d("_step");
    )

  return;
}

}