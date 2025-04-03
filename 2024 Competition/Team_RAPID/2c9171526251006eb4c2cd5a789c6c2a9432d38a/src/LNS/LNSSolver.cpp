#include "LNS/LNSSolver.h"
#include "util/Dev.h"
#include "util/Timer.h"
#include "util/Analyzer.h"
#include "LNS/Parallel/GlobalManager.h"
#include "LNS/Parallel/DataStructure.h"

namespace LNS
{

    LNSSolver::LNSSolver(
        const std::shared_ptr<HeuristicTable> &HT,
        SharedEnvironment *env,
        std::shared_ptr<std::vector<float>> &map_weights,
        nlohmann::json &config,
        std::shared_ptr<LaCAM2::LaCAM2Solver> &lacam2_solver,
        int max_task_completed) : HT(HT),
                                  map_weights(map_weights),
                                  action_model(env),
                                  executor(env),
                                  slow_executor(env),
                                  config(config),
                                  MT(new std::mt19937(read_param_json<uint>(config, "seed", 0))),
                                  lacam2_solver(lacam2_solver),
                                  agent_infos(lacam2_solver->agent_infos),
                                  max_task_completed(max_task_completed) {
                                  };

    void LNSSolver::initialize(const SharedEnvironment &env)
    {
        // obstacle_stats_tree = std::make_shared<StatsTree>(env.cols,env.rows);

        // for (int pos=0;pos<env.map.size();++pos) {
        //     if (env.map[pos]==1) {
        //         obstacle_stats_tree->update(pos,1);
        //     }
        // }

        // agent_stats_tree = std::make_shared<StatsTree>(env.cols,env.rows);
        // lacam2_solver->initialize(env); // it is initialized already outside.
        execution_paths.resize(env.num_of_agents);
        planning_paths.resize(env.num_of_agents);

        // : make it attribute of the class.
        int window_size_for_CT = read_param_json<int>(config, "window_size_for_CT");
        int window_size_for_PATH = read_param_json<int>(config, "window_size_for_PATH");
        int LaCAM2_planning_window = read_param_json<int>(config["LaCAM2"], "planning_window");
        if (window_size_for_CT == -1 || LaCAM2_planning_window != window_size_for_CT || window_size_for_PATH != window_size_for_CT)
        {
            cerr << "not fully supported now! need to modify the padding path code in lns->" << endl;
            exit(-1);
        }

        planning_window = window_size_for_CT;                                // : read from config & initialized in constructor
        execution_window = read_param_json<int>(config, "execution_window"); // : read from config & initialized in constructor

        // instance = std::make_shared<Instance>(env);
        // lns = std::make_shared<Parallel::GlobalManager>(
        //     true,
        //     *instance,
        //     HT,
        //     map_weights,
        //     agent_infos,
        //     read_param_json<int>(config,"neighborSize"),
        //     Parallel::destroy_heuristic::RANDOMWALK, // TODO: always randomwalk
        //     true, // TODO: always Adaptive
        //     0.01, // TODO: decay factor
        //     0.01, // TODO: reaction factor
        //     read_param_json<string>(config,"initAlgo"),
        //     read_param_json<string>(config,"replanAlgo"),
        //     false, // TODO: not sipp
        //     read_param_json<int>(config,"window_size_for_CT"),
        //     read_param_json<int>(config,"window_size_for_CAT"),
        //     read_param_json<int>(config,"window_size_for_PATH"),
        //     execution_window,
        //     lacam2_solver->max_agents_in_use!=env.num_of_agents, // TODO: has disabled agents
        //     read_param_json<bool>(config,"fix_ng_bug"),
        //     0 // TODO: screen
        // );
    }

    int get_neighbor_orientation(const SharedEnvironment *env, int loc1, int loc2)
    {

        // 0:east, 1:south, 2:west, 3:north

        if (loc1 + 1 == loc2)
        {
            return 0;
        }

        if (loc1 + env->cols == loc2)
        {
            return 1;
        }

        if (loc1 - 1 == loc2)
        {
            return 2;
        }

        if (loc1 - env->cols == loc2)
        {
            return 3;
        }

        if (loc1 == loc2)
        {
            return 4;
        }

        return -1;
    }

    std::vector<State> subvector(std::vector<State> &v, int start, int end)
    {
        std::vector<State> ret;
        for (int i = start; i < end; ++i)
        {
            ret.push_back(v[i]);
        }
        return ret;
    }

    void LNSSolver::plan(const SharedEnvironment &env)
    {
        // : make it configurable.
        double time_limit = read_param_json<double>(config, "cutoffTime");

        TimeLimiter time_limiter(time_limit);
        // if (env.curr_timestep == 0 and env.map_name == "brc202d.map") {//竞赛
        //     std::cerr << "brc202d.map" << std::endl;
        //     time_limiter.time_limit = 0.5;
        // }

        ONLYDEV(g_timer.record_p("_plan_s");)

        ONLYDEV(g_timer.record_p("plan_s");)

        std::vector<::State> starts;
        std::vector<::State> goals;

        int disabled_agent_count = 0;
        for (int i = 0; i < env.num_of_agents; ++i)
        {
            if ((*agent_infos)[i].disabled)
            {
                ++disabled_agent_count;
            }
            starts.emplace_back(execution_paths[i].back().location, -1, execution_paths[i].back().orientation);
            // if ((*agent_infos)[i].disabled) {
            //     goals.emplace_back(env.curr_states[i].location,-1,-1);
            // } else {
            // goals.emplace_back(env.goal_locations[i][0].first,-1,-1);
            // }

            // goals.emplace_back(env.goal_locations[i][0].first,-1,-1);

            // 新增判断：env->goal_locations[i]是否为空
            if (env.goal_locations[i].empty())
            {
                // 此智能体当前无任务；将其目标设为自己所在位置，或者让后续逻辑把它当成“不需要移动”
                // orientation 不重要，这里-1即可
                goals.emplace_back(env.curr_states[i].location, -1, -1);
            }
            else
            {
                // 正常情况：env.goal_locations[i]里已有未完成任务
                goals.emplace_back(env.goal_locations[i][0].first, -1, -1);
            }
        }

        ONLYDEV(std::cout << "disabled_agents:" << disabled_agent_count << std::endl;)

        // : we need to replan for all agents that has no plan
        // later we may think of padding all agents to the same length
        if (planning_paths[0].size() < planning_window + 1)
        {
            // 若第一个代理的规划路径（假设各代理规划路径长度相同）小于 planning_window + 1，则说明目前没有足够的规划路径，需要重新规划。

            // std::cout<<"call lacam2: "<<planning_paths[0].size()<<std::endl;
            // : maybe we should directly build lacam2 planner in this class.
            if (read_param_json<string>(config, "initAlgo") == "LaCAM2")
            {
                ONLYDEV(g_timer.record_p("lacam2_plan_s");)
                // use lacam2 to get a initial solution
                // : the following line may need to be optimized. There is no need to rebuild the graph G again.
                lacam2_solver->clear(env);

                // : we should avoid copy here. we may use deque for paths.
                ONLYDEV(g_timer.record_p("copy_paths_1_s");)
                vector<::Path> precomputed_paths;
                precomputed_paths.resize(env.num_of_agents);
                for (int i = 0; i < env.num_of_agents; ++i)
                {
                    if (planning_paths[i][0].location != execution_paths[i].back().location || planning_paths[i][0].orientation != execution_paths[i].back().orientation)
                    {
                        cerr << "agent " << i << "'s current state doesn't match with the plan" << endl; // TODO: modify this cerr.
                        exit(-1);
                    }
                    for (int j = 0; j < planning_paths[i].size(); ++j)
                    {
                        precomputed_paths[i].emplace_back(planning_paths[i][j]);
                        // we could break if we arrive at goal eariler here.
                        // if (env.goal_locations[i].empty() or (j>1 && planning_paths[i][j].location==env.goal_locations[i][0].first)){
                        if (j > 1 and (env.goal_locations[i].empty() or planning_paths[i][j].location == env.goal_locations[i][0].first))
                        {
                            break;
                        }
                    }
                    // std::cerr<<"agent "<<i<<std::endl;
                    // std::cerr<<planning_paths[i]<<std::endl;
                    // std::cerr<<precomputed_paths[i]<<std::endl;
                }
                ONLYDEV(g_timer.record_d("copy_paths_1_s", "copy_paths_1_e", "copy_paths_1");)

                // TODO: lacam2_solver should plan with starts differnt from env.curr_states but goals the same as env.goals because they are up-to-date.
                lacam2_solver->plan(env, &precomputed_paths, &starts, &goals);
                // cout<<"lacam succeed"<<endl;

                // we need to copy the new planned paths into paths
                // std::cerr<<"lacam2 paths:"<<endl;
                // for (int i=0;i<env.num_of_agents;++i) {
                //     std::cerr<<"before agent "<<i<<" "<<env.curr_states[i]<<"->"<<env.goal_locations[i][0].first<<" "<<planning_paths[i].size()<<": "<<planning_paths[i]<<std::endl;
                //     std::cerr<<lacam2_solver->paths[i]<<endl;
                // }
                int num_inconsistent = 0;
                for (int i = 0; i < env.num_of_agents; ++i)
                {
                    for (int j = 0; j < planning_paths[i].size() - 1; ++j)
                    {
                        // 对每个代理，比较原有的 planning_paths 与 LaCAM2 生成的新路径 lacam2_solver->paths
                        // 若发现位置或朝向不一致，则认为该代理的路径有“更新”，计数器 num_inconsistent 加 1，并中断该代理的内部循环。
                        if (planning_paths[i][j].location != lacam2_solver->paths[i][j].location || planning_paths[i][j].orientation != lacam2_solver->paths[i][j].orientation)
                        {
                            ++num_inconsistent;
                            break;
                        }
                    }

                    planning_paths[i].clear();
                    for (int j = 0; j < lacam2_solver->paths[i].size(); ++j)
                    {
                        planning_paths[i].emplace_back(lacam2_solver->paths[i][j].location, j, lacam2_solver->paths[i][j].orientation);
                    }
                    // std::cerr<<"agent "<<i<<" "<<env.curr_states[i]<<"->"<<env.goal_locations[i][0].first<<" "<<planning_paths[i].size()<<": "<<planning_paths[i]<<std::endl;
                }
                ONLYDEV(std::cerr << "num_inconsistent/total: " << num_inconsistent << "/" << env.num_of_agents << "=" << num_inconsistent / (float)env.num_of_agents << std::endl;)
                ONLYDEV(g_timer.record_d("lacam2_plan_s", "lacam2_plan_e", "lacam2_plan");)
                ONLYDEV(double lacam2_plan_time = g_timer.record_d("lacam2_plan_s", "lacam2_plan_e", "lacam2_plan");
                        std::cout << "Time for lacam2_plan phase: " << lacam2_plan_time << " seconds" << std::endl;)
            }

            // ONLYDEV(analyzer.snapshot(
            //     "analysis/ppaths/lacam2",
            //     executed_plan_step,
            //     paths
            // );)
        }

        // : not sure what's bug making it cannot be placed in initialize()
        ONLYDEV(g_timer.record_p("prepare_LNS_s");)
        if (env.curr_timestep == 0)
        {
            // build instace
            // 根据一系列参数构造 LNS（大邻域搜索）的管理对象
            instance = std::make_shared<Instance>(env);
            lns = std::make_shared<Parallel::GlobalManager>(
                true,
                *instance,
                HT,
                map_weights,
                agent_infos,
                read_param_json<int>(config, "neighborSize"),
                Parallel::destroy_heuristic::RANDOMWALK, // TODO: always randomwalk
                true,                                    // TODO: always Adaptive
                0.01,                                    // TODO: decay factor
                0.01,                                    // TODO: reaction factor
                read_param_json<string>(config, "initAlgo"),
                read_param_json<string>(config, "replanAlgo"),
                false, // TODO: not sipp
                read_param_json<int>(config, "window_size_for_CT"),
                read_param_json<int>(config, "window_size_for_CAT"),
                read_param_json<int>(config, "window_size_for_PATH"),
                execution_window,
                lacam2_solver->max_agents_in_use != env.num_of_agents, // TODO: has disabled agents
                read_param_json<bool>(config, "fix_ng_bug"),
                0 // TODO: screen
            );
        }

        lns->reset();
        instance->set_starts_and_goals(starts, goals);
        ONLYDEV(g_timer.record_d("prepare_LNS_s", "prepare_LNS_e", "prepare_LNS");)
        ONLYDEV(double prepare_LNS_time = g_timer.record_d("prepare_LNS_s", "prepare_LNS_e", "prepare_LNS");
                std::cout << "Time for prepare_LNS phase: " << prepare_LNS_time << " seconds" << std::endl;)

        // ONLYDEV(g_timer.record_p("prepare_LNS_s");)
        // build planner
        // PIBTPPS_option pipp_option;
        // pipp_option.windowSize = read_param_json<int>(config,"pibtWindow");
        // pipp_option.winPIBTSoft = read_param_json<int>(config,"winPibtSoftmode");

        // lns = new LNS(
        //     *instance,
        //     read_param_json<double>(config,"cutoffTime"),
        //     read_param_json<string>(config,"initAlgo"),
        //     read_param_json<string>(config,"replanAlgo"),
        //     read_param_json<string>(config,"destoryStrategy"),
        //     read_param_json<int>(config,"neighborSize"),
        //     read_param_json<int>(config,"maxIterations"),
        //     read_param_json<bool>(config,"initLNS"),
        //     read_param_json<string>(config,"initDestoryStrategy"),
        //     read_param_json<bool>(config,"sipp"),
        //     read_param_json<int>(config,"screen"),
        //     pipp_option,
        //     HT,
        //     read_param_json<int>(config,"window_size_for_CT"),
        //     read_param_json<int>(config,"window_size_for_CAT"),
        //     read_param_json<int>(config,"window_size_for_PATH")
        // );

        // ONLYDEV(g_timer.record_d("prepare_LNS_s","prepare_LNS_e","prepare_LNS");)

        // ONLYDEV(g_timer.record_p("modify_goals_s");)
        // : this might not be necessary
        // modify_goals(instance.goal_locations, env);
        // ONLYDEV(g_timer.record_d("modify_goals_s","modify_goals_e","modify_goals");)

        // copy current paths to lns paths
        // we need to do this every timestep because the goal might be updated.
        ONLYDEV(g_timer.record_p("copy_paths_2_s");)
        for (int i = 0; i < lns->agents.size(); i++)
        {
            if (lns->agents[i].id != i)
            {
                cerr << "agents are not ordered at the begining" << endl;
                exit(-1);
            }
            lns->agents[i].path.clear();
            bool goal_arrived = false;
            for (int j = 0; j < planning_paths[i].size(); ++j)
            {
                lns->agents[i].path.nodes.emplace_back(planning_paths[i][j].location, planning_paths[i][j].orientation);
                if (env.goal_locations[i].empty() or (planning_paths[i][j].location == env.goal_locations[i][0].first))
                {
                    goal_arrived = true;
                    // break;
                }
            }
            // : it is not correct on weighted maps
            if (env.goal_locations[i].empty())
            {
                lns->agents[i].path.path_cost = 0; // No task, path cost is zero
            }
            else
            {
                lns->agents[i].path.path_cost = lns->agents[i].getEstimatedPathLength(lns->agents[i].path, env.goal_locations[i][0].first, HT);
            }
            // cerr<<"agent "<<i<<": ";
            // for (int j=0;j<lns->agents[i].path.size();++j){
            //     cerr<<lacam2_solver->paths[i][j].location<<" ";
            // }
            // cerr<<endl;
        }
        ONLYDEV(g_timer.record_d("copy_paths_2_s", "copy_paths_2_e", "copy_paths_2");)

        ONLYDEV(g_timer.record_p("run_LNS_s");)
        // continue optimizing paths
        bool succ = lns->run(time_limiter);
        // if (succ)
        // {
        //     cout<<"lns succeed"<<endl;
        // } else {
        //     cout<<"lns failed"<<endl;
        //     exit(-1);
        // }

        // we cannot do this because it would make result invalid
        // deal with a special case when the goal and the start are the same.
        if (execution_window == 1)
        {
            for (int i = 0; i < lns->agents.size(); ++i)
            {
                if (lns->agents[i].path.size() < planning_window + 1)
                {
                    // in this case, actually the goal is the same as the start
                    lns->agents[i].path.nodes.resize(planning_window + 1, lns->agents[i].path.back());
                }
            }
        }
        ONLYDEV(g_timer.record_d("run_LNS_s", "run_LNS_e", "run_LNS");)

        ONLYDEV(double run_LNS_time = g_timer.record_d("run_LNS_s", "run_LNS_e", "run_LNS");
                std::cout << "Time for run_LNS phase: " << run_LNS_time << " seconds" << std::endl;)

        // save to paths
        ONLYDEV(g_timer.record_p("copy_paths_3_s");)
        // std::cerr<<"lns paths:"<<endl;
        for (int i = 0; i < planning_paths.size(); ++i)
        {
            auto &path = planning_paths[i];
            auto &new_path = lns->agents[i].path;

            // compare
            // for (int j=1;j<new_path.size();++j) {
            //     if (path[executed_plan_step+j].location!=new_path[j].location || path[executed_plan_step+j].orientation!=new_path[j].orientation){
            //         std::cerr<<"agent "<<i<<" has updated lns path"<<endl;
            //         std::cerr<<subvector(path,executed_plan_step,path.size())<<endl;
            //         std::cerr<<new_path<<endl;
            //         // std::cerr<<lns->agents[i].path<<endl;
            //         break;
            //     }
            // }

            // std::cerr<<"agent "<<i<<" has updated lns path"<<endl;
            // std::cerr<<path<<endl;
            // std::cerr<<new_path<<endl;

            path.clear();

            // cerr<<"agent "<<i<<" "<<new_path.size()<<": "<<new_path<<endl;
            for (int j = 0; j < new_path.size(); ++j)
            {
                path.emplace_back(new_path[j].location, j, new_path[j].orientation);
                // if (need_new_execution_paths && new_path[j].location==env.goal_locations[i][0].first){
                //     break;
                // }
            }
            // cerr<<"agent "<<i<<" s:"<<env.curr_states[i]<<" e:"<<env.goal_locations[i][0].first<<" c:"<<executed_plan_step<<endl;
            // std::cerr<<"agent "<<i<<" "<<env.curr_states[i]<<"->"<<env.goal_locations[i][0].first<<" "<<path.size()<<": "<<path<<endl;
            // std::cerr<<path<<endl;
        }
        ONLYDEV(g_timer.record_d("copy_paths_3_s", "copy_paths_3_e", "copy_paths_3");)

        // for (int i=0;i<paths.size();++i){
        //     cerr<<executed_plan_step<<" "<<env.curr_states[i].location<<" "<<env.curr_states[i].orientation<<endl;
        //     cerr<<"agent "<<i<<": ";
        //     for (int j=0;j<paths[i].size();++j){
        //         cerr<<paths[i][j].location<<" ";
        //     }
        //     cerr<<endl;
        // }
        ONLYDEV(g_timer.record_d("plan_s", "plan_e", "plan");)

        ONLYDEV(
            double plan_time = g_timer.record_d("_plan_s", "_plan_e", "_plan");
            if (plan_time > max_plan_time) {
                max_plan_time = plan_time;
            } std::cerr
            << "max_plan_time: " << max_plan_time << endl;
            g_timer.remove_d("_plan");
            // if (plan_time>1.0){
            //     exit(-1);
            // }
        )
    }

    void LNSSolver::observe(const SharedEnvironment &env)
    {
        // for (int i=0;i<env.num_of_agents;++i) {
        //     paths[i].clear();
        // }

        ONLYDEV(g_timer.record_p("observe_s");)

        if (execution_paths[0].size() == 0)
        {
            // 是不是第一次观察（即还没有规划好的路径），
            //  the first step?
            for (int i = 0; i < env.num_of_agents; ++i)
            {
                execution_paths[i].push_back(env.curr_states[i]);
                planning_paths[i].push_back(env.curr_states[i]);
                // std::cerr<<"agent "<<i<<" "<<env.curr_states[i]<<"->"<<env.goal_locations[i][0].first<<std::endl;
            }
            executed_step = 0;
        }
        else
        {
            bool match = true;
            for (int i = 0; i < execution_paths.size(); ++i)
            {
                // cerr<<"agent "<<i<<" curr state:"<<env.curr_states[i]<<", "<<" goal:"<<env.goal_locations[i][0].first<<endl;
                if (executed_step + 1 >= execution_paths[i].size())
                {
                    cerr << "executed_step exceed the execution paths:" << executed_step << execution_paths[i].size() << endl;
                    exit(-1);
                }
                // 在执行过程中是否已经按照预期执行了下一步（更新 executed_step），
                if (execution_paths[i][executed_step + 1].location != env.curr_states[i].location || execution_paths[i][executed_step + 1].orientation != env.curr_states[i].orientation)
                {
                    match = false;
                }
            }
            // otherwise, the previous execution is delayed.
            if (match)
            {
                ++executed_step;
            }
        }

        // 是否已经“耗尽”了当前的执行指令，从而需要生成新的执行路径。
        //  if run out of execution instructions, we need to copy new ones into execution_paths
        if (executed_step == execution_paths[0].size() - 1)
        {
            // std::cerr<<"need new execution paths"<<std::endl;
            need_new_execution_paths = true;
        }
        else
        {
            need_new_execution_paths = false;
        }

        ONLYDEV(g_timer.record_d("observe_s", "observe_e", "observe");)
        ONLYDEV(double observe_time = g_timer.record_d("observe_s", "observe_e", "observe");
                std::cout << "Time for observe phase: " << observe_time << " seconds" << std::endl;)
    }

    void LNSSolver::get_step_actions(const SharedEnvironment &env, vector<Action> &actions)
    {
        ONLYDEV(g_timer.record_p("get_step_actions_s");)
        // cout << "actions.size() = " << actions.size() << endl;
        assert(actions.empty());

#ifndef NO_ROT
        // get current state and current timestep
        // vector<State> planned_next_states;
        // vector<State> next_states;
        // int next_plan_step=executed_plan_step+1;
        // for (int i=0;i<env.num_of_agents;++i) {
        //     if (next_plan_step>=paths[i].size()){
        //         // todo: we need to wait for the new plan to come out.
        //         exit(-1);
        //     }
        //     planned_next_states.emplace_back(paths[i][next_plan_step].location,-1,-1);
        //     next_states.emplace_back(-1,-1,-1);
        // }

        // slow_executor.execute(&(env.curr_states),&planned_next_states,&next_states);

        // for (int i=0;i<env.num_of_agents;++i) {
        //     if (next_states[i].timestep!=env.curr_states[i].timestep+1) {
        //         std::cerr<<"agent "<<i<<"'s plan doesn't show consecutive timesteps: "<<next_states[i].timestep<<" "<<env.curr_states[i].timestep<<endl;
        //         exit(-1);
        //     }
        // }

        if (need_new_execution_paths)
        {
            // copy planning paths to execution paths
            for (int i = 0; i < env.num_of_agents; ++i)
            {
                execution_paths[i].clear();
                for (int j = 0; j < execution_window + 1; ++j)
                {
                    execution_paths[i].emplace_back(planning_paths[i][j].location, j, planning_paths[i][j].orientation);
                }
                if (execution_paths[i].size() != execution_window + 1)
                {
                    std::cerr << "execution paths size is not correct" << std::endl;
                    exit(-1);
                }
                executed_step = 0;
            }

            // keep only the remaining planning paths
            // std::cerr<<"truncated planning paths"<<std::endl;
            for (int i = 0; i < env.num_of_agents; ++i)
            {
                planning_paths[i].erase(planning_paths[i].begin(), planning_paths[i].begin() + execution_window);
                if (
                    // !env.goal_locations[i].empty() and
                    // planning_paths[i].back().location != env.goal_locations[i][0].first and
                    planning_paths[i].size() != planning_window + 1 - execution_window)
                {
                    std::cerr << "planning_paths[" << i << "].size() = " << planning_paths[i].size() << ", expected size = " << planning_window + 1 - execution_window << std::endl;
                    std::cerr << "env.goal_locations[" << i << "]: ";
                    for (const auto &goal : env.goal_locations[i])
                    {
                        std::cerr << goal.first << " ";
                    }
                    std::cerr << std::endl;
                    std::cerr << "env.curr_states[" << i << "].location = " << env.curr_states[i].location << std::endl;
                    std::cerr << "agent " << i << ": " << planning_paths[i] << std::endl;
                    // 检查剩余的规划路径长度是否与预期一致：原本规划窗口长度为 planning_window+1，
                    // 减去已经执行的 execution_window 后，剩余的应为 planning_window+1-execution_window
                    std::cerr << "planning paths size is not correct" << std::endl;
                    exit(-1);
                }
                // std::cerr<<"agent "<<i<<": "<<planning_paths[i]<<std::endl;
            }
        }

        if (num_task_completed >= max_task_completed)
        { // only for competition purpose, don't reveal too much information, otherwise it is too tired to overfit... do something fun instead!
            for (int i = 0; i < env.num_of_agents; ++i)
            {
                // actions.push_back(Action::W);
                actions.at(i) = Action::W;
            }
        }
        else
        {
            // get actions from current state and next state
            for (int i = 0; i < env.num_of_agents; ++i)
            {
                // we will get action indexed at executed_plan_step+1
                if (execution_paths[i].size() <= executed_step + 1)
                {
                    cerr << "wierd error for agent " << i << ". path length: " << execution_paths[i].size() << ", " << "executed_plan_step+1: " << executed_step + 1 << endl;
                    exit(-1);
                }

                if (execution_paths[i][executed_step].location != env.curr_states[i].location || execution_paths[i][executed_step].orientation != env.curr_states[i].orientation)
                {
                    cerr << "agent " << i << "'s current state doesn't match with the executed plan" << endl;
                    exit(-1);
                }

                // actions.push_back(get_action_from_states(execution_paths[i][executed_step],execution_paths[i][executed_step+1]));
                actions.at(i) = get_action_from_states(execution_paths[i][executed_step], execution_paths[i][executed_step + 1]);
                // cout << "actions.size() = " << actions.size() << endl;
                // assume perfect execution
                if (!env.goal_locations[i].empty() and execution_paths[i][executed_step + 1].location == env.goal_locations[i][0].first)
                {
                    ++num_task_completed;
                }
            }
        }
#else

        for (int i = 0; i < env.num_of_agents; ++i)
        {
            // we will get action indexed at executed_plan_step+1
            if (paths[i].size() <= executed_plan_step + 1)
            {
                cerr << "wierd error for agent " << i << ". path length: " << paths[i].size() << ", " << "executed_plan_step+1: " << executed_plan_step + 1 << endl;
                assert(false);
            }
            actions.push_back(get_action_from_states(paths[i][executed_plan_step], paths[i][executed_plan_step + 1]));
        }

#endif

        ONLYDEV(
            if (!action_model.is_valid(env.curr_states, actions)) {
                cerr << "planed actions are not valid in executed_step " << executed_step + 1 << "!" << endl;
                for (int i = 0; i < env.num_of_agents; ++i)
                {
                    cerr << "agent " << i << " " << env.curr_states[i] << " " << actions[i] << endl;
                }
                exit(-1);
            })

        ONLYDEV(g_timer.record_d("get_step_actions_s", "get_step_actions_e", "get_step_actions");)
    }

} // end namespace LNS
