#include "scheduler.h"
#include "util/Timer.h"
#include <omp.h>
#include "MatchingPriorityQueue.h"
#include "BMatchingPriorityQueue.h"
#include <algorithm>
#include <execution>
#include "planner.h"
#include "dynamic_hungarian_assignment.h"
#include "WarehouseMatchingPriorityQueue.h"
#include "nlohmann/json.hpp"

namespace MyPlanner {

using COST_TYPE = float;
const COST_TYPE COST_MAX = FLT_MAX;

nlohmann::json load_map_stats(SharedEnvironment * env) {
    // load configs
	string _path="data/map_stats/"+env->map_name.substr(0,env->map_name.find_last_of("."))+".json";
    std::ifstream f(_path);
    nlohmann::json map_stats;
    try
    {
        map_stats = nlohmann::json::parse(f);
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cout << "Failed to load " << _path << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }

    std::cout<<"Successfully loaded map stats from "<<_path<<std::endl;
    std::cout<<"num empty locs: "<<map_stats["num_empty_locs"]<<std::endl;
    std::cout<<"num E locs: "<<map_stats["num_E_locs"]<<std::endl;
    std::cout<<"num S locs: "<<map_stats["num_S_locs"]<<std::endl;
    
    return map_stats;
}

void MyScheduler::schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    scheduler_name = wppl_planner->config["scheduler_name"];
    std::cout<<"scheduler name: "<<scheduler_name<<std::endl;

    // TODO: make configurable
    if (env->map_name.find("random") != std::string::npos) {
        skip_matching_if_no_free_agents = false;
        no_matching_for_disable_agents = false;
    }

    // TODO: load endpoints_loc_to_id_mapping from file
    auto map_stats = load_map_stats(env);
    endpoint_locs = map_stats["E_locs"].get<std::vector<int> >();

    for (int i=0;i<(int)endpoint_locs.size();++i) {
        endpoints_loc_to_id_mapping[endpoint_locs[i]] = i;
    }

}

void MyScheduler::schedule_plan_default(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
     //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // if (env->curr_timestep==0){
    //     num_task_reveal=float(env->new_tasks.size())/env->num_of_agents;
    //     // std::cout<<"num_task_reveal: "<<num_task_reveal<<std::endl;
    //     avg_num_locs_per_task=0;
    //     for (int i=0;i<env->new_tasks.size();++i){
    //         int num_locs=env->task_pool[env->new_tasks[i]].locations.size();
    //         avg_num_locs_per_task+=num_locs;
    //         if (num_locs>max_num_locs_per_task) max_num_locs_per_task=num_locs;
    //         if (num_locs<min_num_locs_per_task) min_num_locs_per_task=num_locs;
    //         if (num_locs==2) num_locs_2_prob+=1;
    //     }
    //     avg_num_locs_per_task/=env->new_tasks.size();
    //     num_locs_2_prob/=env->new_tasks.size();

    //     // std::cout<<"min_num_locs_per_task: "<<min_num_locs_per_task<<std::endl;
    //     // std::cout<<"avg_num_locs_per_task: "<<avg_num_locs_per_task<<std::endl;
    //     // std::cout<<"num_locs_2_prob: "<<num_locs_2_prob<<std::endl;
    // }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, c_loc, count;
    COST_TYPE dist, min_task_makespan;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = COST_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += wppl_planner->heuristics->get(c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}

void MyScheduler::schedule_plan_default2(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start scheduler default2"<<std::endl;
    std::cout<<"warning: this scheduler use int-type cost"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {
        if (env->curr_task_schedule[i_agent] == -1)
            assignable_agents_indices.push_back(i_agent);
        else {
            auto task_id = env->curr_task_schedule[i_agent];
            auto & task = env->task_pool[task_id];
            if (task.idx_next_loc == 0)
                assignable_agents_indices.push_back(i_agent);
        }
    }

    std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
    std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    std::unordered_map<int, int> assignment;
    std::vector<bool> matched_tasks(assignable_tasks_indices.size(), false);

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    g_timer.record_p("compute_matching_cost_s");
    // std::vector<std::tuple<int,int,int> > edges;
    // #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        COST_TYPE min_cost=COST_MAX;
        int min_cost_task_idx=-1;
        for (int j=0;j<assignable_tasks_indices.size();++j) {
            if (!matched_tasks[j]) {
                int i_agent = assignable_agents_indices[i];
                int i_task = assignable_tasks_indices[j];
                COST_TYPE dist = 0;
                int c_loc = env->curr_states.at(i_agent).location;
                for (int loc : env->task_pool[i_task].locations){
                    dist += wppl_planner->heuristics->get(c_loc, loc);
                    c_loc = loc;
                }
                if (dist<min_cost){
                    min_cost=dist;
                    min_cost_task_idx=j;
                }
            }
        }
        assignment[i] = min_cost_task_idx;
        matched_tasks[min_cost_task_idx] = true;
    }

    std::cout<<"cost matrix is built"<<std::endl;
    g_timer.record_d("compute_matching_cost_s","compute_matching_cost");

    // build the proposed schedule
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
            continue;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

// TODO(rivers): timing and float type
void MyScheduler::schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"warning: this scheduler use int-type cost"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {
        if (env->curr_task_schedule[i_agent] == -1)
            assignable_agents_indices.push_back(i_agent);
        else {
            auto task_id = env->curr_task_schedule[i_agent];
            auto & task = env->task_pool[task_id];
            if (task.idx_next_loc == 0)
                assignable_agents_indices.push_back(i_agent);
        }
    }

    // std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
    // std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    // create cost matrix
    std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        for (int j=0;j<assignable_tasks_indices.size();++j) {
            int i_agent = assignable_agents_indices[i];
            int i_task = assignable_tasks_indices[j];
            COST_TYPE dist = 0;
            int c_loc = env->curr_states.at(i_agent).location;
            for (int loc : env->task_pool[i_task].locations){
                // dist += MyPlanner::get_h(env, c_loc, loc);
                dist += wppl_planner->heuristics->get(c_loc, loc);
                c_loc = loc;
            }
            // always reachable by assumption, so we don't need to use INF7f in dynamic_hungarian_assignment.h
            cost_matrix[i][j] = dist;
        }
    }

    std::cout<<"cost matrix is built"<<std::endl;

    // solve the hungurian assignment
    unordered_map<int, int> assignment;

    // build dynamic hungarian assigner
    DynamicHungarianAssignment assigner;
    assigner.initHungarian(assignable_agents_indices.size(), assignable_tasks_indices.size());
    // auto ans=
    assigner.firstSolution(cost_matrix, assignment);

    std::cout<<"first solution is obtained"<<std::endl;

    // build the proposed schedule

    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

void MyScheduler::schedule_plan_greedy_matching2(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching2"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;
    if (skip_matching_if_no_free_agents && env->new_freeagents.size() == 0){
        std::cout<<"no new free agents: skip matching..."<<std::endl;
        return;
    }
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    auto & agent_infos = *(wppl_planner->lacam2_solver->agent_infos);

    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {   
        // NOTE: for random map, we disable agents whose goals are at the corner.
        // but if new task comes, we can try to reassign new tasks to them.
        if (no_matching_for_disable_agents && agent_infos[i_agent].disabled) {
            int curr_task_id = env->curr_task_schedule[i_agent];
            if (curr_task_id!=-1 && env->task_pool[curr_task_id].is_finished()) {
                proposed_schedule[i_agent] = -1; // no longer assign tasks to disabled agents
            }
        } else {
            // only enabled agents will be assign new tasks
            if (env->curr_task_schedule[i_agent] == -1)
                assignable_agents_indices.push_back(i_agent);
            else {
                auto task_id = env->curr_task_schedule[i_agent];
                auto & task = env->task_pool[task_id];
                if (task.idx_next_loc == 0)
                    assignable_agents_indices.push_back(i_agent);
            }
        }
    }

    // ONLYDEV(
        std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
        std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;
    // )

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    std::vector<COST_TYPE> task_costs(assignable_tasks_indices.size(), 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_tasks_indices.size();++i) {
        int i_task = assignable_tasks_indices[i];
        COST_TYPE & dist = task_costs[i];
        for (int j=1;j<env->task_pool[i_task].locations.size();++j){
            dist += wppl_planner->heuristics->get(
                env->task_pool[i_task].locations[j-1], 
                env->task_pool[i_task].locations[j]
            );
        }
    }

    // create cost matrix
    // std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    ONLYDEV(
        g_timer.record_p("compute_matching_cost_s");
    )
    auto matching_pq=MatchingPriorityQueue(assignable_agents_indices.size(), assignable_tasks_indices.size());
    // std::vector<std::tuple<int,int,int> > edges;
    
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        ONLYDEV(
            if (i==0) {
                int num_threads=omp_get_num_threads();
                std::cout<<"scheduler greedy matching2 uses num_threads: "<<num_threads<<std::endl;
            }
        )

        for (int j=0;j<assignable_tasks_indices.size();++j) {
            int i_agent = assignable_agents_indices[i];
            int i_task = assignable_tasks_indices[j];
            COST_TYPE dist = wppl_planner->heuristics->get(
                env->curr_states.at(i_agent).location, 
                env->curr_states.at(i_agent).orientation,
                env->task_pool[i_task].locations[0]
            )+task_costs[j];
            // always reachable by assumption, so we don't need to use INF7f in dynamic_hungarian_assignment.h
            // cost_matrix[i][j] = dist;
            // edges.emplace_back(dist,i,j);
            matching_pq.push(i,j,dist);
        }
        matching_pq.sort(i);
    }

    ONLYDEV(
        std::cout<<"cost matrix is built"<<std::endl;
        g_timer.record_d("compute_matching_cost_s","compute_matching_cost");
    )

    ONLYDEV(
        g_timer.record_p("matching_s");
    )
    std::unordered_map<int, int> assignment;
    assignment=matching_pq.get_assignment();
    ONLYDEV(
        g_timer.record_d("matching_s","matching");
    )

    // build the proposed schedule
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
            continue;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        // TODO: remove this line? since current we are just proposing, not really assign?
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}


void MyScheduler::schedule_plan_greedy_matching4(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching4"<<std::endl;
    std::cout<<"warning: this scheduler use int-type cost"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;
    if (skip_matching_if_no_free_agents && env->new_freeagents.size() == 0){
        std::cout<<"no new free agents: skip matching..."<<std::endl;
        return;
    }
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    auto & agent_infos = *(wppl_planner->lacam2_solver->agent_infos);

    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {   
        // NOTE: for random map, we disable agents whose goals are at the corner.
        // but if new task comes, we can try to reassign new tasks to them.
        if (no_matching_for_disable_agents && agent_infos[i_agent].disabled) {
            int curr_task_id = env->curr_task_schedule[i_agent];
            if (curr_task_id!=-1 && env->task_pool[curr_task_id].is_finished()) {
                proposed_schedule[i_agent] = -1; // no longer assign tasks to disabled agents
            }
        } else {
            // only enabled agents will be assign new tasks
            if (env->curr_task_schedule[i_agent] == -1)
                assignable_agents_indices.push_back(i_agent);
            else {
                auto task_id = env->curr_task_schedule[i_agent];
                auto & task = env->task_pool[task_id];
                if (task.idx_next_loc == 0)
                    assignable_agents_indices.push_back(i_agent);
            }
        }
    }

    // ONLYDEV(
        std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
        std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;
    // )

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    // ONLYDEV(
        g_timer.record_p("compute_matching_cost_s");
    // )

    // cost, j_endpoint, k_task
    std::vector<std::tuple<int,int,int> > cost_endpoint_task_tuples(assignable_tasks_indices.size());
    #pragma omp parallel for schedule(dynamic,1)
    for (int k_task=0;k_task<assignable_tasks_indices.size();++k_task) {
        int task_id = assignable_tasks_indices[k_task];
        COST_TYPE dist = 0;
        for (int j=1;j<env->task_pool[task_id].locations.size();++j){
            dist += wppl_planner->heuristics->get(
                env->task_pool[task_id].locations[j-1], 
                env->task_pool[task_id].locations[j]
            );
        }
        int j_endpoint = endpoints_loc_to_id_mapping[env->task_pool[task_id].locations[0]];
        cost_endpoint_task_tuples[k_task] = {dist, j_endpoint, k_task};
    }

    std::sort(cost_endpoint_task_tuples.begin(), cost_endpoint_task_tuples.end());

    // for (int i=0;i<cost_endpoint_task_tuples.size();++i) {
    //     auto & tpl=cost_endpoint_task_tuples[i];
    //     std::cout<<"cost_endpoint_task_tuples: "<<std::get<0>(tpl)<<" "<<std::get<1>(tpl)<<" "<<std::get<2>(tpl)<<std::endl;
    // }


    auto matching_pq=BMatchingPriorityQueue(assignable_agents_indices.size(), endpoint_locs.size());

    // push tasks
    for (int i=0;i<cost_endpoint_task_tuples.size();++i) {
        auto & tpl=cost_endpoint_task_tuples[i];
        // NOTE: we push into cost-ordered (endpoint_id,task_id)
        matching_pq.push_endpoint_task(std::get<1>(tpl), std::get<2>(tpl));      
    }

    // push agents
    // TODO: actually, for warehouse we can precompute the closest endpoints to a location
    // #pragma omp parallel for schedule(dynamic,1)
    for (int i_agent=0;i_agent<assignable_agents_indices.size();++i_agent) {
        ONLYDEV(
            if (i_agent==0) {
                int num_threads=omp_get_num_threads();
                std::cout<<"scheduler greedy matching2 uses num_threads: "<<num_threads<<std::endl;
            }
        )

        int agent_id = assignable_agents_indices[i_agent];
        for (int j_endpoint=0;j_endpoint<endpoint_locs.size();++j_endpoint) {
            COST_TYPE dist = wppl_planner->heuristics->get(
                env->curr_states.at(i_agent).location, 
                env->curr_states.at(i_agent).orientation,
                endpoint_locs[j_endpoint]
            );
            // if (dist==0)
            // std::cout<<"push agent endpoint: "<<i_agent<<" "<<j_endpoint<<" "<<dist<<" "<<env->curr_states.at(i_agent).location<<" "<<endpoint_locs[j_endpoint]<<std::endl;

            // if (i_agent==599 && j_endpoint==0)
            //     std::cout<<"xxx push agent endpoint: "<<i_agent<<" "<<j_endpoint<<" "<<dist<<" "<<env->curr_states.at(i_agent).location<<" "<<endpoint_locs[j_endpoint]<<std::endl;


            matching_pq.push_agent_endpoint(i_agent, j_endpoint, dist);
        }
        matching_pq.sort(i_agent);
    }

    // ONLYDEV(
        std::cout<<"cost matrix is built"<<std::endl;
        g_timer.record_d("compute_matching_cost_s","compute_matching_cost");
    // )

    // ONLYDEV(
        g_timer.record_p("matching_s");
        // )
    std::unordered_map<int, int> assignment;
    assignment=matching_pq.get_assignment();
    // ONLYDEV(
        g_timer.record_d("matching_s","matching");
    // )

    // build the proposed schedule
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
            continue;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        // TODO: remove this line? since current we are just proposing, not really assign?
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

// TODO: I guess all the agents that moved to a different location can be reassigined
void MyScheduler::schedule_plan_greedy_matching3(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching3"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;
    
    if (skip_matching_if_no_free_agents && env->new_freeagents.size() == 0){
        std::cout<<"no new free agents: skip matching..."<<std::endl;
        return;
    }
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    for (int agent_id : env->new_freeagents)
    {
        assignable_agents_indices.push_back(agent_id);
    }

    // ONLYDEV(
        std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
        std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;
    // )

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    std::vector<COST_TYPE> task_costs(assignable_tasks_indices.size(), 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_tasks_indices.size();++i) {
        int i_task = assignable_tasks_indices[i];
        COST_TYPE & dist = task_costs[i];
        for (int j=1;j<env->task_pool[i_task].locations.size();++j){
            dist += wppl_planner->heuristics->get(env->task_pool[i_task].locations[j-1], env->task_pool[i_task].locations[j]);
        }
    }

    // create cost matrix
    // std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    ONLYDEV(g_timer.record_p("compute_matching_cost_s");)
    auto matching_pq=MatchingPriorityQueue(assignable_agents_indices.size(), assignable_tasks_indices.size());
    // std::vector<std::tuple<int,int,int> > edges;
    
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        ONLYDEV(
            if (i==0) {
                int num_threads=omp_get_num_threads();
                std::cout<<"scheduler greedy matching2 uses num_threads: "<<num_threads<<std::endl;
            }
        )

        for (int j=0;j<assignable_tasks_indices.size();++j) {
            int i_agent = assignable_agents_indices[i];
            int i_task = assignable_tasks_indices[j];
            COST_TYPE dist = wppl_planner->heuristics->get(
                env->curr_states.at(i_agent).location, 
                env->curr_states.at(i_agent).orientation,
                env->task_pool[i_task].locations[0]
            )+task_costs[j];
            // always reachable by assumption, so we don't need to use INF7f in dynamic_hungarian_assignment.h
            // cost_matrix[i][j] = dist;
            // edges.emplace_back(dist,i,j);
            matching_pq.push(i,j,dist);
        }
        matching_pq.sort(i);
    }

    ONLYDEV(
        std::cout<<"cost matrix is built"<<std::endl;
        g_timer.record_d("compute_matching_cost_s","compute_matching_cost");
    )

    ONLYDEV(g_timer.record_p("matching_s");)
    std::unordered_map<int, int> assignment;
    assignment=matching_pq.get_assignment();
    ONLYDEV(g_timer.record_d("matching_s","matching");)

    // build the proposed schedule
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
            continue;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        // TODO: remove this line? since current we are just proposing, not really assign?
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

void MyScheduler::schedule_plan_greedy_matching_warehouse(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching warehouse"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // make a copy of the current schedule
    proposed_schedule = env->curr_task_schedule;
    if (skip_matching_if_no_free_agents && env->new_freeagents.size() == 0){
        std::cout<<"no new free agents: skip matching..."<<std::endl;
        return;
    }
  
    clock_t start = clock();

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_indices.push_back(task.task_id);
        }
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    auto & agent_infos = *(wppl_planner->lacam2_solver->agent_infos);

    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {   
        // NOTE: for random map, we disable agents whose goals are at the corner.
        // but if new task comes, we can try to reassign new tasks to them.
        if (no_matching_for_disable_agents && agent_infos[i_agent].disabled) {
            int curr_task_id = env->curr_task_schedule[i_agent];
            if (curr_task_id!=-1 && env->task_pool[curr_task_id].is_finished()) {
                proposed_schedule[i_agent] = -1; // no longer assign tasks to disabled agents
            }
        } else {
            // only enabled agents will be assign new tasks
            if (env->curr_task_schedule[i_agent] == -1)
                assignable_agents_indices.push_back(i_agent);
            else {
                auto task_id = env->curr_task_schedule[i_agent];
                auto & task = env->task_pool[task_id];
                if (task.idx_next_loc == 0)
                    assignable_agents_indices.push_back(i_agent);
            }
        }
    }

    // ONLYDEV(
        std::cout<<"assignable_agents_indices.size() "<<assignable_agents_indices.size()<<std::endl;
        std::cout<<"assignable_tasks_indices.size() "<<assignable_tasks_indices.size()<<std::endl;
    // )

    if (assignable_agents_indices.size()>assignable_tasks_indices.size())
    {
        std::cout<<"error: we always ensure more tasks than agents"<<std::endl;
        exit(-1);
    }

    if (assignable_agents_indices.size() == 0 || assignable_tasks_indices.size() == 0)
        return;

    g_timer.record_p("compute_task_cost_s");

    std::vector<COST_TYPE> task_costs(assignable_tasks_indices.size(), 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_tasks_indices.size();++i) {
        int i_task = assignable_tasks_indices[i];
        COST_TYPE & dist = task_costs[i];
        for (int j=1;j<env->task_pool[i_task].locations.size();++j){
            dist += wppl_planner->heuristics->get(
                env->task_pool[i_task].locations[j-1], 
                env->task_pool[i_task].locations[j]
            );
        }
    }

    g_timer.record_d("compute_task_cost_s","compute_task_cost");

    // create cost matrix
    // std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    ONLYDEV(
        g_timer.record_p("compute_matching_cost_s");
    )
    auto matching_pq=WarehouseMatchingPriorityQueue(assignable_agents_indices.size(), endpoint_locs.size());


    // associate all tasks to all endpoints
    for (int i=0;i<assignable_tasks_indices.size();++i) {
        int i_task = assignable_tasks_indices[i];
        int first_loc = env->task_pool[i_task].locations[0];
        int endpoint_id = endpoints_loc_to_id_mapping[first_loc];
        matching_pq.push_task(endpoint_id,i,task_costs[i]);
    }

    // g_timer.record_p("compute_ae_cost_s");

    // associate all agents to all endpoints
    #pragma omp parallel for schedule(dynamic,1)
    for (size_t endpoint_id=0;endpoint_id<endpoint_locs.size();++endpoint_id) {
        int endpoint_loc = endpoint_locs[endpoint_id];
        for (int i=0;i<assignable_agents_indices.size();++i) {
            int i_agent = assignable_agents_indices[i];
            COST_TYPE dist = wppl_planner->heuristics->get(
                env->curr_states.at(i_agent).location, 
                env->curr_states.at(i_agent).orientation,
                endpoint_loc
            );
            matching_pq.push_agent(endpoint_id,i,dist);
        }
    }

    // g_timer.record_d("compute_ae_cost_s","compute_ae_cost");

    // g_timer.record_p("sort_all_s");
    // sort for all endpoints
    matching_pq.sort_all();
    // g_timer.record_d("sort_all_s","sort_all");

    ONLYDEV(
        // std::cout<<"cost matrix is built"<<std::endl;
        g_timer.record_d("compute_matching_cost_s","compute_matching_cost");
    )

    // for (size_t endpoint_id=0;endpoint_id<E_locs.size();++endpoint_id) {
    //     std::cout<<"endpoint_id: "<<endpoint_id<<std::endl;
    //     std::cout<<"num agents: "<<matching_pq.endpoints[endpoint_id].agents.size()<<std::endl;
    //     std::cout<<"num tasks: "<<matching_pq.endpoints[endpoint_id].tasks.size()<<std::endl;
    //     if (endpoint_id==0) {
    //         std::cout<<"agents: ";
    //         for (auto & agent : matching_pq.endpoints[endpoint_id].agents) {
    //             std::cout<<std::get<0>(agent)<<","<<std::get<1>(agent)<<" ";
    //         }
    //         std::cout<<std::endl;
    //         std::cout<<"tasks: ";
    //         for (auto & task : matching_pq.endpoints[endpoint_id].tasks) {
    //             std::cout<<std::get<0>(task)<<","<<std::get<1>(task)<<" ";
    //         }
    //         std::cout<<std::endl;
    //     }
    // }


    ONLYDEV(
        g_timer.record_p("matching_s");
    )
    std::unordered_map<int, int> assignment;
    assignment=matching_pq.get_assignment();
    ONLYDEV(
        g_timer.record_d("matching_s","matching");
    )

    // build the proposed schedule
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
            continue;
        }
        int i_agent = assignable_agents_indices[it.first];
        int i_task = assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = env->task_pool[i_task].task_id;
        // TODO: remove this line? since current we are just proposing, not really assign?
        env->task_pool[i_task].agent_assigned = i_agent;
    }
    
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}


void MyScheduler::schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    ONLYDEV(g_timer.record_p("schedule_plan_s");)

    // we move disable agents here, so that the scheduler could also make use of this information.
    // TODO: perhaps we can disabling more smartly, e.g., dynamically, according to the distance to the tasks
    if (env->curr_timestep==0 && wppl_planner->lacam2_solver->max_agents_in_use<env->num_of_agents) {
        wppl_planner->lacam2_solver->disable_agents(*env);
    }

    if (scheduler_name == "default") {
        schedule_plan_default(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "default2") {
        schedule_plan_default2(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "matching") {
        schedule_plan_matching(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "greedy_matching2") {
        schedule_plan_greedy_matching2(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "greedy_matching3") {
        schedule_plan_greedy_matching3(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "greedy_matching4") {
        schedule_plan_greedy_matching4(time_limit, proposed_schedule, env);
    } else if (scheduler_name == "greedy_matching_warehouse") {
        schedule_plan_greedy_matching_warehouse(time_limit, proposed_schedule, env);
    } else {
        std::cerr<<"unknown scheduler name"<<scheduler_name<<std::endl;
        exit(-1);
    }
    ONLYDEV(g_timer.record_d("schedule_plan_s","schedule_plan");)
    ONLYDEV(g_timer.print_all_d();)
}

}
