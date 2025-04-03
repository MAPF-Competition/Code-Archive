#include "scheduler.h"
#include "dynamic_hungarian_assignment.h"
#include "util/HeuristicTable.h"
#include "util/Timer.h"
#include <omp.h>
#include "MatchingPriorityQueue.h"
#include <algorithm>
#include <execution>
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <unordered_map>
#include "PythonWrapper.h"

namespace MyPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{

    // auto python_wrapper = PythonWrapper();
    // python_wrapper.test();

    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    MyPlanner::init_heuristics(env);
    mt.seed(0);
}

void schedule_plan_dummy(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env) {
    proposed_schedule = std::vector<int>(env->num_of_agents, -1);
}

void schedule_plan_default(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
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
        min_task_makespan = INT_MAX;
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
                dist += UTIL::static_heuristic_table->get(c_loc, loc);
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

// TODO(rivers): timing and float type
void schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule
  
    clock_t start = clock();

    std::unordered_map<int, int> task_id2i_task;
    for (int i=0;i<env->task_pool.size();i++)
    {
        task_id2i_task[env->task_pool[i].task_id] = i;
    }

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (int i_task=0;i_task<env->task_pool.size();i_task++)
    {
        if (env->task_pool[i_task].agent_assigned == -1 
            || env->task_pool[i_task].idx_next_loc==0
            )
            assignable_tasks_indices.push_back(i_task);
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {
        if (env->curr_task_schedule[i_agent] == -1)
            assignable_agents_indices.push_back(i_agent);
        else {
            auto task_id = env->curr_task_schedule[i_agent];
            auto i_task = task_id2i_task[task_id];
            if (env->task_pool[i_task].idx_next_loc == 0)
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
            int dist = 0;
            int c_loc = env->curr_states.at(i_agent).location;
            for (int loc : env->task_pool[i_task].locations){
                // dist += MyPlanner::get_h(env, c_loc, loc);
                dist += UTIL::static_heuristic_table->get(c_loc, loc);
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


template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

// TODO(rivers): timing and float type
void schedule_plan_greedy_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching"<<std::endl;
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule
  
    clock_t start = clock();

    std::unordered_map<int, int> task_id2i_task;
    for (int i=0;i<env->task_pool.size();i++)
    {
        task_id2i_task[env->task_pool[i].task_id] = i;
    }

    // get all the tasks that are not open
    std::vector<int> assignable_tasks_indices;
    for (int i_task=0;i_task<env->task_pool.size();i_task++)
    {
        if (env->task_pool[i_task].agent_assigned == -1 
            || env->task_pool[i_task].idx_next_loc==0
            )
            assignable_tasks_indices.push_back(i_task);
    }

    // get all the agents that don't have a open task
    std::vector<int> assignable_agents_indices;
    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {
        if (env->curr_task_schedule[i_agent] == -1)
            assignable_agents_indices.push_back(i_agent);
        else {
            auto task_id = env->curr_task_schedule[i_agent];
            auto i_task = task_id2i_task[task_id];
            if (env->task_pool[i_task].idx_next_loc == 0)
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

    // create cost matrix
    // std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    UTIL::g_timer.record_p("compute_matching_cost_s");
    std::vector<std::tuple<int,int,int> > edges(
        assignable_agents_indices.size()*assignable_tasks_indices.size()
    );
    // std::vector<std::vector<std::tuple<int,int,int> > > agent_edges(
    //     assignable_agents_indices.size(),
    //     std::vector<std::tuple<int,int,int> >(assignable_tasks_indices.size())
    // );
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        for (int j=0;j<assignable_tasks_indices.size();++j) {
            int i_agent = assignable_agents_indices[i];
            int i_task = assignable_tasks_indices[j];
            int dist = 0;
            int c_loc = env->curr_states.at(i_agent).location;
            for (int loc : env->task_pool[i_task].locations){
                dist += UTIL::static_heuristic_table->get(c_loc, loc);
                c_loc = loc;
            }
            // always reachable by assumption, so we don't need to use INF7f in dynamic_hungarian_assignment.h
            // cost_matrix[i][j] = dist;
            // agent_edges[i][j]={dist,i,j};
            // edges.emplace_back(dist,i,j);
            edges[i*assignable_tasks_indices.size()+j]={dist,i,j};
        }
    }

    // for (int i=0;i<assignable_agents_indices.size();++i) {
    //     for (auto & tpl: agent_edges[i]) {
    //         edges.push_back(tpl);
    //     }
    // }

    std::cout<<"cost matrix is built"<<std::endl;

    UTIL::g_timer.record_d("compute_matching_cost_s","compute_matching_cost");


    UTIL::g_timer.record_p("matching_s");
    UTIL::g_timer.record_p("sort_edges_s");
    std::sort(edges.begin(),edges.end());
    UTIL::g_timer.record_d("sort_edges_s","sort_edges");

    std::vector<bool> matched_agents(assignable_agents_indices.size(),false);
    std::vector<bool> matched_tasks(assignable_tasks_indices.size(),false);

    std::unordered_map<int,int> assignment;

    for (int i_edge=0;i_edge<edges.size();++i_edge) {
        int cost,i,j;
        std::tie(cost,i,j)=edges[i_edge];

        if (!matched_agents[i] && !matched_tasks[j]) {
            matched_agents[i] = true;
            matched_tasks[j] = true;
            assignment[i] = j;
        }

    }
    UTIL::g_timer.record_d("matching_s","matching");

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
// TODO: probably we also want to check stableness if there are multiple edges with the same costs
void schedule_plan_greedy_matching2(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    std::cout<<"start greedy matching2"<<std::endl;
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

    // create cost matrix
    // std::vector<std::vector<int> > cost_matrix(assignable_agents_indices.size(),std::vector<int>(assignable_tasks_indices.size(), 0));

    // greedy matching
    // select the minimum cost 
    // cost, i_agent, i_task
    UTIL::g_timer.record_p("compute_matching_cost_s");
    auto matching_pq=MatchingPriorityQueue(assignable_agents_indices.size(), assignable_tasks_indices.size());
    // std::vector<std::tuple<int,int,int> > edges;
    #pragma omp parallel for schedule(dynamic,1)
    for (int i=0;i<assignable_agents_indices.size();++i) {
        for (int j=0;j<assignable_tasks_indices.size();++j) {
            int i_agent = assignable_agents_indices[i];
            int i_task = assignable_tasks_indices[j];
            int dist = 0;
            int c_loc = env->curr_states.at(i_agent).location;
            for (int loc : env->task_pool[i_task].locations){
                dist += UTIL::static_heuristic_table->get(c_loc, loc);
                c_loc = loc;
            }
            // always reachable by assumption, so we don't need to use INF7f in dynamic_hungarian_assignment.h
            // cost_matrix[i][j] = dist;
            // edges.emplace_back(dist,i,j);
            matching_pq.push(i,j,dist);
        }
        matching_pq.sort(i);
    }

    std::cout<<"cost matrix is built"<<std::endl;
    UTIL::g_timer.record_d("compute_matching_cost_s","compute_matching_cost");

    UTIL::g_timer.record_p("matching_s");
    std::unordered_map<int, int> assignment;
    assignment=matching_pq.get_assignment();
    UTIL::g_timer.record_d("matching_s","matching");

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


void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    UTIL::g_timer.record_p("schedule_plan_s");
    if (configs["scheduler"]=="dummy") {
        schedule_plan_dummy(time_limit, proposed_schedule, env);
    } else if (configs["scheduler"]=="default") {
        schedule_plan_default(time_limit, proposed_schedule, env);
    } else if (configs["scheduler"]=="matching") {
        schedule_plan_matching(time_limit, proposed_schedule, env);
    } else if (configs["scheduler"]=="greedy_matching") {
        schedule_plan_greedy_matching(time_limit, proposed_schedule, env);
    } else if (configs["scheduler"]=="greedy_matching2") {
        schedule_plan_greedy_matching2(time_limit, proposed_schedule, env);
    } else {
        std::cerr<<"error: unknown scheduler: "<<configs<<std::endl;
        exit(-1);
    }
    UTIL::g_timer.record_d("schedule_plan_s","schedule_plan");
    UTIL::g_timer.print_all_d();
}

}
