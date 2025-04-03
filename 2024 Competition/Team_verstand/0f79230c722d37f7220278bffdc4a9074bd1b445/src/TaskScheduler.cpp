#include "TaskScheduler.h"
#include "hungarian.h"
#include "scheduler.h"
#include "const.h"

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 *
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit
 * and adjust for a specified tolerance to account for potential timing errors.
 * It ensures that initialization does not exceed the allocated time.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit)
{
    clock_t start = clock();
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    // 1000ms是留给默认路径规划函数初始化的时间
    int limit = preprocess_time_limit - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE - 1000;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    DefaultPlanner::schedule_initialize(limit, env);
    // cout << "num task initial: " << env->new_tasks.size() << endl; // 由于是0, 所以无法预分配

    agent_task.resize(env->num_of_agents); // initialize all agents to free state
    radius_count_agent = (env->rows + env->cols) / 16;

    // compute map point dist as many as possible
    // compute_map_point_dist(endtime);
    // compute_map_point_dist_symmetry(endtime);

    // rhcr_initialize(); // 带旋转启发式的初始化

    cout << "Scheduler initialize Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

// 带旋转启发式的初始化
void TaskScheduler::rhcr_initialize()
{
    sorting_grid.load_LoRR_map(env);

    namespace po = boost::program_options;
    vm.insert(std::make_pair("single_agent_solver", po::variable_value(std::string("ASTAR"), false)));
    // vm.insert(std::make_pair("single_agent_solver", po::variable_value(std::string("SIPP"), false)));

    // vm.insert(std::make_pair("solver", po::variable_value(std::string("PBS"), false)));
    vm.insert(std::make_pair("lazyP", po::variable_value(false, false)));
    vm.insert(std::make_pair("prioritize_start", po::variable_value(true, false)));
    vm.insert(std::make_pair("hold_endpoints", po::variable_value(false, false)));
    vm.insert(std::make_pair("dummy_paths", po::variable_value(false, false)));
    vm.insert(std::make_pair("CAT", po::variable_value(false, false)));

    vm.insert(std::make_pair("solver", po::variable_value(std::string("ECBS"), false)));
    vm.insert(std::make_pair("potential_function", po::variable_value(std::string("NONE"), false)));
    vm.insert(std::make_pair("potential_threshold", po::variable_value(0.0, false)));
    vm.insert(std::make_pair("suboptimal_bound", po::variable_value(1.0, false)));

    vm.insert(std::make_pair("id", po::variable_value(false, false)));

    MAPFSolver* solver = set_solver(sorting_grid, vm);
    sorting_system = new SortingSystem(sorting_grid, *solver);
    assert(!sorting_system->hold_endpoints);
    assert(!sorting_system->useDummyPaths);

    vm.insert(std::make_pair("output", po::variable_value(std::string("../exp/test"), false)));
    vm.insert(std::make_pair("screen", po::variable_value(1, false)));
    vm.insert(std::make_pair("log", po::variable_value(false, false)));
    vm.insert(std::make_pair("agentNum", po::variable_value(env->num_of_agents, false)));
    vm.insert(std::make_pair("cutoffTime", po::variable_value(60, false)));
    vm.insert(std::make_pair("simulation_window", po::variable_value(1, false)));
    vm.insert(std::make_pair("planning_window", po::variable_value(5, false)));
    vm.insert(std::make_pair("travel_time_window", po::variable_value(0, false)));
    vm.insert(std::make_pair("rotation", po::variable_value(true, false)));
    vm.insert(std::make_pair("robust", po::variable_value(0, false)));
    // hold endpoints已经有了
    // dummy path已经有了
    vm.insert(std::make_pair("seed", po::variable_value(0, false)));

    set_parameters(*sorting_system, vm);
    sorting_grid.preprocessing_LoRR(sorting_system->consider_rotation, env);
    vm.insert(std::make_pair("simulation_time", po::variable_value(20, false)));

    // cout << "map 400: " << env->map[400] << endl;
    // cout << "map 962: " << env->map[962] << endl;
    // cout << "rhcr dist: " << sorting_grid.heuristics[0].at(33) << endl;
    // cout << "default dist: " << DefaultPlanner::get_h(env, 0, 33) << endl;
}

/**
 * Plans a task schedule within a specified time limit.
 *
 * This function schedules tasks by calling schedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 *
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    // give at most half of the entry time_limit to scheduler;
    // -SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    // DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    // greedy_only_first(limit, proposed_schedule);
    greedy_sum(limit, proposed_schedule);
    // greedy_sum_at_once(limit, proposed_schedule);
    // greedy_sum_suburb_first(limit, proposed_schedule);
    // greedy_sum_urban_first(limit, proposed_schedule);
    // greedy_sum_sparse_first(limit, proposed_schedule);
    // greedy_sum_dense_first(limit, proposed_schedule);

    // hungarian_only_first(limit, proposed_schedule);
    // hungarian_pickup_snatch(limit, proposed_schedule, env);
    // hungarian_sum(limit, proposed_schedule);
    // hungarian_sum_at_once(limit, proposed_schedule, env);
    // hungarian_sum_snatch(limit, proposed_schedule);
    // hungarian_sum_snatch_complex(limit, proposed_schedule, env);

    // pickup_jam_based_current(limit, proposed_schedule, env);
    // adaptive_jam_pickup_current_circle(limit, proposed_schedule);
    // adaptive_jam_task_circle_current(limit, proposed_schedule);
    // adaptive_jam_task_circle_current_square(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_current(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_current_preassign(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_current_compare_dist(limit, proposed_schedule);
    // hungarian_sum_snatch_adaptive_jam_task_circle_count_current(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_current_busy(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_current_sample(limit, proposed_schedule, 2048);
    // adaptive_jam_task_circle_count_current_extrapolation(limit, proposed_schedule);
    // adaptive_jam_task_region_count_current(limit, proposed_schedule); // 1.2
    // adaptive_jam_middle_circle_count_current(limit, proposed_schedule);
    // adaptive_jam_task_circle_vector_current(limit, proposed_schedule);
    // adaptive_jam_task_circle_vector_current_busy(limit, proposed_schedule);
    // adaptive_jam_task_circle_vector_current_extrapolation(limit, proposed_schedule);
    // adaptive_jam_task_circle_vector_current_complex(limit, proposed_schedule);

    // pickup_jam_based_goal(limit, proposed_schedule, env);
    // adaptive_jam_task_circle_count_goal(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_goal_compare_dist(limit, proposed_schedule);
    // hungarian_sum_snatch_adaptive_jam_task_circle_count_goal(limit, proposed_schedule);
    // adaptive_jam_task_circle_vector_goal(limit, proposed_schedule);

    // adaptive_jam_task_circle_count_both_current_goal(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_both_current_goal_compare_dist(limit, proposed_schedule);
    // hungarian_sum_snatch_adaptive_jam_task_circle_count_both_current_goal(limit, proposed_schedule);

    // adaptive_jam_task_circle_count_middle_current_goal(limit, proposed_schedule);
    // adaptive_jam_task_circle_count_middle_current_goal_compare_dist(limit, proposed_schedule);
    // hungarian_sum_snatch_adaptive_jam_task_circle_count_middle_current_goal(limit, proposed_schedule);
    // adaptive_jam_task_Manhattan_circle_count_middle_current_goal(limit, proposed_schedule);
    // adaptive_jam_task_Manhattan_circle_count_middle_current_goal_rhcr(limit, proposed_schedule);
    // adaptive_jam_task_fix_Manhattan_circle_count_middle_current_goal(limit, proposed_schedule);

    // adaptive_jam_task_region_count_middle_current_goal(limit, proposed_schedule); // 8

    // adaptive_jam_task_circle_vector_middle_current_goal(limit, proposed_schedule);
    // adaptive_jam_curr_pickup_intersect_curr_goal(limit, proposed_schedule);
    // adaptive_jam_curr_pickup_delivery_intersect_curr_goal(limit, proposed_schedule);

    // choose method according to map size, 128x128 as separator
    /*
    if(env->map.size() <= 128 * 128)
    {
        // Random 1 2 3 4 5
        hungarian_sum_snatch(limit, proposed_schedule);
    }
    else
    {
        // choose method according to agent num, 2048 as separator
        // CITY-01
        if(env->curr_states.size() <= 2048)
        {
            hungarian_sum_snatch(limit, proposed_schedule);
        }
        else
        {
            // 用8192作为adaptive_jam_middle_current_goal_task_circle_count和default的分界点，看看能不能把sortation和warehouse分离出来。
            if(env->curr_states.size() <= 8192) // CITY-02 and GAME
            {
                // CITY-02
                if(env->curr_states.size() <= 4096)
                {
                    adaptive_jam_task_circle_count_middle_current_goal(limit, proposed_schedule);
                }
                else // GAME
                {
                    adaptive_jam_curr_pickup_intersect_curr_goal(limit, proposed_schedule);
                }
            }
            else // SORTATION and WAREHOUSE
            {
                // 观测SORTATION and WAREHOUSE是否小于16384 => 小于
                // 观测SORTATION and WAREHOUSE是否小于12288
                adaptive_jam_task_region_count_current(limit, proposed_schedule);
            }
        }
    }
     //*/

    /*
    cout << "task pool size: " << env->task_pool.size() << endl;
    for(auto element : env->task_pool)
    {
        cout << "task id" << element.first << " " << element.second.idx_next_loc << endl;
    }
     */
}

// 预先计算地图上任意两点之间的距离
void TaskScheduler::compute_map_point_dist(TimePoint _endtime) const
{
    size_t count = 0;
    for(int i=0;i<env->map.size();i++)
    {
        for(int j=0;j<env->map.size();j++)
        {
            // check for timeout every 100 dist computation
            if (count % 100 == 0 && std::chrono::steady_clock::now() > _endtime)
            {
                return;
            }

            if (j != i && env->map[i]==0 && env->map[j]==0)
            {
                DefaultPlanner::get_h(env, i, j);
                count++;
            }
        }
    }
}

// 预先计算地图上任意两点之间的距离, 采用对称性加快计算速度
void TaskScheduler::compute_map_point_dist_symmetry(TimePoint _endtime) const
{
    // cout << DefaultPlanner::global_heuristictable.size() << endl;

    size_t count = 0;
    for(int i=0;i<env->map.size();i++)
    {
        for(int j=i+1;j<env->map.size();j++)
        {
            // check for timeout every 10 dist computation
            if (count % 100 == 0 && std::chrono::steady_clock::now() > _endtime)
            {
                return;
            }

            if (env->map[i]==0 && env->map[j]==0)
            {
                if(DefaultPlanner::global_heuristictable.at(i).htable.empty())
                {
                    DefaultPlanner::global_heuristictable.at(i).htable.resize(
                            env->map.size(), MAX_TIMESTEP);
                }

                // get_h(i,j)填充的是global_heuristictable.at(j).htable[i], 这里使用对称性加快速度
                DefaultPlanner::global_heuristictable.at(i).htable[j] =
                        DefaultPlanner::get_h(env, i, j);
                count++;
                // cout << i << " " << j << endl; // i=0, j=1
                // cout << DefaultPlanner::global_heuristictable.size() << endl;
                // cout << DefaultPlanner::global_heuristictable.at(i).htable[j] << endl; // 0
                // cout << DefaultPlanner::global_heuristictable.at(j).htable[i] << endl; // 65536
                // return;
            }
        }
    }
}

// 默认分配算法，cost只计算agent和任务pickup点的位置。
void TaskScheduler::greedy_only_first(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
                break; // schedule only first
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

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 默认分配算法，从默认代码中复制到TaskScheduler.cpp。
void TaskScheduler::greedy_sum(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
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

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 默认分配算法，用于其他分配算法处理首批任务
void TaskScheduler::greedy_sum_without_newtask(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // 由于new task已经在调用前插入过了, 所以这里不再插入新任务
    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
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

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// compute total distance of task before assignment
void TaskScheduler::greedy_sum_at_once(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    int counter = 0;
    for (const int& new_task_id : env->new_freeagents)
    {
        // this task distance has been calculated
        int total_dist = 0;
        int curr_loc = env->task_pool[new_task_id].locations[0];

        // iterate over the locations (errands) of the task to compute the makespan to finish the task
        // makespan: the time for the agent to complete all the errands of the task t_id in order
        for (int loc : env->task_pool[new_task_id].locations)
        {
            total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
            curr_loc = loc;
        }

        task_distances[new_task_id] = total_dist;
        counter++;
        // cout << counter << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // 遍历
    /*
    for (const auto& pair : task_distances) {
        std::cout << "task " << pair.first << " distance " << pair.second << "\n";
    }
     */

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    int best_total_distance = 0;
    auto it = free_agents.begin();
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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                dist += task_distances[t_id];
                c_loc = loc;
                break;
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
            best_total_distance += min_task_makespan;
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

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "best total distance: " << best_total_distance << endl; // 2338
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 优先给靠近地图角落的agent分配任务, 因为地图边缘不容易堵车。
void TaskScheduler::greedy_sum_suburb_first(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    std::vector<int> free_agents_vector;
    free_agents_vector.assign(free_agents.begin(), free_agents.end());

    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // 每个free agent和地图角落的距离
    vector<int> dist2edges(free_agents_vector.size());
    for(int i=0;i<free_agents_vector.size();i++)
    {
        int agent_loc = env->curr_states.at(free_agents_vector[i]).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        dist2edges[i] = std::min(agent_loc_x, env->cols - 1 - agent_loc_x) +
                            std::min(agent_loc_y, env->rows - 1 - agent_loc_y);
    }

    // 索引数组，表示dist2edges原始索引, 也和free_agents_vector的索引对应
    std::vector<int> indices(dist2edges.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    // 对索引数组进行排序，按照 data 的值排序
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return dist2edges[a] < dist2edges[b];  // 按 data 的值升序排序
    });

    /*
    // 输出排序后的索引顺序
    std::cout << "Sorted indices: ";
    for (int idx : indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // 输出排序后的数组
    std::cout << "Sorted values: ";
    for (int idx : indices) {
        std::cout << dist2edges[idx] << " ";
    }
    std::cout << std::endl;
     //*/

    // iterate over the free agents to decide which task to assign to each of them
    for(int i : indices)
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        auto agent_id = free_agents_vector[i];
        assert(env->curr_task_schedule[agent_id] == -1);

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
            c_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent_id (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[agent_id] = min_task_i;
            free_agents.erase(agent_id);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[agent_id] = -1;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 优先给靠近地图中心的agent分配任务, 因为地图中心更容易堵车。
void TaskScheduler::greedy_sum_urban_first(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    std::vector<int> free_agents_vector;
    free_agents_vector.assign(free_agents.begin(), free_agents.end());

    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // 每个free agent和地图中心的距离
    vector<int> dist2center(free_agents_vector.size());
    for(int i=0;i<free_agents_vector.size();i++)
    {
        int agent_loc = env->curr_states.at(free_agents_vector[i]).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        dist2center[i] = std::abs(agent_loc_x - env->cols / 2)
                                  + std::abs(agent_loc_y - env->rows / 2);
    }

    // 索引数组，表示dist2edges原始索引, 也和free_agents_vector的索引对应
    std::vector<int> indices(dist2center.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    // 对索引数组进行排序，按照 data 的值排序
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return dist2center[a] < dist2center[b];  // 按 data 的值升序排序
    });

    /*
    // 输出排序后的索引顺序
    std::cout << "Sorted indices: ";
    for (int idx : indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // 输出排序后的数组
    std::cout << "Sorted values: ";
    for (int idx : indices) {
        std::cout << dist2center[idx] << " ";
    }
    std::cout << std::endl;
     //*/

    // iterate over the free agents to decide which task to assign to each of them
    for(int i : indices)
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        auto agent_id = free_agents_vector[i];
        assert(env->curr_task_schedule[agent_id] == -1);

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
            c_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent_id (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[agent_id] = min_task_i;
            free_agents.erase(agent_id);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[agent_id] = -1;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 优先给车流稀疏的agent分配任务, 因为车流稀疏区域不容易堵车。
void TaskScheduler::greedy_sum_sparse_first(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    std::vector<int> free_agents_vector;
    free_agents_vector.assign(free_agents.begin(), free_agents.end());

    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // 每个free agent和地图中心的距离
    vector<double> sum_jam_weights(free_agents_vector.size(), 0);
    for(int i=0;i<free_agents_vector.size();i++)
    {
        int agent_id = free_agents_vector[i];
        int agent_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        for(int j=0;j<env->num_of_agents;j++)
        {
            if (j != agent_id)
            {
                int other_loc = env->curr_states.at(j).location;
                int other_loc_x = other_loc % env->cols;
                int other_loc_y = other_loc / env->cols;

                sum_jam_weights[i] += 1.0 / (std::abs(agent_loc_x - other_loc_x)
                                             + std::abs(agent_loc_y - other_loc_y));
            }
        }
    }

    // 索引数组，表示dist2edges原始索引, 也和free_agents_vector的索引对应
    std::vector<int> indices(sum_jam_weights.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    // 对索引数组进行排序，按照 data 的值排序
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return sum_jam_weights[a] < sum_jam_weights[b];  // 按 data 的值升序排序
    });

    /*
    // 输出排序后的索引顺序
    std::cout << "Sorted indices: ";
    for (int idx : indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // 输出排序后的数组
    std::cout << "Sorted values: ";
    for (int idx : indices) {
        std::cout << sum_jam_weights[idx] << " ";
    }
    std::cout << std::endl;
     //*/

    // iterate over the free agents to decide which task to assign to each of them
    for(int i : indices)
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        auto agent_id = free_agents_vector[i];
        assert(env->curr_task_schedule[agent_id] == -1);

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
            c_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent_id (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[agent_id] = min_task_i;
            free_agents.erase(agent_id);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[agent_id] = -1;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 优先给车流密集的agent分配任务, 因为车流密集区域更容易堵车。
void TaskScheduler::greedy_sum_dense_first(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // 如果计算量大到一定程度, 就用默认算法
    if(free_agents.size() * env->num_of_agents > 30 * 5000)
    {
        int min_task_i, min_task_makespan, dist, c_loc, count;
        clock_t start = clock();

        // iterate over the free agents to decide which task to assign to each of them
        auto it = free_agents.begin();
        while (it != free_agents.end())
        {
            // keep assigning until timeout
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
                // check for timeout every 10 task evaluations
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                {
                    break;
                }
                dist = 0;
                c_loc = env->curr_states.at(i).location;

                // iterate over the locations (errands) of the task to compute the makespan to finish the task
                // makespan: the time for the agent to complete all the errands of the task t_id in order
                for (int loc : env->task_pool[t_id].locations){
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
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

        cout << "Default Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
        return;
    }

    std::vector<int> free_agents_vector;
    free_agents_vector.assign(free_agents.begin(), free_agents.end());

    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // 每个free agent和地图中心的距离
    vector<double> sum_jam_weights(free_agents_vector.size(), 0);
    for(int i=0;i<free_agents_vector.size();i++)
    {
        int agent_id = free_agents_vector[i];
        int agent_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        for(int j=0;j<env->num_of_agents;j++)
        {
            if (j != agent_id)
            {
                int other_loc = env->curr_states.at(j).location;
                int other_loc_x = other_loc % env->cols;
                int other_loc_y = other_loc / env->cols;

                sum_jam_weights[i] += 1.0 / (std::abs(agent_loc_x - other_loc_x)
                                     + std::abs(agent_loc_y - other_loc_y));
            }
        }
    }

    // 索引数组，表示dist2edges原始索引, 也和free_agents_vector的索引对应
    std::vector<int> indices(sum_jam_weights.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    // 对索引数组进行排序，按照 data 的值排序
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return sum_jam_weights[a] > sum_jam_weights[b];  // 按 data 的值降序排序
    });

    /*
    // 输出排序后的索引顺序
    std::cout << "Sorted indices: ";
    for (int idx : indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // 输出排序后的数组
    std::cout << "Sorted values: ";
    for (int idx : indices) {
        std::cout << sum_jam_weights[idx] << " ";
    }
    std::cout << std::endl;
     //*/

    // iterate over the free agents to decide which task to assign to each of them
    for(int i : indices)
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        auto agent_id = free_agents_vector[i];
        assert(env->curr_task_schedule[agent_id] == -1);

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
            c_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent_id (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[agent_id] = min_task_i;
            free_agents.erase(agent_id);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[agent_id] = -1;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 匈牙利分配算法，cost只计算agent和任务pickup点的位置。
void TaskScheduler::hungarian_only_first(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    if(free_agents.empty())
    {
        return;
    }

    std::vector<int> free_agents_vec;
    free_agents_vec.assign(free_agents.begin(), free_agents.end());

    std::vector<int> free_tasks_vec;
    free_tasks_vec.assign(free_tasks.begin(), free_tasks.end());

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_vec[i];

        cost_matrix[i].resize(free_tasks.size(), 0);
        for(int j=0;j<free_tasks.size();j++)
        {
            auto task_id = free_tasks_vec[j];

            int dist = 0;
            int curr_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[task_id].locations){
                dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
                break; // schedule only first
            }

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    double cost = HungAlgo.Solve(work_assignment);
    for(int i=0;i<free_agents_vec.size();i++)
    {
        // 该工人被分配了任务
        if(work_assignment[i] != -1)
        {
            free_agents.erase(free_agents_vec[i]);
            free_tasks.erase(free_tasks_vec[work_assignment[i]]);
        }

        proposed_schedule[free_agents_vec[i]] = free_tasks_vec[work_assignment[i]];
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

// hungarian schedule only distance to pickup location and snatch order
void TaskScheduler::hungarian_pickup_snatch(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];

        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];

            int dist = 0;
            int curr_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[task_id].locations){
                dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
                break;
            }

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            proposed_schedule[free_agents_and_before_pickup[i]] = free_tasks_and_before_pickup[work_assignment[i]];

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// 匈牙利分配算法, cost使用默认计算方法。
void TaskScheduler::hungarian_sum(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    if(free_agents.empty())
    {
        return;
    }

    std::vector<int> free_agents_vec;
    free_agents_vec.assign(free_agents.begin(), free_agents.end());

    std::vector<int> free_tasks_vec;
    free_tasks_vec.assign(free_tasks.begin(), free_tasks.end());

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_vec[i];

        cost_matrix[i].resize(free_tasks.size(), 0);
        for(int j=0;j<free_tasks.size();j++)
        {
            auto task_id = free_tasks_vec[j];

            int dist = 0;
            int curr_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[task_id].locations){
                dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    double cost = HungAlgo.Solve(work_assignment);
    for(int i=0;i<free_agents_vec.size();i++)
    {
        // 该工人被分配了任务
        if(work_assignment[i] != -1)
        {
            free_agents.erase(free_agents_vec[i]);
            free_tasks.erase(free_tasks_vec[work_assignment[i]]);
        }

        proposed_schedule[free_agents_vec[i]] = free_tasks_vec[work_assignment[i]];
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

// 匈牙利分配算法，一次性把任务长度计算完储存在unordered_map。
void TaskScheduler::hungarian_sum_at_once(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    if(free_agents.empty())
    {
        return;
    }

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    std::vector<int> free_agents_vec;
    free_agents_vec.assign(free_agents.begin(), free_agents.end());

    std::vector<int> free_tasks_vec;
    free_tasks_vec.assign(free_tasks.begin(), free_tasks.end());

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_vec[i];

        cost_matrix[i].resize(free_tasks.size(), 0);
        for(int j=0;j<free_tasks.size();j++)
        {
            auto task_id = free_tasks_vec[j];

            int dist = 0;
            int curr_loc = env->curr_states.at(agent_id).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[task_id].locations){
                dist += DefaultPlanner::get_h(env, curr_loc, loc);
                dist += task_distances[task_id];
                curr_loc = loc;
                break;
            }

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    double cost = HungAlgo.Solve(work_assignment);
    for(int i=0;i<free_agents_vec.size();i++)
    {
        // 该工人被分配了任务
        if(work_assignment[i] != -1)
        {
            free_agents.erase(free_agents_vec[i]);
            free_tasks.erase(free_tasks_vec[work_assignment[i]]);
        }

        proposed_schedule[free_agents_vec[i]] = free_tasks_vec[work_assignment[i]];
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// hungarian schedule sum at once and snatch order
void TaskScheduler::hungarian_sum_snatch(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];

        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];
            int curr_loc = env->curr_states.at(agent_id).location;

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = DefaultPlanner::get_h(env, curr_loc,
                                  env->task_pool[task_id].locations[0]) + task_distances[task_id];
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            proposed_schedule[free_agents_and_before_pickup[i]] = free_tasks_and_before_pickup[work_assignment[i]];

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// 在hungarian_sum_snatch的基础上，考虑了单据不够、单据超出等多种情况。尚且不稳定，official platform测试会失败。
void TaskScheduler::hungarian_sum_snatch_complex(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    TimePoint task_distance_end_moment = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit / 3);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    int num_free_agents_before_allocated = free_agents.size();
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0; // 每处理10个任务, 计算一次时间
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > task_distance_end_moment)
        {
            // cout << "task distance compute timeout " << task_distances.size() << " been computed" << endl;
            break;
        }

        // this task distance has not been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agents and agents with task but before pickup (only when free agents not enough)
    std::vector<int> free_agents_and_before_pickup;
    if(free_agents.size() <= max_agent_allowed)
    {
        free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());
    }
    else
    {
        int count = 0;
        for (auto it = free_agents.begin(); it != free_agents.end() && count < max_agent_allowed; )
        {
            free_agents_and_before_pickup.emplace_back(*it); // 添加到 vector
            it++;
            count++;
        }
    }

    // free tasks and tasks already assigned but before pickup (only when free tasks not enough)
    std::vector<int> free_tasks_and_before_pickup;
    if(free_tasks.size() <= max_task_allowed)
    {
        free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());
    }
    else
    {
        int count = 0;
        for (auto it = free_tasks.begin(); it != free_tasks.end()
        && count < max_task_allowed
        && task_distances.find(*it) != task_distances.end();) // 已经计算过任务长度的
        {
            free_tasks_and_before_pickup.emplace_back(*it); // 添加到 vector
            it++;
            count++;
        }
    }

    // 不够才需要补充, 够了就不需要了
    if(free_agents_and_before_pickup.size() < max_agent_allowed
    || free_tasks_and_before_pickup.size() < max_task_allowed)
    {
        for(const auto& element : env->task_pool)
        {
            if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
            {
                if(free_agents_and_before_pickup.size() < max_agent_allowed)
                {
                    free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
                }

                if(free_tasks_and_before_pickup.size() < max_task_allowed)
                {
                    free_tasks_and_before_pickup.emplace_back(element.second.task_id);
                }
            }
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    //*
    cout << "free agent num: " << free_agents.size() << endl;
    cout << "free task num: " << free_tasks.size() << endl;

    cout << "free and before pickup agent num: " << free_agents_and_before_pickup.size() << endl;
    cout << "free and before pickup task num: " << free_tasks_and_before_pickup.size() << endl;
    //*/

    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];

        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];

            int curr_loc = env->curr_states.at(agent_id).location;

            if(task_distances.find(task_id) == task_distances.end())
            {
                cerr << "ERROR! use distance not computed" << endl;
                return;
            }

            int dist = DefaultPlanner::get_h(env, curr_loc, env->task_pool[task_id].locations[0])
                    + task_distances[task_id];

            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    int num_free_agents_after_allocated = 0;
    if(free_agents.size() < max_agent_allowed)
    {
        for(auto const& element : work_assignment)
        {
            if(element == -1)
            {
                num_free_agents_after_allocated++;
            }
        }
    }
    else
    {
        num_free_agents_after_allocated = free_agents.size() - max_agent_allowed;
    }

    if (num_free_agents_after_allocated < num_free_agents_before_allocated || cost < prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        if(free_agents.size() < max_agent_allowed)
        {
            free_agents.clear();
            free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        }

        if(free_tasks.size() < max_task_allowed)
        {
            free_tasks.clear();
            free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());
        }

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            proposed_schedule[free_agents_and_before_pickup[i]] = free_tasks_and_before_pickup[work_assignment[i]];

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数之和决定。
void TaskScheduler::pickup_jam_based_current(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    TimePoint jam_end_moment = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit / 2);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // cout << "agent num:" << env->curr_states.size() << endl;
    // cout << "curr time:" << env->curr_timestep << endl;

    clock_t start = clock();

    // compute pickup jam for each free task every timestep
    for (int t_id : free_tasks)
    {
        double pickup_jam = 0;
        int pickup_loc = env->task_pool[t_id].locations[0];
        for(int i=0;i<env->curr_states.size();i++)
        {
            // 上下左右四个格子内的障碍物越多, 对拥堵的影响就越大
            pickup_jam += double(4 - DefaultPlanner::global_neighbors[pickup_loc].size())
                    / DefaultPlanner::get_h(env, pickup_loc, env->curr_states[i].location);
        }

        task_pickup_jams[t_id] = pickup_jam;

        if (std::chrono::steady_clock::now() > jam_end_moment)
        {
            // cout << "jam time not enough" << endl;
            break;
        }
    }

    /*
    for (auto it = task_pickup_jams.begin(); it != task_pickup_jams.end(); ++it) {
        std::cout << "task: " << it->first << ", jam: " << it->second << std::endl;
    }
     */

    // cout << "jam compute duration: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;

    int min_task_i, min_task_makespan, c_loc, count;


    // iterate over the free agents to decide which task to assign to each of them
    int best_total_distance = 0;
    auto it = free_agents.begin();
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

            // distance between current location and task pick up location
            int dist = DefaultPlanner::get_h(env, env->curr_states.at(i).location,
                                          env->task_pool[t_id].locations[0]);
            int temp_dist = dist;

            // TODO: 如果task pickup jams没有来得及算, 这里不要加上去
            dist += int ((task_pickup_jams[t_id] - 1.0 / temp_dist) * jam_coefficient);

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
            best_total_distance += min_task_makespan;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Task assignment duration: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "best total distance: " << best_total_distance << endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 计算adaptive_jam_pickup_current_circle的估计延迟时间
double TaskScheduler::compute_adaptive_jam_pickup_current_circle(int _agent_id, int _agent_loc, int _pickup_loc) const
{
    double sum_jam_weight = 0;

    int agent_loc_x = _agent_loc % env->cols;
    int agent_loc_y = _agent_loc / env->cols;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_loc_x << " " << pickup_loc_y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->curr_states.size();j++)
    {
        if (j != _agent_id) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;
            // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
            // << other_agent_loc_y << endl;

            // vector from current agent location to other agent location
            int agent_other_direction_x = other_agent_loc_x - agent_loc_x;
            int agent_other_direction_y = other_agent_loc_y - agent_loc_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                              + agent_other_direction_y * agent_other_direction_y;

            // 只统计agent-task半圆范围内的agent
            if(agent_other_distance_square < agent_task_direction_square)
            {
                // 其他agent方向与task方向的夹角 < 90°才算入拥堵系数
                int inner_product = agent_other_direction_x * agent_task_direction_x
                                    + agent_other_direction_y * agent_task_direction_y;
                if (inner_product > 0)
                {
                    // double task_direction_length = sqrt(agent_task_direction_square);
                    // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                    // << task_direction_length << endl;

                    sum_jam_weight += inner_product / (agent_other_distance_square
                                                       * sqrt(agent_task_direction_square));
                }
            }
        }
    }

    return sum_jam_weight;
}

// 以agent为圆心，agent-task为半径朝向task画出一个半圆，位于这个半圆内的other agent计入拥堵系数。jam = cos<ao, at> / |ao| = inner(ao, at) / (|ao||ao||at|) if inner(ao, at) > 0 and |ao| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_pickup_current_circle(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
            << " minDist " << agent_task[element].min_task_dist
            << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations.front();

            sum_jam_weight = compute_adaptive_jam_pickup_current_circle(i, agent_loc, pickup_loc);

            // update the new minimum makespan
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 检索以task为圆心，所有agent距离它的倒数之和
void TaskScheduler::adaptive_jam_task_circle_current(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                    // << task_direction_length << endl;

                    sum_jam_weight += 1 / sqrt(other_task_distance_square);
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// 检索以task为圆心，所有agent距离它的倒数平方之和
void TaskScheduler::adaptive_jam_task_circle_current_square(int time_limit,
                                                            std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                    // << task_direction_length << endl;

                    // 检索以task为圆心，所有agent距离它的倒数平方之和
                    sum_jam_weight += 1 / double (other_task_distance_square);
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

int TaskScheduler::compute_jam_task_circle_count_current(int _agent_id, int _agent_loc_x,
                                                         int _agent_loc_y, int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->curr_states.size();j++)
    {
        if (j != _agent_id) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;
            // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
            // << other_agent_loc_y << endl;

            int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
            int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                             + other_task_direction_y * other_task_direction_y;

            // 只统计以task为圆心, agent-task范围内的other agent
            if(other_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 1: 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_current(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_circle_count_current(i, agent_loc_x,
                                                                       agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 1.1: 预分配初始任务。
void TaskScheduler::adaptive_jam_task_circle_count_current_preassign(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    if(free_agents.size() > 50)
    {
        if(first_epoch_done_time == -1)
        {
            // 由于大算例初始任务过多, 在初始任务分配完成前采用默认算法
            greedy_sum_without_newtask(time_limit, proposed_schedule);
            return;
        }
    }
    else
    {
        if(first_epoch_done_time == -1)
        {
            first_epoch_done_time = env->curr_timestep; // 初始阶段的任务分配完毕的时间
        }
    }

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_circle_count_current(i, agent_loc_x,
                                                                       agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_current_compare_dist(int time_limit,
                                                            std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            if (dist < min_task_heuristic)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int sum_jam_weight = compute_jam_task_circle_count_current(i, agent_loc_x,
                                                                           agent_loc_y, pickup_loc);

                // sum_jam_weight * jam_coefficient = guess delay time
                if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                    min_task_i = t_id;
                    min_task_dist = dist;
                    min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                    corresponding_traffic_jam = sum_jam_weight;
                }
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

void TaskScheduler::hungarian_sum_snatch_adaptive_jam_task_circle_count_current(int time_limit,
                                                                 std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<int> > dist_matrix;
    dist_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > jam_matrix;
    jam_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];
        int curr_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = curr_loc % env->cols;
        int agent_loc_y = curr_loc / env->cols;

        dist_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        jam_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);

        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];
            int pickup_loc = env->task_pool[task_id].locations[0];

            int sum_jam_weight = compute_jam_task_circle_count_current(agent_id, agent_loc_x,
                                                                       agent_loc_y, pickup_loc);
            // cout << "traffic jam: " << sum_jam_weight << endl;

            int dist = DefaultPlanner::get_h(env, curr_loc, pickup_loc) + task_distances[task_id];
            dist_matrix[i][j] = dist;
            // 在算代价时, sum_jam_weight要乘以系数; 记录时, 不乘系数
            jam_matrix[i][j] = sum_jam_weight;
            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist + sum_jam_weight * jam_coefficient;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            int assigned_agent = free_agents_and_before_pickup[i];
            int assigned_task = free_tasks_and_before_pickup[work_assignment[i]];

            proposed_schedule[assigned_agent] = assigned_task;

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);

                agent_task[assigned_agent].task_id = assigned_task;
                agent_task[assigned_agent].min_task_dist  = dist_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].task_heuristic = cost_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].assign_moment = env->curr_timestep; // assign task moment
                agent_task[assigned_agent].jam_when_assign = jam_matrix[i][work_assignment[i]];
                // cout << "jam when assign: " << agent_task[assigned_agent].jam_when_assign << endl;
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

int TaskScheduler::compute_jam_task_circle_count_current_busy(int _agent_id, int _agent_loc_x,
                       int _agent_loc_y, int _pickup_loc, std::vector<int> & proposed_schedule) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->curr_states.size();j++)
    {
        if (j != _agent_id && proposed_schedule[j] != -1) // 只统计有任务的agent
        {
            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;
            // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
            // << other_agent_loc_y << endl;

            int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
            int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                             + other_task_direction_y * other_task_direction_y;

            // 只统计以task为圆心, agent-task范围内的other agent
            if(other_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内有任务的other agent的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_current_busy(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_circle_count_current_busy(i, agent_loc_x,
                                                agent_loc_y, pickup_loc, proposed_schedule);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量（只考察前2048个agent）为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_current_sample(int time_limit,
                                                                  std::vector<int> & proposed_schedule, int _num_sample)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    int count = 0;
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // the theoretical lower bound of accomplishing this task
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            int sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // the vector from an agent to its task
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i && j < _num_sample) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        sum_jam_weight++;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_current_extrapolation(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            int sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        sum_jam_weight++;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient * extrapolation = guess delay time
            if (dist + sum_jam_weight * jam_coefficient * dist /
                    DefaultPlanner::get_h(env, agent_loc, pickup_loc) < min_task_heuristic)
            {
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 1.2: 用地图以16x16方形分割区域, task所在区域中agent的数量作为jam
void TaskScheduler::adaptive_jam_task_region_count_current(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    if(free_agents.size() > 50)
    {
        if(first_epoch_done_time == -1)
        {
            // 由于大算例初始任务过多, 在初始任务分配完成前采用默认算法
            greedy_sum_without_newtask(time_limit, proposed_schedule);
            return;
        }
    }
    else
    {
        if(first_epoch_done_time == -1)
        {
            first_epoch_done_time = env->curr_timestep; // 初始阶段的任务分配完毕的时间
        }
    }

    int region_column = 8; // 每个region所占的列数
    int region_row = 8; // 每个region所占的行数

    // 地图有几列region
    int num_region_column = std::ceil((double)env->cols / region_column);
    // 地图有几行region
    int num_region_row = std::ceil((double)env->rows / region_row);

    // 将map分为若干区域, 统计每个区域agent的数量
    vector<int> region_agent_num(num_region_column * num_region_row, 0);

    for (int i=0;i<env->num_of_agents;i++)
    {
        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        int agent_region_x = agent_loc_x / region_column;
        int agent_region_y = agent_loc_y / region_row;

        region_agent_num[agent_region_y * num_region_column + agent_region_x]++;
    }

    /*
    cout << "region agent num: ";
    for(int i : region_agent_num)
    {
        cout << i << " ";
    }
    cout << endl;
     //*/

    if(env->num_of_agents <= 500) // 小算例直接算
    {
        for (int t_id : env->new_tasks)
        {
            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = pickup_loc % env->cols;
            int pickup_loc_y = pickup_loc / env->cols;

            int pickup_region_x = pickup_loc_x / region_column;
            int pickup_region_y = pickup_loc_y / region_row;

            task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
        }
    }
    else // 大算例
    {
        if(env->curr_timestep == first_epoch_done_time) // 在first_epoch_done_time计算完此时的free tasks
        {
            for (int t_id : free_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
        else // first_epoch_done_time之后就只计算新任务
        {
            for (int t_id : env->new_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
    }


    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = region_agent_num[task_region[t_id]];

            // sum_jam_weight * jam_coefficient = estimated delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Task Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 统计以agent-task中点为圆心，|agent-task|/2为半径的圆中other agent的数量作为jam。
void TaskScheduler::adaptive_jam_middle_circle_count_current(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            int sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            int middle_loc_x = (agent_loc_x + pickup_loc_x) / 2;
            int middle_loc_y = (agent_loc_y + pickup_loc_y) / 2;

            // 当前agent指向某个agent-task中点的向量
            int agent_middle_direction_x = pickup_loc_x - agent_loc_x;
            int agent_middle_direction_y = pickup_loc_y - agent_loc_y;
            int agent_middle_direction_square = agent_middle_direction_x * agent_middle_direction_x +
                                              agent_middle_direction_y * agent_middle_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int other_middle_direction_x = middle_loc_x - other_agent_loc_x;
                    int other_middle_direction_y = middle_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_middle_distance_square = other_middle_direction_x * other_middle_direction_x
                                                     + other_middle_direction_y * other_middle_direction_y;

                    // 只统计以|agent-task|中点为圆心, |agent-task|为直径范围内的other agent
                    if(other_middle_distance_square < agent_middle_direction_square)
                    {
                        sum_jam_weight++;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_task_circle_vector_current(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int agent_other_direction_x = other_agent_loc_x - agent_loc_x;
                    int agent_other_direction_y = other_agent_loc_y - agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                                      + agent_other_direction_y * agent_other_direction_y;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        int inner_product = agent_other_direction_x * agent_task_direction_x
                                            + agent_other_direction_y * agent_task_direction_y;

                        // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                        // << task_direction_length << endl;

                        sum_jam_weight += double (inner_product) / agent_other_distance_square;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数，并把这个系数外推出去。jam = (cost<ao,at> * |at| / |ao|) * (total_dist / |at|) = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_task_circle_vector_current_extrapolation(int time_limit,
                                                                          std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int agent_other_direction_x = other_agent_loc_x - agent_loc_x;
                    int agent_other_direction_y = other_agent_loc_y - agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                                      + agent_other_direction_y * agent_other_direction_y;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        int inner_product = agent_other_direction_x * agent_task_direction_x
                                            + agent_other_direction_y * agent_task_direction_y;

                        // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                        // << task_direction_length << endl;

                        sum_jam_weight += double (inner_product) / agent_other_distance_square;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient * extrapolation = guess delay time
            if (dist + sum_jam_weight * jam_coefficient * dist /
            DefaultPlanner::get_h(env, agent_loc, pickup_loc) < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数, 有|agent-task|欧几里得距离作为分母。jam = cost<ao,at> / |ao| = inner(ao, at) / (inner(ao, ao) * |at|) if |to| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_task_circle_vector_current_complex(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);

            total_min_span += temp.min_task_dist;
            total_real_duration += temp.real_duration;
            total_jam += temp.jam_when_assign;

            cout << "complete task " << temp.task_id
                 << " minDist " << temp.min_task_dist
                 << " heuristic " << temp.heuristic_duration
                 << " real " << temp.real_duration
                 << " jam " << temp.jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double traffic_jam = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int agent_other_direction_x = other_agent_loc_x - agent_loc_x;
                    int agent_other_direction_y = other_agent_loc_y - agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                                      + agent_other_direction_y * agent_other_direction_y;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                      + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        int inner_product = agent_other_direction_x * agent_task_direction_x
                                            + agent_other_direction_y * agent_task_direction_y;

                            // double task_direction_length = sqrt(agent_task_direction_square);
                            // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                            // << task_direction_length << endl;

                            traffic_jam += inner_product / (agent_other_distance_square
                                                            * sqrt(agent_task_direction_square));
                    }
                }
            }

            // update the new minimum makespan
            if (dist + traffic_jam * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                // 在算代价时, sum_jam_weight要乘以系数; 记录时, 不乘系数
                min_task_heuristic = dist + traffic_jam * jam_coefficient;
                corresponding_traffic_jam = traffic_jam;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_task_circle_vector_current_busy(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->curr_states.size();j++)
            {
                if (j != i && proposed_schedule[j] != -1) // 只统计有任务的agent
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;
                    // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
                    // << other_agent_loc_y << endl;

                    int agent_other_direction_x = other_agent_loc_x - agent_loc_x;
                    int agent_other_direction_y = other_agent_loc_y - agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                                      + agent_other_direction_y * agent_other_direction_y;

                    int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
                    int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                                     + other_task_direction_y * other_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent
                    if(other_task_distance_square < agent_task_direction_square)
                    {
                        int inner_product = agent_other_direction_x * agent_task_direction_x
                                            + agent_other_direction_y * agent_task_direction_y;

                        // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                        // << task_direction_length << endl;

                        sum_jam_weight += double (inner_product) / agent_other_distance_square;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent目标距离该任务的倒数之和决定。
void TaskScheduler::pickup_jam_based_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    TimePoint jam_end_moment = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit / 2);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // cout << "agent num:" << env->curr_states.size() << endl;
    // cout << "curr time:" << env->curr_timestep << endl;
    /*
    cout << "num goal locs: " << env->goal_locations.size() << endl;
    for(int i=0;i<env->goal_locations.size();i++)
    {
        cout << "agent " << i << " goal locs: ";
        for(auto element : env->goal_locations[i])
        {
            cout << element.first << " ";
        }
        cout << endl;
    }
     */

    clock_t start = clock();

    // compute pickup jam for each free task every timestep
    for (int t_id : free_tasks)
    {
        double pickup_jam = 0;
        int pickup_loc = env->task_pool[t_id].locations[0];
        for(int i=0;i<env->goal_locations.size();i++)
        {
            if (!env->goal_locations[i].empty())
            {
                // 上下左右四个格子内的障碍物越多, 对拥堵的影响就越大
                pickup_jam += double(4 - DefaultPlanner::global_neighbors[pickup_loc].size())
                              / DefaultPlanner::get_h(env, pickup_loc, env->goal_locations[i][0].first);

                // cout << "pickup jam: " << pickup_jam << endl;
            }
        }

        task_pickup_jams[t_id] = pickup_jam;

        if (std::chrono::steady_clock::now() > jam_end_moment)
        {
            // cout << "jam time not enough" << endl;
            break;
        }
    }

    /*
    for (auto it = task_pickup_jams.begin(); it != task_pickup_jams.end(); ++it) {
        std::cout << "task: " << it->first << ", jam: " << it->second << std::endl;
    }
     */

    // cout << "jam compute duration: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;

    int min_task_i, min_task_makespan, c_loc, count;


    // iterate over the free agents to decide which task to assign to each of them
    int best_total_distance = 0;
    auto it = free_agents.begin();
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

            // distance between current location and task pick up location
            int dist = DefaultPlanner::get_h(env, env->curr_states.at(i).location,
                                             env->task_pool[t_id].locations[0]);
            int temp_dist = dist;

            // TODO: 如果task pickup jams没有来得及算, 这里不要加上去
            dist += int ((task_pickup_jams[t_id] - 1.0 / temp_dist) * jam_coefficient);

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
            best_total_distance += min_task_makespan;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "best total distance: " << best_total_distance << endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

int TaskScheduler::compute_jam_task_circle_count_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                       int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int other_task_direction_x = pickup_loc_x - other_agent_goal_x;
            int other_task_direction_y = pickup_loc_y - other_agent_goal_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                             + other_task_direction_y * other_task_direction_y;

            // 只统计以task为圆心, agent-task范围内的other agent
            if(other_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_count_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];

            int sum_jam_weight = compute_jam_task_circle_count_goal(i, agent_loc_x,
                                                                    agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

void TaskScheduler::adaptive_jam_task_circle_count_goal_compare_dist(int time_limit,
                                                      std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            if (dist < min_task_heuristic)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];

                int sum_jam_weight = compute_jam_task_circle_count_goal(i, agent_loc_x,
                                                                        agent_loc_y, pickup_loc);

                // sum_jam_weight * jam_coefficient = guess delay time
                if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                    min_task_i = t_id;
                    min_task_dist = dist;
                    min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                    corresponding_traffic_jam = sum_jam_weight;
                }
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

void TaskScheduler::hungarian_sum_snatch_adaptive_jam_task_circle_count_goal(int time_limit,
                                                              std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<int> > dist_matrix;
    dist_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > jam_matrix;
    jam_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];
        int curr_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = curr_loc % env->cols;
        int agent_loc_y = curr_loc / env->cols;

        dist_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        jam_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);

        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];
            int pickup_loc = env->task_pool[task_id].locations[0];

            int sum_jam_weight = compute_jam_task_circle_count_goal(agent_id, agent_loc_x,
                                                                    agent_loc_y, pickup_loc);
            // cout << "traffic jam: " << sum_jam_weight << endl;

            int dist = DefaultPlanner::get_h(env, curr_loc, pickup_loc) + task_distances[task_id];
            dist_matrix[i][j] = dist;
            // 在算代价时, sum_jam_weight要乘以系数; 记录时, 不乘系数
            jam_matrix[i][j] = sum_jam_weight;
            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist + sum_jam_weight * jam_coefficient;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            int assigned_agent = free_agents_and_before_pickup[i];
            int assigned_task = free_tasks_and_before_pickup[work_assignment[i]];

            proposed_schedule[assigned_agent] = assigned_task;

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);

                agent_task[assigned_agent].task_id = assigned_task;
                agent_task[assigned_agent].min_task_dist  = dist_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].task_heuristic = cost_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].assign_moment = env->curr_timestep; // assign task moment
                agent_task[assigned_agent].jam_when_assign = jam_matrix[i][work_assignment[i]];
                // cout << "jam when assign: " << agent_task[assigned_agent].jam_when_assign << endl;
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

[[nodiscard]] double TaskScheduler::compute_jam_task_circle_vector_goal(int _agent_id, int _agent_loc_x,
                                                             int _agent_loc_y, int _pickup_loc) const
{
    double sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int agent_other_direction_x = other_agent_goal_x - _agent_loc_x;
            int agent_other_direction_y = other_agent_goal_y - _agent_loc_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int agent_other_distance_square = agent_other_direction_x * agent_other_direction_x
                                              + agent_other_direction_y * agent_other_direction_y;

            int other_task_direction_x = pickup_loc_x - other_agent_goal_x;
            int other_task_direction_y = pickup_loc_y - other_agent_goal_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                             + other_task_direction_y * other_task_direction_y;

            // 只统计以task为圆心, agent-task范围内的other agent
            if(other_task_distance_square < agent_task_direction_square)
            {
                int inner_product = agent_other_direction_x * agent_task_direction_x
                                    + agent_other_direction_y * agent_task_direction_y;

                // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                // << task_direction_length << endl;

                sum_jam_weight += double (inner_product) / agent_other_distance_square;
            }
        }
    }

    return sum_jam_weight;
}

// 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的数量为拥堵系数。
void TaskScheduler::adaptive_jam_task_circle_vector_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            double sum_jam_weight = compute_jam_task_circle_vector_goal(i, agent_loc_x,
                                                                    agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

int TaskScheduler::compute_jam_task_circle_count_both_current_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                    int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 仅计入有目标点的agent
        {
            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;
            // cout << "other agent loc " << other_agent_loc << " " << other_agent_loc_x << " "
            // << other_agent_loc_y << endl;

            int other_task_direction_x = pickup_loc_x - other_agent_loc_x;
            int other_task_direction_y = pickup_loc_y - other_agent_loc_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int other_task_distance_square = other_task_direction_x * other_task_direction_x
                                             + other_task_direction_y * other_task_direction_y;

            // 统计以task为圆心, agent-task范围内的other agent
            if(other_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }

            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int goal_task_direction_x = pickup_loc_x - other_agent_goal_x;
            int goal_task_direction_y = pickup_loc_y - other_agent_goal_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int goal_task_distance_square = goal_task_direction_x * goal_task_direction_x
                                            + goal_task_direction_y * goal_task_direction_y;

            // 统计以task为圆心, agent-task范围内的other agent goal
            if(goal_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 统计以task为圆心，|agent-task|为半径的圆中other agent current location 和 goal的数量作为jam.
void TaskScheduler::adaptive_jam_task_circle_count_both_current_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_circle_count_both_current_goal(i, agent_loc_x,
                                                                                 agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 统计以task为圆心，|agent-task|为半径的圆中other agent current location 和 goal的数量作为jam.
void TaskScheduler::adaptive_jam_task_circle_count_both_current_goal_compare_dist(int time_limit,
                                                              std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            if (dist < min_task_heuristic)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int sum_jam_weight = compute_jam_task_circle_count_both_current_goal(i,
                                                             agent_loc_x, agent_loc_y, pickup_loc);

                // sum_jam_weight * jam_coefficient = guess delay time
                if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                    min_task_i = t_id;
                    min_task_dist = dist;
                    min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                    corresponding_traffic_jam = sum_jam_weight;
                }
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

void TaskScheduler::hungarian_sum_snatch_adaptive_jam_task_circle_count_both_current_goal(
        int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<int> > dist_matrix;
    dist_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > jam_matrix;
    jam_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];
        int curr_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = curr_loc % env->cols;
        int agent_loc_y = curr_loc / env->cols;

        dist_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        jam_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);

        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];
            int pickup_loc = env->task_pool[task_id].locations[0];

            int sum_jam_weight = compute_jam_task_circle_count_both_current_goal(agent_id,
                                                             agent_loc_x, agent_loc_y, pickup_loc);
            // cout << "traffic jam: " << sum_jam_weight << endl;

            int dist = DefaultPlanner::get_h(env, curr_loc, pickup_loc) + task_distances[task_id];
            dist_matrix[i][j] = dist;
            // 在算代价时, sum_jam_weight要乘以系数; 记录时, 不乘系数
            jam_matrix[i][j] = sum_jam_weight;
            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist + sum_jam_weight * jam_coefficient;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            int assigned_agent = free_agents_and_before_pickup[i];
            int assigned_task = free_tasks_and_before_pickup[work_assignment[i]];

            proposed_schedule[assigned_agent] = assigned_task;

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);

                agent_task[assigned_agent].task_id = assigned_task;
                agent_task[assigned_agent].min_task_dist  = dist_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].task_heuristic = cost_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].assign_moment = env->curr_timestep; // assign task moment
                agent_task[assigned_agent].jam_when_assign = jam_matrix[i][work_assignment[i]];
                // cout << "jam when assign: " << agent_task[assigned_agent].jam_when_assign << endl;
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// 计算adaptive_jam_middle_current_goal_task_circle_count的估计延迟时间
int TaskScheduler::compute_jam_task_circle_count_middle_current_goal(int _agent_id,
                                                                     int _agent_loc_x, int _agent_loc_y, int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                      agent_task_direction_y * agent_task_direction_y;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;

            // 其他agent当前位置和目标点位置的中点
            int other_agent_middle_x = (other_agent_loc_x + other_agent_goal_x) / 2;
            int other_agent_middle_y = (other_agent_loc_y + other_agent_goal_y) / 2;

            // 其他agent当前位置和目标点位置的中点到pickup的向量
            int middle_task_direction_x = pickup_loc_x - other_agent_middle_x;
            int middle_task_direction_y = pickup_loc_y - other_agent_middle_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int middle_task_distance_square = middle_task_direction_x * middle_task_direction_x
                                              + middle_task_direction_y * middle_task_direction_y;

            // 统计以task为圆心, agent-task范围内的other agent当前位置和目标点位置的中点的数量
            if(middle_task_distance_square < agent_task_direction_square)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
void TaskScheduler::adaptive_jam_task_circle_count_middle_current_goal(int time_limit,
                                                                       std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_circle_count_middle_current_goal(i,
                                                           agent_loc_x, agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
void TaskScheduler::adaptive_jam_task_circle_count_middle_current_goal_compare_dist(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            // 排除dist >= min_task_heuristic的情况
            if (dist < min_task_heuristic)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int sum_jam_weight = compute_jam_task_circle_count_middle_current_goal(i,
                                                                                       agent_loc_x, agent_loc_y,
                                                                                       pickup_loc);

                // sum_jam_weight * jam_coefficient = guess delay time
                if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                    min_task_i = t_id;
                    min_task_dist = dist;
                    min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                    corresponding_traffic_jam = sum_jam_weight;
                }
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// hungarian_sum_snatch计算cost matrix加上adaptive_jam_middle_current_goal_task_circle_count
void TaskScheduler::hungarian_sum_snatch_adaptive_jam_task_circle_count_middle_current_goal(
        int time_limit, std::vector<int> & proposed_schedule)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;
    clock_t start = clock();

    // 没有新agent就不用抢单了
    if(env->new_freeagents.empty())
    {
        return;
    }

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int counter = 0;
    for (const int& free_task_id : free_tasks)
    {
        if (counter % 10 == 0 && std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        // this task distance has been calculated
        if (task_distances.find(free_task_id) == task_distances.end())
        {
            int total_dist = 0;
            int curr_loc = env->task_pool[free_task_id].locations[0];

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[free_task_id].locations)
            {
                total_dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }

            task_distances[free_task_id] = total_dist;
            counter++;
            // cout << counter << endl;
        }
        else
        {
            // cout << "already in" << endl;
        }
    }

    // free agent and agent with task but before pickup
    std::vector<int> free_agents_and_before_pickup;
    free_agents_and_before_pickup.assign(free_agents.begin(), free_agents.end());

    // free tasks and tasks already assigned but before pickup
    std::vector<int> free_tasks_and_before_pickup;
    free_tasks_and_before_pickup.assign(free_tasks.begin(), free_tasks.end());

    for(const auto& element : env->task_pool)
    {
        if(element.second.agent_assigned != -1 && element.second.idx_next_loc == 0)
        {
            free_agents_and_before_pickup.emplace_back(element.second.agent_assigned);
            free_tasks_and_before_pickup.emplace_back(element.second.task_id);
        }
    }

    // 每个时间步，空闲agent数量等于新出现任务的数量。
    // cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    vector< vector<int> > dist_matrix;
    dist_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > jam_matrix;
    jam_matrix.resize(free_agents_and_before_pickup.size());
    vector< vector<double> > cost_matrix;
    cost_matrix.resize(free_agents_and_before_pickup.size());
    for(int i=0;i<cost_matrix.size();i++)
    {
        auto agent_id = free_agents_and_before_pickup[i];
        int curr_loc = env->curr_states.at(agent_id).location;
        int agent_loc_x = curr_loc % env->cols;
        int agent_loc_y = curr_loc / env->cols;

        dist_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        jam_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);
        cost_matrix[i].resize(free_tasks_and_before_pickup.size(), 0);

        for(int j=0;j<free_tasks_and_before_pickup.size();j++)
        {
            auto task_id = free_tasks_and_before_pickup[j];
            int pickup_loc = env->task_pool[task_id].locations[0];

            int sum_jam_weight = compute_jam_task_circle_count_middle_current_goal(agent_id,
                                                                                   agent_loc_x, agent_loc_y, pickup_loc);
            // cout << "traffic jam: " << sum_jam_weight << endl;

            int dist = DefaultPlanner::get_h(env, curr_loc, pickup_loc) + task_distances[task_id];
            dist_matrix[i][j] = dist;
            // 在算代价时, sum_jam_weight要乘以系数; 记录时, 不乘系数
            jam_matrix[i][j] = sum_jam_weight;
            // 计算agent id完成task id的成本
            cost_matrix[i][j] = dist + sum_jam_weight * jam_coefficient;
        }
    }

    vector<int> work_assignment; // "worker " << i << ", assignment " << work_assignment[i]
    Hungarian HungAlgo(cost_matrix);
    // HungAlgo.print_cost_matrix();
    // cout << "prev cost: " << prev_best_total_distance << endl;
    int cost = HungAlgo.Solve(work_assignment); // 如果后续算拥堵系数, cost和prev_best的类型改成double
    // cout << "new cost: " << cost << endl;

    if (cost != prev_best_total_distance)
    {
        prev_best_total_distance = cost;
        free_agents.clear();
        free_agents.insert(free_agents_and_before_pickup.begin(), free_agents_and_before_pickup.end());
        free_tasks.clear();
        free_tasks.insert(free_tasks_and_before_pickup.begin(), free_tasks_and_before_pickup.end());

        for(int i=0; i < free_agents_and_before_pickup.size(); i++)
        {
            int assigned_agent = free_agents_and_before_pickup[i];
            int assigned_task = free_tasks_and_before_pickup[work_assignment[i]];

            proposed_schedule[assigned_agent] = assigned_task;

            // 该工人被分配了任务
            if(work_assignment[i] != -1)
            {
                free_agents.erase(free_agents_and_before_pickup[i]);
                free_tasks.erase(free_tasks_and_before_pickup[work_assignment[i]]);

                agent_task[assigned_agent].task_id = assigned_task;
                agent_task[assigned_agent].min_task_dist  = dist_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].task_heuristic = cost_matrix[i][work_assignment[i]];
                agent_task[assigned_agent].assign_moment = env->curr_timestep; // assign task moment
                agent_task[assigned_agent].jam_when_assign = jam_matrix[i][work_assignment[i]];
                // cout << "jam when assign: " << agent_task[assigned_agent].jam_when_assign << endl;
            }
        }
    }

    /*
    cout << "work assignment: ";
    for(int i=0;i<work_assignment.size();i++)
    {
        cout << work_assignment[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "proposed schedule: ";
    for(int i=0;i<proposed_schedule.size();i++)
    {
        cout << proposed_schedule[i] << " ";
    }
    cout << endl;
     //*/

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "hungarian total distance: " << cost << endl; // 2087, 比默认方法确实缩小了
}

// 7: 计算jam_task_Manhattan_circle_count_middle_current_goal的估计延迟时间
int TaskScheduler::compute_jam_task_Manhattan_circle_count_middle_current_goal(int _agent_id,
                                                                               int _agent_loc_x, int _agent_loc_y, int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    // 当前agent指向某个任务的向量
    int agent_task_direction_x = pickup_loc_x - _agent_loc_x;
    int agent_task_direction_y = pickup_loc_y - _agent_loc_y;
    int agent_task_Manhattan_distance = std::abs(agent_task_direction_x) +
                                        std::abs(agent_task_direction_y);

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;

            // 其他agent当前位置和目标点位置的中点
            int other_agent_middle_x = (other_agent_loc_x + other_agent_goal_x) / 2;
            int other_agent_middle_y = (other_agent_loc_y + other_agent_goal_y) / 2;

            // 其他agent当前位置和目标点位置的中点到pickup的向量
            int middle_task_direction_x = pickup_loc_x - other_agent_middle_x;
            int middle_task_direction_y = pickup_loc_y - other_agent_middle_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int middle_task_Manhattan_distance = std::abs(middle_task_direction_x)
                                                 + std::abs(middle_task_direction_y);

            // 统计以task为圆心, agent-task曼哈顿范围内的other agent当前位置和目标点位置的中点的数量
            if(middle_task_Manhattan_distance < agent_task_Manhattan_distance)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 统计以task为圆心，|agent-task|为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
void TaskScheduler::adaptive_jam_task_Manhattan_circle_count_middle_current_goal(int time_limit,
                                                                                 std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                // dist += sorting_grid.heuristics[c_loc].at(loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_Manhattan_circle_count_middle_current_goal(i,
                                                                                             agent_loc_x, agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 使用RHCR启发式
void TaskScheduler::adaptive_jam_task_Manhattan_circle_count_middle_current_goal_rhcr(
        int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

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
                // 如果sorting_grid.heuristics[loc]没有计算, 就计算
                if(sorting_grid.heuristics.find(loc) == sorting_grid.heuristics.end())
                {
                    sorting_grid.heuristics[loc] =
                            sorting_grid.compute_heuristics(loc);
                }

                dist += sorting_grid.heuristics[loc].at(c_loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_Manhattan_circle_count_middle_current_goal(i,
                                                                                             agent_loc_x, agent_loc_y, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 8: 计算jam_task_fix_Manhattan_circle_count_middle_current_goal的估计延迟时间
int TaskScheduler::compute_jam_task_fix_Manhattan_circle_count_middle_current_goal(int _agent_id, int _pickup_loc) const
{
    int sum_jam_weight = 0;

    int pickup_loc_x = _pickup_loc % env->cols;
    int pickup_loc_y = _pickup_loc / env->cols;
    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
        {
            int other_agent_goal = env->goal_locations[j][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int other_agent_loc = env->curr_states.at(j).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;

            // 其他agent当前位置和目标点位置的中点
            int other_agent_middle_x = (other_agent_loc_x + other_agent_goal_x) / 2;
            int other_agent_middle_y = (other_agent_loc_y + other_agent_goal_y) / 2;

            // 其他agent当前位置和目标点位置的中点到pickup的向量
            int middle_task_direction_x = pickup_loc_x - other_agent_middle_x;
            int middle_task_direction_y = pickup_loc_y - other_agent_middle_y;
            // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
            int middle_task_Manhattan_distance = std::abs(middle_task_direction_x)
                                                 + std::abs(middle_task_direction_y);

            // 统计以task为圆心, radius_count_agent曼哈顿范围内的other agent当前位置和目标点位置的中点的数量
            if(middle_task_Manhattan_distance < radius_count_agent)
            {
                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 8: 统计以task为圆心，|num_rows + num_columns| / 8为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
void TaskScheduler::adaptive_jam_task_fix_Manhattan_circle_count_middle_current_goal(int time_limit,
                                                                                 std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
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
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = compute_jam_task_fix_Manhattan_circle_count_middle_current_goal(
                    i, pickup_loc);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }

            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// 9: 将地图分成16x16的区域, task所在区域中agent当前位置和目标点位置的中点的数量作为作为sum jam weight
void TaskScheduler::adaptive_jam_task_region_count_middle_current_goal(int time_limit,
                                                                       std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    if(free_agents.size() > 50)
    {
        if(first_epoch_done_time == -1)
        {
            // 由于大算例初始任务过多, 在初始任务分配完成前采用默认算法
            greedy_sum_without_newtask(time_limit, proposed_schedule);
            return;
        }
    }
    else
    {
        if(first_epoch_done_time == -1)
        {
            first_epoch_done_time = env->curr_timestep; // 初始阶段的任务分配完毕的时间
        }
    }

    int region_column = 16; // 每个region所占的列数
    int region_row = 16; // 每个region所占的行数

    // 地图有几列region
    int num_region_column = std::ceil((double)env->cols / region_column);
    // 地图有几行region
    int num_region_row = std::ceil((double)env->rows / region_row);

    // 将map分为若干区域, 统计每个区域agent当前位置和目标点位置的中点的数量
    vector<int> region_middle_num(num_region_column * num_region_row, 0);

    for (int i=0;i<env->num_of_agents;i++)
    {
        if (!env->goal_locations[i].empty()) // 只统计有目标的agent
        {
            int other_agent_goal = env->goal_locations[i][0].first;
            int other_agent_goal_x = other_agent_goal % env->cols;
            int other_agent_goal_y = other_agent_goal / env->cols;
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            int other_agent_loc = env->curr_states.at(i).location;
            int other_agent_loc_x = other_agent_loc % env->cols;
            int other_agent_loc_y = other_agent_loc / env->cols;

            // 其他agent当前位置和目标点位置的中点
            int other_agent_middle_x = (other_agent_loc_x + other_agent_goal_x) / 2;
            int other_agent_middle_y = (other_agent_loc_y + other_agent_goal_y) / 2;

            int middle_region_x = other_agent_middle_x / region_column;
            int middle_region_y = other_agent_middle_y / region_row;

            region_middle_num[middle_region_y * num_region_column + middle_region_x]++;
        }
    }

    /*
    cout << "region agent num: ";
    for(int i : region_middle_num)
    {
        cout << i << " ";
    }
    cout << endl;
     //*/

    if(env->num_of_agents <= 500) // 小算例直接算
    {
        for (int t_id : env->new_tasks)
        {
            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = pickup_loc % env->cols;
            int pickup_loc_y = pickup_loc / env->cols;

            int pickup_region_x = pickup_loc_x / region_column;
            int pickup_region_y = pickup_loc_y / region_row;

            task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
        }
    }
    else // 大算例
    {
        if(env->curr_timestep == first_epoch_done_time) // 在first_epoch_done_time计算完此时的free tasks
        {
            for (int t_id : free_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
        else // first_epoch_done_time之后就只计算新任务
        {
            for (int t_id : env->new_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
    }


    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            int sum_jam_weight = region_middle_num[task_region[t_id]];

            // sum_jam_weight * jam_coefficient = estimated delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Task Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// (10) 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent当前位置和目标点位置的中点的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
void TaskScheduler::adaptive_jam_task_circle_vector_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            double sum_jam_weight = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = env->task_pool[t_id].locations[0] % env->cols;
            int pickup_loc_y = env->task_pool[t_id].locations[0] / env->cols;
            // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

            // 当前agent指向某个任务的向量
            int agent_task_direction_x = pickup_loc_x - agent_loc_x;
            int agent_task_direction_y = pickup_loc_y - agent_loc_y;
            int agent_task_direction_square = agent_task_direction_x * agent_task_direction_x +
                                              agent_task_direction_y * agent_task_direction_y;

            for(int j=0;j<env->goal_locations.size();j++)
            {
                if (j != i && !env->goal_locations[j].empty()) // 所有agent都会有任务, 所以都要计入
                {
                    int other_agent_loc = env->curr_states.at(j).location;
                    int other_agent_loc_x = other_agent_loc % env->cols;
                    int other_agent_loc_y = other_agent_loc / env->cols;

                    int other_agent_goal = env->goal_locations[j][0].first;
                    int other_agent_goal_x = other_agent_goal % env->cols;
                    int other_agent_goal_y = other_agent_goal / env->cols;
                    // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
                    // << other_agent_goal_y << endl;

                    // 其他agent当前位置和目标点位置的中点
                    int other_agent_middle_x = (other_agent_loc_x + other_agent_goal_x) / 2;
                    int other_agent_middle_y = (other_agent_loc_y + other_agent_goal_y) / 2;

                    // 当前agent指向其他agent当前位置和目标点中点的向量
                    int agent_middle_direction_x = other_agent_middle_x - agent_loc_x;
                    int agent_middle_direction_y = other_agent_middle_y - agent_loc_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int agent_middle_distance_square = agent_middle_direction_x * agent_middle_direction_x
                                                      + agent_middle_direction_y * agent_middle_direction_y;

                    // 其他agent当前位置和目标点中点指向任务点的向量
                    int middle_task_direction_x = pickup_loc_x - other_agent_goal_x;
                    int middle_task_direction_y = pickup_loc_y - other_agent_goal_y;
                    // cout << "other agent direction " << agent_other_direction_x << " " << other_agent_loc_y << endl;
                    int middle_task_distance_square = middle_task_direction_x * middle_task_direction_x
                                                     + middle_task_direction_y * middle_task_direction_y;

                    // 只统计以task为圆心, agent-task范围内的other agent当前位置和目标点的中点
                    if(middle_task_distance_square < agent_task_direction_square)
                    {
                        int inner_product = agent_middle_direction_x * agent_task_direction_x
                                            + agent_middle_direction_y * agent_task_direction_y;

                        // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                        // << task_direction_length << endl;

                        sum_jam_weight += double (inner_product) / agent_middle_distance_square;
                    }
                }
            }

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// compute pickup jam by counting whether other agent-task line intersect with this agent-task line
[[nodiscard]] int TaskScheduler::compute_jam_curr_pickup_intersect_curr_goal(int _agent_id,
                                                                 Point _agent_loc, Point _agent_end)
{
    int sum_jam_weight = 0;

    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // only consider the agent with goals
        {
            int other_agent_loc = env->curr_states.at(j).location;
            Point other_agent_start{other_agent_loc % env->cols, other_agent_loc / env->cols};

            int other_agent_goal = env->goal_locations[j][0].first;
            Point other_agent_end{other_agent_goal % env->cols, other_agent_goal / env->cols};
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            // 统计other agent和它目标点的连线与agent-pickup连线发生交叉的数量
            if(isIntersecting(_agent_loc, _agent_end, other_agent_start, other_agent_end))
            {
                // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                // << task_direction_length << endl;

                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 11: compute pickup jam by counting whether other agent-task line intersect with this agent-task line
void TaskScheduler::adaptive_jam_curr_pickup_intersect_curr_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // test whether two vectors intersect or not
    /*
    Point startA{0, 0};
    Point endA{0, 2};
    Point startB{2, 0};
    Point endB{2, 2};
    cout << "is intersect: " << isIntersecting(startA, endA, startB, endB) << endl;
     （*/

    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        Point agent_point{agent_loc % env->cols, agent_loc / env->cols};
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            Point pickup_point{pickup_loc % env->cols, pickup_loc / env->cols};
            int sum_jam_weight = compute_jam_curr_pickup_intersect_curr_goal(i,
                                                                             agent_point,
                                                                             pickup_point);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

void TaskScheduler::adaptive_jam_curr_pickup_delivery_intersect_curr_goal(int time_limit,
                                                              std::vector<int> & proposed_schedule)
{
    // test whether two vectors intersect or not
    /*
    Point startA{0, 0};
    Point endA{0, 2};
    Point startB{2, 0};
    Point endB{2, 2};
    cout << "is intersect: " << isIntersecting(startA, endA, startB, endB) << endl;
     （*/

    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout << "schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        Point agent_point{agent_loc % env->cols, agent_loc / env->cols};
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            /*
            if(OneDim2TwoDim.find(env->task_pool[t_id].locations[0]) == OneDim2TwoDim.end())
            {
                cerr << "map error!" << endl;
            }
            */

            int pickup_loc = env->task_pool[t_id].locations[0];
            Point pickup_point{pickup_loc % env->cols, pickup_loc / env->cols};
            int delivery_loc = env->task_pool[t_id].locations[1];
            Point delivery_point{delivery_loc % env->cols, delivery_loc / env->cols};
            int sum_jam_weight = compute_jam_curr_pickup_intersect_curr_goal(i,
                                             agent_point, pickup_point)
                                + compute_jam_curr_pickup_intersect_curr_goal(i,
                                               pickup_point, delivery_point);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
            // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}

// RHCR functions
MAPFSolver* TaskScheduler::set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
    string solver_name = vm["single_agent_solver"].as<string>();
    SingleAgentSolver* path_planner;
    MAPFSolver* mapf_solver;
    if (solver_name == "ASTAR")
    {
        path_planner = new StateTimeAStar();
    }
    else if (solver_name == "SIPP")
    {
        path_planner = new SIPP();
    }
    else
    {
        cout << "Single-agent solver " << solver_name << "does not exist!" << endl;
        exit(-1);
    }

    solver_name = vm["solver"].as<string>();
    if (solver_name == "ECBS")
    {
        ECBS* ecbs = new ECBS(G, *path_planner);
        ecbs->potential_function = vm["potential_function"].as<string>();
        ecbs->potential_threshold = vm["potential_threshold"].as<double>();
        ecbs->suboptimal_bound = vm["suboptimal_bound"].as<double>();
        mapf_solver = ecbs;
    }
    else if (solver_name == "PBS")
    {
        PBS* pbs = new PBS(G, *path_planner);
        pbs->lazyPriority = vm["lazyP"].as<bool>();
        auto prioritize_start = vm["prioritize_start"].as<bool>();
        if (vm["hold_endpoints"].as<bool>() || vm["dummy_paths"].as<bool>())
            prioritize_start = false;
        pbs->prioritize_start = prioritize_start;
        pbs->setRT(vm["CAT"].as<bool>(), prioritize_start);
        mapf_solver = pbs;
    }
    else if (solver_name == "WHCA")
    {
        mapf_solver = new WHCAStar(G, *path_planner);
    }
    else if (solver_name == "LRA")
    {
        mapf_solver = new LRAStar(G, *path_planner);
    }
    else
    {
        cout << "Solver " << solver_name << "does not exist!" << endl;
        exit(-1);
    }

    if (vm["id"].as<bool>())
    {
        return new ID(G, *path_planner, *mapf_solver);
    }
    else
    {
        return mapf_solver;
    }
}

void TaskScheduler::set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
    system.outfile = vm["output"].as<std::string>();
    system.screen = vm["screen"].as<int>();
    system.log = vm["log"].as<bool>();
    system.num_of_drives = vm["agentNum"].as<int>();
    system.time_limit = vm["cutoffTime"].as<int>();
    system.simulation_window = vm["simulation_window"].as<int>();
    system.planning_window = vm["planning_window"].as<int>();
    system.travel_time_window = vm["travel_time_window"].as<int>();
    system.consider_rotation = vm["rotation"].as<bool>();
    system.k_robust = vm["robust"].as<int>();
    system.hold_endpoints = vm["hold_endpoints"].as<bool>(); // 如果只plan一步, 不用担心hold endpoints
    system.useDummyPaths = vm["dummy_paths"].as<bool>();
    if (vm.count("seed"))
        system.seed = vm["seed"].as<int>();
    else
        system.seed = (int)time(0);
    srand(system.seed);
}