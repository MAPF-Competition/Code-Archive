#include "TaskScheduler.h"

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
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
    get_anchor(env);
    // get_sector(env);
    time_step = 0;
}

/**
 * Plans a task schedule within a specified time limit.
 *
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 *
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    // DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    my_plan(limit, proposed_schedule, env);
}

int TaskScheduler::cul_score(int robot_id, int task_id, SharedEnvironment *env, int cur_task_dis)
{
    // return 0.1;

    int dist = 0;
    int c_loc = env->curr_states.at(robot_id).location;

    int assigned_task_id = env->curr_task_schedule[robot_id];
    if (assigned_task_id >= 0)
    {
        int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
        if (i_loc > 0 && env->task_pool[assigned_task_id].locations.size()>0)
        {
            // dist += ceil(0.5 * DefaultPlanner::manhattanDistance(c_loc, env->task_pool[assigned_task_id].locations.back(), env));
            // dist += ceil(0.5 * DefaultPlanner::get_h(env, c_loc, env->task_pool[assigned_task_id].locations.back()));
            dist += get_est_dist(c_loc, env->task_pool[assigned_task_id].locations.back(), env);
            c_loc = env->task_pool[assigned_task_id].locations.back();
        }
    }
    // iterate over the locations (errands) of the task to compute the makespan to finish the task
    // makespan: the time for the agent to complete all the errands of the task t_id in order
    // for (int loc : env->task_pool[task_id].locations)
    // {
    //     dist += DefaultPlanner::get_h(env, c_loc, loc);
    //     c_loc = loc;
    // }
    dist += get_est_dist(c_loc, env->task_pool[task_id].locations[0], env);
    // dist += DefaultPlanner::manhattanDistance(c_loc, env->task_pool[task_id].locations[0], env);
    // dist += DefaultPlanner::get_h(env, c_loc, env->task_pool[task_id].locations[0]);
    dist += cur_task_dis;
    // cout<<","<<endl;
    return 200 - dist;
}

void TaskScheduler::my_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    // cout << "!!!!!!";
    // for (int i = 0; i < proposed_schedule.size(); i++)
    //     cout << proposed_schedule[i] << ", ";
    // cout << endl;
    all_task_ids.insert(all_task_ids.end(), env->new_tasks.begin(), env->new_tasks.end());
    for (int t_id : all_task_ids)
    {
        int i_loc = env->task_pool[t_id].idx_next_loc;
        if (i_loc > 0 || env->task_pool[t_id].task_id != t_id || env->task_pool[t_id].locations.size() <= 0)
            all_task_ids.erase(std::remove(all_task_ids.begin(), all_task_ids.end(), t_id), all_task_ids.end());
    }
    if (time_step == 0)
    {
        robot_ids.insert(robot_ids.end(), env->new_freeagents.begin(), env->new_freeagents.end());
        task_ids = all_task_ids;
    }
    else
    {
        int loc_num = env->new_tasks.size() + env->new_freeagents.size();
        if (loc_num <= 0)
            return;

        robot_ids.clear();
        task_ids.clear();
        robot_ids.insert(robot_ids.end(), env->new_freeagents.begin(), env->new_freeagents.end());
        task_ids.insert(task_ids.end(), env->new_tasks.begin(), env->new_tasks.end());
        for (int t_id : task_ids)
        {
            int i_loc = env->task_pool[t_id].idx_next_loc;
            if (i_loc > 0 || env->task_pool[t_id].task_id != t_id || env->task_pool[t_id].locations.size() <= 0)
                task_ids.erase(std::remove(task_ids.begin(), task_ids.end(), t_id), task_ids.end());
        }
        int robot_limit_per_loc = ceil((double)robot_size_limit / (double)loc_num);
        int task_limit_per_loc = ceil((double)task_size_limit / (double)loc_num);

        vector<int> center_locs;
        int cur_task_id;
        for (int i = 0; i < task_ids.size(); i++)
        {
            cur_task_id = task_ids[i];
            if (env->task_pool[cur_task_id].task_id == cur_task_id || env->task_pool[cur_task_id].locations.size() > 0)
                center_locs.push_back(env->task_pool[cur_task_id].locations[0]);
        }
        for (int i = 0; i < robot_ids.size(); i++)
        {
            center_locs.push_back(env->curr_states.at(robot_ids[i]).location);
        }

        int dist, i_loc;
        std::vector<std::pair<int, int>> loc_dists;
        for (int i = 0; i < center_locs.size(); i++)
        {
            // add robot
            loc_dists.clear();
            for (int j = 0; j < env->num_of_agents; j++)
            {
                int assigned_task_id = env->curr_task_schedule[j];
                if (assigned_task_id >= 0)
                {
                    if (env->task_pool[assigned_task_id].locations.size()<=0)
                    {
                        robot_ids.push_back(j);
                        continue;
                    }

                    i_loc = env->task_pool[assigned_task_id].idx_next_loc;
                    if (i_loc <= 0)
                        dist = get_est_dist(center_locs[i], env->curr_states.at(j).location, env);
                    else if (i_loc >= env->task_pool[assigned_task_id].locations.size() - 1)
                    {
                        int dist_to_go = get_est_dist(env->curr_states.at(j).location, env->task_pool[assigned_task_id].locations.back(), env);
                        if (dist_to_go > 20)
                            continue;
                        dist = get_est_dist(center_locs[i], env->task_pool[assigned_task_id].locations.back(), env);
                    }
                    else
                        continue;
                }
                else
                    dist = get_est_dist(center_locs[i], env->curr_states.at(j).location, env);

                if (dist > 0 && dist < MAX_TIMESTEP)
                {
                    loc_dists.emplace_back(dist, j);
                }
            }

            std::sort(loc_dists.begin(), loc_dists.end(),
                      [](std::pair<int, int> &a, std::pair<int, int> &b)
                      {
                          return a.first < b.first;
                      });
            for (int j = 0; j < min((int)loc_dists.size(), robot_limit_per_loc); j++)
            {
                int robot_id = loc_dists[j].second;
                if (robot_id>=500)
                    cout<<"!!"<<endl;
                int assigned_task_id = env->curr_task_schedule[robot_id];
                if (assigned_task_id < 0)
                    robot_ids.push_back(robot_id);
                else
                {
                    i_loc = env->task_pool[assigned_task_id].idx_next_loc;
                    if (i_loc <= 0)
                    {
                        robot_ids.push_back(robot_id);
                        task_ids.push_back(assigned_task_id);
                    }
                    else
                    {
                        robot_ids.push_back(robot_id);
                        // task_ids.push_back(assigned_task_id);
                        // cout << "wrong task" << assigned_task_id << endl;
                    }
                }
            }

            // add task
            loc_dists.clear();
            for (int j = 0; j < all_task_ids.size(); j++)
            {
                int assigned_robot_id = env->task_pool[all_task_ids[j]].agent_assigned;
                int i_loc = env->task_pool[all_task_ids[j]].idx_next_loc;
                if (assigned_robot_id < 0 || (assigned_robot_id >= 0 && i_loc <= 0))
                    dist = get_est_dist(center_locs[i], env->task_pool[all_task_ids[j]].locations[0], env);
                else
                    continue;

                if (dist > 0 && dist < MAX_TIMESTEP)
                {
                    loc_dists.emplace_back(dist, all_task_ids[j]);
                }
            }

            std::sort(loc_dists.begin(), loc_dists.end(),
                      [](std::pair<int, int> &a, std::pair<int, int> &b)
                      {
                          return a.first < b.first;
                      });
            for (int j = 0; j < min((int)loc_dists.size(), task_limit_per_loc); j++)
            {
                int task_id = loc_dists[j].second;
                int assigned_robot_id = env->task_pool[task_id].agent_assigned;
                if (assigned_robot_id < 0)
                    task_ids.push_back(task_id);
                else
                {
                    task_ids.push_back(task_id);
                    robot_ids.push_back(assigned_robot_id);
                    if (assigned_robot_id>=500)
                        cout<<"!!"<<endl;
                }
            }
        }
        cout << "size!!!!!!!!!!!!!!!!!!!!!" << task_ids.size() << "," << robot_ids.size() << endl;

        std::unordered_set<int> set_robot(robot_ids.begin(), robot_ids.end());
        std::unordered_set<int> set_task(task_ids.begin(), task_ids.end());
        robot_ids.assign(set_robot.begin(), set_robot.end());
        task_ids.assign(set_task.begin(), set_task.end());
        std::sort(robot_ids.begin(), robot_ids.end());
        std::sort(task_ids.begin(), task_ids.end());

        // incase robot size>task size
        while (robot_ids.size() > task_ids.size())
        {
            cout << "size!!!!!!!!!!!!!!!!!!!!!" << task_ids.size() << "," << robot_ids.size() << endl;
            robot_ids.pop_back();
        }
    }
    cout << "size~~~" << task_ids.size() << "," << robot_ids.size() << endl;
    // cout << "tasks ";
    // for (int i = 0; i < task_ids.size(); i++)
    //     cout << task_ids[i] << ", ";
    // cout << endl;
    // cout << "robots ";
    // for (int i = 0; i < robot_ids.size(); i++)
    //     cout << robot_ids[i] << ", ";
    // cout << endl;

    // double t1 = 0.0;
    // double t2 = 0.0;
    clock_t start = clock();
    vector<int> task_dis, dis_error, manh_dis_error;
    // double dis_error_sum, manh_dis_error_sum;
    for (int i = 0; i < task_ids.size(); i++)
    {
        if (env->task_pool[task_ids[i]].locations.size()<=0)
            cout<<"?????????????????????"<<endl;
        int dist = 0;
        // double dist1 = 0;
        // double dist2 = 0;
        int c_loc = env->task_pool[task_ids[i]].locations[0];
        for (int loc : env->task_pool[task_ids[i]].locations)
        {
            // start = clock();
            // dist += DefaultPlanner::manhattanDistance(c_loc, loc, env);
            dist += get_est_dist(c_loc, loc, env);
            // t1 += double(clock() - start) / CLOCKS_PER_SEC;
            // start = clock();
            // dist1 += DefaultPlanner::get_h(env, c_loc, loc);
            // t2 += double(clock() - start) / CLOCKS_PER_SEC;
            // c_loc = loc;
        }
        task_dis.push_back(dist);
        // dis_error.push_back(abs(dist1 - dist2) / dist1);
        // dis_error_sum += abs(dist1 - dist2) / dist1;
        // manh_dis_error_sum += abs(dist1 - dist) / dist1;
    }
    // cout << "dis error " << dis_error_sum / (double)dis_error.size() << "manh dis error " << manh_dis_error_sum / (double)dis_error.size() << endl;
    // for (int i = 0; i < dis_error.size(); i++)
    // {
    //     cout << dis_error[i] << ", ";
    // }
    // cout << endl;

    // std::cout << "t1: " << t1 << " t2: " << t2 << std::endl;

    int N = robot_ids.size();
    int M = task_ids.size();
    int robot_id, task_id;
    int cur_task_dis;
    KuhnMunkres km(N, M);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            robot_id = robot_ids[i];
            task_id = task_ids[j];
            cur_task_dis = task_dis[j];
            // cout<<"task id "<<task_id<<", robot id "<<robot_id<<", cur_task_dis "<<cur_task_dis;
            int weight = cul_score(robot_id, task_id, env, cur_task_dis);
            // cout<<", weight "<<weight<<endl;
            km.setEdge(i, j, weight);
            // cout << "task id " << task_ids[j] << " weight " << weight << ", ";
        }
        // cout << endl;
    }

    km.solve();
    for (int i = 0; i < N; i++)
    {
        if (km.get_result()[i] < 0)
            continue;
        robot_id = robot_ids[i];
        task_id = task_ids[km.get_result()[i]];
        // cout << "idx " << i << ", robot id" << robot_id << ", task id" << task_id << endl;

        // 若当前robot有正在执行的任务，不分配
        int assigned_task_id = env->curr_task_schedule[robot_id];
        if (assigned_task_id >= 0)
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc > 0)
                continue;
        }
        proposed_schedule[robot_id] = task_id;
    }

    clock_t end = clock();
    double duration = double(end - start) / CLOCKS_PER_SEC;
    std::cout << "程序运行时间: " << duration << " 秒" << std::endl;
    time_step += 1;
}
int TaskScheduler::get_est_dist(int loc, int c_loc, SharedEnvironment *env)
{
    if (env->rows < 200)
        return DefaultPlanner::manhattanDistance(c_loc, loc, env);

    int dis_est = DefaultPlanner::manhattanDistance(c_loc, loc, env);
    // dist += dis_est;
    dis_est = max(dis_est, abs(DefaultPlanner::get_h(env, loc, anchor_loc_1) - DefaultPlanner::get_h(env, c_loc, anchor_loc_1)));
    dis_est = max(dis_est, abs(DefaultPlanner::get_h(env, loc, anchor_loc_2) - DefaultPlanner::get_h(env, c_loc, anchor_loc_2)));
    dis_est = max(dis_est, abs(DefaultPlanner::get_h(env, loc, anchor_loc_3) - DefaultPlanner::get_h(env, c_loc, anchor_loc_3)));
    dis_est = max(dis_est, abs(DefaultPlanner::get_h(env, loc, anchor_loc_4) - DefaultPlanner::get_h(env, c_loc, anchor_loc_4)));
    return dis_est;
}
void TaskScheduler::get_anchor(SharedEnvironment *env)
{
    int loc;
    anchor_loc_1 = -1;
    for (int i = 0; i < min(env->cols, env->rows); i++)
    {
        if (anchor_loc_1 > 0)
            break;
        for (int j = 0; j < i; j++)
        {

            loc = j + i * env->cols;
            if (env->map[loc] == 0)
                anchor_loc_1 = loc;
        }
    }

    int max_dis = 0;
    int dis_tmp;
    for (int i = 0; i < env->rows; i++)
        for (int j = 0; j < env->cols; j++)
        {
            loc = j + i * env->cols;
            if (env->map[loc] == 1)
                continue;
            dis_tmp = DefaultPlanner::get_h(env, loc, anchor_loc_1);
            if (dis_tmp > max_dis && dis_tmp < MAX_TIMESTEP)
            {
                max_dis = dis_tmp;
                anchor_loc_2 = loc;
            }
        }
    max_dis = 0;
    for (int i = 0; i < env->rows; i++)
        for (int j = 0; j < env->cols; j++)
        {
            loc = j + i * env->cols;
            if (env->map[loc] == 1)
                continue;
            dis_tmp = min(DefaultPlanner::get_h(env, loc, anchor_loc_1), DefaultPlanner::get_h(env, loc, anchor_loc_2));
            if (dis_tmp > max_dis && dis_tmp < MAX_TIMESTEP)
            {
                max_dis = dis_tmp;
                anchor_loc_3 = loc;
            }
        }
    max_dis = 0;
    for (int i = 0; i < env->rows; i++)
        for (int j = 0; j < env->cols; j++)
        {
            loc = j + i * env->cols;
            if (env->map[loc] == 1)
                continue;
            dis_tmp = min(DefaultPlanner::get_h(env, loc, anchor_loc_1), DefaultPlanner::get_h(env, loc, anchor_loc_2));
            dis_tmp = min(dis_tmp, DefaultPlanner::get_h(env, loc, anchor_loc_3));
            if (dis_tmp > max_dis && dis_tmp < MAX_TIMESTEP)
            {
                max_dis = dis_tmp;
                anchor_loc_4 = loc;
            }
        }
    dis_tmp = DefaultPlanner::get_h(env, anchor_loc_1, anchor_loc_4);
    dis_tmp = DefaultPlanner::get_h(env, anchor_loc_2, anchor_loc_4);
    dis_tmp = DefaultPlanner::get_h(env, anchor_loc_3, anchor_loc_4);
    cout << "p1: " << anchor_loc_1 / env->cols << "," << anchor_loc_1 % env->cols << endl;
    cout << "p1: " << anchor_loc_2 / env->cols << "," << anchor_loc_2 % env->cols << endl;
    cout << "p1: " << anchor_loc_3 / env->cols << "," << anchor_loc_3 % env->cols << endl;
    cout << "p1: " << anchor_loc_4 / env->cols << "," << anchor_loc_4 % env->cols << endl;
}
void TaskScheduler::get_sector(SharedEnvironment *env)
{
    int sector_num;
    if (env->num_of_agents <= 300)
        sector_num = 1;
    else if (env->num_of_agents <= 500)
        sector_num = 1;
    else if (env->num_of_agents <= 2000)
        sector_num = 10;
    else if (env->num_of_agents <= 5000)
        sector_num = 30;

    sector_anchor_list.clear();
    sector_anchor_list.push_back(anchor_loc_1);
    int max_dis, loc, dis_tmp, anchor_loc;
    for (int anchor_id = 1; anchor_id < sector_num; anchor_id++)
    {
        max_dis = 0;
        for (int i = 0; i < env->rows; i++)
            for (int j = 0; j < env->cols; j++)
            {
                loc = j + i * env->cols;
                if (env->map[loc] == 1)
                    continue;
                if (DefaultPlanner::get_h(env, loc, anchor_loc_1) >= MAX_TIMESTEP)
                    continue;

                dis_tmp = MAX_TIMESTEP;
                for (int k = 0; k < anchor_id; k++)
                    dis_tmp = min(dis_tmp, DefaultPlanner::get_h(env, loc, sector_anchor_list[k]));
                if (dis_tmp > max_dis && dis_tmp < MAX_TIMESTEP)
                {
                    max_dis = dis_tmp;
                    anchor_loc = loc;
                }
            }
        sector_anchor_list.push_back(anchor_loc);
    }
    std::vector<int> sector_loc_num;
    sector_loc_num.resize(sector_num, 0);

    loc_sector.resize(env->cols * env->rows, 0);
    int min_dis, sector_choose;
    for (int i = 0; i < env->rows; i++)
        for (int j = 0; j < env->cols; j++)
        {
            min_dis = MAX_TIMESTEP;
            sector_choose = -1;
            loc = j + i * env->cols;
            for (int k = 0; k < sector_num; k++)
            {
                dis_tmp = DefaultPlanner::get_h(env, loc, sector_anchor_list[k]);
                if (dis_tmp < min_dis)
                {
                    min_dis = dis_tmp;
                    sector_choose = k;
                }
            }
            if (sector_choose > 0)
            {
                loc_sector[loc] = sector_choose;
                sector_loc_num[sector_choose] += 1;
            }
            else
                loc_sector[loc] = -1;
        }
    for (int k = 0; k < sector_num; k++)
        cout << "sector anchor is " << sector_anchor_list[k] / env->cols << "," << sector_anchor_list[k] % env->cols << ", loc num is " << sector_loc_num[k] << endl;
}
