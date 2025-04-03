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
    get_sector(env);
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

int TaskScheduler::cul_score(int robot_id, int task_id, SharedEnvironment *env, double cur_task_dis)
{
    // return 0.1;

    int dist = 0;
    int c_loc = env->curr_states.at(robot_id).location;

    int assigned_task_id = env->curr_task_schedule[robot_id];
    if (assigned_task_id >= 0)
    {
        int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
        if (i_loc > 0)
        {
            // dist += ceil(0.5 * DefaultPlanner::manhattanDistance(c_loc, env->task_pool[assigned_task_id].locations.back(), env));
            // dist += ceil(0.5 * DefaultPlanner::get_h(env, c_loc, env->task_pool[assigned_task_id].locations.back()));
            dist += ceil(0.5 * get_est_dist(c_loc, env->task_pool[assigned_task_id].locations.back(), env));
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

    robot_ids.clear();
    task_ids.clear();

    for (int i = 0; i < env->num_of_agents; i++)
    {
        // if (loc_sector[env->curr_states.at(i).location] != time_step % (int)sector_anchor_list.size())
        //     continue;
        int assigned_task_id = env->curr_task_schedule[i];
        if (assigned_task_id < 0)
            robot_ids.push_back(i);
        else if (robot_ids.size() < robot_size_limit)
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc <= 0)
            {
                robot_ids.push_back(i);
                task_ids.push_back(assigned_task_id);
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; i++)
    {
        // if (loc_sector[env->curr_states.at(i).location] != time_step % (int)sector_anchor_list.size())
        //     continue;
        // cout << loc_sector[env->curr_states.at(i).location] << ",  " << time_step << "." << sector_anchor_list.size();
        if (robot_ids.size() >= robot_size_limit)
            break;
        int assigned_task_id = env->curr_task_schedule[i];
        if (assigned_task_id < 0)
            continue;
        else
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc >= env->task_pool[assigned_task_id].locations.size() - 1) // 进行最后一个任务时，可分配下个任务
            {
                int dis_tmp = DefaultPlanner::manhattanDistance(i_loc, env->curr_states.at(i).location, env);
                if (dis_tmp < 20)
                    robot_ids.push_back(i);
            }
        }
    }

    for (int i = 0; i < all_task_ids.size(); i++)
    {
        if (env->task_pool[all_task_ids[i]].agent_assigned < 0)
            task_ids.push_back(all_task_ids[i]);
    }
    for (int i = 0; i < all_task_ids.size(); i++)
    {
        int assigned_robot_id = env->task_pool[all_task_ids[i]].agent_assigned;
        int i_loc = env->task_pool[all_task_ids[i]].idx_next_loc;
        if (assigned_robot_id >= 0 && i_loc <= 0 &&
            (task_ids.size() < task_size_limit || task_ids.size() < robot_ids.size()))
        {
            auto it = std::find(robot_ids.begin(), robot_ids.end(), assigned_robot_id);
            // 本轮未分配
            if (it == robot_ids.end())
            {
                task_ids.push_back(all_task_ids[i]);
                robot_ids.push_back(assigned_robot_id);
            }
        }
    }
    // int cur_sector_idx = time_step % (int)sector_anchor_list.size();
    // get_sector_tasks(cur_sector_idx, env);

    // vector<int> near_sectors;
    // near_sectors.push_back(cur_sector_idx);
    // int near_sector_idx, min_dis, dis_tmp;
    // while (task_ids.size() < robot_ids.size())
    // {
    //     min_dis = MAX_TIMESTEP;
    //     for (int i = 0; i < sector_anchor_list.size(); i++)
    //     {
    //         auto it = std::find(near_sectors.begin(), near_sectors.end(), i);
    //         if (it != near_sectors.end())
    //             continue;
    //         dis_tmp = DefaultPlanner::get_h(env, sector_anchor_list[i], sector_anchor_list[cur_sector_idx]);
    //         if (dis_tmp < min_dis)
    //         {
    //             min_dis = dis_tmp;
    //             near_sector_idx = i;
    //         }
    //     }
    //     get_sector_tasks(near_sector_idx, env);
    //     near_sectors.push_back(near_sector_idx);
    // }
    // cout << "size~~~" << task_ids.size() << "," << robot_ids.size() << endl;

    // double t1 = 0.0;
    // double t2 = 0.0;
    clock_t start = clock();
    vector<int> assigned_robots;
    double score_cul_t = 0;
    vector<int> task_dis, dis_error, manh_dis_error;
    // double dis_error_sum, manh_dis_error_sum;
    for (int i = 0; i < task_ids.size(); i++)
    {
        double dist = 0;
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
        task_dis.push_back(ceil(0.3 * dist));
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

    int N = min(robot_ids.size(), task_ids.size());
    int M = max(robot_ids.size(), task_ids.size());
    int robot_id, task_id;
    double cur_task_dis;
    KuhnMunkres km(N, M);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            if (robot_ids.size() <= task_ids.size())
            {
                robot_id = robot_ids[i];
                task_id = task_ids[j];
                cur_task_dis = task_dis[j];
            }
            else
            {
                robot_id = robot_ids[j];
                task_id = task_ids[i];
                cur_task_dis = task_dis[i];
            }
            int weight = cul_score(robot_id, task_id, env, cur_task_dis);
            km.setEdge(i, j, weight);
            // cout << "task id " << task_ids[j] << " weight " << weight << ", ";
        }
        // cout << endl;
    }
    // score_cul_t = double(clock() - start) / CLOCKS_PER_SEC;
    // std::cout << "score_cul时间: " << score_cul_t << " 秒" << std::endl;

    km.solve();
    for (int i = 0; i < N; i++)
    {
        if (km.get_result()[i] < 0)
            continue;

        if (robot_ids.size() <= task_ids.size())
        {
            robot_id = robot_ids[i];
            task_id = task_ids[km.get_result()[i]];
        }
        else
        {
            robot_id = robot_ids[km.get_result()[i]];
            task_id = task_ids[i];
        }

        // 若当前robot有正在执行的任务，不分配
        int assigned_task_id = env->curr_task_schedule[robot_id];
        if (assigned_task_id >= 0)
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc > 0)
                continue;
        }

        // // 若当前任务已分配到其他robot（未开始执行），对应robot任务取消
        // int other_robot_id = -1;
        // for (int j = 0; j < env->curr_task_schedule.size(); j++)
        //     if (env->curr_task_schedule[j] == task_id)
        //         other_robot_id = j;
        // if (other_robot_id >= 0 && other_robot_id != robot_id)
        // {
        //     auto it = std::find(assigned_robots.begin(), assigned_robots.end(), other_robot_id);
        //     // 本轮未分配
        //     if (it == assigned_robots.end())
        //         proposed_schedule[other_robot_id] = -1;
        // }

        proposed_schedule[robot_id] = task_id;
        assigned_robots.push_back(robot_id);
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

// void TaskScheduler::get_sector_tasks(int sector_idx, SharedEnvironment *env)
// {
//     for (int i = 0; i < all_task_ids.size(); i++)
//     {
//         if (loc_sector[env->task_pool[all_task_ids[i]].locations[0]] != sector_idx)
//             continue;
//         if (env->task_pool[all_task_ids[i]].agent_assigned < 0)
//             task_ids.push_back(all_task_ids[i]);
//     }
//     for (int i = 0; i < all_task_ids.size(); i++)
//     {
//         if (loc_sector[env->task_pool[all_task_ids[i]].locations[0]] != sector_idx)
//             continue;
//         int assigned_robot_id = env->task_pool[all_task_ids[i]].agent_assigned;
//         int i_loc = env->task_pool[all_task_ids[i]].idx_next_loc;
//         if (assigned_robot_id >= 0 && i_loc <= 0 &&
//             (task_ids.size() < task_size_limit || task_ids.size() < robot_ids.size()))
//         {
//             auto it = std::find(robot_ids.begin(), robot_ids.end(), assigned_robot_id);
//             // 本轮未分配
//             if (it == robot_ids.end())
//             {
//                 task_ids.push_back(all_task_ids[i]);
//                 robot_ids.push_back(assigned_robot_id);
//             }
//         }
//     }
// }
// /*
void TaskScheduler::get_sector_tasks(int sector_idx, SharedEnvironment *env)
{

    int sector_row, sector_col, row_num, col_num;
    if (env->num_of_agents == 5000)
    {
        sector_row = 47;
        sector_col = 50;
        row_num = 3;
        col_num = 10;
    }
    else if (env->num_of_agents == 2000)
    {
        sector_row = 70;
        sector_col = 100;
        row_num = 2;
        col_num = 5;
    }
    else
    {
        sector_row = env->rows;
        sector_col = env->cols;
        row_num = 1;
        col_num = 1;
    }
    int sector_x, sector_y;
    sector_x = sector_idx / col_num;
    sector_y = sector_idx % col_num;
    int inflate_size = 5;
    int lb_x = max(0, sector_x * sector_row) - inflate_size;
    int ub_x = min(env->rows, (sector_x + 1) * sector_row + inflate_size) - 1;
    int lb_y = max(0, sector_y * sector_col) - inflate_size;
    int ub_y = min(env->cols, (sector_y + 1) * sector_col + inflate_size) - 1;

    robot_ids.clear();
    task_ids.clear();
    for (int i = 0; i < env->num_of_agents; i++)
    {
        int loc = env->curr_states.at(i).location;
        int loc_x = loc / env->cols;
        int loc_y = loc % env->cols;
        if (loc_x < lb_x || loc_x > ub_x || loc_y < lb_y || loc_y > ub_y)
            continue;

        int assigned_task_id = env->curr_task_schedule[i];
        if (assigned_task_id < 0)
            robot_ids.push_back(i);
        else if (robot_ids.size() < robot_size_limit)
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc <= 0)
            {
                robot_ids.push_back(i);
                task_ids.push_back(assigned_task_id);
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (robot_ids.size() >= robot_size_limit)
            break;
        int assigned_task_id = env->curr_task_schedule[i];
        if (assigned_task_id < 0)
            continue;
        else
        {
            int i_loc = env->task_pool[assigned_task_id].idx_next_loc;
            if (i_loc >= env->task_pool[assigned_task_id].locations.size() - 1) // 进行最后一个任务时，可分配下个任务
                robot_ids.push_back(i);
        }
    }
    for (int i = 0; i < all_task_ids.size(); i++)
    {
        if (env->task_pool[all_task_ids[i]].agent_assigned < 0)
            task_ids.push_back(all_task_ids[i]);
    }
    for (int i = 0; i < all_task_ids.size(); i++)
    {
        int assigned_robot_id = env->task_pool[all_task_ids[i]].agent_assigned;
        int i_loc = env->task_pool[all_task_ids[i]].idx_next_loc;
        if (assigned_robot_id >= 0 && i_loc <= 0 &&
            (task_ids.size() < task_size_limit || task_ids.size() < robot_ids.size()))
        {
            auto it = std::find(robot_ids.begin(), robot_ids.end(), assigned_robot_id);
            // 本轮未分配
            if (it == robot_ids.end())
            {
                task_ids.push_back(all_task_ids[i]);
                robot_ids.push_back(assigned_robot_id);
            }
        }
    }
}
// */