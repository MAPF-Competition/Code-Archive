#pragma once
#include "ID.h"
#include "Tasks.h"
#include "SharedEnv.h"
#include "SortingSystem.h"
#include <unordered_set>
#include <boost/program_options.hpp>


class TaskScheduler
{
private:

    bool delete_left = true; // 如果是true就删掉左边的邻居, 如果是false就删掉右边的邻居

    struct AgentTask
    {
        int task_id = -1;
        int min_task_dist = -1;
        double jam_when_assign = -1; // 当该任务分配时, agent前往该任务的拥堵系数
        double task_heuristic = -1; // 启发值必然是double, 因为即使sum jam weight是int, 乘以系数后还是要变成double
        int assign_moment = -1;
        int complete_moment = -1;
    };
    vector<AgentTask> agent_task;

    int numTaskFinished = 0;
    struct FinishedTask
    {
        int task_id = -1; // 完成的任务id
        int min_task_dist = -1;
        double jam_when_assign = -1; // 当该任务分配时, agent前往该任务的拥堵系数
        double heuristic_duration = -1;
        int real_duration = -1;
    };
    vector<FinishedTask> finished_tasks;
    int total_min_span = 0; // 已完成任务的理论完成时间下界之和
    int total_real_duration = 0; // 已完成任务的实际完成时间之和
    double total_jam = 0;

    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;

    std::unordered_map<int, int> task_region; // 记录每个task所属的区域

    int prev_best_total_distance = INT_MAX; // 上一轮分配的最小距离
    std::unordered_map<int, double> task_pickup_jams; // <task_id, jam>每个任务起点附近道路的拥堵情况
    std::vector<int> num_nearby_obstacles; // 上下左右格子中障碍物的数量
    double jam_coefficient = 1; // 在多大程度上考虑拥堵
    std::unordered_map<int, int> task_distances;
    int max_agent_allowed = 128; // 规划时间所允许的, 匈牙利算法的最大agent边界
    int max_task_allowed = 128; // 规划时间所允许的, 匈牙利算法的最大任务边界

    int radius_count_agent = 0; // 统计半径范围内agent的数量, 用于计算jam
    int first_epoch_done_time = -1; // 如果初始阶段的任务较多, 先等它们分配完成. 本变量记录初始分配结束时间。

    // RHCR members
    SortingGrid sorting_grid;
    boost::program_options::variables_map vm;
    SortingSystem* sorting_system;

public:
    SharedEnvironment* env;

    TaskScheduler(SharedEnvironment* env): env(env){};
    TaskScheduler(){env = new SharedEnvironment();};
    virtual ~TaskScheduler(){delete env;};
    virtual void initialize(int preprocess_time_limit);
    void rhcr_initialize(); // 带旋转启发式的初始化
    virtual void plan(int time_limit, std::vector<int> & proposed_schedule);

    // 预先计算地图上任意两点之间的距离
    void compute_map_point_dist(TimePoint _endtime) const;

    // 预先计算地图上任意两点之间的距离, 采用对称性加快计算速度
    void compute_map_point_dist_symmetry(TimePoint _endtime) const;

    // only compute distance between agent and the first target
    void greedy_only_first(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，从默认代码中复制到TaskScheduler.cpp。
    void greedy_sum(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，用于其他分配算法处理首批任务
    void greedy_sum_without_newtask(int time_limit, std::vector<int> & proposed_schedule);

    // compute total distance of task before assignment
    void greedy_sum_at_once(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给靠近地图角落的agent分配任务, 因为地图边缘不容易堵车。
    void greedy_sum_suburb_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给靠近地图中心的agent分配任务, 因为地图中心更容易堵车。
    void greedy_sum_urban_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给车流稀疏的agent分配任务, 因为车流稀疏区域不容易堵车。
    void greedy_sum_sparse_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给车流密集的agent分配任务, 因为车流密集区域更容易堵车。
    void greedy_sum_dense_first(int time_limit, std::vector<int> & proposed_schedule);

    // 匈牙利分配算法，cost只计算agent和任务pickup点的位置。
    void hungarian_only_first(int time_limit, std::vector<int> & proposed_schedule);

    // 匈牙利分配算法，cost只计算agent和任务pickup点的位置；加入了抢单机制，也就是把被指派任务但还没有取货的agent也考虑在内。
    void hungarian_pickup_snatch(int time_limit, std::vector<int> & proposed_schedule);

    // 匈牙利分配算法, cost使用默认计算方法。
    void hungarian_sum(int time_limit, std::vector<int> & proposed_schedule);

    // 匈牙利分配算法，一次性把任务长度计算完储存在unordered_map。
    void hungarian_sum_at_once(int time_limit, std::vector<int> & proposed_schedule);

    // hungarian schedule sum at once and snatch order
    void hungarian_sum_snatch(int time_limit, std::vector<int> & proposed_schedule);

    //  在hungarian_sum_snatch的基础上，考虑了单据不够、单据超出等多种情况。尚且不稳定，official platform测试会失败。
    void hungarian_sum_snatch_complex(int time_limit, std::vector<int> & proposed_schedule);

    // 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数之和决定。
    void pickup_jam_based_current(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以agent为圆心，|agent-task|为半径的半圆
    void adaptive_jam_pickup_current_circle(int time_limit, std::vector<int> & proposed_schedule);
    // adaptive_pickup_jam_current_vector配套的计算jam函数
    [[nodiscard]] double compute_adaptive_jam_pickup_current_circle(int _agent_id, int _agent_loc, int _pickup_loc) const;

    // 检索以task为圆心，所有agent距离它的倒数之和
    void adaptive_jam_task_circle_current(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以task为圆心，所有agent距离它的倒数平方之和
    void adaptive_jam_task_circle_current_square(int time_limit, std::vector<int> & proposed_schedule);

    // 1: 统计以task为圆心，|agent-task|为半径的圆中other agent的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_current(int _agent_id, int _agent_loc_x,
                                                            int _agent_loc_y, int _pickup_loc) const;
    void adaptive_jam_task_circle_count_current(int time_limit, std::vector<int> & proposed_schedule);
    // 1.1预分配初始任务
    void adaptive_jam_task_circle_count_current_preassign(int time_limit, std::vector<int> & proposed_schedule);
    void adaptive_jam_task_circle_count_current_compare_dist(int time_limit,
                                                             std::vector<int> & proposed_schedule);
    void hungarian_sum_snatch_adaptive_jam_task_circle_count_current(int time_limit,
                                                             std::vector<int> & proposed_schedule);

    // 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量（只考察前2048个agent）为拥堵系数。
    void adaptive_jam_task_circle_count_current_sample(int time_limit, std::vector<int> & proposed_schedule,
                                                       int _num_sample);

    // 统计以task为圆心，|agent-task|为半径的圆中other agent的数量作为jam，并把这个系数外推出去
    void adaptive_jam_task_circle_count_current_extrapolation(int time_limit, std::vector<int> & proposed_schedule);

    // 1.2: 将地图分成16x16的区域, task所在区域中agent的数量作为sum jam weight
    void adaptive_jam_task_region_count_current(int time_limit, std::vector<int> & proposed_schedule);

    // 2: 统计以task为圆心，|agent-task|为半径的圆中有任务的other agent的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_current_busy(int _agent_id, int _agent_loc_x,
                        int _agent_loc_y, int _pickup_loc, std::vector<int> & proposed_schedule) const;
    void adaptive_jam_task_circle_count_current_busy(int time_limit, std::vector<int> & proposed_schedule);

    // 统计以agent-task中点为圆心，|agent-task|/2为半径的圆中other agent的数量作为jam
    void adaptive_jam_middle_circle_count_current(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以task为圆心，|agent-task|为半径的圆
    void adaptive_jam_task_circle_vector_current(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以task为圆心，|agent-task|为半径的圆中有任务的agent向量相乘
    void adaptive_jam_task_circle_vector_current_busy(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以task为圆心，|agent-task|为半径的圆; 外推到整个任务执行长度
    void adaptive_jam_task_circle_vector_current_extrapolation(int time_limit, std::vector<int> & proposed_schedule);

    // 检索以task为圆心，|agent-task|为半径的圆, 有|agent-task|欧几里得距离作为分母
    void adaptive_jam_task_circle_vector_current_complex(int time_limit, std::vector<int> & proposed_schedule);

    // 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent目标距离该任务的倒数之和决定。
    void pickup_jam_based_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 3: 统计以task为圆心，|agent-task|为半径的圆中other agent goal的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                      int _pickup_loc) const;
    void adaptive_jam_task_circle_count_goal(int time_limit, std::vector<int> & proposed_schedule);
    void adaptive_jam_task_circle_count_goal_compare_dist(int time_limit,
                                                          std::vector<int> & proposed_schedule);
    void hungarian_sum_snatch_adaptive_jam_task_circle_count_goal(int time_limit,
                                                                 std::vector<int> & proposed_schedule);

    // 4: 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
    [[nodiscard]] double compute_jam_task_circle_vector_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                      int _pickup_loc) const;
    void adaptive_jam_task_circle_vector_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 5: 统计以task为圆心，|agent-task|为半径的圆中other agent current location 和 goal的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_both_current_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                        int _pickup_loc) const;
    void adaptive_jam_task_circle_count_both_current_goal(int time_limit,
                                                          std::vector<int> & proposed_schedule);
    void adaptive_jam_task_circle_count_both_current_goal_compare_dist(int time_limit,
                                                          std::vector<int> & proposed_schedule);
    // hungarian_sum_snatch计算cost matrix加上adaptive_jam_task_circle_count_both_current_goal
    void hungarian_sum_snatch_adaptive_jam_task_circle_count_both_current_goal(int time_limit,
                                                                  std::vector<int> & proposed_schedule);

    // 6: 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_circle_count_middle_current_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                        int _pickup_loc) const;
    void adaptive_jam_task_circle_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);
    void adaptive_jam_task_circle_count_middle_current_goal_compare_dist(int time_limit, std::vector<int> & proposed_schedule);

    // hungarian_sum_snatch计算cost matrix加上adaptive_jam_middle_current_goal_task_circle_count
    void hungarian_sum_snatch_adaptive_jam_task_circle_count_middle_current_goal(int time_limit,
                                                             std::vector<int> & proposed_schedule);

    // 7: 统计以task为圆心，|agent-task|为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_Manhattan_circle_count_middle_current_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                        int _pickup_loc) const;
    void adaptive_jam_task_Manhattan_circle_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);
    // 使用RHCR启发式
    void adaptive_jam_task_Manhattan_circle_count_middle_current_goal_rhcr(int time_limit, std::vector<int> & proposed_schedule);

    // 8: 统计以task为圆心，|num_rows + num_columns| / 8为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_fix_Manhattan_circle_count_middle_current_goal(int _agent_id, int _pickup_loc) const;
    void adaptive_jam_task_fix_Manhattan_circle_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 9: 将地图分成16x16的区域, task所在区域中agent当前位置和目标点位置的中点的数量作为作为sum jam weight
    void adaptive_jam_task_region_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 10: 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent当前位置和目标点位置的中点的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
    void adaptive_jam_task_circle_vector_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    struct Point {
        int x, y;
    };

    int cross(const Point& startA, const Point& endA, const Point& pointB) {
        return (endA.x - startA.x) * (pointB.y - startA.y) - (endA.y - startA.y) * (pointB.x - startA.x);
    }

    bool isIntersecting(const Point& startA, const Point& endA, const Point& startB, const Point& endB) {
        // Bounding box filter
        if (std::max(startA.x, endA.x) < std::min(startB.x, endB.x) ||
            std::max(startA.y, endA.y) < std::min(startB.y, endB.y) ||
            std::max(startB.x, endB.x) < std::min(startA.x, endA.x) ||
            std::max(startB.y, endB.y) < std::min(startA.y, endA.y)) {
            return false;
        }

        // Step 1: Check general intersection condition
        int cross1 = cross(startA, endA, startB);
        int cross2 = cross(startA, endA, endB);
        int cross3 = cross(startB, endB, startA);
        int cross4 = cross(startB, endB, endA);

        if (cross1 * cross2 < 0 && cross3 * cross4 < 0)
            return true;

        // Step 2: Check collinear overlap
        auto isBetween = [](int a, int b, int c) {
            return std::min(a, b) <= c && c <= std::max(a, b);
        };

        if (cross1 == 0 && isBetween(startA.x, endA.x, startB.x) && isBetween(startA.y, endA.y, startB.y)) return true;
        if (cross2 == 0 && isBetween(startA.x, endA.x, endB.x) && isBetween(startA.y, endA.y, endB.y)) return true;
        if (cross3 == 0 && isBetween(startB.x, endB.x, startA.x) && isBetween(startB.y, endB.y, startA.y)) return true;
        if (cross4 == 0 && isBetween(startB.x, endB.x, endA.x) && isBetween(startB.y, endB.y, endA.y)) return true;

        return false;
    }

    // 11: compute pickup jam by counting whether other agent-task line intersect with this agent-task line
    [[nodiscard]] int compute_jam_curr_pickup_intersect_curr_goal(int _agent_id, Point _agent_loc,
                                                                      Point _agent_end);
    void adaptive_jam_curr_pickup_intersect_curr_goal(int time_limit, std::vector<int> & proposed_schedule);
    void adaptive_jam_curr_pickup_delivery_intersect_curr_goal(int time_limit,
                                                               std::vector<int> & proposed_schedule);

    // RHCR functions
    MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm);
    void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm);
};