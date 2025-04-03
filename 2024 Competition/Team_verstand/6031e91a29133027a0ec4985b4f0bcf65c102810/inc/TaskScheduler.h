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
    vector<vector<int>> grid; // 地图的二维表示, 可用来寻找长直道   


    bool delete_left = true; // 如果是true就删掉左边的邻居, 如果是false就删掉右边的邻居

    struct AgentTask
    {
        int task_id = -1;
        int min_task_dist = -1;
        double jam_when_assign = -1; // 当该任务分配时, agent前往该任务的拥堵系数
        double task_heuristic = -1; // 启发值必然是double, 因为即使sum jam weight是int, 乘以系数后还是要变成double
        int dist_agent_pickup = -1; // 从agent当前位置到pickup点的启发式距离
        int dist_pickup_delivery = -1; // task pickup点到delivery点的启发式距离
        int assign_moment = -1;
        int complete_moment = -1;
    };
    vector<AgentTask> agent_task;
    vector<int> task_wait; // 每个task等待的时间
    vector<std::pair<int, int>> extra_add; // <source, target>。

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
    int min_delay = 1000; // 最小延迟, 用来估算停止的边界

    vector<int> x; // jam
    vector<int> y; // real_duration - min_task_dist

    int total_dist_agent_pickup = 0; // 已完成任务的agent当前位置到pickup点的启发式距离之和
    int total_dist_pickup_delivery = 0; // 已完成任务的pickup点到delivery点的启发式距离之和
    double average_dist_agent_pickup = 0; // 已完成任务的agent当前位置到pickup点的启发式距离平均数
    double average_dist_pickup_delivery = 0; // 已完成任务的pickup点到delivery点的启发式距离平均数

    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;

    std::unordered_map<int, int> task_region; // 记录每个task所属的历史区域
    std::unordered_map<int, int> task_current_region; // 记录每个task所属的当前区域
    std::unordered_map<int, int> pickup_region; // 记录每个task pickup所属的当前区域
    std::unordered_map<int, int> delivery_region; // 记录每个task delivery所属的当前区域
    std::unordered_map<int, int> delivery_near_edge; // 记录每个task delivery是否靠近边缘
    std::unordered_map<int, vector<int>> task_errand_region; // 记录每个task errand所属的区域序列
    std::unordered_map<int, vector<int>> task_errand_crossed_region; // 记录每个task errand连线穿过的区域序列
    std::unordered_map<int, vector<int>> task_middle_region; // 记录task相邻errand点的中点所属的区域序列
    vector<State> prev_states; // 上一轮各个agent的位置

    int small_region_column = 12; // (errand点)(小)每个region所占的列数
    int small_region_row = 12; // (errand点)(小)每个region所占的行数
    // (历史统计)地图有几列small region
    int num_small_region_column;
    // (历史统计)地图有几行small region
    int num_small_region_row;
    vector<int> small_region_wait_num; // 记录每个区域历史等待点的数量
    vector<int> small_region_cell_num; // 记录每个区域地图点的数量
    vector<int> small_region_blank_num; // (历史统计)记录每个区域空点的数量
    vector<int> small_region_obstacle_num; // 记录每个区域空点的数量

    int large_region_column = 12; // (middle of pickup and delivery)(大)每个region所占的列数
    int large_region_row = 12; // (middle of pickup and delivery)(大)每个region所占的行数
    // (当下时刻)地图有几列large region
    int num_large_region_column;
    // (当下时刻)地图有几行large region
    int num_large_region_row;
    vector<int> large_region_wait_num; // 记录每个区域历史等待点的数量
    vector<int> large_region_blank_num; // (当下时刻)每个区域空点的数量

    // 定长队列
    class FixedQueue {
    private:
        std::deque<std::vector<int>> data; // 每个元素是一个长度为5的vector<int>
        size_t max_size;

    public:

        void set_max_size(int _max_size)
        {
            max_size = _max_size;
        }

        int get_size()
        {
            return data.size();
        }

        std::vector<int> push(const std::vector<int>& value) {
            std::vector<int> removed; // 用于记录被移除的值
            if (data.size() == max_size) {
                removed = data.front(); // 记录被移除的值
                data.pop_front();       // 从头部移除旧元素
            }
            data.push_back(value); // 从尾部添加新元素
            return removed;        // 返回被移除的值，若没有则返回空vector
        }

        void print() const {
            for (const auto& vec : data) {
                std::cout << "[ ";
                for (int val : vec) {
                    std::cout << val << " ";
                }
                std::cout << "] ";
            }
            std::cout << std::endl;
        }
    };

    FixedQueue period_small_region_wait_num; // 储存若干个时间步内的small_region_wait_num信息
    FixedQueue period_large_region_wait_num; // 储存若干个时间步内的large_region_wait_num信息

    int prev_best_total_distance = INT_MAX; // 上一轮分配的最小距离
    std::unordered_map<int, double> task_pickup_jams; // <task_id, jam>每个任务起点附近道路的拥堵情况
    double jam_coefficient = 1; // 在多大程度上考虑拥堵
    double fit_a = 1; // y = a*x + b中拟合出的a
    double fit_b = 0; // y = a*x + b中拟合出的b

    std::unordered_map<int, int> task_distances;
    vector<int> task_lengths; // 用数组计算快一些
    int max_agent_allowed = 128; // 规划时间所允许的, 匈牙利算法的最大agent边界
    int max_task_allowed = 128; // 规划时间所允许的, 匈牙利算法的最大任务边界

    int radius_count_agent = 0; // 统计半径范围内agent的数量, 用于计算jam
    int first_epoch_done_time = -1; // 如果初始阶段的任务较多, 先等它们分配完成. 本变量记录初始分配结束时间。

    bool initial_task_separator = false;

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

    // 寻找grid一行中最大连续0的个数
    static std::tuple<int, int, int> find_longest_zeros(const std::vector<int>& vec);

    std::vector<std::vector<int>> find_segments(const std::vector<int>& vec) {
    std::vector<std::vector<int>> segments;
    int n = vec.size();
    
    // 遍历整个vector
    for (int i = 0; i < n; ++i) {
        // 如果当前元素是4，尝试找到一个符合条件的片段
        if (vec[i] == 4) {
            for (int j = i + 1; j < n; ++j) {
                // 找到以4结束的片段
                if (vec[j] == 4) {
                    // 检查这个片段是否包含1或2
                    bool valid = true;
                    for (int k = i + 1; k < j; ++k) {
                        if (vec[k] == 1 || vec[k] == 2) {
                            valid = false;
                            break;
                        }
                    }
                    // 如果有效，加入结果
                    if (valid) {
                        std::vector<int> segment(vec.begin() + i, vec.begin() + j + 1);
                        segments.push_back(segment);
                    }
                    break; // 找到一个片段后继续查找下一个4
                }
            }
        }
    }
    
    return segments;
}

    void build_row_highway_delete(TimePoint _endtime);

    void build_row_highway_add(TimePoint _endtime);

    void build_row_highway_compute(TimePoint _endtime); // 计算所有对

    void build_column_highway_delete(TimePoint _endtime);

    void build_column_highway_add(TimePoint _endtime);

    void assign_extra_add();

    void rhcr_initialize(); // 带旋转启发式的初始化
    virtual void plan(int time_limit, std::vector<int> & proposed_schedule);

    // 预先计算地图上任意两点之间的距离
    void compute_map_point_dist(TimePoint _endtime) const;

    // 预先逆序计算地图上任意两点之间的距离
    void compute_map_point_dist_reverse(TimePoint _endtime) const;

    // 预先计算地图上任意两点之间的距离, 采用对称性加快计算速度
    void compute_map_point_dist_symmetry(TimePoint _endtime) const;

    // only compute distance between agent and the first target
    void greedy_only_first(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，从默认代码中复制到TaskScheduler.cpp。
    void greedy_sum(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，用于其他分配算法处理首批任务
    void greedy_sum_without_newtask(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，用于其他分配算法处理首批任务
    void greedy_sum_at_once_without_newtask(int time_limit, std::vector<int>& proposed_schedule);

    // 默认分配算法，从默认代码中复制到TaskScheduler.cpp。
    void greedy_sum_vector(int time_limit, std::vector<int>& proposed_schedule);

    // compute total distance of task before assignment
    void greedy_sum_at_once(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给靠近地图角落的agent分配任务, 因为地图边缘不容易堵车。
    void greedy_sum_suburb_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给靠近地图中心的agent分配任务, 因为地图中心更容易堵车。
    void greedy_sum_urban_first(int time_limit, std::vector<int> & proposed_schedule);

    // 结合greedy_sum_urban_first和adaptive_jam_task_pickup_region_count_current
    void urban_first_adaptive_jam_task_pickup_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 完不成的就不接了
    void urban_first_adaptive_jam_task_pickup_region_count_current_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // 优先给区域车流稀疏的agent分配任务, 因为车流稀疏区域不容易堵车。
    void greedy_sum_sparse_region_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给区域等待点少的agent分配任务, 因为区域等待点少更不容易堵车。
    void greedy_sum_sparse_wait_region_first(int time_limit, std::vector<int>& proposed_schedule);

    // 优先给历史区域等待点少的agent分配任务, 因为历史区域等待点少不容易堵车。
    void greedy_sum_all_time_sparse_wait_region_first(int time_limit, std::vector<int>& proposed_schedule);

    // 优先给区域车流密集的agent分配任务, 因为车流密集区域更容易堵车。
    void greedy_sum_dense_region_first(int time_limit, std::vector<int> & proposed_schedule);

    // 优先给区域等待点多的agent分配任务, 因为区域等待点多更容易堵车。
    void greedy_sum_dense_wait_region_first(int time_limit, std::vector<int>& proposed_schedule);

    // 优先给历史区域等待点多的agent分配任务, 因为历史区域等待点多容易堵车。
    void greedy_sum_all_time_dense_wait_region_first(int time_limit, std::vector<int>& proposed_schedule);

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

    // 来不及完成的任务就不接了
    void hungarian_sum_snatch_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // combine hungarian schedule sum at once and snatch order and pickup jam based current
    void hungarian_sum_snatch_and_pickup_jam_based_current(int time_limit, std::vector<int>& proposed_schedule);

    //  在hungarian_sum_snatch的基础上，考虑了单据不够、单据超出等多种情况。尚且不稳定，official platform测试会失败。
    void hungarian_sum_snatch_complex(int time_limit, std::vector<int> & proposed_schedule);

    // 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数之和决定。
    void pickup_jam_based_current(int time_limit, std::vector<int> & proposed_schedule);

    // 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数之和决定。
    void pickup_jam_based_current_without_newtask(int time_limit, std::vector<int>& proposed_schedule);

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

    void adaptive_jam_task_circle_count_current_without_newtask(int time_limit, std::vector<int>& proposed_schedule);

    // 1.1预分配初始任务
    void adaptive_jam_task_circle_count_current_preassign(int time_limit, std::vector<int> & proposed_schedule);
    // 1.2 加入compare dist加快判断速度
    void adaptive_jam_task_circle_count_current_compare_dist(int time_limit,
                                                             std::vector<int> & proposed_schedule);
    void hungarian_sum_snatch_adaptive_jam_task_circle_count_current(int time_limit,
                                                             std::vector<int> & proposed_schedule);

    // 1.1.0: 统计以task为圆心，|agent-task|/2为半径的圆中other agent的数量作为jam
    [[nodiscard]] int compute_jam_task_half_circle_count_current(int _agent_id, int _agent_loc_x,
        int _agent_loc_y, int _pickup_loc) const;
    void adaptive_jam_task_half_circle_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.1.1: 统计以task为圆心，|agent-task|*2为半径的圆中other agent的数量作为jam
    [[nodiscard]] int compute_jam_task_double_circle_count_current(int _agent_id, int _agent_loc_x,
        int _agent_loc_y, int _pickup_loc) const;
    void adaptive_jam_task_double_circle_count_current(int time_limit, std::vector<int>& proposed_schedule);

    void hungarian_sum_snatch_adaptive_jam_task_double_circle_count_current(int time_limit,
        std::vector<int>& proposed_schedule); // 1.1.1.1

    // 1.1.2: 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量（只考察前2048个agent）为拥堵系数。
    void adaptive_jam_task_circle_count_current_sample(int time_limit, std::vector<int> & proposed_schedule,
                                                       int _num_sample);

    // 1.1.3: 统计以task为圆心，|agent-task|为半径的圆中other agent的数量作为jam，并把这个系数外推出去
    void adaptive_jam_task_circle_count_current_extrapolation(int time_limit, std::vector<int> & proposed_schedule);

    // 1.1.4: 统计以pickup为圆心，|pickup-delivery|为半径的圆中other agent的数量作为jam
    [[nodiscard]] int compute_jam_pickup_delivery_circle_count_current(int _agent_id, int _pickup_loc_x, int _pickup_loc_y,
        int _dist_pickup_delivery_square) const;
    void adaptive_jam_pickup_delivery_circle_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.1.5: 统计以pickup为圆心，|agent-pickup|和|pickup-delivery|为半径的圆中other agent的数量作为jam
    [[nodiscard]] int compute_jam_both_circle_count_current(int _agent_id, int _pickup_loc_x, int _pickup_loc_y,
        int _dist_pickup_delivery_square) const;
    void adaptive_jam_both_circle_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2: 将地图分成12x12的区域, task所在区域中agent的数量作为sum jam weight
    void adaptive_jam_task_pickup_region_count_current(int time_limit, std::vector<int> & proposed_schedule);

    // 1.2.-3: 将地图分成12x12的区域, task所在区域中goal的数量作为sum jam weight
    void adaptive_jam_task_pickup_region_count_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.-2: 将地图分成12x12的区域, task所在区域中agent的数量作为sum jam weight. 用agent区域数量修正估计
    void adaptive_jam_agent_pickup_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.-1: 将地图分成大小可变的方块区域, 边长为|agent-pickup|均值, pickup所在区域中agent的数量作为sum jam weight
    void adaptive_jam_adaptive_pickup_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.0: 将地图分成8x8的区域, task pickup区域agent数量作为sum jam weight, 叠加匈牙利抢单算法
    void hungarian_sum_snatch_adaptive_jam_task_pickup_region_count_current(int time_limit,
        std::vector<int>& proposed_schedule);

    // 1.2.1: 将地图分成12x12的区域, task所在区域agent密度作为sum jam weight
    void adaptive_jam_task_region_current_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.1.1: 将地图分成8x8的区域, task pickup区域agent密度作为sum jam weight, 叠加匈牙利抢单算法
    void hungarian_sum_snatch_adaptive_jam_task_pickup_region_current_density(int time_limit,
        std::vector<int>& proposed_schedule);

    // 1.2.2: 将地图分成12x12的区域, task所在区域agent密度 x |agent-pickup|作为sum jam weight
    void adaptive_jam_task_region_current_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.2.-1: 来不及做的任务就不算了
    void adaptive_jam_task_region_current_density_times_agent_pickup_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.2.-1.1: 来不及做的任务就不算了+最小延迟
    void adaptive_jam_task_region_current_density_times_agent_pickup_not_get_delay(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.2.0: 一次性算完
    void adaptive_jam_task_region_current_density_times_agent_pickup_at_once(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.2.1: 将地图分成8x8的区域, task pickup区域agent密度x |agent pickup|作为sum jam weight, 叠加匈牙利抢单算法
    void hungarian_sum_snatch_adaptive_jam_task_pickup_region_current_density_times_agent_pickup(int time_limit,
        std::vector<int>& proposed_schedule);

    // 1.2.2.2: 将地图分成12x12的区域, task所在区域agent密度 x agent-pickup-delivery距离作为sum jam weight
    void adaptive_jam_task_region_current_density_times_agent_pickup_delivery(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.3: 将地图分成12x12的区域, task所在区域时间平均agent密度作为sum jam weight
    void adaptive_jam_task_region_all_time_average_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4: 将地图分成12x12的区域, task所在区域时间窗平均agent密度作为sum jam weight
    void adaptive_jam_task_region_time_window_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.1: 将地图分成12x12的区域, task所在区域时间窗平均agent密度 x |agent-pickup|作为sum jam weight
    void adaptive_jam_task_region_time_window_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.2.0: 将地图进行12x12和32x32两次划分, task所在两次划分的agent数量加权和作为sum jam weight
    void adaptive_jam_pickup_region_large_small_partition_count(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.2: 将地图分成12x12的区域, task所在区域长短时平均agent数量之和作为sum jam weight
    void adaptive_jam_task_region_long_short_term_count(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.3: 将地图分成12x12的区域, task所在区域长短时平均agent密度作为sum jam weight
    void adaptive_jam_task_region_long_short_term_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.4: 将地图分成12x12的区域, task所在区域长短时平均agent密度 x |agent-pickup|作为sum jam weight
    void adaptive_jam_task_region_long_short_term_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.4.5: 将地图分成12x12的区域, task所在区域历史大面积现在小面积平均agent密度作为sum jam weight
    void adaptive_jam_pickup_region_long_large_short_small_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5: 将地图分成12x12的区域, agent-pickup经过区域中agent的数量之和作为sum jam weight
    void adaptive_jam_agent_pickup_crossed_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5.1: 将地图分成12x12的区域, agent-pickup经过区域中agent的平均密度作为sum jam weight
    // 显然, 只有crossed_region才需要计算平均密度。单一region是不需要的。
    void adaptive_jam_agent_pickup_crossed_region_current_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5.2: 将地图分成12x12的区域, agent-pickup经过区域中agent的平均密度 x |agent pickup|作为sum jam weight
    void adaptive_jam_agent_pickup_crossed_region_current_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5.2.1: 将地图分成12x12的区域, agent-pickup经过区域中agent的加权平均密度 x |agent pickup|作为sum jam weight
    void adaptive_jam_agent_pickup_crossed_region_current_weighted_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5.3: 将地图分成12x12的区域, agent-pickup经过区域中agent的平均密度 x |agent pickup delivery|作为sum jam weight
    void adaptive_jam_agent_pickup_crossed_region_current_density_times_agent_pickup_delivery(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.5.4: 将地图分成12x12的区域, agent-pickup经过区域中agent的平均密度作为sum jam weight
    // 显然, 只有crossed_region才需要计算平均密度。单一region是不需要的。
    // void adaptive_jam_agent_pickup_crossed_region_current_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.6: 将地图分成12x12的区域, pickup-delivery经过区域中agent的数量之和作为sum jam weight
    void adaptive_jam_pickup_delivery_crossed_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.6.1: 将地图分成12x12的区域, pickup-delivery经过区域中agent goal的数量之和作为sum jam weight
    void adaptive_jam_pickup_delivery_crossed_region_count_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.7: 将地图分成12x12的区域, agent-pickup-delivery经过区域中agent的数量之和作为sum jam weight
    void adaptive_jam_agent_pickup_delivery_crossed_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.8: 将地图分成12x12的区域, agent-pickup-delivery经过区域中agent的平均密度作为sum jam weight
    void adaptive_jam_agent_pickup_delivery_crossed_region_current_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.9: 将地图分成12x12的区域, agent-pickup-delivery经过区域中agent的加权平均密度作为sum jam weight
    void adaptive_jam_agent_pickup_delivery_crossed_region_current_weighted_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.2.10: 将地图分成12x12的区域, agent-pickup-delivery经过区域中agent的加权平均密度 x |agent task|作为sum jam weight
    void adaptive_jam_agent_pickup_delivery_crossed_region_current_weighted_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 1.3: 将地图分成12x12的区域, task pickup区域中时间平均agent等待点的数量作为sum jam weight
    void adaptive_jam_task_pickup_region_count_average_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.3.1: 将地图分成12x12的区域, task pickup区域中时间平均agent等待点的密度作为sum jam weight
    void adaptive_jam_task_pickup_region_average_wait_density(int time_limit, std::vector<int>& proposed_schedule);

    // 1.3.2: 将地图分成12x12的区域, task pickup区域中时间窗agent等待点的数量作为sum jam weight
    void adaptive_jam_task_pickup_region_count_time_window_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.4: 将地图分成8x8的区域, task delivery区域中agent等待点的数量作为sum jam weight
    void adaptive_jam_task_delivery_region_count_average_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.4.1: 将地图分成12x12的区域, task delivery区域中时间窗agent等待点的数量作为sum jam weight
    void adaptive_jam_task_delivery_region_count_time_window_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5: 将地图分成12x12的区域, task errand区域中agent等待点的数量作为sum jam weight
    void adaptive_jam_task_errand_region_count_average_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.1: 将地图分成12x12的区域, task errand区域时间窗agent等待点的数量作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_task_errand_region_count_time_window_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.2: 将地图分成12x12的区域, task errand区域中agent的数量作为sum jam weight
    void adaptive_jam_task_errand_region_count_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.3: 将地图进行12x12和16x16两次划分, 统计pickup区域小方块中agent数量与delivery区域大方块中agent数量的加权和
    void adaptive_jam_count_near_small_current_far_large_current(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.4.-3: 将地图分成方形区域, task pickup区域统计当前agent数量, delivery区域统计agent goal数量, 两者和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_pickup_current_delivery_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.4.-2: 将地图分成方形区域, task pickup区域统计当前agent数量, delivery区域统计agent goal数量, 两者加权和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_weighted_pickup_current_delivery_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.4.-1: 将地图分成方形区域, task pickup区域统计当前agent数量, delivery区域统计时间窗agent等待点数量, 两者和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_pickup_current_delivery_time_window_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.4: 将地图分成方形区域, task pickup区域统计当前agent数量, 后面的delivery区域统计时间窗agent等待点数量, 两者和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_near_current_far_time_window_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.5: 将地图分成方形区域, task pickup区域统计当前agent数量, delivery区域统计时间窗平均agent等待点数量, 两者加权和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_pickup_current_delivery_time_window_average_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.6: 将地图分成方形区域, task pickup区域统计当前agent数量 减去 delivery区域统计时间窗平均agent等待点数量, 作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_pickup_current_minus_delivery_time_window_average_wait(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.7: 将地图分成方形区域, task pickup区域统计当前agent数量 减去 delivery区域统计时间窗平均agent等待点数量, 作为sum jam weight
    // 等待权重的系数是恒定的
    void adaptive_jam_count_pickup_current_minus_delivery_time_window_average_wait_constant(int time_limit, std::vector<int>& proposed_schedule);

    // 1.5.8: 将地图分成方形区域, task pickup区域统计当前agent数量, delivery区域统计时间窗平均agent等待点数量, middle区域统计时间窗平均agent等待点数量, 三者加权和作为sum jam weight
    // 三个步骤。(1) 统计每个区域agent的数量。(2) 计算每个task归属于哪个区域。(3) 根据task所属区域估计执行成本。
    void adaptive_jam_count_pickup_current_delivery_time_window_average_wait_middle_large(int time_limit, std::vector<int>& proposed_schedule);


    struct Point {
        int x, y;
    };

    // Bresenham's Line Algorithm
    vector<Point> get_line_crossed_cell(int _start, int _goal) 
    {
        int x0 = _start % env->cols;
        int y0 = _start / env->cols;
        int x1 = _goal % env->cols;
        int y1 = _goal / env->cols;

        vector<Point> points;
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            points.push_back({ x0, y0 });
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
        return points;
    }

    // 按顺序存储经过的 region id
    vector<int> get_line_crossed_regions(const vector<Point>& linePoints) 
    {
        vector<int> regions;                  // 用于按顺序存储经过的 region id
        std::unordered_set<int> visited;     // 用于避免重复插入
        for (const auto& point : linePoints) 
        {
            int cell_region_x = point.x / small_region_column;
            int cell_region_y = point.y / small_region_row;
            int region_id = cell_region_y * num_small_region_column + cell_region_x;

            // 如果这个 region 尚未访问过，则添加到结果中
            if (visited.find(region_id) == visited.end()) 
            {
                regions.push_back(region_id);
                visited.insert(region_id);
            }
        }

        return regions; // 按照经过的顺序输出
    }

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

    // 3.0: 使用adaptive_jam_task_circle_count_current预分配
    void adaptive_jam_task_circle_count_goal_preassign(int time_limit, std::vector<int>& proposed_schedule);

    // 3.1: 统计以task为圆心，|agent-task|为半径的圆中other agent next goal的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_next_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc) const;
    void adaptive_jam_task_circle_count_next_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 3.2: 统计以task为圆心，|agent-task|为半径的圆中other agent goals的数量作为jam
    [[nodiscard]] int compute_jam_task_circle_count_all_goals(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc) const;
    void adaptive_jam_task_circle_count_all_goals(int time_limit, std::vector<int>& proposed_schedule);

    // 3.2.-2: 完不成的任务就不接了
    void adaptive_jam_task_circle_count_all_goals_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // 3.2.-1: 规避末段靠近边缘的任务
    void adaptive_jam_task_circle_count_all_goals_delivery_edge(int time_limit, std::vector<int>& proposed_schedule);

    // 3.2.0: 减去任务等待时间
    void adaptive_jam_task_circle_count_all_goals_minus_wait(int time_limit, std::vector<int>& proposed_schedule);

    void linear_regression(const vector<int>& x, const vector<int>& y, double& a, double& b)
    {
        int n = x.size();
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;

        for (int i = 0; i < n; ++i) {
            sum_x += x[i];
            sum_y += y[i];
            sum_xx += x[i] * x[i];
            sum_xy += x[i] * y[i];
        }

        // 计算a和b
        a = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        b = (sum_y - a * sum_x) / n;
    }

    // 3.2.1: 线性回归拟合
    void linear_regression_jam_task_circle_count_all_goals(int time_limit, std::vector<int>& proposed_schedule);

    // 3.2.2: 一口气把长度算完
    void adaptive_jam_task_circle_count_all_goals_at_once(int time_limit, std::vector<int>& proposed_schedule);

    // 3.3: 统计以task为圆心，|agent-pickup|为半径的圆中goal的数量与delivery所在方块中goal的数量作为jam
    [[nodiscard]] int compute_jam_count_pickup_circle_goal_delivery_square_goal(int _agent_id, int _agent_loc_x, 
        int _agent_loc_y, int _pickup_loc) const;
    void adaptive_jam_count_pickup_circle_goal_delivery_square_goal(int time_limit, std::vector<int>& proposed_schedule);
    
    // 3.4: 统计以task为圆心，|agent-task|*2为半径的圆中other agent goal的数量作为jam
    [[nodiscard]] int compute_jam_task_double_circle_count_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc) const;
    void adaptive_jam_task_double_circle_count_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 3.5: 统计以|agent-pickup|中心点和|pickup-delivery|中心点的中点为圆心，|agent-task|*2为半径的圆中other agent goal的数量作为jam
    [[nodiscard]] int compute_jam_task_triangle_circle_count_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc, int _delivery_loc) const;
    void adaptive_jam_task_triangle_circle_count_goal(int time_limit, std::vector<int>& proposed_schedule);

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

    // 6.0: 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和下一个目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_circle_count_middle_current_next_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc) const;
    void adaptive_jam_task_circle_count_middle_current_next_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 6.1: 计算以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的密度作为jam.
    [[nodiscard]] double compute_jam_task_circle_middle_current_goal_density(int _agent_id, int _agent_loc_x, int _agent_loc_y,
        int _pickup_loc) const;
    void adaptive_jam_task_circle_middle_current_goal_density(int time_limit, std::vector<int>& proposed_schedule);

    // 6.2: 以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的密度 x |agent pickup|作为jam.
    void adaptive_jam_task_circle_middle_current_goal_density_times_agent_pickup(int time_limit, std::vector<int>& proposed_schedule);

    // 6.3: 以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的密度 x |agent pickup delivery|作为jam.
    void adaptive_jam_task_circle_middle_current_goal_density_times_agent_pickup_delivery(int time_limit, std::vector<int>& proposed_schedule);

    // 7: 统计以task为圆心，|agent-task|为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_Manhattan_circle_count_middle_current_goal(int _agent_id, int _agent_loc_x, int _agent_loc_y,
                                                                        int _pickup_loc) const;
    void adaptive_jam_task_Manhattan_circle_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);
    // 使用RHCR启发式
    void adaptive_jam_task_Manhattan_circle_count_middle_current_goal_rhcr(int time_limit, std::vector<int> & proposed_schedule);

    // 8: 统计以task为圆心，|num_rows + num_columns| / 8为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_fix_Manhattan_circle_count_middle_current_goal(int _agent_id, int _pickup_loc) const;
    void adaptive_jam_task_fix_Manhattan_circle_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 8.1: 统计get_h(other middle, pickup) < get_h(agent, pickup), other agent 当前位置和目标点位置的中点的数量作为jam.
    [[nodiscard]] int compute_jam_task_h_circle_count_middle_current_goal(int _agent_id, int _pickup_loc, int _h_radius) const;
    void adaptive_jam_task_h_circle_count_middle_current_goal(int time_limit, std::vector<int>& proposed_schedule);

    // 9: 将地图分成16x16的区域, task所在区域中agent当前位置和目标点位置的中点的数量作为作为sum jam weight
    void adaptive_jam_task_region_count_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 10: 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent当前位置和目标点位置的中点的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
    void adaptive_jam_task_circle_vector_middle_current_goal(int time_limit, std::vector<int> & proposed_schedule);

    int cross(const Point& startA, const Point& endA, const Point& pointB) {
        return (endA.x - startA.x) * (pointB.y - startA.y) - (endA.y - startA.y) * (pointB.x - startA.x);
    }

    int angleType(const Point& startA, const Point& endA, const Point& startB, const Point& endB) {
        // 计算向量 A = (endA.x - startA.x, endA.y - startA.y) 和 B = (endB.x - startB.x, endB.y - startB.y) 的点积
        int vectorA_x = endA.x - startA.x;
        int vectorA_y = endA.y - startA.y;
        int vectorB_x = endB.x - startB.x;
        int vectorB_y = endB.y - startB.y;

        int dotProduct = vectorA_x * vectorB_x + vectorA_y * vectorB_y;

        // 判断夹角类型
        if (dotProduct > 0) {
            return 1; // 锐角
        }
        else if (dotProduct < 0) {
            return 2; // 钝角
        }
        else {
            return 0; // 直角
        }
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

    // 11.-1: 来不及做的任务就不接了
    void adaptive_jam_curr_pickup_intersect_curr_goal_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // 11.0: 一次性把加和做完
    void adaptive_jam_curr_pickup_intersect_curr_goal_at_once(int time_limit, std::vector<int>& proposed_schedule);

    // 11.0.-2: 来不及做的任务就不接了
    void adaptive_jam_curr_pickup_intersect_curr_goal_at_once_not_get(int time_limit, std::vector<int>& proposed_schedule);

    // 11.0.1: 锐角夹角加一，钝角夹角加二
    [[nodiscard]] int compute_jam_vector_curr_pickup_intersect_curr_goal(int _agent_id, Point _agent_loc,
        Point _agent_end);
    void adaptive_jam_vector_curr_pickup_intersect_curr_goal_at_once(int time_limit, std::vector<int>& proposed_schedule);

    // 11.1: 用adaptive_jam_task_circle_count_current预分配
    void adaptive_jam_curr_pickup_intersect_curr_goal_preassign(int time_limit, std::vector<int>& proposed_schedule);

    // 11.2: compute pickup jam by counting whether other agent-task line intersect with this agent-task line density
    void adaptive_jam_curr_pickup_intersect_curr_goal_density(int time_limit, std::vector<int>& proposed_schedule);

    void adaptive_jam_curr_pickup_delivery_intersect_curr_goal(int time_limit,
                                                               std::vector<int> & proposed_schedule);

    // RHCR functions
    MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm);
    void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm);
};