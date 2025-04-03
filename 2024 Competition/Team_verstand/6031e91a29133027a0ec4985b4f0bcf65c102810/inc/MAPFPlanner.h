#pragma once
#include "SharedEnv.h"
#include "ActionModel.h"
// #include "flow.h"
#include "tfo.h"
// #include "pibt.h"
#include "causal_pibt.h"
// #include "util.h"
#include "usage.h"
// #include "TrajLNS.h"
#include "path.h"
#include "ID.h"
#include "SortingSystem.h"
#include <ctime>
#include <random>
#include <fstream>
#include <queue>
#include <set>
#include <boost/program_options.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

class MAPFPlanner
{
    //*
private:

    std::vector<int> decision; // 根据每个agent决定的target, 每个地图点受否会被占据
    std::vector<int> prev_decision; // 根据之前每个agent的目标点, 每个地图点是否会被占据
    std::vector<double> priorities; // agent被规划的优先级
    std::vector<State> prev_states; // 每个agent下达指令前的位置
    std::vector<State> next_states; // 每个agent下达指令后的位置
    std::vector<int> ids; // agent正在前往任务中第几个目标点
    std::vector<double> priorities_copy; // agent被规划的优先级备份
    std::vector<bool> occupied; // 被占据的地图点
    std::vector<DCR> decided; // 给每个agent定下的目标点
    std::vector<bool> checked; // agent是否可以执行移动
    std::vector<bool> require_guide_path; // 需要提供路径的agent
    std::vector<int> dummy_goals; // 如果agent没有被分配任务, 就返回起点
    TrajLNS trajLNS; // agent轨迹信息, 包含拥堵情况
    std::mt19937 mt1;
     //*/

    vector<vector<int>> grid; // 地图的二维表示, 可用来寻找长直道
    vector<int> cell_wait_count; // 每个地图点发生了几次等待。可以用来衡量拥堵和设计highway.
    vector<vector<int>> cell_wait_direction_count; // 每个地图点发生了几次某种方向的等待。确认highway的朝向
    vector<int> prev_cell_wait_count; // 上次运行时每个地图点发生了几次等待。
    vector<vector<int>> prev_cell_wait_direction_count; // 上次运行时每个地图点发生了几次某种方向的等待。

    struct ObstacleBlock {
        int contour_rotation = -1; // 外轮廓highway的旋转方向 0: clockwise; 1: /; 2: \; 3: counter-clockwise
        unordered_set<pair<int, int>> content;   // 障碍物内部单元格
        unordered_set<pair<int, int>> contour;  // 障碍物外轮廓（紧邻的0）

        // 外轮廓用来构筑highway
        unordered_set<pair<int, int>> north_contour;      // 北(3)外轮廓
        unordered_set<pair<int, int>> south_contour;   // 南(1)外轮廓
        unordered_set<pair<int, int>> west_contour;     // 西(2)外轮廓
        unordered_set<pair<int, int>> east_contour;    // 东(0)外轮廓
        unordered_map<pair<int, int>, int> point_contour; // 一个点所属的contour是东(0)南(1)西(2)北(3)中的哪个
    };

    vector<ObstacleBlock> obstacleBlocks; // 地图中的障碍物块

    vector<vector<int>> forbids; // 每个地图点限制朝某个方向移动. 如果为空, 说明这个地图点无限制。
    // agent向几个方向移动或等待时, 额外的weights. 可用来越来限制agent朝某个方向移动
     vector<vector<int>> extra_weights;

     // RHCR members
    SortingGrid sorting_grid;
    boost::program_options::variables_map vm;
    SortingSystem* sorting_system;

public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};

    static MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm);
    static void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm);

    virtual void initialize(int preprocess_time_limit);

    void initialize_TFO(int preprocess_time_limit);

    // compute map point dist
    void compute_map_point_dist(TimePoint _endtime) const;

    // 预先计算地图上任意两点之间的距离, 采用对称性加快计算速度
    void compute_map_point_dist_symmetry(TimePoint _endtime) const;

    // 找出地图中所有obstacle block的内核部分和外轮廓部分, 并将外轮廓分为上下左右四部分。
    void find_obstacle_blocks_with_contours(const vector<vector<int>>& _grid);

    // 把obstacle blocks按照由中心向边缘的顺序排序, 后面标定highway也是按照这个顺序
    void sort_obstacle_blocks_by_distance();

    // 打印obstacle blocks的信息
    static void print_obstacle_blocks(vector<ObstacleBlock> _obstacleBlocks);

    // 返回重叠边在未定block所属的东南西北位置, 并定下未定block的旋转方向
    static int are_blocks_overlapping(ObstacleBlock& _block_decided, ObstacleBlock& _block_to_decide);

    // 找出邻接的obstacle block组成group
    // 这种方法不适用于sortation和warehouse. 那两个直接横纵交错画就可以了
    vector<vector<int>> find_connected_blocks();

    // 根据一个block的contour rotation, 构造围绕它的highway
    void build_block_highway(const ObstacleBlock& _block);

    void draw_obstacle_block() const; // 打印障碍物块

    void generate_extra_weights_from_forbids(bool _print); // 打印禁忌边

    // 依据每个obstacle block的外轮廓建立highway.
    void build_contour_highway_from_center(TimePoint _endtime);

    // 从给定位置开始，一圈一圈向外搜索，直到找到非障碍物点
    pair<int, int> find_nearest_non_obstacle(int _start_row, int _start_column);

    // 使用分割地图为几个区块的highway. 这里的分割线起点和终点都在地图四边上。
    void build_partition_highway(TimePoint _endtime);

    // 使用分割地图为几个区块的highway. 这里的分割线起点和终点有的在地图四边上, 有的在中线上。
    void build_region_partition_highway(TimePoint _endtime);

    // 计算agent向几个方向移动或等待时, 额外的weights. 可用来越来限制agent朝某个方向移动
    void initialize_extra_weights();

    // 输出每个地图点的extra_weights
    void print_extra_weights();

    // 寻找grid一行中最大连续0的个数
    static std::tuple<int, int, int> find_longest_zeros(const std::vector<int>& vec);

    // 读取上次运行的cell wait count
    void read_prev_cell_wait_count(const string& _map_name);

    // 保存cell wait count
    void save_cell_wait_count();

    // 限制横向车道的方向
    void build_row_highway(TimePoint _endtime);

    // 限制纵向车道的方向
    void build_column_highway(TimePoint _endtime);

    // 限制横向纵向车道的方向
    void build_row_column_highway(TimePoint _endtime);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    void my_plan(int time_limit,vector<Action> & _actions);
    void rhcr_plan(int time_limit,vector<Action> & _actions);
};
