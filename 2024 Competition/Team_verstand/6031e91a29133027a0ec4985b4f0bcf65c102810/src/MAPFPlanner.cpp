#include "SortingSystem.h"
#include <random>
#include <Entry.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
//default planner includes
#include "planner.h"
// #include "const.h"
#include "constant.h"

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 * 
 * This function call the planner initialize function with a time limit fore preprocessing.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    clock_t start = clock();
    // 留下1000ms的余量
    int limit = preprocess_time_limit
            - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now()
            - env->plan_start_time).count()
            - PLANNER_TIMELIMIT_TOLERANCE - 1000;
    // cout << "planner limit: " << limit << endl;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    // DefaultPlanner::initialize(limit, env);
    initialize_TFO(limit);

    // 用1d map给2d grid赋值。0表示通过, 1表示障碍。
    grid.resize(env->rows);
    for(int i=0;i<env->rows;i++)
    {
        grid[i].resize(env->cols);
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            grid[i][j] = env->map[index];
        }
    }
    // cout << endl;

    // 预先计算地图上任意两点之间的距离
    // compute_map_point_dist(endtime);
    // compute_map_point_dist_symmetry(endtime);

    initialize_extra_weights();

    // 记录每个地图点上发生等待的时间
    cell_wait_count.resize(env->rows * env->cols, 0);
    cell_wait_direction_count.resize(env->rows * env->cols);
    for(auto & this_cell : cell_wait_direction_count)
    {
        // 根据usage.cpp, 0: east。1: south。2: west。3: north
        this_cell.resize(4, 0);
    }

    // 分析各个地图的等待点密集处
    // read_prev_cell_wait_count("../jam_wait/random-32-32-20.map");

    // cout << "before building highway: " << endl;
    // print_extra_weights();

    // build_row_highway_delete(endtime);
    // build_column_highway_delete(endtime);
    // build_row_column_highway(endtime);
    // build_contour_highway_from_center(endtime);
    // build_partition_highway(endtime);
    // build_region_partition_highway(endtime);

    /*// 用名称区分
    if(env->map_name[0] == 's' || env->map_name[0] == 'w')
    {
        build_row_column_highway(endtime);
    }
    else
    {
        if(env->map_name[0] != 'P')
        {
            build_partition_highway(endtime);
        }
    }
    //*/

    // 用地图尺寸和agent数量区分算例
    /*
    if(env->map.size() <= 128 * 128)
    {
        // Random 1 2 3 4 5
        cout << "random map" << endl;
        build_partition_highway(endtime);
    }
    else
    {
        // choose method according to agent num, 2048 as separator
        // CITY-01
        if(env->num_of_agents <= 2048)
        {
            cout << "city 01" << endl;
            // build_partition_highway(endtime);
        }
        else
        {
            // 用8192作为adaptive_jam_middle_current_goal_task_circle_count和default的分界点，看看能不能把sortation和warehouse分离出来。
            if(env->num_of_agents <= 8192) // CITY-02 and GAME
            {
                // CITY-02
                if(env->num_of_agents <= 4096)
                {
                    cout << "city 02" << endl;
                    // build_partition_highway(endtime);
                }
                else // GAME
                {
                    cout << "game" << endl;
                    // build_partition_highway(endtime);
                }
            }
            else // SORTATION and WAREHOUSE
            {
                // 观测SORTATION and WAREHOUSE是否小于16384 => 小于
                // 观测SORTATION and WAREHOUSE是否小于12288
                cout << "sortation and warehouse" << endl;
                build_row_column_highway(endtime);
            }
        }
    }
    //*/

    // cout << "after building highway: " << endl;
    // print_extra_weights();
    // random-32-32-20.map, Paris_1_256.map, brc202d.map, sortation_large.map, warehouse_large.map
    // cout << "map_name: " << env->map_name << endl;

    // 李娇阳的RHCR代码
    // if (!sorting_grid.load_map("sorting_map.grid"))
    //    return;

    /*
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
    cout << "rhcr dist: " << sorting_grid.heuristics[0].at(33) << endl;
    cout << "default dist: " << DefaultPlanner::get_h(env, 0, 33) << endl;
    // 既然能在plan中算, 就没有必要在初始化时算了
    // sorting_system->simulate_LoRR(vm["simulation_time"].as<int>());
     */

    cout << "Planner initialize Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}

void MAPFPlanner::initialize_TFO(int preprocess_time_limit)
{
    // initialise all required data structures
    assert(env->num_of_agents != 0);
    priorities.resize(env->num_of_agents); // 优先级
    decision.resize(env->map.size(), -1); // 根据每个agent决定的target, 每个地图点受否会被占据
    prev_states.resize(env->num_of_agents); // agent前一个状态
    next_states.resize(env->num_of_agents); // agent后一个状态
    decided.resize(env->num_of_agents,DCR({-1,DoneState::DONE}));
    occupied.resize(env->map.size(),false); // 地图点是否被占据
    checked.resize(env->num_of_agents,false);
    ids.resize(env->num_of_agents); // agent的序列?
    require_guide_path.resize(env->num_of_agents,false); // agent是否需要指引路径
    for (int i = 0; i < ids.size();i++){
        ids[i] = i;
    }

    // initialise the heuristics tables containers
    // 记录地图中每个点可到达的邻居节点, 不包括动态障碍物(other agent)
    DefaultPlanner::init_heuristics(env);
    init_heuristics(env);
    mt1.seed(0);
    srand(0);

    new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
    trajLNS.init_mem();

    // assign initial (random) priority to each agent
    std::shuffle(ids.begin(), ids.end(), mt1);
    for (int i = 0; i < ids.size();i++){
        priorities[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
    }
    // 保存优先级副本
    priorities_copy = priorities;
}

MAPFSolver* MAPFPlanner::set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
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

void MAPFPlanner::set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
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

void MAPFPlanner::compute_map_point_dist(TimePoint _endtime) const
{
    size_t count = 0;
    // cout << "env->map.size(): " << env->map.size() << endl;
    for(int i=0;i<env->map.size();i++)
    {
        for(int j=0;j<env->map.size();j++)
        {
            // check for timeout every 10 dist computation
            if (count % 100 == 0 && std::chrono::steady_clock::now() > _endtime)
            {
                return;
            }

            if (j != i && env->map[i]==0 && env->map[j]==0)
            {
                // cout << i << " " << j << endl;
                DefaultPlanner::get_h(env, i, j);
                count++;
            }
        }
    }
}

// 预先计算地图上任意两点之间的距离, 采用对称性加快计算速度
void MAPFPlanner::compute_map_point_dist_symmetry(TimePoint _endtime) const
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

void MAPFPlanner::find_obstacle_blocks_with_contours(const vector<vector<int>>& _grid)
{
    int rows = _grid.size();
    int cols = _grid[0].size();
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));

    // 八邻域方向
    vector<pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}, // 四邻域
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // 对角线邻域
    };

    auto is_in_range = [&](int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    };

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (_grid[i][j] == 1 && !visited[i][j])
            {
                // 找到新的障碍物块
                ObstacleBlock block;
                std::queue<pair<int, int>> q;
                q.emplace(i, j);
                visited[i][j] = true;

                while (!q.empty())
                {
                    auto [x, y] = q.front();
                    q.pop();
                    block.content.insert(std::make_pair(x, y));

                    for (auto [dx, dy] : directions)
                    {
                        int nx = x + dx, ny = y + dy;
                        if (is_in_range(nx, ny))
                        {
                            if (_grid[nx][ny] == 0)
                            {
                                block.contour.insert({nx, ny}); // 紧邻的0为外轮廓
                            }
                            else if (!visited[nx][ny] && _grid[nx][ny] == 1)
                            {
                                visited[nx][ny] = true;
                                q.emplace(nx, ny);
                            }
                        }
                    }
                }

                // 计算障碍物中心点
                double cx = 0, cy = 0;
                for (const auto& cell : block.content)
                {
                    cx += cell.first;
                    cy += cell.second;
                }
                cx /= block.content.size();
                cy /= block.content.size();

                // 将外轮廓点分配到上下左右
                for (const auto& cell : block.contour)
                {
                    int x = cell.first, y = cell.second;
                    double dx = x - cx, dy = y - cy;
                    if (abs(dx) >= abs(dy))
                    {
                        // 更接近于垂直方向
                        if (dx < 0)
                        {
                            block.north_contour.insert(cell); // 北(3)
                            block.point_contour[cell] = 3;
                        }
                        else
                        {
                            block.south_contour.insert(cell); // 南(1)
                            block.point_contour[cell] = 1;
                        }
                    }
                    else
                    {
                        // 更接近于水平方向
                        if (dy < 0)
                        {
                            block.west_contour.insert(cell); // 西(2)
                            block.point_contour[cell] = 2;
                        }
                        else
                        {
                            block.east_contour.insert(cell); // 东(0)
                            block.point_contour[cell] = 0;
                        }
                    }
                }

                obstacleBlocks.push_back(block);
            }
        }
    }
}

// 把obstacle blocks按照由中心向边缘的顺序排序, 后面标定highway也是按照这个顺序
void MAPFPlanner::sort_obstacle_blocks_by_distance() {
    // 地图中心点
    double center_x = env->rows / 2.0;
    double center_y = env->cols / 2.0;

    // 计算每个障碍物块的几何中心并排序
    auto calculate_distance = [&](const ObstacleBlock& block) {
        double cx = 0, cy = 0;
        for (const auto& cell : block.content) {
            cx += cell.first;
            cy += cell.second;
        }
        cx /= block.content.size();
        cy /= block.content.size();
        return sqrt(pow(cx - center_x, 2) + pow(cy - center_y, 2));
    };

    sort(obstacleBlocks.begin(), obstacleBlocks.end(), [&](const ObstacleBlock& a, const ObstacleBlock& b) {
        return calculate_distance(a) < calculate_distance(b);
    });
}

// 打印obstacle blocks的信息
void MAPFPlanner::print_obstacle_blocks(vector<ObstacleBlock> _obstacleBlocks)
{
    cout << "Number of obstacle blocks: " << _obstacleBlocks.size() << endl;
    for (int i = 0; i < _obstacleBlocks.size(); ++i) {
        cout << "Block " << i + 1 << ":\n";
        cout << "  Content:";
        for (auto [x, y] : _obstacleBlocks[i].content) {
            cout << " (" << x << "," << y << ")";
        }
        cout << "\n  Contour:";
        for (auto [x, y] : _obstacleBlocks[i].contour) {
            cout << " (" << x << "," << y << ")";
        }
        cout << endl;

        cout << "  Top:";
        for (auto [x, y] : _obstacleBlocks[i].north_contour) {
            cout << " (" << x << "," << y << ")";
        }
        cout << "\n  Bottom:";
        for (auto [x, y] : _obstacleBlocks[i].south_contour) {
            cout << " (" << x << "," << y << ")";
        }
        cout << "\n  Left:";
        for (auto [x, y] : _obstacleBlocks[i].west_contour) {
            cout << " (" << x << "," << y << ")";
        }
        cout << "\n  Right:";
        for (auto [x, y] : _obstacleBlocks[i].east_contour) {
            cout << " (" << x << "," << y << ")";
        }
        cout << endl;
    }
}

// 返回重叠边在未定block所属的东南西北位置, 并定下未定block的旋转方向
int MAPFPlanner::are_blocks_overlapping(ObstacleBlock& _block_decided, ObstacleBlock& _block_to_decide)
{
    // 遍历block2的外轮廓点，检查是否有交集
    for (const auto& point : _block_to_decide.contour)
    {
        if (_block_decided.contour.find(point) != _block_decided.contour.end())
        {
            // 如果有交集, 顺便把block2的contour rotation也定下来
            int overlap_direction = _block_to_decide.point_contour[point];

            if(_block_decided.contour_rotation == 0) // 0: clockwise
            {
                if(overlap_direction == 0 || overlap_direction == 2) // east or west contour of block to decide
                {
                    _block_to_decide.contour_rotation = 1; // 1: /
                }
                else if (overlap_direction == 1 || overlap_direction == 3) // south or north contour of block to decide
                {
                    _block_to_decide.contour_rotation = 2; // 2: \;
                }
            }
            else if(_block_decided.contour_rotation == 1) // 1: /
            {
                if(overlap_direction == 0 || overlap_direction == 2) // east or west contour of block to decide
                {
                    _block_to_decide.contour_rotation = 0; // 0: clockwise ;
                }
                else if (overlap_direction == 1 || overlap_direction == 3) // south or north contour of block to decide
                {
                    _block_to_decide.contour_rotation = 3; // 3: counterclockwise;
                }
            }
            else if(_block_decided.contour_rotation == 2) // 2: \ ;
            {
                if(overlap_direction == 0 || overlap_direction == 2) // east or west contour of block to decide
                {
                    _block_to_decide.contour_rotation = 3; // 3: counterclockwise ;
                }
                else if (overlap_direction == 1 || overlap_direction == 3) // south or north contour of block to decide
                {
                    _block_to_decide.contour_rotation = 0; // 0: clockwise;
                }
            }
            else if(_block_decided.contour_rotation == 3) // 3: counterclockwise
            {
                if(overlap_direction == 0 || overlap_direction == 2) // east or west contour of block to decide
                {
                    _block_to_decide.contour_rotation = 2; // 2: \ ;
                }
                else if (overlap_direction == 1 || overlap_direction == 3) // south or north contour of block to decide
                {
                    _block_to_decide.contour_rotation = 1; // 1: /;
                }
            }

            return overlap_direction; // 返回重叠边在block2所属的东南西北位置
        }
    }

    return -1; // 无交集
}

vector<vector<int>> MAPFPlanner::find_connected_blocks() {
    auto n = obstacleBlocks.size();
    vector<vector<int>> connectedComponents; // 存储所有相连的块组
    vector<bool> visited(n, false);          // 标记每个 block 是否已访问

    // BFS 查找相连的 block
    auto bfs = [&](int start) {
        vector<int> component;        // 当前连通块
        std::queue<int> q;                 // 待处理的 block id
        q.push(start);
        visited[start] = true;

        while (!q.empty())
        {
            int current = q.front();
            q.pop();
            component.push_back(current);

            for (int i = 0; i < n; ++i)
            {
                if (!visited[i] &&
                are_blocks_overlapping(obstacleBlocks[current], obstacleBlocks[i]) != -1)
                {
                    q.push(i);
                    visited[i] = true;
                }
            }
        }

        return component;
    };

    // 遍历所有 block，找到所有连通块
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            connectedComponents.push_back(bfs(i));
        }
    }

    return connectedComponents;
}


void MAPFPlanner::initialize_extra_weights()
{
    forbids.resize(env->rows * env->cols); // 每个地图点限制朝某个方向移动

    extra_weights.resize(env->rows * env->cols);
    for (int row=0; row<env->rows; row++){
        for (int col=0; col<env->cols; col++){
            int loc = row*env->cols+col;
            extra_weights[loc].resize(4, 0);
        }
    }
}

// 输出每个地图点的extra_weights
void MAPFPlanner::print_extra_weights()
{
    for(int i=0;i<env->rows;i++)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";

            bool is_found = false; // 这个地图点是否被highway约束
            for(auto element : extra_weights[index])
            {
                if (element == MAX_TIMESTEP)
                {
                    is_found = true;
                    break;
                }
            }

            if(is_found)
            {
                cout << "1";
            }
            else
            {
                cout << ".";
            }
            if (j != env->cols - 1) cout << " "; // 用空格分隔
        }
        cout << "\n";
    }
}

// 根据一个block的contour rotation, 构造围绕它的highway (forbids)
void MAPFPlanner::build_block_highway(const ObstacleBlock& _block)
{
    if(_block.contour_rotation == 0) // 0: clockwise。
    {
        // north contour禁止向西移动。根据usage.cpp, 西方对应2.
        for(auto const& point : _block.north_contour)
        {
            int point_north_contour = point.first * env->cols + point.second;
            forbids[point_north_contour].emplace_back(2);
        }

        // south contour禁止向东移动。根据usage.cpp, 东方对应0.
        for(auto const& point : _block.south_contour)
        {
            int point_south_contour = point.first * env->cols + point.second;
            forbids[point_south_contour].emplace_back(0);
        }

        // west contour禁止向南移动。根据usage.cpp, 南方对应1.
        for(auto const& point : _block.west_contour)
        {
            int point_west_contour = point.first * env->cols + point.second;
            forbids[point_west_contour].emplace_back(1);
        }

        // east contour禁止向北移动。根据usage.cpp, 北方对应3.
        for(auto const& point : _block.east_contour)
        {
            int point_east_contour = point.first * env->cols + point.second;
            forbids[point_east_contour].emplace_back(3);
        }
    }
    else if (_block.contour_rotation == 1) // 1: /
    {
        // north contour禁止向西移动。根据usage.cpp, 西方对应2.
        for(auto const& point : _block.north_contour)
        {
            int point_north_contour = point.first * env->cols + point.second;
            forbids[point_north_contour].emplace_back(2);
        }

        // south contour禁止向东移动。根据usage.cpp, 东方对应0.
        for(auto const& point : _block.south_contour)
        {
            int point_south_contour = point.first * env->cols + point.second;
            forbids[point_south_contour].emplace_back(0);
        }

        // west contour禁止向北移动。根据usage.cpp, 北方对应3.
        for(auto const& point : _block.west_contour)
        {
            int point_west_contour = point.first * env->cols + point.second;
            forbids[point_west_contour].emplace_back(3);
        }

        // east contour禁止向南移动。根据usage.cpp, 南方对应1.
        for(auto const& point : _block.east_contour)
        {
            int point_east_contour = point.first * env->cols + point.second;
            forbids[point_east_contour].emplace_back(1);
        }
    }
    else if (_block.contour_rotation == 2) // 2: \ ;
    {
        // north contour禁止向东移动。根据usage.cpp, 东方对应0.
        for(auto const& point : _block.north_contour)
        {
            int point_north_contour = point.first * env->cols + point.second;
            forbids[point_north_contour].emplace_back(0);
        }

        // south contour禁止向西移动。根据usage.cpp, 西方对应2.
        for(auto const& point : _block.south_contour)
        {
            int point_south_contour = point.first * env->cols + point.second;
            forbids[point_south_contour].emplace_back(2);
        }

        // west contour禁止向南移动。根据usage.cpp, 南方对应1.
        for(auto const& point : _block.west_contour)
        {
            int point_west_contour = point.first * env->cols + point.second;
            forbids[point_west_contour].emplace_back(1);
        }

        // east contour禁止向北移动。根据usage.cpp, 北方对应3.
        for(auto const& point : _block.east_contour)
        {
            int point_east_contour = point.first * env->cols + point.second;
            forbids[point_east_contour].emplace_back(3);
        }
    }
    else if (_block.contour_rotation == 3) // 3: counterclockwise ;
    {
        // north contour禁止向东移动。根据usage.cpp, 东方对应0.
        for(auto const& point : _block.north_contour)
        {
            int point_north_contour = point.first * env->cols + point.second;
            forbids[point_north_contour].emplace_back(0);
        }

        // south contour禁止向西移动。根据usage.cpp, 西方对应2.
        for(auto const& point : _block.south_contour)
        {
            int point_south_contour = point.first * env->cols + point.second;
            forbids[point_south_contour].emplace_back(2);
        }

        // west contour禁止向北移动。根据usage.cpp, 北方对应3.
        for(auto const& point : _block.west_contour)
        {
            int point_west_contour = point.first * env->cols + point.second;
            forbids[point_west_contour].emplace_back(3);
        }

        // east contour禁止向南移动。根据usage.cpp, 南方对应1.
        for(auto const& point : _block.east_contour)
        {
            int point_east_contour = point.first * env->cols + point.second;
            forbids[point_east_contour].emplace_back(1);
        }
    }

}

void MAPFPlanner::draw_obstacle_block() const
{
    vector<int> map_cell(env->rows*env->cols, 0);
     /*
    for(auto element : obstacleBlocks[0].content)
    {
        map_cell[element.first * env->cols + element.second] = 1;
    }

    for(auto element : obstacleBlocks[2].content)
    {
        map_cell[element.first * env->cols + element.second] = 1;
    }
     */

    for(auto& obstacleBlock : obstacleBlocks)
    {
        for(auto element : obstacleBlock.content)
        {
            map_cell[element.first * env->cols + element.second] = 1;
        }
    }

    for(int i=0;i<env->rows;i++)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            if(map_cell[index] == 0)
            {
                cout << ".";
            }
            else
            {
                cout << "1";
            }
            if (j != env->cols - 1) cout << " "; // 用空格分隔
        }
        cout << "\n";
    }
}

void MAPFPlanner::generate_extra_weights_from_forbids(bool _print)
{
    if(_print)
    {
        for(int i=0;i<env->rows;i++)
        {
            for(int j=0;j<env->cols;j++)
            {
                int index = i * env->cols + j;
                // cout << index << " ";
                if(forbids[index].empty())
                {
                    if(env->map[index] == 1)
                    {
                        cout << "@";
                    }
                    else if(env->map[index] == 0)
                    {
                        cout << ".";
                    }
                }
                else
                {
                    // 画的是禁止向的反向, 也就是鼓励向
                    if(forbids[index][0] == 0)
                    {
                        cout << "\u2190"; // 禁止向东, 鼓励向西。<
                    }
                    else if(forbids[index][0] == 1)
                    {
                        cout << "\u2191"; // 禁止向南, 鼓励向北 ^
                    }
                    else if(forbids[index][0] == 2)
                    {
                        cout << "\u2192"; // 禁止向西, 鼓励向东 >
                    }
                    else if(forbids[index][0] == 3)
                    {
                        cout << "\u2193"; // 禁止向北, 鼓励向南 v
                    }
                }
                if (j != env->cols - 1) cout << " "; // 用空格分隔
            }
            cout << "\n";
        }
    }

    int penalty_distance = env->rows + env->cols;

    for(int i=0;i<env->rows;i++)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            if(!forbids[index].empty())
            {
                if(forbids[index][0] == 0)
                {
                    // cout << "\u2192"; // 禁止向东, 鼓励向西
                    extra_weights[index][0] = penalty_distance;
                }
                else if(forbids[index][0] == 1)
                {
                    // cout << "\u2191"; // 禁止向南, 鼓励向北
                    extra_weights[index][1] = penalty_distance;
                }
                else if(forbids[index][0] == 2)
                {
                    // cout << "\u2190"; // 禁止向西, 鼓励向东
                    extra_weights[index][2] = penalty_distance;
                }
                else if(forbids[index][0] == 3)
                {
                    // cout << "\u2193"; // 禁止向北, 鼓励向南
                    extra_weights[index][3] = penalty_distance;
                }
            }
            // if (j != env->cols - 1) cout << " "; // 用空格分隔
        }
        // cout << "\n";
    }
}

// 寻找grid一行中最大连续0的个数
std::tuple<int, int, int> MAPFPlanner::find_longest_zeros(const std::vector<int>& vec)
{
    int max_length = 0;       // 最大连续0的长度
    int start_index = -1;     // 最大连续0的起点索引
    int end_index = -1;       // 最大连续0的终点索引

    int current_start = -1;   // 当前连续0的起点索引
    int current_length = 0;   // 当前连续0的长度

    for (int i = 0; i < vec.size(); ++i) {
        if (vec[i] == 0) {
            // 如果是0，更新当前连续0的长度
            if (current_length == 0) {
                current_start = i;  // 标记当前连续0的起点
            }
            current_length++;

            // 如果当前连续0长度超过最大长度，则更新最大长度和起终点
            if (current_length > max_length) {
                max_length = current_length;
                start_index = current_start;
                end_index = i;
            }
        } else {
            // 如果不是0，重置当前连续0的长度
            current_length = 0;
        }
    }

    return {max_length, start_index, end_index};
}

void MAPFPlanner::read_prev_cell_wait_count(const string& _map_name)
{
    // 读取上次运行时的cell wait (direction) count
    std::ifstream ifs(_map_name);
    boost::archive::text_iarchive ia(ifs);
    ia >> prev_cell_wait_count; // 加载上次运行时每个地图点发生了几次等待。
    ia >> prev_cell_wait_direction_count; // 加载上次运行时每个地图点发生了几次某种方向的等待。
    ifs.close();

    cout << "prev cell wait count: " << prev_cell_wait_count.size() << endl;
    cout << "prev cell wait direction count: " << prev_cell_wait_direction_count.size() << endl;

    std::ofstream file("../jam_wait/random-32-32-20.csv");
    for(int i=0;i<env->rows;i++)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            file << prev_cell_wait_count[index];
            if (j != env->cols - 1) file << ","; // 用逗号分隔
        }
        file << "\n";
    }
    file.close();
}

void MAPFPlanner::save_cell_wait_count()
{
    std::ofstream ofs(env->map_name);
    boost::archive::text_oarchive oa(ofs);
    oa << cell_wait_count; // 保存第一个变量
    oa << cell_wait_direction_count; // 保存第二个变量
    ofs.close();
}

void MAPFPlanner::build_row_highway(TimePoint _endtime)
{
    /*
    cout << "num global neighbors: " << DefaultPlanner::global_neighbors.size() << endl;
    cout << "global neighbor[0] num: " << DefaultPlanner::global_neighbors[0].size() << endl;
    cout << "global neighbor[1] num: " << DefaultPlanner::global_neighbors[1].size() << endl;
    cout << "global htable size: " << DefaultPlanner::global_heuristictable.size() << endl;
    cout << "global htable[0] empty: " << DefaultPlanner::global_heuristictable[0].empty() << endl;
    cout << "global htable[1] empty: " << DefaultPlanner::global_heuristictable[1].empty() << endl;
     */
    // 给grid赋值
    grid.resize(env->rows);
    for(int i=0;i<env->rows;i++)
    {
        grid[i].resize(env->cols);
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            grid[i][j] = env->map[index];
        }
    }
    // cout << endl;

    // 统计每行最长连续0的个数, 用来找出走廊行。
    vector<int> rowsLongestZeros(env->rows, 0); // 每行最长连续0的个数
    for(int i=0;i<env->rows;i++)
    {
        auto [max_length, start_index, end_index] = find_longest_zeros(grid[i]);

        /*
        std::cout << "row " << i << " Maximum length of consecutive zeros: " << max_length << "\n";
        std::cout << "Start index: " << start_index << "\n";
        std::cout << "End index: " << end_index << "\n";
        // */

        rowsLongestZeros[i] = max_length;
    }

    vector<int> passageRows; // 走廊行的特点是, 自己最长零串的长度过半, 而上下邻居行的最长零串长度都不过半
    for(int i=1;i<env->rows-1;i++)
    {
        if(rowsLongestZeros[i] > env->cols / 2
           && rowsLongestZeros[i-1] < env->cols / 2
           && rowsLongestZeros[i+1] < env->cols / 2)
        {
            passageRows.emplace_back(i);
        }
    }

    /*
    cout << "passage row: ";
    for(int i=0;i<passageRows.size();i++)
    {
        cout << passageRows[i] << " ";
    }
    cout << endl;
     */

    for(auto row : passageRows)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = row * env->cols + j;
            // cout << index << " ";
            // cout << DefaultPlanner::global_neighbors[index].size() << " ";
            /*
            cout << "(";
            for(auto neighbor : DefaultPlanner::global_neighbors[index])
            {
                cout << neighbor << " ";
            }
            cout << ")";
             */
        }
        // cout << endl;
    }

    vector<int> oddPassageRows;
    vector<int> evenPassageRows;
    for(int i=0;i<passageRows.size();i++)
    {
        if(i % 2==0)
        {
            evenPassageRows.emplace_back(passageRows[i]);
        }
        else
        {
            oddPassageRows.emplace_back(passageRows[i]);
        }
    }

    /*
    cout << "even passage rows: ";
    for(auto row : evenPassageRows)
    {
        cout << row << " ";
    }
    cout << endl;

    cout << "odd passage rows: ";
    for(auto row : oddPassageRows)
    {
        cout << row << " ";
    }
    cout << endl;
     //*/

    // even行给左向移动增加cost
    //*
    for(auto row : evenPassageRows)
    {
        for(int j=1;j<env->cols-1;j++)
        {
            int index = row * env->cols + j;
            // 根据usage.cpp, 向西对应2
            extra_weights[index][2] = MAX_TIMESTEP;
        }
    }
     //*/

    // odd行给右向移动增加cost
    //*
    for(auto row : oddPassageRows)
    {
        for(int j=1;j<env->cols-1;j++)
        {
            int index = row * env->cols + j;
            // 根据usage.cpp, 向东对应0
            extra_weights[index][0] = MAX_TIMESTEP;
        }
    }
     //*/

}

void MAPFPlanner::build_column_highway(TimePoint _endtime)
{
    /*
    cout << "num global neighbors: " << DefaultPlanner::global_neighbors.size() << endl;
    cout << "global neighbor[0] num: " << DefaultPlanner::global_neighbors[0].size() << endl;
    cout << "global neighbor[1] num: " << DefaultPlanner::global_neighbors[1].size() << endl;
    cout << "global htable size: " << DefaultPlanner::global_heuristictable.size() << endl;
    cout << "global htable[0] empty: " << DefaultPlanner::global_heuristictable[0].empty() << endl;
    cout << "global htable[1] empty: " << DefaultPlanner::global_heuristictable[1].empty() << endl;
     */
    // 给grid赋值
    grid.resize(env->rows);
    for(int i=0;i<env->rows;i++)
    {
        grid[i].resize(env->cols);
        for(int j=0;j<env->cols;j++)
        {
            int index = i * env->cols + j;
            // cout << index << " ";
            grid[i][j] = env->map[index];
        }
    }
    // cout << endl;

    // 统计每列最长连续0的个数, 用来找出走廊列。
    vector<int> columnsLongestZeros(env->cols, 0); // 每列最长连续0的个数
    for(int i=0;i<env->cols;i++)
    {
        vector<int> each_column(env->rows, 0);
        for(int j=0;j<env->rows;j++)
        {
            each_column[j] = grid[j][i];
        }

        auto [max_length, start_index, end_index] = find_longest_zeros(each_column);

        /*
        std::cout << "column " << i << " Maximum length of consecutive zeros: " << max_length << "\n";
        std::cout << "Start index: " << start_index << "\n";
        std::cout << "End index: " << end_index << "\n";
        // */

        columnsLongestZeros[i] = max_length;
    }

    vector<int> passageColumns; // 走廊列的特点是, 自己最长零串的长度过半, 而左右邻居列的最长零串长度都不过半
    for(int i=1;i<env->cols-1;i++)
    {
        if(columnsLongestZeros[i] > env->rows / 2
           && columnsLongestZeros[i-1] < env->rows / 2
           && columnsLongestZeros[i+1] < env->rows / 2)
        {
            passageColumns.emplace_back(i);
        }
    }

    /*
    cout << "passage columns: ";
    for(int i=0;i<passageColumns.size();i++)
    {
        cout << passageColumns[i] << " ";
    }
    cout << endl;
     //*/

    vector<int> oddPassageColumns;
    vector<int> evenPassageColumns;
    for(int i=0;i<passageColumns.size();i++)
    {
        if(i % 2==0)
        {
            evenPassageColumns.emplace_back(passageColumns[i]);
        }
        else
        {
            oddPassageColumns.emplace_back(passageColumns[i]);
        }
    }

    /*
    cout << "even passage columns: ";
    for(auto column : evenPassageColumns)
    {
        cout << column << " ";
    }
    cout << endl;

    cout << "odd passage columns: ";
    for(auto column : oddPassageColumns)
    {
        cout << column << " ";
    }
    cout << endl;
     //*/

    // even列给向上移动增加cost
    //*
    for(auto column : evenPassageColumns)
    {
        for(int i=1;i<env->rows-1;i++)
        {
            int index = i * env->cols + column;
            // 根据usage.cpp, 向北对应3
            extra_weights[index][3] = MAX_TIMESTEP;
        }
    }
    //*/

    // odd列给向下移动增加cost
    //*
    for(auto column : oddPassageColumns)
    {
        for(int i=1;i<env->rows-1;i++)
        {
            int index = i * env->cols + column;
            // 根据usage.cpp, 向南对应1
            extra_weights[index][1] = MAX_TIMESTEP;
        }
    }
    //*/
}

// 限制横向纵向车道的方向
void MAPFPlanner::build_row_column_highway(TimePoint _endtime)
{
    // 统计每行最长连续0的个数, 用来找出走廊行。
    vector<int> rowsLongestZeros(env->rows, 0); // 每行最长连续0的个数
    for(int i=0;i<env->rows;i++)
    {
        auto [max_length, start_index, end_index] = find_longest_zeros(grid[i]);

        /*
        std::cout << "row " << i << " Maximum length of consecutive zeros: " << max_length << "\n";
        std::cout << "Start index: " << start_index << "\n";
        std::cout << "End index: " << end_index << "\n";
        // */

        rowsLongestZeros[i] = max_length;
    }

    vector<int> passageRows; // 走廊行的特点是, 自己最长零串的长度过半, 而上下邻居行的最长零串长度都不过半
    for(int i=1;i<env->rows-1;i++)
    {
        if(rowsLongestZeros[i] > env->cols / 2
           && rowsLongestZeros[i-1] < env->cols / 2
           && rowsLongestZeros[i+1] < env->cols / 2)
        {
            passageRows.emplace_back(i);
        }
    }

    /*
    cout << "passage row: ";
    for(int i=0;i<passageRows.size();i++)
    {
        cout << passageRows[i] << " ";
    }
    cout << endl;
     */

    /*
    cout << "passage row wait count: ";
    for(int i=0;i<passageRows.size();i++)
    {
        for(int j=0;j<env->cols;j++)
        {
            int index = passageRows[i] * env->cols + j;
            cout << prev_cell_wait_count[index] << " ";
        }
        cout << endl;
    }
    cout << endl;
     //*/

    vector<int> oddPassageRows;
    vector<int> evenPassageRows;
    for(int i=0;i<passageRows.size();i++)
    {
        if(i % 2==0)
        {
            evenPassageRows.emplace_back(passageRows[i]);
        }
        else
        {
            oddPassageRows.emplace_back(passageRows[i]);
        }
    }

    /*
    cout << "even passage rows: ";
    for(auto row : evenPassageRows)
    {
        cout << row << " ";
    }
    cout << endl;

    cout << "odd passage rows: ";
    for(auto row : oddPassageRows)
    {
        cout << row << " ";
    }
    cout << endl;
     //*/

    // even行给左向移动增加cost
    //*
    for(auto row : evenPassageRows)
    {
        for(int j=1;j<env->cols-1;j++)
        {
            int index = row * env->cols + j;
            // 根据usage.cpp, 向西对应2
            extra_weights[index][2] = MAX_TIMESTEP;
        }
    }
    //*/

    // odd行给右向移动增加cost
    //*
    for(auto row : oddPassageRows)
    {
        for(int j=1;j<env->cols-1;j++)
        {
            int index = row * env->cols + j;
            // 根据usage.cpp, 向东对应0
            extra_weights[index][0] = MAX_TIMESTEP;
        }
    }
    //*/

    // 统计每列最长连续0的个数, 用来找出走廊列。
    vector<int> columnsLongestZeros(env->cols, 0); // 每列最长连续0的个数
    for(int i=0;i<env->cols;i++)
    {
        vector<int> each_column(env->rows, 0);
        for(int j=0;j<env->rows;j++)
        {
            each_column[j] = grid[j][i];
        }

        auto [max_length, start_index, end_index] = find_longest_zeros(each_column);

        /*
        std::cout << "column " << i << " Maximum length of consecutive zeros: " << max_length << "\n";
        std::cout << "Start index: " << start_index << "\n";
        std::cout << "End index: " << end_index << "\n";
        // */

        columnsLongestZeros[i] = max_length;
    }

    vector<int> passageColumns; // 走廊列的特点是, 自己最长零串的长度过半, 而左右邻居列的最长零串长度都不过半
    for(int i=1;i<env->cols-1;i++)
    {
        if(columnsLongestZeros[i] > env->rows / 2
           && columnsLongestZeros[i-1] < env->rows / 2
           && columnsLongestZeros[i+1] < env->rows / 2)
        {
            passageColumns.emplace_back(i);
        }
    }

    /*
    cout << "passage columns: ";
    for(int i=0;i<passageColumns.size();i++)
    {
        cout << passageColumns[i] << " ";
    }
    cout << endl;
     //*/

    vector<int> oddPassageColumns;
    vector<int> evenPassageColumns;
    for(int i=0;i<passageColumns.size();i++)
    {
        if(i % 2==0)
        {
            evenPassageColumns.emplace_back(passageColumns[i]);
        }
        else
        {
            oddPassageColumns.emplace_back(passageColumns[i]);
        }
    }

    /*
    cout << "even passage columns: ";
    for(auto column : evenPassageColumns)
    {
        cout << column << " ";
    }
    cout << endl;

    cout << "odd passage columns: ";
    for(auto column : oddPassageColumns)
    {
        cout << column << " ";
    }
    cout << endl;
     //*/

    // even列给向上移动增加cost
    //*
    for(auto column : evenPassageColumns)
    {
        for(int i=1;i<env->rows-1;i++)
        {
            int index = i * env->cols + column;
            // 根据usage.cpp, 向北对应3
            extra_weights[index][3] = MAX_TIMESTEP;
        }
    }
    //*/

    // odd列给向下移动增加cost
    //*
    for(auto column : oddPassageColumns)
    {
        for(int i=1;i<env->rows-1;i++)
        {
            int index = i * env->cols + column;
            // 根据usage.cpp, 向南对应1
            extra_weights[index][1] = MAX_TIMESTEP;
        }
    }
    //*/
}

// 依据每个obstacle block的外轮廓建立highway.
void MAPFPlanner::build_contour_highway_from_center(TimePoint _endtime)
{
    vector<vector<int>> test_grid = {
            {1, 1, 0, 0, 0},
            {1, 1, 0, 1, 1},
            {0, 0, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 0, 0, 0}
    };

    // Step 1: 找出地图中所有obstacle block的内核部分和外轮廓部分, 并将外轮廓分为上下左右四部分。
    // obstacleBlocks = find_obstacle_blocks_with_contours(test_grid);
    find_obstacle_blocks_with_contours(grid);

    // Step 2: 把obstacle blocks按照由中心向边缘的顺序排序, 后面标定highway也是按照这个顺序
    sort_obstacle_blocks_by_distance();

    /*// 检查block重叠函数是否正确
    cout << 0 << " and " << 1 << ": " << are_blocks_overlapping(obstacleBlocks[0],
                                                                obstacleBlocks[1]) << endl;
    cout << 0 << " and " << 10 << ": " << are_blocks_overlapping(obstacleBlocks[0],
                                                                 obstacleBlocks[10]) << endl;]
                                                                 */

    // print_obstacle_blocks(obstacleBlocks);

    // Test: 找出邻接的obstacle block组成group
    // 感觉不找也可以
    // vector<vector<int>> connectedBlocks = find_connected_blocks();

    // 输出每个连通块的 block id
    /*
    for (size_t i = 0; i < connectedBlocks.size(); ++i) {
        cout << "Connected Component " << i + 1 << ": ";
        for (int id : connectedBlocks[i]) {
            cout << id << " ";
        }
        cout << endl;
    }
     */

    // Step 3: 按地图中心向地图边缘的顺序, 给obstacle block的外轮廓绘制方向禁忌
    // 给地图最中央的block赋值。后面也可以考虑首先给最大块的block赋值。
    obstacleBlocks[0].contour_rotation = 0; // 初始的设为0: clockwise。
    build_block_highway(obstacleBlocks[0]);

    // Test: 给2画上highway
    /*
    int overlapping_type = are_blocks_overlapping(obstacleBlocks[0], obstacleBlocks[8]);
    if (overlapping_type != -1)
    {
        // cout << "overlap type: " << overlapping_type << endl; // 北(3)外轮廓
        // cout << "2 contour direction: " << obstacleBlocks[2].contour_rotation << endl;
        build_block_highway(obstacleBlocks[8]);
    }
    //*/
    // draw_obstacle_block();

    // 找出和0相邻的block, 根据相邻的外轮廓添加forbids
    /*// 给0的所有neighbor画上highway.
    cout << 0 << " neighbor: ";
    for (int i = 1; i < obstacleBlocks.size(); ++i)
    {
        int overlapping_type = are_blocks_overlapping(obstacleBlocks[0], obstacleBlocks[i]);
        if (overlapping_type != -1)
        {
            cout << i << " ";
            build_block_highway(obstacleBlocks[i]);
        }
    }
    cout << endl;
     //*/

    auto n = obstacleBlocks.size();
    vector<bool> visited(n, false);          // 标记每个 block 是否已访问

    // BFS 查找相连的 block
    auto bfs = [&](int start) {
        // vector<int> component;        // 当前连通块
        std::queue<int> q;                 // 待处理的 block id
        q.push(start);
        visited[start] = true;

        while (!q.empty())
        {
            int current = q.front();
            q.pop();
            // component.push_back(current);
            // cout << current << " rotation: " << obstacleBlocks[current].contour_rotation << endl;

            for (int i = 0; i < n; ++i)
            {
                if (!visited[i] &&
                are_blocks_overlapping(obstacleBlocks[current], obstacleBlocks[i]) != -1)
                {
                    q.push(i);
                    visited[i] = true;
                    // cout << i << " ";
                    build_block_highway(obstacleBlocks[i]);
                }
            }
            // cout << endl;
        }

        // return component;
    };

    // 遍历所有 block，找到所有连通(共享部分外轮廓)块
    for (int i = 0; i < n; ++i)
    {
        if (!visited[i])
        {
            if(obstacleBlocks[i].contour_rotation == -1)
            {
                // cout << "block " << i << " contour rotation not allocated" << endl;
                // 如果没有定向, 就定为0
                obstacleBlocks[i].contour_rotation = 0; // error in astar: re-expansion
                build_block_highway(obstacleBlocks[i]);
            }

            bfs(i);
        }
    }

    generate_extra_weights_from_forbids(true);
}

// 从给定位置开始，一圈一圈向外搜索，直到找到非障碍物点
pair<int, int> MAPFPlanner::find_nearest_non_obstacle(int _start_row, int _start_column)
{
    int rows = grid.size();
    int cols = grid[0].size();

    // 如果起始点已经是非障碍物，直接返回
    if (grid[_start_row][_start_column] == 0) {
        return {_start_row, _start_column};
    }

    // 八邻域方向 (上下左右 + 对角线)
    vector<pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    // 队列用于BFS
    std::queue<pair<int, int>> q;
    q.emplace(_start_row, _start_column);

    // 记录访问过的点
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    visited[_start_row][_start_column] = true;

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        // 遍历当前点的所有邻居
        for (const auto& [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            // 检查是否在范围内且未访问过
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && !visited[nx][ny]) {
                visited[nx][ny] = true;

                // 如果找到非障碍物点，返回该点
                if (grid[nx][ny] == 0) {
                    return {nx, ny};
                }

                // 否则将该点加入队列
                q.emplace(nx, ny);
            }
        }
    }

    // 如果没有找到非障碍物点，返回无效位置
    return {-1, -1}; // 根据实际需求调整返回值
}

// 使用分割地图为几个区块的highway
void MAPFPlanner::build_partition_highway(TimePoint _endtime)
{
    std::pair<int, int> true_start, true_goal;
    //*
    int num_highway_row = 8; // 有多少个highway行
    TrajLNS highway_row(env, global_heuristictable, global_neighbors, num_highway_row);
    highway_row.init_mem();
    // cout << "partition agent num: " << highway_row.trajs.size() << endl; // 一开始agent num是知道的

    // cout << "grid 0 1: " << grid[1][0] << endl;
    // partition highway成对出现, 有向东则有向西, 有向北则有向南
    // 绘制横向highway。第一个横向highway是零行
    int highway_row_counter = 0;
    for(int i=0;i<num_highway_row;i++)
    {
        // 如果是偶数行, 则起点列在左, 终点列在右
        if (highway_row_counter % 2 == 0)
        {
            int approximate_start_column = 0;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            } else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols - 1;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            } else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else // 如果是奇数行, 则起点列在右, 终点列在左
        {
            int approximate_start_column = env->cols - 1;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            } else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = 0;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            } else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_row.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_row.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_row.env, highway_row.flow, highway_row.heuristics[goal], highway_row.trajs[i],
                                 extra_weights, highway_row.mem, start, goal, &(highway_row.neighbors));

        if(!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_row_counter++;
        }

        // cout << "highway: ";
        for(int j=0; j < highway_row.trajs[i].size() - 1; j++)
        {
            // cout << highway_row.trajs[i][j] << " ";
            int diff = highway_row.trajs[i][j+1] - highway_row.trajs[i][j];
            // cout << "diff: " << diff << endl;
            // cout << get_d(diff, env) << endl;
            forbids[highway_row.trajs[i][j]].emplace_back(get_d(-diff, env)); // -diff是因为highway逆向是禁止向
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false); 
        // break;
    }
    //*/

    //*
    int num_highway_column = 8; // 有多少个highway列
    TrajLNS highway_column(env, global_heuristictable, global_neighbors, num_highway_column);
    highway_column.init_mem();
    // cout << "partition agent num: " << highway_row.trajs.size() << endl; // 一开始agent num是知道的

    // 绘制纵向highway
    int highway_column_counter = 0;
    for(int i=0;i<num_highway_column;i++)
    {
        // 如果是偶数列, 则起点行在下, 终点行在上
        if (highway_column_counter % 2 == 0)
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = env->rows - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            } else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = 0;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            } else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = 0;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            } else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = env->rows - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            } else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_column.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_column.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_column.env, highway_column.flow, highway_column.heuristics[goal], highway_column.trajs[i],
                                 extra_weights, highway_column.mem, start, goal, &(highway_column.neighbors));

        if(!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_column_counter++;
        }

        // cout << "highway: ";
        for(int j=0; j < highway_column.trajs[i].size() - 1; j++)
        {
            int diff = highway_column.trajs[i][j+1] - highway_column.trajs[i][j];
            forbids[highway_column.trajs[i][j]].emplace_back(get_d(-diff, env));
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false);
    }
     //*/

    generate_extra_weights_from_forbids(true);
}

// 使用分割地图为几个区块的highway
void MAPFPlanner::build_region_partition_highway(TimePoint _endtime)
{
    std::pair<int, int> true_start, true_goal;
    //*
    int num_highway_row = 8; // 有多少个highway行
    TrajLNS highway_row(env, global_heuristictable, global_neighbors, num_highway_row);
    highway_row.init_mem();
    // cout << "partition agent num: " << highway_row.trajs.size() << endl; // 一开始agent num是知道的

    // cout << "grid 0 1: " << grid[1][0] << endl;
    // partition highway成对出现, 有向东则有向西, 有向北则有向南
    // 绘制横向highway。第一个横向highway是零行
    int highway_row_counter = 0;
    for (int i = 0; i < num_highway_row; i++)
    {
        // 如果是偶数行, 则起点列在左, 终点列在中间
        if (highway_row_counter % 2 == 0)
        {
            int approximate_start_column = 0;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols / 2 - 1;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else // 如果是奇数行, 则起点列在中间, 终点列在左
        {
            int approximate_start_column = env->cols / 2 - 1;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = 0;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_row.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_row.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_row.env, highway_row.flow, highway_row.heuristics[goal], highway_row.trajs[i],
            extra_weights, highway_row.mem, start, goal, &(highway_row.neighbors));

        if (!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_row_counter++;
        }

        // cout << "highway: ";
        for (int j = 0; j < highway_row.trajs[i].size() - 1; j++)
        {
            // cout << highway_row.trajs[i][j] << " ";
            int diff = highway_row.trajs[i][j + 1] - highway_row.trajs[i][j];
            // cout << "diff: " << diff << endl;
            // cout << get_d(diff, env) << endl;
            forbids[highway_row.trajs[i][j]].emplace_back(get_d(-diff, env)); // -diff是因为highway逆向是禁止向
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false);
        // break;
    }

    highway_row_counter = 0;
    for (int i = 0; i < num_highway_row; i++)
    {
        // 如果是偶数行, 则起点列在中间, 终点列在右
        if (highway_row_counter % 2 == 0)
        {
            int approximate_start_column = env->cols / 2 - 1;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols - 1;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else // 如果是奇数行, 则起点列在右, 终点列在中间
        {
            int approximate_start_column = env->cols - 1;
            int approximate_start_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols / 2 - 1;
            int approximate_goal_row = env->rows * (2 * i + 1) / (2 * num_highway_row) - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_row.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_row.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_row.env, highway_row.flow, highway_row.heuristics[goal], highway_row.trajs[i],
            extra_weights, highway_row.mem, start, goal, &(highway_row.neighbors));

        if (!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_row_counter++;
        }

        // cout << "highway: ";
        for (int j = 0; j < highway_row.trajs[i].size() - 1; j++)
        {
            // cout << highway_row.trajs[i][j] << " ";
            int diff = highway_row.trajs[i][j + 1] - highway_row.trajs[i][j];
            // cout << "diff: " << diff << endl;
            // cout << get_d(diff, env) << endl;
            forbids[highway_row.trajs[i][j]].emplace_back(get_d(-diff, env)); // -diff是因为highway逆向是禁止向
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false);
        // break;
    }
    //*/

    //*
    int num_highway_column = 8; // 有多少个highway列
    TrajLNS highway_column(env, global_heuristictable, global_neighbors, num_highway_column);
    highway_column.init_mem();
    // cout << "partition agent num: " << highway_row.trajs.size() << endl; // 一开始agent num是知道的

    // 绘制纵向highway
    int highway_column_counter = 0;
    for (int i = 0; i < num_highway_column; i++)
    {
        // 如果是偶数列, 则起点行在下, 终点行在中间
        if (highway_column_counter % 2 == 0)
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = env->rows - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = env->rows / 2 - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else // 如果是奇数列, 则起点行在中间, 终点行在下
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = env->rows / 2 - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = env->rows - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_column.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_column.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_column.env, highway_column.flow, highway_column.heuristics[goal], highway_column.trajs[i],
            extra_weights, highway_column.mem, start, goal, &(highway_column.neighbors));

        if (!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_column_counter++;
        }

        // cout << "highway: ";
        for (int j = 0; j < highway_column.trajs[i].size() - 1; j++)
        {
            int diff = highway_column.trajs[i][j + 1] - highway_column.trajs[i][j];
            forbids[highway_column.trajs[i][j]].emplace_back(get_d(-diff, env));
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false);
    }

    highway_column_counter = 0;
    for (int i = 0; i < num_highway_column; i++)
    {
        // 如果是偶数列, 则起点行在中间, 终点行在上
        if (highway_column_counter % 2 == 0)
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = env->rows / 2 - 1;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = 0;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }
        else // 如果是奇数列, 则起点行在上, 终点行在中间
        {
            int approximate_start_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_start_row = 0;
            true_start = find_nearest_non_obstacle(approximate_start_row, approximate_start_column);
            if (true_start.first == -1 && true_start.second == -1) {
                cout << "No non-obstacle start found.\n";
            }
            else {
                cout << "Nearest non-obstacle start: (" << true_start.first << ", " << true_start.second << ")\n";
            }

            int approximate_goal_column = env->cols * (2 * i + 1) / (2 * num_highway_column) - 1;
            int approximate_goal_row = env->rows / 2 - 1;
            true_goal = find_nearest_non_obstacle(approximate_goal_row, approximate_goal_column);
            if (true_goal.first == -1 && true_goal.second == -1) {
                cout << "No non-obstacle goal found.\n";
            }
            else {
                cout << "Nearest non-obstacle goal: (" << true_goal.first << ", " << true_goal.second << ")\n";
            }
        }

        int start = true_start.first * env->cols + true_start.second;
        int goal = true_goal.first * env->cols + true_goal.second;

        if (highway_column.heuristics.at(goal).empty())
        {
            // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
            init_heuristic(highway_column.heuristics[goal], env, goal);
        }

        bool highway_found = astar_highway(highway_column.env, highway_column.flow, highway_column.heuristics[goal], highway_column.trajs[i],
            extra_weights, highway_column.mem, start, goal, &(highway_column.neighbors));

        if (!highway_found)
        {
            cout << "high way not found" << endl;
            continue;
        }
        else
        {
            highway_column_counter++;
        }

        // cout << "highway: ";
        for (int j = 0; j < highway_column.trajs[i].size() - 1; j++)
        {
            int diff = highway_column.trajs[i][j + 1] - highway_column.trajs[i][j];
            forbids[highway_column.trajs[i][j]].emplace_back(get_d(-diff, env));
        }
        // cout << endl;

        // 为了避免highway中的对向车流, 每生成一个highway都要生成一遍forbids
        generate_extra_weights_from_forbids(false);
    }
    //*/

    generate_extra_weights_from_forbids(true);
}

/**
 * Plans a path using default planner
 * 
 * This function performs path planning within the timelimit given, and call the plan function in default planner.
 * The planned actions are output to the provided actions vector.
 * 
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - PLANNER_TIMELIMIT_TOLERANCE;

    // DefaultPlanner::plan(limit, actions, env);
    my_plan(limit, actions);
    // rhcr_plan(limit, actions);
}

void MAPFPlanner::my_plan(int time_limit,vector<Action> & _actions)
{
    // calculate the time planner should stop optimising traffic flows and return the plan.
    TimePoint start_time = std::chrono::steady_clock::now();
    // cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
    // traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
    TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);

    // record the initial location of each agent as dummy goals in case no goal is assigned to the agent.
    // 如果没有分配目标, agent将返回初始位置
    // 这可能意味着agent不单是避让其他有任务的agent, 可能会有冲突
    if (env->curr_timestep == 0){
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
        {
            dummy_goals.at(i) = env->curr_states.at(i).location;
        }
    }

    // data structure for record the previous decision of each agent
    prev_decision.clear();
    prev_decision.resize(env->map.size(), -1); // 根据之前每个agent的目标点, 每个地图点是否会被占据

    // update the status of each agent and prepare for planning
    int count = 0;
    for(int i=0; i<env->num_of_agents; i++)
    {
        // initialise the shortest distance heuristic table for the goal location of the agent
        // 路径启发式表的初始化
        if ( ( std::chrono::steady_clock::now() < end_time) )
        {
            for(int j=0; j<env->goal_locations[i].size(); j++)
            {
                int goal_loc = env->goal_locations[i][j].first;
                if (trajLNS.heuristics.at(goal_loc).empty())
                {
                    // 初始化地图中每个点和目标点的启发式距离, 目标点作为根节点
                    init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                    count++;
                }
            }
        }

        // set the goal location of each agent
        // 如果智能体没有目标，分配一个虚拟目标（初始位置）。
        // trajLNS.tasks[i] 表示第 i 个代理的目标位置
        if (env->goal_locations[i].empty()){
            trajLNS.tasks[i] = dummy_goals.at(i);
            priorities[i] = priorities_copy[i];
        }
        else
        {
            trajLNS.tasks[i] = env->goal_locations[i].front().first;
        }

        // check if the agent need a guide path update, when the agent has no guide path or the guide path does not end at the goal location
        // 如果路径引导 guide path 需要更新（目标发生变化），将 require_guide_path[i] 设置为 true
        require_guide_path[i] = false;
        if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
            require_guide_path[i] = true;

        // check if the agent completed the action in the previous timestep
        // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
        assert(env->curr_states[i].location >=0);
        prev_states[i] = env->curr_states[i]; // 每个agent下达指令前的状态
        next_states[i] = State();
        prev_decision[env->curr_states[i].location] = i;
        // 如果没有新下达新目标点, 就维持当前目标点
        if (decided[i].loc == -1){
            decided[i].loc = env->curr_states[i].location;
            assert(decided[i].state == DoneState::DONE);
        }
        // 如果指令地点和当前位置相同, 就标记完成
        if (prev_states[i].location == decided[i].loc){
            decided[i].state = DoneState::DONE; // DONE的意思是已经到达下达的指令地点
        }
        // 没有到达之前下达的指令地点
        if (decided[i].state == DoneState::NOT_DONE){
            decision.at(decided[i].loc) = i;
            next_states[i] = State(decided[i].loc,-1,-1);
        }

        // reset the PIBT priority if the agent reached previous goal location and switch to new goal location
        if(require_guide_path[i])
            priorities[i] = priorities_copy[i];
        else if (!env->goal_locations[i].empty())
            priorities[i] = priorities[i]+1; // 给没有被重新规划且有目标的agent更高的优先级

        // give priority bonus to the agent if the agent is in a deadend location
        // 给进入胡同的agent更高的优先级
        if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1)
        {
            priorities[i] = priorities[i] + 10;
            // priorities[i] = priorities[i] + 3;
        }
    }

    // compute the congestion minimised guide path for the agents that need guide path update
    // 仅更新需要路径更新的智能体的路径。
    for (int i = 0; i < env->num_of_agents;i++){
        if (std::chrono::steady_clock::now() >end_time)
            break;
        if (require_guide_path[i]){
            if (!trajLNS.trajs[i].empty())
                remove_traj(trajLNS, i);
            update_traj(trajLNS, extra_weights, i);
        }
    }

    // iterate and recompute the guide path to optimise traffic flow
    // 使用 Frank-Wolfe 算法迭代优化路径分配，从而优化交通流量，减少路径冲突。
    std::unordered_set<int> updated;
    frank_wolfe(trajLNS, extra_weights, updated,end_time);

    // sort agents based on the current priority
    // 将智能体按优先级排序，以优先处理需要高优先级更新的智能体。
    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                  return priorities.at(a) > priorities.at(b);
              }
    );

    // compute the targeted next location for each agent using PIBT
    // 使用 PIBT 计算每个智能体的下一步目标位置，避免冲突。需要用到优先级和路径权重
    for (int i : ids){
        if (decided[i].state == DoneState::NOT_DONE){
            continue;
        }
        if (next_states[i].location==-1){
            assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
            causalPIBT(i,-1,prev_states,next_states,
                       prev_decision,decision,
                       occupied, trajLNS);
        }
    }

    // post processing the targeted next location to turning or moving actions
    // 基于路径规划结果，生成智能体的具体行动，如前进（FW）、转向等。
    _actions.resize(env->num_of_agents);
    for (int id : ids){
        //clear the decision table based on which agent has next_states
        if (next_states.at(id).location!= -1)
            decision.at(next_states.at(id).location) = -1;
        // if agent is newly assigned a targeted next location, record the decision as not done yet
        if (next_states.at(id).location >=0){
            decided.at(id) = DCR({next_states.at(id).location,DoneState::NOT_DONE});
        }

        // post process the targeted next location to turning or moving actions
        _actions.at(id) = getAction(prev_states.at(id), decided.at(id).loc, env);
        checked.at(id) = false;

    }

    // recursively check if the FW action can be executed by checking whether all agents in the front of the agent can move forward
    // if any agent cannot move forward due to turning, all agents behind the turning agent will not move forward.
    for (int id=0;id < env->num_of_agents ; id++){
        if (!checked.at(id) && _actions.at(id) == Action::FW){
            moveCheck(id, checked, decided, _actions, prev_decision);
        }
    }

    // 记录每个地图点发生拥堵的次数
    for (int id=0;id < env->num_of_agents ; id++)
    {
        if(_actions.at(id) == Action::W)
        {
            cell_wait_count[env->curr_states[id].location]++;
            cell_wait_direction_count[env->curr_states[id].location][env->curr_states[id].orientation]++;
        }
    }

    prev_states = next_states; // 保存新状态

    // save wait count and wait direction count
    /*
    if(env->curr_timestep == 4999)
    {
        save_cell_wait_count();
    }
    */

    // 获取结束时间
    TimePoint real_end_time = std::chrono::steady_clock::now();

    // 计算运行时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(real_end_time - start_time);

    // 输出运行时间（以毫秒为单位）
    std::cout << "Path Plan time: " << duration.count() << " milliseconds" << std::endl;
}

void MAPFPlanner::rhcr_plan(int time_limit,vector<Action> & _actions)
{
    // calculate the time planner should stop optimising traffic flows and return the plan.
    TimePoint start_time = std::chrono::steady_clock::now();
    // cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
    // traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
    TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);

    // cout << 462 << endl;

    sorting_system->simulate_LoRR(vm["simulation_time"].as<int>(), env, _actions);

    // cout << 466 << endl;

    // record the initial location of each agent as dummy goals in case no goal is assigned to the agent.
    // 如果没有分配目标, agent将返回规划前位置
    // 这可能意味着agent不单是避让其他有任务的agent, 可能会有冲突
    if (env->curr_timestep == 0){
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
        {
            dummy_goals.at(i) = env->curr_states.at(i).location;
        }
    }

}


// Test on windows platform: 
// lifelong.exe --inputFile ../../../example_problems/random.domain/random_32_32_20_100.json -o test.json -s 5000 -t 1000
// lifelong.exe --inputFile ../../../example_problems/city.domain/paris_1_256_250.json -o test.json -s 5000 -t 1000
// lifelong.exe --inputFile ../../../example_problems/game.domain/brc202d_500.json -o test.json -s 5000 -t 1000
// lifelong.exe --inputFile ../../../example_problems/warehouse.domain/sortation_large_2000.json -o test.json -s 5000 -t 1000
// lifelong.exe --inputFile ../../../example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 5000 -t 1000
