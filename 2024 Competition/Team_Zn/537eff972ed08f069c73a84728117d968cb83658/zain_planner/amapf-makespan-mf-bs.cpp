#include <bits/stdc++.h>
#include "z_scheduler.h"

using namespace std;

namespace ZainPlanner
{

    const int INF = INT_MAX / 2;
    int map_width;
    int map_height;
    int map_size, number_of_free_cells;
    int T;
    int *heuristic;

    // Classes with operators:
    struct Edge
    {
        int cell_id, time_step;
        Edge(int cell_id, int time_step) : cell_id(cell_id), time_step(time_step) {}
        bool operator==(const Edge &other) const
        {
            return cell_id == other.cell_id && time_step == other.time_step;
        }
    };
    bool operator<(Edge a, Edge b)
    {
        return a.time_step != b.time_step ? a.time_step < b.time_step : a.cell_id < b.cell_id;
    }
    struct Interval
    {
        int l, h;
        Interval() {}
        Interval(int l, int h) : l(l), h(h) {}
    };
    struct TimestepCostPair
    {
        int time_step;
        bool blockedToMove; // Added to mark the nodes which are blocked to use the move actions because they are created in the infinite blocked interval of a goal cell (and no inverse edge is used yet).
        TimestepCostPair() {}
        TimestepCostPair(int time_step) : time_step(time_step)
        {
            blockedToMove = false;
        }
        TimestepCostPair(int time_step, bool blockedToMove) : time_step(time_step), blockedToMove(blockedToMove) {}
    };
    bool operator<(TimestepCostPair a, TimestepCostPair b)
    {
        return a.time_step < b.time_step;
    }
    struct Node
    {
        int parent, cell_id;
        TimestepCostPair timestep_cost_pair;
        Node() {}
        Node(TimestepCostPair timestep_cost_pair, int parent, int cell_id) : timestep_cost_pair(timestep_cost_pair), parent(parent), cell_id(cell_id) {}
    };
    bool operator<(Node a, Node b)
    { // inversed to inverse the default priority_queue
        return a.timestep_cost_pair.time_step + heuristic[a.cell_id] > b.timestep_cost_pair.time_step + heuristic[b.cell_id];
    }

    // storage::
    bool *map_cell; // cell=map_cell[id]: true -> obstacle, false -> free
    vector<int> *neighbors;
    int sink_id;
    vector<int> start_cells, goal_cells;
    bool *is_goal_cell;
    // The edges will be stored in the 'edges[destination]'.
    set<Edge> *edges;             // between other cells, it is neccessary to store the time step and the id of the next cell.
    set<TimestepCostPair> *visited_time_steps;
    set<int> *blocks;
    int *time_assigning_goal_cells;
    int *free_cells, *inverse_free_cells;

    void init1()
    {
        map_cell = new bool[map_size]();
        free_cells = new int[map_size]();
        inverse_free_cells = new int[map_size]();
    }
    void init2()
    {
        sink_id = number_of_free_cells;
        neighbors = new vector<int>[number_of_free_cells]();
        is_goal_cell = new bool[number_of_free_cells]();
        time_assigning_goal_cells = new int[number_of_free_cells]();
        edges = new set<Edge>[number_of_free_cells + 1]();
        visited_time_steps = new set<TimestepCostPair>[number_of_free_cells + 1]();
        blocks = new set<int>[number_of_free_cells + 1]();
        heuristic = new int[number_of_free_cells]();
    }
    void dynInit(int num_of_goals)
    {
    }
    void dynDel()
    {
    }
    void del()
    {
        delete[] map_cell;
        delete[] free_cells;
        delete[] inverse_free_cells;
        delete[] neighbors;
        delete[] is_goal_cell;
        delete[] time_assigning_goal_cells;
        delete[] edges;
        delete[] visited_time_steps;
        delete[] blocks;
    }

    inline bool isSink(int cell_id)
    {
        return cell_id == sink_id;
    }
    void compressFreeCells()
    {
        number_of_free_cells = 0;
        for (int i = 0; i < map_size; ++i)
        {
            if (!map_cell[i])
            {
                free_cells[i] = number_of_free_cells;
                inverse_free_cells[number_of_free_cells] = i;
                number_of_free_cells++;
            }
        }
        init2();
    }
    void genAllNeighbors()
    { // fill 'map' and 'is_goal_cell' sequentially (in 1D arrays), 'map_width' and 'map_height' and you are ready to call this function.
        int cnt = 0;
        int di[4] = {0, 0, 1, -1}, dj[4] = {1, -1, 0, 0};
        for (int i = 0; i < map_height; ++i)
        {
            for (int j = 0; j < map_width; ++j)
            {
                if (!map_cell[cnt])
                {
                    int cell = free_cells[cnt];
                    for (int k = 0; k < 4; ++k)
                    {
                        int ii = i + di[k];
                        int jj = j + dj[k];
                        int id = cnt + dj[k] + di[k] * map_width;
                        if (ii < map_height && ii >= 0 && jj < map_width && jj >= 0)
                        {
                            if (!map_cell[id])
                            {
                                neighbors[cell].push_back(free_cells[id]);
                            }
                        }
                    }
                }
                ++cnt;
            }
        }
    }

    void dynGenAllNeighbors(int num_of_goals)
    {
    }
    pair<bool, Interval> checkTimestepAndGetUnvisitedSafeInterval(TimestepCostPair timestep_cost_pair, int cell_id)
    {
        int prevBlock = -1, nxtBlock = T;
        auto itr = blocks[cell_id].lower_bound(timestep_cost_pair.time_step);
        if (itr != blocks[cell_id].end())
        {
            nxtBlock = *itr;
        }
        if (itr != blocks[cell_id].begin())
        {
            --itr;
            prevBlock = *itr;
        }
        int next_visited_timestep = T + 1;
        auto itrt = visited_time_steps[cell_id].upper_bound(TimestepCostPair(timestep_cost_pair.time_step));
        if (itrt != visited_time_steps[cell_id].begin())
        {
            --itrt;
            if (itrt->time_step > prevBlock)
            {
                return make_pair(false, Interval());
            }
            ++itrt;
        }

        if (itrt != visited_time_steps[cell_id].end() && itrt->time_step <= nxtBlock)
        {
            next_visited_timestep = itrt->time_step;
            return {true, Interval(timestep_cost_pair.time_step, next_visited_timestep - 1)};
        }
        return {true, Interval(timestep_cost_pair.time_step, nxtBlock)};
    }
    bool checkTimestep(TimestepCostPair timestep_cost_pair, int cell_id)
    {
        int prevBlock = -1;
        auto itr = blocks[cell_id].lower_bound(timestep_cost_pair.time_step);
        if (itr != blocks[cell_id].begin())
        {
            --itr;
            prevBlock = *itr;
        }
        auto itrt = visited_time_steps[cell_id].upper_bound(TimestepCostPair(timestep_cost_pair.time_step));
        if (itrt != visited_time_steps[cell_id].begin())
        {
            --itrt;
            if (itrt->time_step > prevBlock) //&& prvCost <= timestep_cost_pair.cost - timestep_cost_pair.time_step)
            {
                return false;
            }
        }
        return true;
    }
    bool isThereAnEdge(int source, int destination, int time_step)
    {
        return edges[destination].count(Edge(source, time_step));
    }
    void visit(int cell_id, TimestepCostPair timestep_cost_pair)
    {
        visited_time_steps[cell_id].insert(timestep_cost_pair);
    }
    void getAllTimeSteps(int lowestTimeStep, int highestTimeStep, int cell_id, vector<pair<int, bool>> &validtimeStepsWithMoveBlocked)
    {
        if (is_goal_cell[cell_id] && time_assigning_goal_cells[cell_id] < INF)
        {
            for (auto &it : blocks[cell_id])
            {
                if (lowestTimeStep > highestTimeStep)
                    break;
                if (it >= lowestTimeStep)
                {
                    validtimeStepsWithMoveBlocked.push_back(make_pair(lowestTimeStep, false));
                    lowestTimeStep = it + 2;
                }
            }
            if (lowestTimeStep <= highestTimeStep)
            {
                validtimeStepsWithMoveBlocked.push_back(make_pair(lowestTimeStep, true));
            }
        }
        else
        {
            for (auto &it : blocks[cell_id])
            {
                if (lowestTimeStep > highestTimeStep)
                    break;
                if (it >= lowestTimeStep)
                {
                    validtimeStepsWithMoveBlocked.push_back(make_pair(lowestTimeStep, false));
                    lowestTimeStep = it + 2;
                }
            }
            if (lowestTimeStep <= highestTimeStep)
            {
                validtimeStepsWithMoveBlocked.push_back(make_pair(lowestTimeStep, false));
            }
        }
    }
    void getAllTimestepCostPairs(int cellSource, int cellDestination, Interval it, vector<TimestepCostPair> &result)
    {
        vector<pair<int, bool>> timeStepsWithMoveBlocked; // When we move to the infinite blocked interval of a goal cell (the last waiting before connecting to sink node), we don't allow the resulting node to make move actions.
        int lowestTimeStep, highestTimeStep;
        if (it.h % 2 == 0) // the high time step is in the copy 0 of the layer.
        {
            if (isThereAnEdge(cellDestination, cellSource, it.h - 1))
            {
                timeStepsWithMoveBlocked.push_back(make_pair(it.h - 1, it.h - 1 >= time_assigning_goal_cells[cellDestination]));
            }
            highestTimeStep = it.h;
        }
        else
        {
            if (isThereAnEdge(cellSource, cellDestination, it.h))
            {
                highestTimeStep = it.h - 1;
            }
            else
            {
                highestTimeStep = it.h + 1;
            }
        }
        if (it.l % 2)
        {
            if (isThereAnEdge(cellSource, cellDestination, it.l))
            {
                lowestTimeStep = it.l + 3;
            }
            else
            {
                lowestTimeStep = it.l + 1;
            }
        }
        else
        {
            if (isThereAnEdge(cellDestination, cellSource, it.l - 1))
                lowestTimeStep = it.l - 1;
            else
                lowestTimeStep = it.l + 2;
        }
        if (highestTimeStep >= lowestTimeStep)
        {
            getAllTimeSteps(lowestTimeStep, highestTimeStep, cellDestination, timeStepsWithMoveBlocked);
        }
        for (auto t : timeStepsWithMoveBlocked)
        {
            result.push_back(TimestepCostPair(t.first, t.second));
        }
    }
        void addPathEdges(vector<Node> &nodes, vector<int> &sink_nodes_id, int &number_of_remained_paths) // the second step in SSPA method.
    {
        bool *marked = new bool[nodes.size()]();
        for (auto sink_node_id : sink_nodes_id)
        {
            Node node1, node2 = nodes[sink_node_id];
            int par = node2.parent;
            bool flag_failed = false;
            vector<Node> path_nodes;
            path_nodes.push_back(node2);
            while (par != -1)
            {
                node1 = nodes[par];
                if (marked[par])
                {
                    flag_failed = true;
                    break;
                }
                marked[par] = true;

                Node node_tmp = node1;
                if (node1.timestep_cost_pair.time_step <= node2.timestep_cost_pair.time_step)
                {
                    if (node2.timestep_cost_pair.time_step % 2)
                    { // Case when go to the upper bound of the time interval then go downward using negative edge.
                        for (int time_step = node2.timestep_cost_pair.time_step + 1; time_step >= node1.timestep_cost_pair.time_step; time_step--)
                        {
                            node_tmp.timestep_cost_pair.time_step = time_step;
                            path_nodes.push_back(node_tmp);
                        }
                    }
                    else
                    {
                        for (int time_step = node2.timestep_cost_pair.time_step - 1; time_step >= node1.timestep_cost_pair.time_step; time_step--)
                        {
                            node_tmp.timestep_cost_pair.time_step = time_step;
                            path_nodes.push_back(node_tmp);
                        }
                    }
                }
                else
                {
                    for (int time_step = node2.timestep_cost_pair.time_step + 1; time_step <= node1.timestep_cost_pair.time_step; time_step++)
                    {
                        node_tmp.timestep_cost_pair.time_step = time_step;
                        path_nodes.push_back(node_tmp);
                    }
                }
                node2 = node1;
                par = node1.parent;
            }
            if (flag_failed)
                continue;
            --number_of_remained_paths;
            std::reverse(path_nodes.begin(), path_nodes.end());
            for (int i = 0; i + 1 < path_nodes.size(); ++i)
            {
                node1 = path_nodes[i];
                node2 = path_nodes[i + 1];
                if (is_goal_cell[node1.cell_id])
                {
                    if (time_assigning_goal_cells[node1.cell_id] < node1.timestep_cost_pair.time_step)
                    {
                        for (int t = time_assigning_goal_cells[node1.cell_id] + 1; t <= node1.timestep_cost_pair.time_step; ++t)
                        {
                            edges[node1.cell_id].insert({node1.cell_id, t});
                            blocks[node1.cell_id].insert(t);
                        }
                        time_assigning_goal_cells[node1.cell_id] = node1.timestep_cost_pair.time_step;
                    }
                }
                if (node1.timestep_cost_pair.time_step > node2.timestep_cost_pair.time_step)
                { // --> the edges is between map-cells (none-hub-none-goal nodes).
                    edges[node1.cell_id].erase({node2.cell_id, node2.timestep_cost_pair.time_step});
                    if (node1.cell_id == node2.cell_id)
                    {
                        blocks[node1.cell_id].erase(node2.timestep_cost_pair.time_step);
                    }
                }
                else // if (node1.timestep_cost_pair.time_step < node2.timestep_cost_pair.time_step)
                {    // --> the edge is between map-cells (none-hub-none-goal nodes).
                    edges[node2.cell_id].insert({node1.cell_id, node1.timestep_cost_pair.time_step});
                    if (node1.cell_id == node2.cell_id)
                    {
                        blocks[node1.cell_id].insert(node1.timestep_cost_pair.time_step);
                    }
                }
            }
            blocks[node2.cell_id].insert(node2.timestep_cost_pair.time_step);
            edges[node2.cell_id].insert({node2.cell_id, node2.timestep_cost_pair.time_step});
            time_assigning_goal_cells[node2.cell_id] = node2.timestep_cost_pair.time_step;
            // cout<<"T = "<<T<<", assign = "<<time_assigning_goal_cells[node2.cell_id]<<endl;
        }
        delete[] marked;
    }

    vector<Node> nodes;
    priority_queue<Node> q;
    void partialReset()
    {
        for (int i = 0; i < number_of_free_cells; ++i)
        {
            blocks[i].clear();
            edges[i].clear();
            time_assigning_goal_cells[i] = INF; // This value means that this goal is not used yet.
        }
    }
    void partialResetKeepPaths()
    {
        // for (int i = 0; i < number_of_free_cells; ++i)
        // {
        //     if(time_assigning_goal_cells[i]<INF){
        //         blocks[i].erase(time_assigning_goal_cells[i]);
        //         edges[i].erase({i, time_assigning_goal_cells[i]});
        //         time_assigning_goal_cells[i] = T+1;
        //         blocks[i].insert(T+1);
        //         edges[i].insert({i, T+1});
        //     }
        // }
    }
    void calHeuristics()
    {
        queue<pair<int, int>> q;
        for (int i = 0; i < number_of_free_cells; ++i)
        {
            if (is_goal_cell[i] && time_assigning_goal_cells[i] == INF)
            {
                q.push({i, 0});
            }
            heuristic[i] = -1;
        }
        while (!q.empty())
        {
            auto tmp = q.front();
            q.pop();
            if (heuristic[tmp.first] != -1)
            {
                continue;
            }
            heuristic[tmp.first] = tmp.second;
            for (auto it : neighbors[tmp.first])
            {
                if (heuristic[it] == -1)
                {
                    q.push({it, tmp.second + 1});
                }
            }
        }
    }
    int remainingGoals;
    int solveT(int min_T)
    {
        T = min_T * 2 - 1;
        while (remainingGoals)
        {
            // cout<<"Agents remaining: "<<remainingGoals<<" "<<endl;
            while (!q.empty())
            {
                q.pop();
            }
            nodes.clear();
            for (auto it : start_cells)
            {
                q.push(Node(TimestepCostPair(0, 0), -1, it));
            }
            vector<int> sink_nodes_ids;
            for (int i = 0; i <= sink_id; ++i)
            {
                visited_time_steps[i].clear();
            }
            while (!q.empty())
            {
                Node tp = q.top();
                int nodeID = nodes.size();
                nodes.push_back(tp);
                q.pop();
                int currentCell = tp.cell_id;
                TimestepCostPair timestep_cost_pair = tp.timestep_cost_pair;
                pair<bool, Interval> res = checkTimestepAndGetUnvisitedSafeInterval(timestep_cost_pair, currentCell);
                // cout<<currentCell<<" "<<timestep_cost_pair.time_step<<" "<<res.first<<endl;
                if (!res.first)
                { // visited time step in ths cell.
                    continue;
                }
                if (is_goal_cell[currentCell] && time_assigning_goal_cells[currentCell] == INF && res.second.h == T)
                {
                    sink_nodes_ids.push_back(nodeID);
                    visit(currentCell, timestep_cost_pair);
                    continue;
                    // break;
                }
                visit(currentCell, timestep_cost_pair);
                vector<int> &nebs = neighbors[currentCell];
                Interval currentInterval = res.second;
                // special edge: inverse of wait action.
                if (isThereAnEdge(currentCell, currentCell, currentInterval.l - 1) || currentInterval.l - 1 >= time_assigning_goal_cells[currentCell])
                {
                    q.push(Node(TimestepCostPair(currentInterval.l - 1, false), nodeID, currentCell));
                }
                // special edge: infinite node (came from infinite interval and projected to infinite blocked interval).
                if (tp.timestep_cost_pair.blockedToMove)
                {
                    if (timestep_cost_pair.time_step + 2 <= currentInterval.h)
                    {
                        q.push(Node(TimestepCostPair(timestep_cost_pair.time_step + 2, true), tp.parent, currentCell));
                    }
                }
                else
                {
                    for (auto neborCell : nebs)
                    {
                        vector<TimestepCostPair> result;
                        getAllTimestepCostPairs(currentCell, neborCell, currentInterval, result);
                        for (auto it : result)
                        {
                            if (checkTimestep(it, neborCell))
                                q.push(Node(it, nodeID, neborCell));
                        }
                    }
                }
            }
            if (sink_nodes_ids.size() == 0)
            {
                return false;
            }
            addPathEdges(nodes, sink_nodes_ids, remainingGoals);
        }
        return true;
    }
    void dfs(Edge edge, int &start)
    {
        start = edge.cell_id;
        for (auto it : edges[edge.cell_id])
        {
            if (it.time_step == edge.time_step - 1)
            {
                dfs(it, start);
                return;
            }
        }
    }
    void getAssignments(vector<int> &assignments)
    {
        assignments.resize(map_size, -1);
        for (auto goal_cell : goal_cells)
        {
            int start;
            if (time_assigning_goal_cells[goal_cell] < INF)
            {
                Edge last_edge = Edge(goal_cell, time_assigning_goal_cells[goal_cell]);
                dfs(last_edge, start);
                assignments[inverse_free_cells[start]] = inverse_free_cells[last_edge.cell_id];
            }
        }
        // exit(0);
    }
    void readMap(SharedEnvironment *env)
    {
        map_height = env->rows;
        map_width = env->cols;
        map_size = env->cols * env->rows;
        init1();
        int cnt = 0;
        for (int i = 0; i < map_size; ++i)
        {
            map_cell[i] = env->map[i];
        }
    }
    void print(int cell)
    {
        cout << cell / map_width << " " << cell % map_width << endl;
    }
    void fillInputs(vector<int> &agents_locations, vector<pair<int, int>> &task_locations_and_costs)
    {
        start_cells.clear();
        for (auto it : agents_locations)
        {
            start_cells.push_back(free_cells[it]);
        }
        for (auto it : goal_cells)
        {
            is_goal_cell[it] = false;
        }
        goal_cells.clear();
        for (auto it : task_locations_and_costs)
        {
            is_goal_cell[free_cells[it.first]] = 1;
            goal_cells.push_back(free_cells[it.first]);
        }
        calHeuristics();
    }
    void dynReset()
    {
    }

    int solve()
    {
        int st = 1, nd = 1, tmp = INF;
        for(auto it:goal_cells){
            tmp = INF;
            for(auto it1:start_cells){
                nd = max(nd, distancesBetweenCells[it][it1]);
                tmp = min(tmp, distancesBetweenCells[it][it1]);
            }
            st = max(st, tmp);
        }
        partialReset();
        remainingGoals = min(start_cells.size(), goal_cells.size());
        while(remainingGoals){
            solveT(st);
            cout<<st<<": "<<remainingGoals<<endl;
            ++st;
        }
        // while (st < nd)
        // {
        //     int md = (st + nd) / 2;
        //     bool l = solveT(md);
        //     cout << md << ":::" << l << endl;
        //     if (l)
        //     {
        //         nd = md;
        //         partialReset();
        //         remainingGoals = min(start_cells.size(), goal_cells.size());
        //     }
        //     else
        //     {
        //         st = md + 1;
        //         // partialResetKeepPaths();
        //         // remainingGoals = min(start_cells.size(), goal_cells.size());
        //     }
        // }
        // bool problem_is_solved = solveT(nd);
        // assert(problem_is_solved == true);
        return st;
    }
}

namespace std
{
    template <>
    struct hash<ZainPlanner::Edge>
    {
        auto operator()(const ZainPlanner::Edge &e) const -> size_t
        {
            return e.time_step * ZainPlanner::number_of_free_cells + e.cell_id;
        }
    };
}