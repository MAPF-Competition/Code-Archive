#include <RandomPlanner2.h>
#include <random>
#include <scc_dag_random.h>
using namespace std::chrono;

struct AstarNode
{
    int location;
    int direction;
    int f, g, h;
    AstarNode *parent;
    AstarNode(int _location, int _direction, int _g, int _h, AstarNode *_parent) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent) {}
};

struct cmp
{
    bool operator()(AstarNode *a, AstarNode *b)
    {
        if (a->g * 70 + a->h == b->g * 70 + b->h)
            return a->h > b->h;
        else
            return a->g * 70 + a->h > b->g * 70 + b->h;
    }
};

void RandomPlanner2::initialize(int preprocess_time_limit)
{
    shift_id = 0;
    flagReplan = false;
    W1 = 1;
    W2 = 70;
    max_time_limit = 970000;
    timeStep = -1;
    freeCells = new int[env->map.size()]();
    compressFreeCells();
    genAllNeighors();
}

void RandomPlanner2::compressFreeCells()
{
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows * env->cols; ++i)
    {
        if (!env->map[i])
        {
            freeCells[i] = numOfFreeCells;
            ++numOfFreeCells;
        }
    }
    mapFree = new bool[numOfFreeCells]();
    nebors = new int[numOfFreeCells * 4]();
    passes = new int[numOfFreeCells * 4]();
    emergency_nebors = new int[numOfFreeCells * 4]();
    closed_list = new bool[numOfFreeCells * 4]();
    occupied = new int[numOfFreeCells]();
    future_occupied = new int[numOfFreeCells]();
    nxt_occupied = new vector<int>[numOfFreeCells]();
    row = new int[numOfFreeCells]();
    col = new int[numOfFreeCells]();
    isGoal = new bool[numOfFreeCells]();
    dangerousPlaceFree = new bool[numOfFreeCells]();
    escape_place = new int[numOfFreeCells]();
    is_escape_place = new bool[numOfFreeCells]();
    cellCost = new int[numOfFreeCells];
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows * env->cols; ++i)
    {
        if (!env->map[i])
        {
            row[numOfFreeCells] = i / env->cols;
            col[numOfFreeCells] = i % env->cols;
            ++numOfFreeCells;
        }
    }
}
void RandomPlanner2::genAllNeighors()
{
    int cnt = 0;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            int cell = i * m + j;
            if (!env->map[cell])
            {
                int cnt = 0, nxt;
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
                    {
                        ++cnt;
                        nxt = freeCells[ii * env->cols + jj];
                        emergency_nebors[freeCells[cell] * 4 + k] = nxt;
                        nebors[freeCells[cell] * 4 + k] = nxt;
                    }
                    else
                    {
                        nebors[freeCells[cell] * 4 + k] = -1;
                        emergency_nebors[freeCells[cell] * 4 + k] = -1;
                    }
                }
                if (cnt == 1)
                {
                    dangerousPlaceFree[freeCells[cell]] = 1;
                    escape_place[freeCells[cell]] = nxt;
                    is_escape_place[nxt] = 0;
                }
            }
        }
    }
}
void RandomPlanner2::dfs_fill_sizes(int x)
{
    if (sz[x])
    {
        return;
    }
    sz[x] = 1; //! occupied[goal_locations[x]];
    if (dangerousPlaceFree[curr_states[x].first])
        sz[x] = 50;
    int mx = 0;
    for (auto it : adj[x])
    {
        dfs_fill_sizes(it);
        mx = max(mx, sz[it]);
    }
    sz[x] += mx;
}
void RandomPlanner2::dfs_mark_valids(int x)
{
    allowedToMove[x] = 1;
    if (adj[x].size() == 0)
        return;
    int mx_id = adj[x][0], mx = sz[adj[x][0]];
    for (auto it : adj[x])
    {
        if ((sz[it] > mx || isForbiddenToMove(mx_id)) && (!isForbiddenToMove(it)))
        {
            mx_id = it;
            mx = sz[it];
        }
    }
    if (!isForbiddenToMove(mx_id))
        dfs_mark_valids(mx_id);
}
bool RandomPlanner2::dfs_fill_cycles(int x, int pa)
{
    if (vis_local[x] == 1)
    {
        allowedToMove[x] = 1;
        return true;
    }
    if (vis[x])
        return false;
    vis[x] = 1;
    vis_local[x] = 1;
    for (auto it : adj[x])
    {
        if (it == x)
        { // maybe no need to this. not tested yet.
            allowedToMove[x] = 1;
            return true;
        }
        if (it != pa)
        {
            if (dfs_fill_cycles(it, x))
            {
                allowedToMove[x] = 1;
                return true;
            }
        }
    }
    vis_local[x] = false;
    return false;
}
void RandomPlanner2::markWhoAllowedToMove()
{
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        allowedToMove[i] = 0;
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (!occupied[old_paths[i].front().first])
        {
            nxt_occupied[old_paths[i].front().first].push_back(i + 1);
        }
    }
    vector<int> cnt(env->num_of_agents, 0);
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (occupied[old_paths[i].front().first])
        {
            adj[occupied[old_paths[i].front().first] - 1].push_back(i);
            ++cnt[i];
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        // assert(nxt_occupied[old_paths[i].front().first].size()>0);
        if (nxt_occupied[old_paths[i].front().first].size() < 2)
            continue;
        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            ++cnt[it - 1];
            dfs_fill_sizes(it - 1);
        }
        int mx, mx_id;
        mx_id = nxt_occupied[old_paths[i].front().first][0] - 1;
        mx = sz[mx_id];

        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            if (((sz[it - 1] > mx) || isForbiddenToMove(mx_id)) && (!isForbiddenToMove(it - 1)))
            {
                mx_id = it - 1;
                mx = sz[it - 1];
            }
        }
        if (!isForbiddenToMove(mx_id))
            dfs_mark_valids(mx_id);
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (cnt[i] == 0)
        {
            dfs_fill_sizes(i);
            dfs_mark_valids(i);
        }
        else
        {
            dfs_fill_cycles(i, -1);
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (old_paths[i].front().first == curr_states[i].first)
        {
            if (allowedToMove[i] != 1)
                exit(0);
        }
        vis[i] = 0;
        vis_local[i] = 0;
        sz[i] = 0;
        nxt_occupied[old_paths[i].front().first].clear();
        adj[i].clear();
    }
}
void RandomPlanner2::writeMap()
{
    std::ofstream out;
    out.open("map.txt");
    bool l = 1;
    int m = env->cols;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (env->map[i * m + j])
            {
                if (!l)
                    out << " ";
                l = 0;
                out << j;
            }
        }
    }
    out << endl;
    l = 1;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (env->map[i * m + j])
            {
                if (!l)
                    out << " ";
                l = 0;
                out << env->rows - 1 - i;
            }
        }
    }
    out << endl;
    l = 1;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (!env->map[i * m + j])
            {
                if (!l)
                    out << " ";
                l = 0;
                out << j;
            }
        }
    }
    out << endl;
    l = 1;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (!env->map[i * m + j])
            {
                if (!l)
                    out << " ";
                out << env->rows - 1 - i;
                l = 0;
            }
        }
    }
    out << endl;
    out << scc_random::cyc << endl;
    for (int i = 0; i < scc_random::cyc; ++i)
    {
        l = 1;
        for (int j = 0; j < env->rows; ++j)
        {
            for (int k = 0; k < env->cols; ++k)
            {
                if (env->map[j * env->cols + k] == 0 && scc_random::comp_depth[scc_random::sccs[freeCells[j * env->cols + k]]] == i)
                {
                    if (!l)
                        out << " ";
                    l = 0;
                    out << k;
                }
            }
        }
        out << endl;
        l = 1;
        for (int j = 0; j < env->rows; ++j)
        {
            for (int k = 0; k < env->cols; ++k)
            {
                if (env->map[j * env->cols + k] == 0 && scc_random::comp_depth[scc_random::sccs[freeCells[j * env->cols + k]]] == i)
                {
                    if (!l)
                        out << " ";
                    l = 0;
                    out << env->rows - 1 - j;
                }
            }
        }
        out << endl;
    }
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (env->map[i * env->cols + j])
                continue;
            int cell = freeCells[i * env->cols + j];
            int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
            for (int k = 0; k < 4; ++k)
            {
                if (nebors[cell * 4 + k] != -1 && passes[cell * 4 + k] > 0)
                {
                    // if(dx[k]<0 || dy[k]<0)
                    //     out<<i+dx[k]<<" "<<j+dy[k]<<" "<<i<<" "<<j<<endl;
                    // else
                    out << j << " " << env->rows - 1 - i << " " << j + dy[k] << " " << env->rows - 1 - i - dx[k] << endl;
                }
            }
        }
    }
    out.close();
}

void RandomPlanner2::writePaths()
{
    std::ofstream out;
    out.open("paths.txt");
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        out << old_paths[i].size() + 1 << " ";
        out << col[curr_states[i].first] << " " << env->rows - 1 - row[curr_states[i].first] << " ";
        for (auto it : old_paths[i])
        {
            out << col[it.first] << " " << env->rows - 1 - row[it.first];
            if (it != old_paths[i].back())
                out << " ";
        }
        out << endl;
    }
    out.close();
}

void RandomPlanner2::writeGaolPositions()
{
    std::ofstream out;
    out.open("goals" + to_string(timeStep) + ".txt");
    map<int, int> mp;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        ++mp[goal_locations[i]];
        out << col[curr_states[i].first] << " " << env->rows - 1 - row[curr_states[i].first] << " " << path_cost(i, old_paths[i]).first << " " << path_cost(i, old_paths[i]).second << endl;
    }
    // for(int i=0; i<env->rows; ++i){
    //     for(int j=0; j<env->cols; ++j){
    //         if(mp[freeCells[i*env->cols+j]]>0){
    //             out<<j<<" "<<env->rows-1-i<<" "<<mp[freeCells[i*env->cols+j]]<<endl;
    //         }
    //     }
    // }
    out.close();
}
void RandomPlanner2::writePathsSteps()
{
    std::ofstream out;
    out.open("paths_steps" + to_string(timeStep) + ".txt");
    out << env->num_of_agents << endl;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        out << col[curr_states[i].first] << " " << env->rows - 1 - row[curr_states[i].first] << " ";
        out << col[old_paths[i][0].first] << " " << env->rows - 1 - row[old_paths[i][0].first];
        out << endl;
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (allowedToMove[i])
        {
            out << col[curr_states[i].first] << " " << env->rows - 1 - row[curr_states[i].first] << endl;
        }
    }
    out.close();
}
void RandomPlanner2::process_path(int start, deque<pair<int, int>> &path)
{
    int tmp1 = 0;
    for (int i = 0; i + 1 < path.size(); ++i)
    {
        if (path[i].first == start)
        {
            tmp1 = i;
        }
    }
    for (int i = 0; i < tmp1; ++i)
    {
        path[i].first = start;
    }
    for (int i = tmp1; i < path.size(); ++i)
    {
        int tmp = i;
        for (int j = i + 1; j < path.size(); ++j)
        {
            if (path[j].first == path[i].first)
            {
                tmp = j;
            }
        }
        for (int j = i + 1; j <= tmp; ++j)
        {
            path[j].first = path[i].first;
        }
        i = tmp;
    }
}

bool RandomPlanner2::isForbiddenToMove(int agent)
{
    return dangerousPlaceFree[goal_locations[agent]] && occupied[goal_locations[agent]] &&
           escape_place[goal_locations[agent]] == old_paths[agent].front().first;
}
void RandomPlanner2::sortAgents()
{
    const int partition = 8;
    vector<pair<pair<int, int>, int>> v[partition][partition];
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        v[row[goal_locations[i]] / ((env->rows) / partition)][col[goal_locations[i]] / ((env->cols) / partition)].push_back({{min(min(row[goal_locations[i]], env->rows - 1 - row[goal_locations[i]]), min(col[goal_locations[i]], env->cols - 1 - col[goal_locations[i]])),
                                                                                                                              abs(row[goal_locations[i]] - row[curr_states[i].first]) + abs(col[goal_locations[i]] - col[curr_states[i].first])},
                                                                                                                             i});
    }
    for (int i = 0; i < partition; ++i)
    {
        for (int j = 0; j < partition; ++j)
        {
            sort(v[i][j].begin(), v[i][j].end());
            reverse(v[i][j].begin(), v[i][j].end());
        }
    }
    set<pair<int, int>> set_ids;
    vector<pair<int, int>> ids;
    pair<int, int> start_pairs[4] = {{partition / 2, 0}, {0, partition / 2}, {partition / 2, partition - 1}, {partition - 1, partition / 2}};
    ids.push_back(start_pairs[0]);
    set_ids.insert(start_pairs[0]);
    while (ids.size() < partition * partition)
    {
        int x = ids.back().first;
        int y = ids.back().second;
        int mx = -1;
        pair<int, int> tmp_pair;
        for (int i = 0; i < partition; ++i)
        {
            for (int j = 0; j < partition; ++j)
            {
                if (set_ids.find({i, j}) != set_ids.end())
                    continue;
                int dis = abs(i - x) + abs(j - y);
                if (dis > mx)
                {
                    mx = dis;
                    tmp_pair = {i, j};
                }
            }
        }
        ids.push_back(tmp_pair);
        set_ids.insert(tmp_pair);
    }
    int cnt = 0;
    agents_vector.clear();
    while (agents_vector.size() < env->num_of_agents)
    {
        int i = ids[cnt].first;
        int j = ids[cnt].second;
        if (v[i][j].size() > 0)
        {
            agents_vector.push_back(v[i][j].back());
            v[i][j].pop_back();
        }
        cnt = (cnt + 1) % (partition * partition);
    }
}
void RandomPlanner2::checkPasses()
{
    int tmp = 0;

    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (curr_states[i].first != old_paths[i][0].first)
        {
            ++tmp;
        }
        for (int j = 0; j + 1 < old_paths[i].size(); ++j)
        {
            tmp += old_paths[i][j].first != old_paths[i][j + 1].first;
        }
    }
    for (int i = 0; i < numOfFreeCells; ++i)
    {
        for (int k = 0; k < 4; ++k)
        {
            tmp -= passes[i * 4 + k];
        }
    }
    assert(tmp == 0);
    for (int i = 0; i < numOfFreeCells; ++i)
    {
        tmp += future_occupied[i];
    }
    assert(tmp == env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (old_paths[i].size() == 0)
        {
            assert(future_occupied[curr_states[i].first] > 0);
        }
        else
        {
            assert(future_occupied[old_paths[i][0].first] > 0);
        }
    }
}
// plan using simple A* that ignores the time dimension
void RandomPlanner2::plan(int time_limit, vector<Action> &actions)
{
    start_plan_time = high_resolution_clock::now();
    ++timeStep;
    if (timeStep == 0)
    {
        old_paths = new deque<pair<int, int>>[env->num_of_agents]();
        curr_states = new pair<int, int>[env->num_of_agents]();
        goal_locations = new int[env->num_of_agents]();
        adj = new vector<int>[env->num_of_agents]();
        sz = new int[env->num_of_agents]();
        vis = new bool[env->num_of_agents]();
        vis_local = new bool[env->num_of_agents]();
        allowedToMove = new bool[env->num_of_agents]();
        ignore = new bool[env->num_of_agents]();
        solved = new bool[env->num_of_agents]();
        shift_id = 0;
        agents_vector.resize(env->num_of_agents);
    }
    bool reset = false;
    if (timeStep > 0)
    {
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            occupied[curr_states[i].first] = 0;
            if (goal_locations[i] != -1)
            {
                isGoal[goal_locations[i]] = 0;
            }
            if (old_paths[i].size() > 0)
            {
                future_occupied[old_paths[i][0].first]--;
            }
            else
            {
                future_occupied[curr_states[i].first]--;
            }
        }
    }
    // map<int, int> mp;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        curr_states[i] = make_pair(freeCells[env->curr_states[i].location], env->curr_states[i].orientation);
        occupied[curr_states[i].first] = i + 1;
        if (env->goal_locations[i].empty())
        {
            goal_locations[i] = -1;
        }
        else
        {
            if (goal_locations[i] != freeCells[env->goal_locations[i].front().first])
            {
                reset = true;
            }
            goal_locations[i] = freeCells[env->goal_locations[i].front().first];
            isGoal[goal_locations[i]] = 1;
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        if (allowedToMove[i] && it.size() > 0)
        {
            if ((it.front().first != curr_states[i].first) || (it.front().second != curr_states[i].second))
            {
                actions = lastActions;
                --timeStep;
                return;
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (allowedToMove[i] && old_paths[i].size() > 0)
        {
            if (old_paths[i].back().first != old_paths[i].front().first)
                assert(nebors[old_paths[i].back().first * 4 + old_paths[i].back().second] == old_paths[i].front().first);
            old_paths[i].push_back(old_paths[i].front());
            old_paths[i].pop_front();
        }
        if (old_paths[i].size() > 0)
        {
            future_occupied[old_paths[i][0].first]++;
        }
        else
        {
            future_occupied[curr_states[i].first]++;
        }
    }
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W);
    if (timeStep == 0 || reset)
    {
        sortAgents();
    }

    int num_of_clears = 100;
    for (int i = shift_id; i < min(env->num_of_agents, shift_id + num_of_clears); ++i)
    {
        if (old_paths[i].size() > 0)
        {
            future_occupied[old_paths[i][0].first]--;
            future_occupied[curr_states[i].first]++;
        }
        if (old_paths[i].size() > 0 && curr_states[i].first != old_paths[i].front().first)
        {
            auto it = curr_states[i];
            auto nxt = old_paths[i].front();
            if (it.first != nxt.first)
            {
                --passes[it.first * 4 + it.second];
            }
        }
        for (int j = 0; j + 1 < old_paths[i].size(); ++j)
        {
            auto it = old_paths[i][j];
            auto nxt = old_paths[i][j + 1];
            if (it.first != nxt.first)
            {
                --passes[it.first * 4 + it.second];
            }
        }
        old_paths[i].clear();
    }
    shift_id = (shift_id + num_of_clears);
    if (shift_id >= env->num_of_agents)
        shift_id = 0;

    for (int i = 0; i < numOfFreeCells; ++i)
    {
        cellCost[i] = 0;
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        int cnt = 0;
        ++cellCost[curr_states[i].first];
        for (auto it : old_paths[i])
        {
            ++cnt;
            ++cellCost[it.first];
            if (cnt == 5)
                break;
        }
    }
    for (int repeat = 0; true; ++repeat)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        for (int ii = 0; ii < env->num_of_agents; ++ii)
        {
            if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
            {
                break;
            }
            int i = agents_vector[ii].second;
            if (old_paths[i].size() > 0 && curr_states[i].first != old_paths[i].front().first)
            {
                auto it = curr_states[i];
                auto nxt = old_paths[i].front();
                if (it.first != nxt.first)
                {
                    --passes[it.first * 4 + it.second];
                }
            }
            for (int j = 0; j + 1 < old_paths[i].size(); ++j)
            {
                auto it = old_paths[i][j];
                auto nxt = old_paths[i][j + 1];
                if (it.first != nxt.first)
                {
                    --passes[it.first * 4 + it.second];
                }
            }
            --cellCost[curr_states[i].first];
            int cnt = 0;
            for (auto it : old_paths[i])
            {
                ++cnt;
                --cellCost[it.first];
                if (cnt == 5)
                    break;
            }
            auto old_cost = path_cost(i, old_paths[i]);
            auto tmp_path = old_paths[i];
            old_paths[i].clear();
            occupied[curr_states[i].first] = 0;
            if ((dangerousPlaceFree[goal_locations[i]] && occupied[goal_locations[i]] && curr_states[i].first == escape_place[goal_locations[i]]))
            {
                auto path1 = single_agent_plan_nonempty(curr_states[i].first, curr_states[i].second);
                process_path(curr_states[i].first, path1);
                if (path1.size() > 0)
                {
                    auto it = curr_states[i];
                    auto nxt = path1.front();
                    if (it.first != nxt.first)
                    {
                        assert(nebors[it.first * 4 + it.second] == nxt.first);
                        ++passes[it.first * 4 + it.second];
                        if (dangerousPlaceFree[it.first] == 0 && dangerousPlaceFree[nxt.first] == 0)
                            assert(passes[nxt.first * 4 + (it.second + 2) % 4] == 0);
                    }
                }
                for (int j = 0; j + 1 < path1.size(); ++j)
                {
                    auto it = path1[j];
                    auto nxt = path1[j + 1];
                    if (it.first != nxt.first)
                    {
                        assert(nebors[it.first * 4 + it.second] == nxt.first);
                        ++passes[it.first * 4 + it.second];
                        if (dangerousPlaceFree[it.first] == 0 && dangerousPlaceFree[nxt.first] == 0)
                            assert(passes[nxt.first * 4 + (it.second + 2) % 4] == 0);
                    }
                }
                auto path2 = single_agent_plan(path1.back().first, path1.back().second, curr_states[i].first);
                process_path(path1.back().first, path2);
                for (int j = 0; j + 1 < path2.size(); ++j)
                {
                    auto it = path2[j];
                    auto nxt = path2[j + 1];
                    if (it.first != nxt.first)
                    {
                        ++passes[it.first * 4 + it.second];
                    }
                }
                if (path2.size() > 0)
                {
                    auto it = path1.back();
                    auto nxt = path2.front();
                    if (it.first != nxt.first)
                    {
                        ++passes[it.first * 4 + it.second];
                    }
                }
                old_paths[i] = path1;
                for (auto it : path2)
                {
                    old_paths[i].push_back(it);
                }
                if (old_paths[i][0].first != curr_states[i].first)
                {
                    int dir1 = old_paths[i].back().second;
                    int dir2 = old_paths[i][0].second;
                    if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) > 1)
                    {
                        old_paths[i].push_back({curr_states[i].first, (dir1 + dir2) / 2});
                        old_paths[i].push_back({curr_states[i].first, dir2});
                    }
                    else if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) == 1)
                    {
                        old_paths[i].push_back({curr_states[i].first, dir2});
                    }
                }
                else
                {
                    int dir1 = old_paths[i].back().second;
                    int dir2 = old_paths[i][0].second;
                    if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) > 1)
                    {
                        old_paths[i].push_back({curr_states[i].first, (dir1 + dir2) / 2});
                    }
                }
            }
            else
            {
                auto path1 = single_agent_plan(curr_states[i].first, curr_states[i].second, goal_locations[i]);
                process_path(curr_states[i].first, path1);
                auto cost_path = path_cost(i, path1);
                if (old_cost.second <= cost_path.second && old_cost.first <= cost_path.first)
                {
                    old_paths[i] = tmp_path;
                    if (old_paths[i].size() > 0)
                    {
                        auto it = curr_states[i];
                        auto nxt = old_paths[i].front();
                        if (it.first != nxt.first)
                        {
                            ++passes[it.first * 4 + it.second];
                        }
                        for (int j = 0; j + 1 < old_paths[i].size(); ++j)
                        {
                            auto it = old_paths[i][j];
                            auto nxt = old_paths[i][j + 1];
                            if (it.first != nxt.first)
                            {
                                ++passes[it.first * 4 + it.second];
                            }
                        }
                    }
                }
                else
                {
                    if (path1.size() > 0)
                    {
                        auto it = curr_states[i];
                        auto nxt = path1.front();
                        if (it.first != nxt.first)
                        {
                            ++passes[it.first * 4 + it.second];
                        }
                        for (int j = 0; j + 1 < path1.size(); ++j)
                        {
                            auto it = path1[j];
                            auto nxt = path1[j + 1];
                            if (it.first != nxt.first)
                            {
                                ++passes[it.first * 4 + it.second];
                            }
                        }
                        auto path2 = single_agent_plan(goal_locations[i], path1.back().second, curr_states[i].first);
                        process_path(goal_locations[i], path2);
                        if (path2.size() > 0)
                        {
                            auto it = path1.back();
                            auto nxt = path2.front();
                            if (it.first != nxt.first)
                            {
                                ++passes[it.first * 4 + it.second];
                            }
                            for (int j = 0; j + 1 < path2.size(); ++j)
                            {
                                auto it = path2[j];
                                auto nxt = path2[j + 1];
                                if (it.first != nxt.first)
                                {
                                    ++passes[it.first * 4 + it.second];
                                }
                            }
                            old_paths[i] = path1;
                            for (auto it : path2)
                            {
                                old_paths[i].push_back(it);
                            }
                            // if the first action is turn, then no need to begin from curr_states
                            if (old_paths[i][0].first != curr_states[i].first)
                            {
                                int dir1 = old_paths[i].back().second;
                                int dir2 = old_paths[i][0].second;
                                if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) > 1)
                                {
                                    old_paths[i].push_back({curr_states[i].first, (dir1 + dir2) / 2});
                                    old_paths[i].push_back({curr_states[i].first, dir2});
                                }
                                else if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) == 1)
                                {
                                    old_paths[i].push_back({curr_states[i].first, dir2});
                                }
                            }
                            else
                            {
                                int dir1 = old_paths[i].back().second;
                                int dir2 = old_paths[i][0].second;
                                if (min(abs(dir1 - dir2), 4 - abs(dir1 - dir2)) > 1)
                                {
                                    old_paths[i].push_back({curr_states[i].first, (dir1 + dir2) / 2});
                                }
                            }
                        }
                    }
                }
            }
            occupied[curr_states[i].first] = i + 1;
            ++cellCost[curr_states[i].first];
            cnt = 0;
            for (auto it : old_paths[i])
            {
                ++cnt;
                ++cellCost[it.first];
                if (cnt == 5)
                    break;
            }
            if (old_paths[i].size() > 0)
            {
                future_occupied[old_paths[i][0].first]++;
            }
            else
            {
                future_occupied[curr_states[i].first]++;
            }
        }
    }
    int cnt = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (old_paths[i].size() == 0)
        {
            old_paths[i].push_back({curr_states[i].first, (curr_states[i].second + 1) % 4});
        }
        else
        {
            ++cnt;
        }
    }
    // cout << cnt << "///" << env->num_of_agents << endl;
    markWhoAllowedToMove();
    // int cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
    // int cnt5 = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        // if (it.size() > 0)
        // {
        //     if (old_paths[i][0].first == curr_states[i].first)
        //     {
        //         ++cnt3;
        //         if (old_paths[i][0].second != curr_states[i].second)
        //             ++cnt5;
        //     }
        //     for(auto it1:old_paths[i]){
        //         if (it1.first == goal_locations[i])
        //         {
        //             ++cnt2;
        //             break;
        //         }
        //     }
        // }
        // else
        // {
        //     ++cnt4;
        // }
        if (allowedToMove[i])
        {
            // ++cnt1;
            pair<int, int> tmp = it.front();
            if (tmp.first != curr_states[i].first)
            {
                lastActions[i] = actions[i] = Action::FW; // forward action
            }
            else if (tmp.second != curr_states[i].second)
            {
                int incr = tmp.second - curr_states[i].second;
                if (incr == 1 || incr == -3)
                {
                    lastActions[i] = actions[i] = Action::CR; // C--counter clockwise rotate
                }
                else if (incr == -1 || incr == 3)
                {
                    lastActions[i] = actions[i] = Action::CCR; // CCR--clockwise rotate
                }
            }
        }
    }
    // cout << "number of allowed to move = " << cnt1 << endl;
    // cout << "number to move to goal = " << cnt2 << endl;
    // cout << "number of constant agents = (turning)" << cnt5 << "/ " << cnt3 << endl;
    // cout << "number of cleared agents = " << cnt4 << endl;
}
void RandomPlanner2::print_location(int loc, int dr)
{
    cout << col[loc] << " " << env->rows - 1 - row[loc] << ", direction:" << dr << endl;
}
void RandomPlanner2::print_agent_information(int ag)
{
    cout << "Agent :" << ag << endl;
    cout << "current location:";
    print_location(curr_states[ag].first, curr_states[ag].second);
    cout << "Goal:";
    print_location(goal_locations[ag], -1);
    cout << "Path size:" << old_paths[ag].size() << "::" << endl;
    for (auto it : old_paths[ag])
    {
        print_location(it.first, it.second);
    }
    cout << "----" << endl;
}
pair<int, int> RandomPlanner2::path_cost(int agent, deque<pair<int, int>> &path)
{
    pair<int, int> ans = {1000000000, 1000000000};
    int cost_obs = 0, cost_dis = 0;
    for (int i = 0; i < path.size(); ++i)
    {
        auto it = path[i];
        ++cost_dis;
        if (i == 0)
        {
            if (curr_states[agent].first != it.first)
            {
                cost_obs += (int)(cellCost[it.first]);
            }
            else
            {
                ++cost_obs;
            }
        }
        else
        {
            if (it.first == path[i - 1].first)
            {
                ++cost_obs;
            }
            else
            {
                cost_obs += (int)(cellCost[it.first]);
            }
        }
        if (it.first == goal_locations[agent])
        {
            ans = {cost_obs, cost_dis};
            break;
        }
    }
    return ans;
}
deque<pair<int, int>> RandomPlanner2::single_agent_plan(int start, int start_direct, int end)
{
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, 0, 0, nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    AstarNode *ans_node = nullptr;
    bool solved = false;
    while (!open_list.empty())
    {
        AstarNode *curr = open_list.top();
        open_list.pop();
        int id = curr->location * 4 + curr->direction, cost = curr->g;
        if (closed_list[id])
        {
            continue;
        }
        if (curr->location == end)
        {
            ans_node = curr;
            solved = true;
            break;
        }
        closed_list[id] = 1;
        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction, curr->location == start);
        for (const pair<int, int> &neighbor : neighbors)
        {
            if (neighbor.first != curr->location)
            {
                AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                     curr->g + (int)(cellCost[neighbor.first]), curr->h + 1, curr);
                open_list.push(next_node);
                all_nodes.push_back(next_node);
            }
            else
            {
                AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                     curr->g + 1, curr->h + 1, curr);
                open_list.push(next_node);
                all_nodes.push_back(next_node);
            }
        }
    }
    deque<pair<int, int>> path;
    if (solved)
    {
        while (ans_node->parent != NULL)
        {
            path.emplace_front(make_pair(ans_node->location, ans_node->direction));
            ans_node = ans_node->parent;
        }
    }
    for (auto n : all_nodes)
    {
        closed_list[n->location * 4 + n->direction] = 0;
        delete n;
    }
    all_nodes.clear();
    return path;
}

deque<pair<int, int>> RandomPlanner2::single_agent_plan_nonempty(int start, int start_direct)
{
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, 0, 0, nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    AstarNode *ans_node = nullptr;
    bool solved = false;
    while (!open_list.empty())
    {
        AstarNode *curr = open_list.top();
        open_list.pop();
        int id = curr->location * 4 + curr->direction, cost = curr->g;
        if (closed_list[id])
        {
            continue;
        }
        if (curr->location != start)
        {
            ans_node = curr;
            solved = true;
            break;
        }
        closed_list[id] = 1;
        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction, curr->location == start);
        for (const pair<int, int> &neighbor : neighbors)
        {
            AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                 curr->g + (int)(neighbor.first == curr->location),
                                                 curr->h + 1, // + W2 * (int)(neighbor.first==curr->location),
                                                 curr);
            open_list.push(next_node);
            all_nodes.push_back(next_node);
        }
    }
    deque<pair<int, int>> path;
    if (solved)
    {
        while (ans_node->parent != NULL)
        {
            path.emplace_front(make_pair(ans_node->location, ans_node->direction));
            ans_node = ans_node->parent;
        }
    }
    for (auto n : all_nodes)
    {
        closed_list[n->location * 4 + n->direction] = 0;
        delete n;
    }
    all_nodes.clear();
    return path;
}
int RandomPlanner2::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

list<pair<int, int>> RandomPlanner2::getNeighbors(int location, int direction, bool is_start)
{
    list<pair<int, int>> neighbors;
    // forward
    int forward = nebors[location * 4 + direction];
    if (forward != -1)
    {
        int nm_of_reverse_passes = passes[forward * 4 + (direction + 2) % 4];
        if ((nm_of_reverse_passes == 0 && !dangerousPlaceFree[forward]) || (dangerousPlaceFree[forward] && (!occupied[forward] || !is_start)) || dangerousPlaceFree[location])
        {
            neighbors.push_back({forward, direction});
        }
    }
    int new_direction = direction;
    // turn left
    new_direction = direction - 1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.push_back({location, new_direction});
    // turn right
    new_direction = direction + 1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.push_back({location, new_direction});
    return neighbors;
}
