#include <SortationPlanner.h>
#include <thread>
#include <random>
using namespace std::chrono;
using namespace std;

struct AstarNode
{
    int location;
    int direction;
    int f, g, h;
    AstarNode *parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location, int _direction, int _g, int _h, AstarNode *_parent) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent) {}
    AstarNode(int _location, int _direction, int _g, int _h, int _t, AstarNode *_parent) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), t(_t), parent(_parent) {}
};

struct cmp
{
    bool operator()(AstarNode *a, AstarNode *b)
    {
        if (a->f == b->f)
            return a->g <= b->g;
        else
            return a->f > b->f;
    }
};

void SortationPlanner::initialize(int preprocess_time_limit)
{
    flagReplan = false;
    W1 = 1;
    W2 = 10;
    max_time_limit = 900000;
    safeValues.push_back(2);
    safeValues.push_back(3);
    safeExtraLine = 5;
    // safeValues.push_back(5);
    mx_edge_x = 4;
    mx_edge_y = 6;

    safePlace = new bool[(env->cols) * (env->rows)]();
    dangerousPlace = new bool[(env->cols) * (env->rows)]();
    findSafePlaces();
    timeStep = -1;
    freeCells = new int[env->map.size()]();
    compressFreeCells();
    genAllNeighors();
    dir_h = dir_v = dr = 0;
}

void SortationPlanner::findSafePlaces()
{
    for (int x = 0; x < env->rows; ++x)
    {
        for (int y = 0; y < env->cols; ++y)
        {
            bool l = 0;
            for (auto it : safeValues)
            {
                if ((x == it || x == env->rows - 1 - it) && (y >= mx_edge_x + 2 && y <= env->cols - 1 - mx_edge_x - 2 && y != env->cols / 5 && y != 2 * env->cols / 5 && y != 3 * env->cols / 5 && y != 4 * env->cols / 5))
                {
                    l = 1;
                    break;
                }
                if ((y == it || y == env->cols - 1 - it) && (x >= mx_edge_y + 2 && x <= env->rows - 1 - mx_edge_y - 2 && x != env->rows / 5 && x != 2 * env->rows / 5 && x != 3 * env->rows / 5 && x != 4 * env->rows / 5))
                {
                    l = 1;
                    break;
                }
            }
            if ((x == env->rows - 1 - safeExtraLine) && (y >= mx_edge_x + 2 && y <= env->cols - 1 - mx_edge_x - 2 && y != env->cols / 5 && y != 2 * env->cols / 5 && y != 3 * env->cols / 5 && y != 4 * env->cols / 5))
            {
                l = 1;
            }
            if (l)
            {
                safePlace[x * env->cols + y] = 1;
            }
        }
    }
    // for(int i=0; i<env->rows; ++i){
    //     for(int j=0; j<env->cols; ++j){
    //         if(env->map[i*env->cols+j]==0){
    //             vector<int> v;
    //             for(int k=0; k<8; ++k){
    //                 int l1 = -1;
    //                 int ii = i+dx[k];
    //                 int jj = j+dy[k];
    //                 if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //                     if(k%2==0)
    //                         l1 = 0;
    //                 }
    //                 else{
    //                     l1=1;
    //                 }
    //                 if(l1!=-1)
    //                     v.push_back(l1);
    //             }
    //             int tmp = 0;
    //             for(int i=0;i<v.size();++i){
    //                 if(v[i]!=v[(i+1)%v.size()])++tmp;
    //             }
    //             if(tmp>=3){
    //                 dangerousPlace[i*env->cols+j]=1;
    //             }
    //         }
    //     }
    // }
}

void SortationPlanner::compressFreeCells()
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
    emergency_nebors = new int[numOfFreeCells * 4]();
    for(int i=0; i<processor_count; ++i)
        closed_list[i] = new bool[numOfFreeCells * 4]();
    occupied = new int[numOfFreeCells]();
    nxt_occupied = new vector<int>[numOfFreeCells]();
    row = new int[numOfFreeCells]();
    col = new int[numOfFreeCells]();
    isGoal = new bool[numOfFreeCells]();
    safePlaceFree = new bool[numOfFreeCells]();
    dangerousPlaceFree = new bool[numOfFreeCells]();
    out_nebors_cnt = new int[numOfFreeCells]();
    for (int i = 0; i < 4; ++i)
    {
        blocked_edges[i] = new std::set<int>[numOfFreeCells]();
    }
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows * env->cols; ++i)
    {
        if (!env->map[i])
        {
            row[numOfFreeCells] = i / env->cols;
            col[numOfFreeCells] = i % env->cols;
            if (safePlace[i])
            {
                safePlaceFree[numOfFreeCells] = 1;
            }
            // if(dangerousPlace[i]){
            //     dangerousPlaceFree[numOfFreeCells]=1;
            // }
            ++numOfFreeCells;
        }
    }
}

void SortationPlanner::genAllNeighors()
{

    int cnt = 0;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    vector<int> blocked_cells;
    fill(nebors, nebors+numOfFreeCells*4, -1);
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            int cell = i * m + j;
            if (!env->map[cell])
            {
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
                    {
                        emergency_nebors[freeCells[cell] * 4 + k] = freeCells[ii*env->cols+jj];
                    }
                    else{
                        emergency_nebors[freeCells[cell] * 4 + k] = -1;
                    }
                }
            }
        }
    }
    vector<int> v(env->cols, 0), h(env->rows, 0);
    for (int i = 0; i < env->cols; ++i)
    {
        v[i] = (i + dir_v) % 2;
    }
    for (int i = 0; i < env->rows; ++i)
    {
        h[i] = (i + dir_h) % 2;
    }
    vector<int> tmp;
    for (int i = 0; i < env->rows; ++i)
    {
        if (env->map[i * env->cols + 6] == 0)
        {
            tmp.push_back(i);
        }
    }
    for (int i = 0; i < tmp.size(); ++i)
    {
        h[tmp[i]] = (i + dir_h) % 2;
    }
    tmp.clear();
    for (int i = 0; i < env->cols; ++i)
    {
        if (env->map[6 * env->cols + i] == 0)
        {
            tmp.push_back(i);
        }
    }
    for (int i = 0; i < tmp.size(); ++i)
    {
        v[tmp[i]] = (i + dir_v) % 2;
    }
    cnt = 0;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (!env->map[cnt])
            {
                int cell = freeCells[cnt];
                for (int k = 0; k < 4; ++k)
                {
                    if ((k == 0 && !h[i]) || (k == 1 && v[j]) || (k == 2 && h[i]) || (k == 3 && !v[j]))
                    {
                        int ii = i + di[k];
                        int jj = j + dj[k];
                        int id = cnt + dj[k] + di[k] * env->cols;
                        if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
                        {
                            if (!env->map[id])
                            {
                                nebors[cell * 4 + k] = freeCells[id];
                            }
                            else
                            {
                                nebors[cell * 4 + k] = -1;
                            }
                        }
                        else
                        {
                            nebors[cell * 4 + k] = -1;
                        }
                    }
                    else
                    {
                        nebors[cell * 4 + k] = -1;
                    }
                }
            }
            ++cnt;
        }
    }
    // cout<<"Finished generating nebors"<<endl;
    // std::ofstream out;
    // out.open("map.txt");
    // bool l = 1;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         if (env->map[i * m + j])
    //         {
    //             if (!l)
    //                 out << " ";
    //             l = 0;
    //             out << j;
    //         }
    //     }
    // }
    // out << endl;
    // l = 1;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         if (env->map[i * m + j])
    //         {
    //             if (!l)
    //                 out << " ";
    //             l = 0;
    //             out << env->rows - 1 - i;
    //         }
    //     }
    // }
    // out << endl;
    // l = 1;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         if (!env->map[i * m + j])
    //         {
    //             if (!l)
    //                 out << " ";
    //             l = 0;
    //             out << j;
    //         }
    //     }
    // }
    // out << endl;
    // l = 1;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         if (!env->map[i * m + j])
    //         {
    //             if (!l)
    //                 out << " ";
    //             out << env->rows - 1 - i;
    //             l = 0;
    //         }
    //     }
    // }
    // out << endl;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         if (env->map[i * env->cols + j])
    //             continue;
    //         int cell = freeCells[i * env->cols + j];
    //         int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    //         for (int k = 0; k < 4; ++k)
    //         {
    //             if (nebors[cell * 4 + k] != -1)
    //             {
    //                 out << j << " " << env->rows - 1 - i << " " << j + dy[k] << " " << env->rows - 1 - i - dx[k] << endl;
    //             }
    //         }
    //     }
    // }
    // out.close();
}
void SortationPlanner::dfs_fill_sizes(int x)
{
    if (sz[x])
    {
        return;
    }
    sz[x] = 1;
    int mx = 0;
    for (auto it : adj[x])
    {
        dfs_fill_sizes(it);
        mx = max(mx, sz[it]);
    }
    sz[x] += mx;
}
void SortationPlanner::dfs_mark_valids(int x)
{
    allowedToMove[x] = 1;
    if (adj[x].size() == 0)
        return;
    int nxt = adj[x][0], mx = sz[adj[x][0]];
    for (auto it : adj[x])
    {
        if (sz[it] > mx)
        {
            mx = sz[it];
            nxt = it;
        }
    }
    dfs_mark_valids(nxt);
}
bool SortationPlanner::dfs_fill_cycles(int x, int pa)
{
    // if(timeStep == 450){
    //     cout<<x<<" "<<vis[x]<<", "<<vis_local[x]<<" "<<pa<<endl;
    // }
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
void SortationPlanner::markWhoAllowedToMove()
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
        int mx = 0, mx_id = i;
        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            ++cnt[it - 1];
            dfs_fill_sizes(it - 1);
        }
        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            if (sz[it - 1] > mx)
            {
                mx_id = it - 1;
                mx = sz[it - 1];
            }
        }
        dfs_mark_valids(mx_id);
        // int tmp = nxt_occupied[old_paths[i].front().first];
        // if(tmp && tmp != i+1 && old_paths[i].front().first!=curr_states[i].first){
        //     ++cnt[i];
        //     ++cnt[tmp-1];
        //     dfs_fill_sizes(i);
        //     dfs_fill_sizes(tmp-1);
        //     if(sz[i]>sz[tmp-1] || dangerousPlaceFree[curr_states[i].first]){
        //         dfs_mark_valids(i);
        //     }
        //     else{
        //         dfs_mark_valids(tmp-1);
        //     }
        // }
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
            assert(allowedToMove[i] == 1);
        }
        vis[i] = 0;
        vis_local[i] = 0;
        sz[i] = 0;
        nxt_occupied[old_paths[i].front().first].clear();
        adj[i].clear();
    }
}
void SortationPlanner::move_away(int id)
{
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int cell = curr_states[id].first, dr = curr_states[id].second;
    list<pair<int, int>> new_path;
    // old_paths[id].clear();
    // old_paths[id].push_back({cell, dr});
    // return;
    for (int i = 1; i <= 4; ++i)
    {
        new_path.push_back({cell, (dr + i) % 4});
        if (nebors[cell * 4 + (dr + i) % 4] != -1 && !occupied[nebors[cell * 4 + (dr + i) % 4]])
        {
            new_path.push_back({nebors[cell * 4 + (dr + i) % 4], (dr + i) % 4});
            // assert(dangerousPlaceFree[nebors[cell*4+(dr+i)%4]]==0);
            break;
        }
    }
    // cout<<id<<" "<<old_paths[id].size()<<" "<<new_path.size()<<endl;
    if (old_paths[id].size() == 0 || new_path.back().first != curr_states[id].first)
    {
        // cout<<"changed"<<endl;
        old_paths[id] = new_path;
    }
}

void SortationPlanner::partial_plan(int thread_id, int start_id, int end_id)
{
    // cout<<thread_id<<", "<<start_id<<" "<<end_id<<endl;
    // auto start_plan_time = high_resolution_clock::now();
    vector<pair<int, int>> v;
    for (int i = start_id; i < end_id; ++i)
    {
        if(old_paths[i].size()>0){
            if(old_paths[i].front().first != curr_states[i].first){
                if(nebors[curr_states[i].first*4+curr_states[i].second]==-1 && occupied[old_paths[i].front().first]){
                    old_paths[i].clear();
                }
            }
        }
        if (old_paths[i].size() == 0)
            v.push_back({abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first]), i});
    }
    sort(v.begin(), v.end());
    for (int ii=0; ii<v.size(); ii++)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = v[ii].second;
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i]);
    }
    int tmp_shift_id = shift_id[thread_id];
    for (int ii = 0; ii < end_id-start_id; ii++)
    {
        int i = (ii + tmp_shift_id);
        if (i >= end_id-start_id)
        {
            i -= end_id-start_id;
        }
        shift_id[thread_id] = i;
        i += start_id;
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            return;
        }
        if (goal_locations[i] != -1 && allowedToMove[i]==0)
        {
            auto tmp_path = old_paths[i];
            old_paths[i].clear();
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i]);
        }
    }
    for (int i = start_id; i < end_id; ++i)
    {
        if (old_paths[i].size() == 0)
        {
            int tmp = nebors[curr_states[i].first * 4 + curr_states[i].second];
            if (tmp == -1)
            {
                tmp = nebors[curr_states[i].first * 4 + (curr_states[i].second + 1) % 4];
                if (tmp == -1)
                {
                    old_paths[i].push_back({curr_states[i].first, (curr_states[i].second + 3) % 4});
                }
                else
                {
                    old_paths[i].push_back({curr_states[i].first, (curr_states[i].second + 1) % 4});
                }
            }
            else
            {
                old_paths[i].push_back({tmp, curr_states[i].second});
            }
        }
    }
}

// plan using simple A* that ignores the time dimension
void SortationPlanner::plan(int time_limit, vector<Action> &actions)
{
    start_plan_time = high_resolution_clock::now();
    ++timeStep;
    if (timeStep == 0)
    {
        old_paths = new list<pair<int, int>>[env->num_of_agents]();
        curr_states = new pair<int, int>[env->num_of_agents]();
        goal_locations = new int[env->num_of_agents]();
        adj = new vector<int>[env->num_of_agents]();
        sz = new int[env->num_of_agents]();
        vis = new bool[env->num_of_agents]();
        vis_local = new bool[env->num_of_agents]();
        allowedToMove = new bool[env->num_of_agents]();
        ignore = new bool[env->num_of_agents]();
        solved = new bool[env->num_of_agents]();
        for(int i=0; i<processor_count; ++i){
            shift_id[i] = 0;
        }
    }
    if (timeStep > 0)
    {
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            occupied[curr_states[i].first] = 0;
            if (goal_locations[i] != -1)
            {
                isGoal[goal_locations[i]] = 0;
            }
        }
    }
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
                solved[i] = false;
            }
            goal_locations[i] = freeCells[env->goal_locations[i].front().first];
            isGoal[goal_locations[i]] = 1;
        }
    }
    if (timeStep == 0)
    {
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            mapFree[curr_states[i].first] = 1;
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
            old_paths[i].pop_front();
        }
    }
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W);
    thread threads[processor_count];
    for(int i=0; i<processor_count; ++i){

        if(i < processor_count-1){
            threads[i] = thread(&SortationPlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents/processor_count*(i+1));
        }
        else{
            threads[i] = thread(&SortationPlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents);
        }    
    }
    for(int i=0; i<processor_count; ++i){
        threads[i].join();
    }
    int cnt = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (old_paths[i].size() == 0)
        {
            // if(solved[i]){
            //     cout<<i<<" "<<col[curr_states[i].first]<<" "<<env->rows-1-row[curr_states[i].first]<<endl;
            //     cout<<i<<" "<<col[goal_locations[i]]<<" "<<env->rows-1-row[goal_locations[i]]<<endl;
            //     cout<<"is danger current position: "<<(dangerousPlaceFree[curr_states[i].first]?"YES":"NO")<<endl;
            //     cout<<"is goal = current position: "<<(goal_locations[i]==curr_states[i].first?"YES":"NO")<<endl;
            // }
            // ++cnt;
            // assert(i>shift_id);
            // int tmp = nebors[curr_states[i].first*4+curr_states[i].second];
            // if((tmp==-1) || (bridge[curr_states[i].first*4+curr_states[i].second] && !available[curr_states[i].first*4+curr_states[i].second])){
            //     tmp = nebors[curr_states[i].first*4+(curr_states[i].second+1)%4];
            //     if((tmp==-1)){
            //         old_paths[i].push_back({curr_states[i].first, (curr_states[i].second+3)%4});
            //     }
            //     else{
            old_paths[i].push_back({curr_states[i].first, (curr_states[i].second + 1) % 4});
            //     }
            // }
            // else{
            // old_paths[i].push_back({tmp, curr_states[i].second});
            // // available[tmp*4+(curr_states[i].second+2)%4]=0;
            // if(bridge[curr_states[i].first*4+(curr_states[i].second)%4]){
            // assert(available[curr_states[i].first*4+(curr_states[i].second)%4]);
            // }

            // }
            if (solved[i] && goal_locations[i] != curr_states[i].first)
            {
                ignore[i] = 1;
            }
        }
        else
        {
            ++cnt;
        }
        // cout<<i<<" path size = "<<old_paths[i].size()<<endl;
        int nxt = old_paths[i].front().first;
        // if(dangerousPlaceFree[nxt] && occupied[nxt] && occupied[nxt]!=i+1){
        //     assert(false);
        //     move_away(i);
        // }
    }
    // cout<<cnt<<"///";
    // for(int i=0; i<processor_count; ++i){
    //     cout<<shift_id[i]<<" ";
    // }
    // cout<<"/"<<env->num_of_agents<<endl;
    // cnt1 = 0;
    // for(int i=0; i<env->num_of_agents; ++i){
    //     if(old_paths[i].back().first != goal_locations[i]){
    //         ++cnt1;
    //         cout<<i<<" "<<row[curr_states[i].first]<<" "<<col[curr_states[i].first]<<endl;
    //     }
    //     // if(old_paths[i].front() == curr_states[i]){
    //     //     cout<<"stop["<<i<<"]"<<endl;
    //     // }
    // }
    // cout<<"Num of not to goals = "<<cnt1<<endl;
    markWhoAllowedToMove();
    // int cnt1 = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        // assert(it.size()>0);
        if (allowedToMove[i])
        {
            // ++cnt1;
            // cout<<i<<"::"<<endl;
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
    // cout<<"number of allowed to move = "<<cnt1<<endl;
    // cnt = 0;
    // for (int i = 0; i < env->num_of_agents; ++i)
    // {
    //     cnt += allowedToMove[i] && !ignore[i];
    // }
    // if (cnt == 0)
    // {
    //     for (int i = 0; i < env->num_of_agents; ++i)
    //     {
    //         ignore[i] = 0;
    //         old_paths[i].clear();
    //         solved[i] = 0;
    //     }
    //     // exit(0);
    //     dir_h = drh[dr];
    //     dir_v = drv[dr];
    //     dr = (dr + 1) % 4;
    //     genAllNeighors();
    // }
    return;
}

void SortationPlanner::single_agent_plan(int thread_id, int start, int start_direct, int end, list<pair<int, int>> &path)
{
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, timeStep, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    AstarNode *ans_node = nullptr;
    bool solved = false;
    while (!open_list.empty())
    {
        AstarNode *curr = open_list.top();
        open_list.pop();
        int id = curr->location * 4 + curr->direction, cost = curr->g;
        if (closed_list[thread_id][id])
        {
            continue;
        }
        if (curr->location == end)
        {
            ans_node = curr;
            solved = true;
            break;
        }
        closed_list[thread_id][id] = 1;
        cost += W1 * int((occupied[curr->location] > 0) && (curr->location != start));
        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction, curr->location==start);
        for (const pair<int, int> &neighbor : neighbors)
        {
            AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                 cost + W2, W2 * getManhattanDistance(neighbor.first, end), curr);
            open_list.push(next_node);
            all_nodes.push_back(next_node);
        }
    }
    assert(solved);
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
        closed_list[thread_id][n->location * 4 + n->direction] = 0;
        delete n;
    }
    all_nodes.clear();
    return;
}

int SortationPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

list<pair<int, int>> SortationPlanner::getNeighbors(int location, int direction, bool is_start)
{
    list<pair<int, int>> neighbors;
    // forward
    int forward = nebors[location * 4 + direction];
    if (forward != -1)
    {
        neighbors.push_back({forward, direction});
    }
    else if(is_start){
        int forward = emergency_nebors[location*4+direction];
        if(forward != -1 && !occupied[forward]){
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
