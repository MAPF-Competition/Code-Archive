#include <CityPlanner.h>
#include <random>
#include <scc_dag_city.h>
using namespace std::chrono;

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
        // return a->g > b->g;
        if (a->f == b->f)
            return a->g <= b->g;
        else
            return a->f > b->f;
    }
};

void CityPlanner::initialize(int preprocess_time_limit)
{
    flagReplan = false;
    W1 = 2;
    W2 = 4;
    max_time_limit = 900000;
    safePlace = new bool[(env->cols) * (env->rows)]();
    cellCost = new int[env->cols * env->rows]();
    dangerousPlace = new bool[(env->cols) * (env->rows)]();
    findSafePlaces();
    timeStep = -1;
    freeCells = new int[env->map.size()]();
    compressFreeCells();
    genAllNeighors();
    dir_h = dir_v = dr = 0;
}

void CityPlanner::findSafePlaces()
{
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    auto data = env->map;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (env->map[i * env->cols + j])
            {
                for (int k = 0; k < 8; ++k)
                {
                    int ii = i + dx[k];
                    int jj = j + dy[k];
                    if (ii >= 0 && ii < env->rows && jj >= 0 && jj < env->cols)
                    {
                        data[ii * env->cols + jj] = 1;
                    }
                }
            }
        }
    }
    Image img(env->cols, env->rows, data);
    ZhangSuen zs;
    Image mp = std::move(zs(img));
    for (int i = 0; i < mp.height(); ++i)
    {
        for (int j = 0; j < mp.width(); ++j)
        {
            if (mp(j, i) == 1)
            {
                safePlace[i * (env->cols) + j] = 1;
            }
        }
    }
}


void  CityPlanner::dfs_safe_places(int x, bool *vis){
    if(vis[x])return;
    vis[x]=1;
    safe_places_vector.push_back(freeCells[x]);
    for(int k=0; k<4; ++k){
        int i = x/env->cols + di[k];
        int j = x%env->cols + dj[k];
        if(i>=0 && i<env->rows && j>=0 && j<env->cols){
            int xx = x + di[k]*env->cols + dj[k];
            if(safePlace[xx]){
                dfs_safe_places(xx, vis);
            }
        }
    }
}
void CityPlanner::compressFreeCells()
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
    costEdge = new int[numOfFreeCells * 4]();
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
    bridge = new bool[numOfFreeCells * 4]();
    available = new bool[numOfFreeCells * 4]();
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
                safe_places_vector.push_back(numOfFreeCells);
            }
            // if(dangerousPlace[i]){
            //     dangerousPlaceFree[numOfFreeCells]=1;
            // }
            ++numOfFreeCells;
        }
    }
    bool *vis = new bool[env->cols * env->rows];
    for(int i=0; i<env->cols * env->rows; ++i){
        if(safePlace[i] && !vis[i]){
            dfs_safe_places(i,vis);
        }
    }
    delete vis;
}

bool CityPlanner::connect(int i, int k, int ii)
{
    // assert(env->map[i]==0);
    // assert(env->map[ii]==0);
    if (nebors[freeCells[ii] * 4 + (k + 2) % 4] != -1)
        return false;
    nebors[freeCells[i] * 4 + k] = freeCells[ii];
    return true;
}
void CityPlanner::dfs_similar(int x, int y, int id, vector<int> &vis, vector<int> &mp)
{
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    // assert(vis[x*m+y]==0);
    // assert(mp[x*m+y]==id);
    // assert(env->map[x*m+y]==0);
    vis[x * m + y] = 1;
    for (int i = 0; i < 4; ++i)
    {
        int ii = x + dx[i];
        int jj = y + dy[i];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj] && mp[ii * m + jj] == id)
        {
            if (connect(x * m + y, i, ii * m + jj))
            {
                if (!vis[ii * m + jj])
                    dfs_similar(ii, jj, id, vis, mp);
            }
        }
    }
}
void CityPlanner::dfs_full(int x, int y, vector<int> &vis)
{
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    // assert(vis[x*m+y]==0);
    // assert(env->map[x*m+y]==0);
    vis[x * m + y] = 1;
    for (int i = 0; i < 4; ++i)
    {
        int ii = x + dx[i];
        int jj = y + dy[i];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
        {
            if (connect(x * m + y, i, ii * m + jj))
            {
                if (!vis[ii * m + jj])
                    dfs_full(ii, jj, vis);
            }
        }
    }
}
void CityPlanner::dfs_count_reach(int x, int y, vector<int> &vis, int &cnt)
{
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    // assert(vis[x*m+y]==0);
    // assert(env->map[x*m+y]==0);
    vis[x * m + y] = 1;
    ++cnt;
    for (int i = 0; i < 4; ++i)
    {
        int ii = x + dx[i];
        int jj = y + dy[i];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && vis[ii * m + jj] == 0)
        {
            if (nebors[freeCells[x * m + y] * 4 + i] != -1)
            {
                assert(nebors[freeCells[ii * m + jj] * 4 + (i + 2) % 4] == -1);
                dfs_count_reach(ii, jj, vis, cnt);
            }
        }
    }
}

bool CityPlanner::onTheEdge(int i, int j, int dir, int ccw, vector<int> &mp, int id)
{
    int m = env->cols, n = env->rows;
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    bool onEdge = 0;
    if (ccw)
    {
        // for(int k=1; k<2; ++k)
        {
            int ii = i + dx[(dir + 1) % 4];
            int jj = j + dy[(dir + 1) % 4];
            if (ii < 0 || ii >= n || jj < 0 || jj >= m || (mp[ii * m + jj] != id && mp[ii * m + jj]))
                onEdge = 1;
        }
        int k1 = (dir + 1) % 4;
        int k2 = (dir + 2) % 4;
        int di = dx[k1];
        if (di == 0)
            di = dx[k2];
        int dj = dy[k1];
        if (dj == 0)
            dj = dy[k2];
        int ii = i + di;
        int jj = j + dj;
        if (ii < 0 || ii >= n || jj < 0 || jj >= m || (mp[ii * m + jj] != id && mp[ii * m + jj]))
            onEdge = 1;
    }
    else
    {
        // for(int k=3; k<4; ++k)
        {
            int ii = i + dx[(dir + 3) % 4];
            int jj = j + dy[(dir + 3) % 4];
            if (ii < 0 || ii >= n || jj < 0 || jj >= m || (mp[ii * m + jj] != id && mp[ii * m + jj]))
                onEdge = 1;
        }
        int k1 = (dir + 3) % 4;
        int k2 = (dir + 2) % 4;
        int di = dx[k1];
        if (di == 0)
            di = dx[k2];
        int dj = dy[k1];
        if (dj == 0)
            dj = dy[k2];
        int ii = i + di;
        int jj = j + dj;
        if (ii < 0 || ii >= n || jj < 0 || jj >= m || (mp[ii * m + jj] != id && mp[ii * m + jj]))
            onEdge = 1;
    }
    if (!onEdge)
    {
        return false;
    }
    return true;
}
void CityPlanner::dfs_one_cycle(int i, int j, int dir, bool ccw, vector<int> &mp, int id, int cnt, int i_st, int j_st, vector<int> &vis)
{
    int m = env->cols, n = env->rows;
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int cell = freeCells[i * m + j];
    // cout<<id<<"::"<<endl;
    // cout<<cnt<<" "<<mp[i*m+j]<<" "<<i<<" "<<j<<" "<<i_st<<" "<<j_st<<endl;
    // if(mp[i*m+j] && i==i_st && j==j_st){
    //     cout<<cnt<<endl;
    //     return true;
    // }
    mp[i * m + j] = id;
    bool onTheEnd = true, lastEnd = false; //, touchedTheEnd = false;
    bool didConnect = false;
    if (!ccw)
    {
        for (int k = -1; k < 3; ++k)
        {
            int kk = (dir + k + 4) % 4;
            int ii = i + dx[kk];
            int jj = j + dy[kk];
            if (ii < 0 || ii >= env->rows || jj < 0 || jj >= m)
                continue;
            int nxt = ii * m + jj;
            // cout<<dx[kk]<<" **** "<<dy[kk]<<" "<<i<<" "<<j<<" "<<ii<<" "<<jj<<" "<<id<<endl;
            if (ii == i_st && jj == j_st)
            {
                if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                    nebors[cell * 4 + kk] = freeCells[nxt];
                lastEnd = true;
                // break;
            }
            if (mp[nxt])
            {
                if (!env->map[nxt])
                {
                    if (mp[nxt] == id)
                    {
                        if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        {
                            nebors[cell * 4 + kk] = freeCells[nxt];
                        }
                    }
                    else
                    {
                        // if(id==4){
                        //     cout<<"###"<<cnt<<endl;
                        // }
                        if (cnt % 2)
                        {
                            if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                            {
                                nebors[cell * 4 + kk] = freeCells[nxt];
                                // didConnect = 1;
                                ++cnt;
                            }
                        }
                        else
                        {
                            if (nebors[cell * 4 + kk] == -1)
                            {
                                nebors[freeCells[nxt] * 4 + (kk + 2) % 4] = cell;
                                // didConnect = 1;
                                ++cnt;
                            }
                        }
                    }
                }
                continue;
            }
            // if(onTheEdge(ii, jj, kk, ccw, mp, id)){
            if (vis[nxt])
            {
                nebors[cell * 4 + kk] = freeCells[nxt];
                onTheEnd = false;
                assert(nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1);
                // if(dfs_one_cycle(ii, jj, kk, ccw, mp, id, cnt+1, i_st, j_st, vis)){
                dfs_one_cycle(ii, jj, kk, ccw, mp, id, cnt + didConnect, i_st, j_st, vis);
                // touchedTheEnd = true;
                // break;
                // }
            }
        }
    }
    else
    {
        for (int k = -1; k < 3; ++k)
        {
            int kk = (dir - k + 4) % 4;
            int ii = i + dx[kk];
            int jj = j + dy[kk];
            if (ii < 0 || ii >= env->rows || jj < 0 || jj >= m)
                continue;
            int nxt = ii * m + jj;
            // cout<<dx[kk]<<" **** "<<dy[kk]<<" "<<i<<" "<<j<<" "<<ii<<" "<<jj<<endl;
            if (ii == i_st && jj == j_st)
            {
                if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                    nebors[cell * 4 + kk] = freeCells[nxt];
                lastEnd = true;
                // break;
            }
            if (mp[nxt])
            {
                if (!env->map[nxt])
                {
                    if (mp[nxt] == id)
                    {
                        if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        {
                            nebors[cell * 4 + kk] = freeCells[nxt];
                        }
                    }
                    else
                    {
                        // if(id==4){
                        //     cout<<"###"<<cnt<<endl;
                        // }
                        if (cnt % 2)
                        {
                            if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                            {
                                nebors[cell * 4 + kk] = freeCells[nxt];
                                // didConnect = 1;
                                ++cnt;
                            }
                        }
                        else
                        {
                            if (nebors[cell * 4 + kk] == -1)
                            {
                                nebors[freeCells[nxt] * 4 + (kk + 2) % 4] = cell;
                                // didConnect = 1;
                                ++cnt;
                            }
                        }
                    }
                }
                continue;
            }
            // cout<<ii<<",,,"<<jj<<endl;
            // if(onTheEdge(ii, jj, kk, ccw, mp, id)){
            if (vis[nxt])
            {
                nebors[cell * 4 + kk] = freeCells[nxt];
                onTheEnd = false;
                assert(nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1);
                // if(dfs_one_cycle(ii, jj, kk, ccw, mp, id, cnt+1, i_st, j_st, vis)){
                dfs_one_cycle(ii, jj, kk, ccw, mp, id, cnt + didConnect, i_st, j_st, vis);
                // touchedTheEnd = true;
                // break;
                // }
            }
        }
    }
    if (lastEnd || onTheEnd)
    {
        bool ok = false;
        for (int k = 0; k < 4; ++k)
        {
            int ii = i + dx[k];
            int jj = j + dy[k];
            if (ii < 0 || ii >= n || jj < 0 || jj >= m)
                continue;
            if (!env->map[ii * m + jj] && mp[ii * m + jj] != id && mp[ii * m + jj] && nebors[freeCells[i * m + j] * 4 + k] == freeCells[ii * m + jj])
            {
                ok = 1;
                break;
            }
        }
        if (!ok)
            for (int k = 0; k < 4; ++k)
            {
                int ii = i + dx[k];
                int jj = j + dy[k];
                if (ii < 0 || ii >= n || jj < 0 || jj >= m)
                    continue;
                if (!env->map[ii * m + jj] && mp[ii * m + jj] != id && mp[ii * m + jj])
                {
                    nebors[freeCells[i * m + j] * 4 + k] = freeCells[ii * m + jj];
                    // assert(mp[ii*m+jj]<id);
                    // assert(nebors[freeCells[ii*m+jj]*4+(k+2)%4]==-1);
                    nebors[freeCells[ii * m + jj] * 4 + (2 + k) % 4] = -1;
                    // notOnTheEnd = true;
                    break;
                    // return true;
                }
            }
    }
    // return lastEnd || touchedTheEnd;
}
void CityPlanner::bfs_outer_cycle()
{
    int n = env->rows, m = env->cols;
    auto mp = env->map;
    int dir = 0;
    int id = 3;
    vector<int> g_vis;
    int dx[8] = {1, 1, 1, 0, 0, -1, -1, -1}, dy[8] = {0, 1, -1, 1, -1, 0, 1, -1};
    vector<int> vis(mp.size(), 0);
    vector<int> v;
    std::queue<pair<int, int>> q;
    for (int i = 0; i < n; ++i)
    {
        q.push({i, 0});
        q.push({i, m - 1});
    }
    for (int j = 0; j < m; ++j)
    {
        q.push({0, j});
        q.push({n - 1, j});
    }
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(mp[i*env->cols+j]){
                q.push({i,j});
            }
        }
    }
    while (!q.empty())
    {
        auto tmp = q.front();
        int x = tmp.first, y = tmp.second;
        int cell = x * m + y;
        q.pop();
        if (vis[cell])
        {
            continue;
        }
        vis[cell] = 1;
        if (!mp[cell])
        {
            v.push_back(cell);
            continue;
        }
        for (int i = 0; i < 8; ++i)
        {
            int xx = x + dx[i];
            int yy = y + dy[i];
            if (xx >= 0 && xx < n && yy >= 0 && yy < m)
            {
                q.push({xx, yy});
            }
        }
    }
    for (auto it : v)
    {
        if (mp[it] != id)
            dfs_one_cycle(it / m, it % m, dir * 2, dir, mp, id, 0, it / m, it % m, vis);
    }
}
void CityPlanner::bfs()
{
    int n = env->rows, m = env->cols;
    auto mp = env->map;
    int dir = 0;
    int id = 3;

    // std::ofstream out;
    // out.open("map.txt");

    vector<int> g_vis;
    int dx[8] = {1, 1, 1, 0, 0, -1, -1, -1}, dy[8] = {0, 1, -1, 1, -1, 0, 1, -1};
    while (1)
    {
        bool l = 0;
        vector<int> vis(mp.size(), 0);
        vector<int> v;
        std::queue<pair<int, int>> q;
        for (int i = 0; i < n; ++i)
        {
            q.push({i, 0});
            q.push({i, m - 1});
        }
        for (int j = 0; j < m; ++j)
        {
            q.push({0, j});
            q.push({n - 1, j});
        }
        while (!q.empty())
        {
            auto tmp = q.front();
            int x = tmp.first, y = tmp.second;
            int cell = x * m + y;
            q.pop();
            if (vis[cell])
            {
                continue;
            }
            vis[cell] = 1;
            if (!mp[cell])
            {
                v.push_back(cell);
                continue;
            }
            for (int i = 0; i < 8; ++i)
            {
                int xx = x + dx[i];
                int yy = y + dy[i];
                if (xx >= 0 && xx < n && yy >= 0 && yy < m)
                {
                    q.push({xx, yy});
                }
            }
        }
        if (v.size() == 0)
        {
            break;
        }
        for (auto it : v)
        {
            //     mp[it]=id;
            // }
            // ++id;
            if (mp[it] != id)
                dfs_one_cycle(it / m, it % m, dir * 2, dir, mp, id, 0, it / m, it % m, vis);
        }
        // for(int ii=0; ii<env->rows; ++ii){
        //     for(int ji=0; ji<env->cols; ++ji){
        //         out<<char('A'+mp[ii*env->cols+ji]);
        //     }
        //     out<<endl;
        // }
        // out<<endl;
        // out<<endl;
        // for(int ii=0; ii<env->rows; ++ii){
        //     for(int ji=0; ji<env->cols; ++ji){
        //         out<<char('A'+vis[ii*env->cols+ji]);
        //     }
        //     out<<endl;
        // }
        // out<<endl;
        // out<<endl;
        // for(auto it:v){
        //     assert(mp[it]==id);
        // }
        dir = !dir;
        ++id;
        // while(1){
        //     int dx[4] = {0, 1, 0, -1}, dy[4]={1,0,-1,0};
        //     bool loop = false;
        //     for(int j=0; j<n; ++j){
        //         for(int k=0; k<m; ++k){
        //             if(mp[j*m+k])continue;
        //             vector<pair<int, pair<int, int>>> v;
        //             for(int l=0; l<4; ++l){
        //                 int ii = j+dx[l];
        //                 int jj = k+dy[l];
        //                 if(ii>=0 && ii<n && jj>=0 && jj<m && !env->map[ii*m+jj] && mp[ii*m+jj]){
        //                     v.push_back({l,{ii,jj}});
        //                 }
        //             }
        //             if(v.size()>=2){
        //                 int cnt = 0;
        //                 for(auto it:v){
        //                     if(cnt%2==0){
        //                         assert(mp[it.second.first*m+it.second.second]!=0);
        //                         assert(mp[j*m+k]==0);
        //                         assert(nebors[freeCells[it.second.first*m+it.second.second]*4+(it.first+2)%4]==-1);
        //                         nebors[freeCells[j*m+k]*4+it.first]=freeCells[it.second.first*m+it.second.second];
        //                     }
        //                     else{
        //                         assert(nebors[freeCells[j*m+k]*4+it.first]==-1);
        //                         nebors[freeCells[it.second.first*m+it.second.second]*4+(it.first+2)%4]=freeCells[j*m+k];
        //                     }
        //                     ++cnt;
        //                 }
        //                 mp[j*m+k]=id++;
        //                 loop = 1;
        //                 // for(int ii=0; ii<env->rows; ++ii){
        //                 //     for(int ji=0; ji<env->cols; ++ji){
        //                 //         out<<char('A'+mp[ii*env->cols+ji]);
        //                 //     }
        //                 //     out<<endl;
        //                 // }
        //                 // out<<endl;
        //                 // out<<endl;
        //             }
        //         }
        //     }
        //     if(!loop)break;
        // }
        g_vis = vis;
    }
}
bool CityPlanner::dfs_fix_in(int i, int j, int pa)
{
    int m = env->cols, n = env->rows;
    int cell = freeCells[i * m + j], cnt = 0;
    for (int k = 0; k < 4; ++k)
    {
        int ii = i + di[k];
        int jj = j + dj[k];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
        {
            int tmp = freeCells[ii * env->cols + jj];
            if (nebors[tmp * 4 + (k + 2) % 4] != -1)
            {
                assert(nebors[tmp * 4 + (k + 2) % 4] == cell);
                ++cnt;
            }
        }
    }
    if (cnt > 0)
        return 1;
    for (int k = 0; k < 4; ++k)
    {
        int ii = i + di[k];
        int jj = j + dj[k];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
        {
            int tmp = freeCells[ii * env->cols + jj];
            if (tmp != pa)
            {
                nebors[tmp * 4 + (k + 2) % 4] = cell;
                nebors[cell * 4 + k] = -1;
                return dfs_fix_in(ii, jj, cell);
            }
        }
    }
    return false;
}

bool CityPlanner::dfs_fix_out(int i, int j, int pa)
{
    int m = env->cols, n = env->rows;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int cell = freeCells[i * m + j], cnt = 0;
    for (int k = 0; k < 4; ++k)
    {
        int ii = i + di[k];
        int jj = j + dj[k];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj] && nebors[cell * 4 + k] != -1)
        {
            int tmp = freeCells[ii * env->cols + jj];
            // if(nebors[tmp*4+(k+2)%4]!=-1){
            assert(nebors[tmp * 4 + (k + 2) % 4] == -1);
            ++cnt;
            // }
        }
    }
    if (cnt > 0)
        return 1;
    for (int k = 0; k < 4; ++k)
    {
        int ii = i + di[k];
        int jj = j + dj[k];
        if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
        {
            int tmp = freeCells[ii * env->cols + jj];
            if (tmp != pa)
            {
                nebors[tmp * 4 + (k + 2) % 4] = -1;
                nebors[cell * 4 + k] = tmp;
                return dfs_fix_out(ii, jj, cell);
            }
        }
    }
    return false;
}

bool CityPlanner::force_connect(int i, int k)
{
    // assert(env->map[i]==0);
    // assert(env->map[ii]==0);
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int nxt = i+dx[k]*env->cols+dy[k];
    if(env->map[i] || env->map[nxt])return false;
    nebors[freeCells[i] * 4 + k] = freeCells[nxt];
    nebors[freeCells[nxt] * 4 + (k + 2) % 4] = -1;
    return true;
}
void CityPlanner::genAllNeighors()
{
    cout << "NUm of free places " << numOfFreeCells << endl;
    int cnt = 0;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int n = env->rows, m = env->cols;
    vector<int> blocked_cells;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            int cell = i * m + j;
            if (!env->map[cell])
            {
                dangerousPlaceFree[freeCells[cell]] = 0;
                out_nebors_cnt[freeCells[cell]] = 0;
                int cnt = 0;
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    if (ii >= 0 && ii < n && jj >= 0 && jj < m && !env->map[ii * m + jj])
                    {
                        ++cnt;
                    }
                    nebors[freeCells[cell] * 4 + k] = -1;
                    emergency_nebors[freeCells[cell] * 4 + k] = -1;
                }
                // assert(cnt>0);
                if (cnt == 1)
                {
                    blocked_cells.push_back(cell);
                    // dangerousPlace[cell] = 1;
                    // dangerousPlaceFree[freeCells[cell]] = 1;
                }
            }
        }
    }
    // for (auto it : blocked_cells)
    // {
    //     env->map[it] = 1;
    //     --numOfFreeCells;
    // }
    //     // writeMap();
    //     // exit(0);
    // }
    if (false)
    {
        bfs();
    }
    else
    {
        // bfs_outer_cycle();
        vector<int> v(env->cols, 0), h(env->rows, 0);
        for (int i = 0; i < env->cols; ++i)
        {
            v[i] = (i + dir_v) % 2;
        }
        for (int i = 0; i < env->rows; ++i)
        {
            h[i] = (i + dir_h) % 2;
        }
        cnt = 0;
        // int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
        for (int i = 0; i < env->rows; ++i)
        {
            for (int j = 0; j < env->cols; ++j)
            {
                if (!env->map[cnt])
                {
                    int cell = freeCells[cnt];
                    for (int k = 0; k < 4; ++k)
                    {
                        if ((k == 0 && h[i]) || (k == 1 && v[j]) || (k == 2 && !h[i]) || (k == 3 && !v[j]))
                        {
                            int ii = i + di[k];
                            int jj = j + dj[k];
                            int id = cnt + dj[k] + di[k] * env->cols;
                            if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
                            {
                                if (!env->map[id])
                                {
                                    if (nebors[freeCells[id] * 4 + (k + 2) % 4] == -1)
                                    {
                                        nebors[cell * 4 + k] = freeCells[id];
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
                        // else
                        // {
                        //     nebors[cell * 4 + k] = -1;
                        // }
                    }
                }
                ++cnt;
            }
        }
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            if (env->map[i * m + j] == 0)
                // assert(dfs_fix_in(i, j, -1)==1);
                dfs_fix_in(i, j, -1);
        }
    }
    // cout<<dfs_fix_out(27,1, -1)<<endl;
    // exit(0);
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            if (env->map[i * m + j] == 0)
            {
                // assert(dfs_fix_out(i, j, -1)==1);
                dfs_fix_out(i, j, -1);
            }
        }
    }
    ifstream in;
    vector<pair<pair<int, int>, pair<int, int>>> vv, vh;
    // in.open("data/city_lanes_h_new.txt");
    // if (!in.good())
    // {
    //     cout << "wrong file" << endl;
    //     exit(0);
    // }
    int y1, y2, x1, x2;
    // while (in >> y1 >> y2 >> x1 >> x2)
    // {
    //     int i1 = env->rows - 1 - y1;
    //     int i2 = env->rows - 1 - y2;
    //     int j1 = x1;
    //     int j2 = x2;
    //     vh.push_back({{j1, j2}, {i1, i2}});
    // }
    // in.close();
    // in.open("data/city_lanes_v_new.txt");
    // if (!in.good())
    // {
    //     cout << "wrong file" << endl;
    //     exit(0);
    // }
    // while (in >> x1 >> x2 >> y1 >> y2)
    // {
    //     int i1 = env->rows - 1 - y1;
    //     int i2 = env->rows - 1 - y2;
    //     int j1 = x1;
    //     int j2 = x2;
    //     vv.push_back({{i1, i2}, {j1, j2}});
    // }
    // in.close();
    //  for(auto it:vh){
    //     int tmp = (it.second.first + it.second.second) / 2;
    //     for (int j = it.first.first; j + 1 <= it.first.second; ++j)
    //     {
    //         int tmp1 = tmp * env->cols + j;
    //         int tmp2 = (tmp+1) * env->cols + j;
    //         nebors[freeCells[tmp1]*4+1] = -1;
    //         nebors[freeCells[tmp2]*4+3] = -1;
    //     }
    // }
    // for(auto it:vv){
    //     int tmp = (it.second.first + it.second.second) / 2;
    //     for (int i = it.first.first; i + 1 <= it.first.second; ++i)
    //     {
    //         int tmp1 = i * env->cols + tmp;
    //         int tmp2 = i * env->cols + tmp + 1;
    //         nebors[freeCells[tmp1]*4]=-1;
    //         nebors[freeCells[tmp2]*4+2]=-1;
    //     }
    // }
    // for (auto it : vh)
    // {
    //     int tmp = (it.second.first + it.second.second) / 2;
    //     for (int i = it.second.first; i <= tmp; ++i)
    //     {
    //         for (int j = it.first.first; j + 1 <= it.first.second; ++j)
    //         {
    //             int tmp1 = i * env->cols + j;
    //             force_connect(tmp1, 0);
    //         }
    //     }
    //     for (int i = tmp + 1; i <= it.second.second; ++i)
    //     {
    //         for (int j = it.first.first; j + 1 <= it.first.second; ++j)
    //         {
    //             int tmp2 = i * env->cols + j + 1;
    //             force_connect(tmp2, 2);
    //         }
    //     }
    // }
    // for (auto it : vv)
    // {
    //     int tmp = (it.second.first + it.second.second) / 2;
    //     for (int j = it.second.first; j <= tmp; ++j)
    //     {
    //         for (int i = it.first.first; i + 1 <= it.first.second; ++i)
    //         {
    //             int tmp2 = (i + 1) * env->cols + j;
    //             force_connect(tmp2, 3);
    //         }
    //     }
    //     for (int j = tmp + 1; j <= it.second.second; ++j)
    //     {
    //         for (int i = it.first.first; i + 1 <= it.first.second; ++i)
    //         {
    //             int tmp1 = i * env->cols + j;
    //             force_connect(tmp1, 1);
    //         }
    //     }
        
    //     // cout << it.first.first << " " << it.second.first << " " << tmp << " " << it.second.second << endl;
    // }
   
    vector<pair<pair<int, int>, pair<int, int>>> vv_one_direction[2], hh_one_direction[2];
    in.open("data/city_lanes_v_one_direction.txt");
    if (!in.good())
    {
        cout << "wrong file" << endl;
        exit(0);
    }
    int direction;
    while (in >> direction >> x1 >> x2 >> y1 >> y2)
    {
        int j1 = x1;
        int j2 = x2;
        int i1 = env->rows - 1 - y1;
        int i2 = env->rows - 1 - y2;
        vv_one_direction[direction].push_back({{j1, j2}, {i1, i2}});
    }
    in.close();
    
    in.open("data/city_lanes_h_one_direction.txt");
    if (!in.good())
    {
        cout << "wrong file" << endl;
        exit(0);
    }
    while (in >> direction >> y1 >> y2 >> x1 >> x2)
    {
        int j1 = x1;
        int j2 = x2;
        int i1 = env->rows - 1 - y1;
        int i2 = env->rows - 1 - y2;
        hh_one_direction[direction].push_back({{j1, j2}, {i1, i2}});
    }
    in.close();

    for(auto it:vv_one_direction[1]){
        for(int j=it.first.first; j<=it.first.second; ++j){
            for(int i=it.second.first; i+1<=it.second.second; ++i){
                int tmp1 = i * env->cols + j;
                force_connect(tmp1, 1);
            }
        }
    }
    for(auto it:vv_one_direction[0]){
        for(int j=it.first.first; j<=it.first.second; ++j){
            for(int i=it.second.first; i+1<=it.second.second; ++i){
                int tmp2 = (i+1) * env->cols + j;
                force_connect(tmp2, 3);
            }
        }
    }
    for(auto it:hh_one_direction[1]){
        for(int i=it.second.first; i<=it.second.second; ++i){
            for(int j=it.first.first; j+1<=it.first.second; ++j){
                int tmp1 = i * env->cols + j;
                force_connect(tmp1, 0);
            }
        }
    }
    for(auto it:hh_one_direction[0]){
        for(int i=it.second.first; i<=it.second.second; ++i){
            for(int j=it.first.first; j+1<=it.first.second; ++j){
                int tmp2 = i * env->cols + j+1;
                force_connect(tmp2, 2);
            }
        }
    }

    vector<vector<pair<int, int>>> lines[2];
    in.open("data/city_lines.txt");
    if (!in.good())
    {
        cout << "wrong file" << endl;
        exit(0);
    }
    int x, y;
    while (in >> direction)
    {
        auto v = vector<pair<int, int>>();
        while(true){
            in >> y;
            if(y == -1){
                break;
            }
            in >> x;
            int i = env->rows-1-y;
            int j = x;
            v.push_back({i,j});
        }
        lines[direction].push_back(v);
    }
    in.close();
    for(auto it:lines[1]){
        for(int l=1; l+1<it.size(); ++l){
            auto tmp = it[l];
            for(int k=0; k<4; ++k){
                int nxt = (tmp.first+di[k])*env->cols + tmp.second+dj[k];
                nebors[freeCells[nxt]*4+(k+2)%4] = -1;
            }
        }
    }
    for(auto it:lines[0]){
        for(int l=1; l+1<it.size(); ++l){
            auto tmp = it[l];
            for(int k=0; k<4; ++k){
                int nxt = (tmp.first+di[k])*env->cols + tmp.second+dj[k];
                nebors[freeCells[nxt]*4+(k+2)%4] = -1;
            }
        }
    }
    auto mp_tmp = env->map;
    cnt = 2;    
    for(auto it:lines[1]){
        for(int l=0; l+1<it.size(); ++l){
            auto tmp = it[l];
            auto nxt = it[l+1];
            mp_tmp[tmp.first*env->cols+tmp.second]=cnt;
            mp_tmp[nxt.first*env->cols+nxt.second]=cnt;
            if(tmp.first == nxt.first){
                int i = tmp.first;
                if(tmp.second > nxt.second){
                    for(int j=tmp.second; j-1>=nxt.second; --j){
                        force_connect(i*env->cols+j, 2);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
                else{
                    for(int j=tmp.second; j+1<=nxt.second; ++j){
                        force_connect(i*env->cols+j, 0);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
            }
            else{
                assert(tmp.second == nxt.second);
                int j = tmp.second;
                if(tmp.first > nxt.first){
                    for(int i=tmp.first; i-1>=nxt.first; --i){
                        force_connect(i*env->cols+j, 3);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
                else{
                    for(int i=tmp.first; i+1<=nxt.first; ++i){
                        force_connect(i*env->cols+j, 1);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
            }
        }
        ++cnt;
    }
    for(auto it:lines[0]){
        for(int l=0; l+1<it.size(); ++l){
            auto nxt = it[l];
            auto tmp = it[l+1];
            mp_tmp[tmp.first*env->cols+tmp.second]=cnt;
            mp_tmp[nxt.first*env->cols+nxt.second]=cnt;
            if(tmp.first == nxt.first){
                int i = tmp.first;
                if(tmp.second > nxt.second){
                    for(int j=tmp.second; j-1>=nxt.second; --j){
                        force_connect(i*env->cols+j, 2);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
                else{
                    for(int j=tmp.second; j+1<=nxt.second; ++j){
                        force_connect(i*env->cols+j, 0);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
            }
            else{
                assert(tmp.second == nxt.second);
                int j = tmp.second;
                if(tmp.first > nxt.first){
                    for(int i=tmp.first; i-1>=nxt.first; --i){
                        force_connect(i*env->cols+j, 3);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
                else{
                    for(int i=tmp.first; i+1<=nxt.first; ++i){
                        force_connect(i*env->cols+j, 1);
                        mp_tmp[i*env->cols+j]=cnt;
                    }
                }
            }
        }
        ++cnt;
    }


    scc_city::cal_sccs(numOfFreeCells, nebors);
    cout << "number of cycles= " << scc_city::cyc << endl;
    cnt = 0;
    for (int i = 0; i < scc_city::cyc; ++i)
    {
        cnt += scc_city::scc[i].size();
    }
    assert(cnt == numOfFreeCells);
    cnt = 0;
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (!env->map[cnt])
            {
                int cell = freeCells[cnt];
                bool l = 0;
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    int id = cnt + dj[k] + di[k] * env->cols;
                    if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0 && !env->map[id])
                    {
                        id = freeCells[id];
                        assert(id != -1);
                        assert(cell != -1);
                        if (cell < id && scc_city::sccs[cell] != scc_city::sccs[id])
                        {
                            // cout << col[cell] << " " << env->rows - 1 - row[cell] << endl;
                            // cout << "+ " << col[id] << " " << env->rows - 1 - row[id] << endl;
                            nebors[cell * 4 + k] = id;
                            nebors[id * 4 + (k + 2) % 4] = cell;
                            bridge[cell * 4 + k] = 1;
                            bridge[id * 4 + (k + 2) % 4] = 1;
                            bridges.push_back({cell, k});
                            if (scc_city::scc[scc_city::sccs[cell]].size() < scc_city::scc[scc_city::sccs[id]].size())
                            {
                                available[cell * 4 + k] = 1;
                            }
                            else
                            {
                                available[id * 4 + (k + 2) % 4] = 1;
                            }
                        }
                    }
                }
            }
            ++cnt;
        }
    }
    checkBridges();
    writeMap();
    // for(int i=0; i<n; ++i){
    //     for(int j=0; j<m; ++j){
    //         if(!env->map[i*m+j]){
    //             for(int k=0; k<4; ++k){
    //                 int tmp = freeCells[i*m+j]*4+k;
    //                 if(nebors[tmp]!=-1){
    //                     assert(dangerousPlaceFree[nebors[tmp]]==0);
    //                 }
    //             }
    //         }
    //     }
    // }
    // cout<<"Finished generating nebors"<<endl;
}
void CityPlanner::dfs_fill_sizes(int x)
{
    if (sz[x])
    {
        return;
    }
    sz[x] = 1;
    dangerous_branch[x] = 0;
    if(dangerousPlaceFree[curr_states[x].first] && bridge[curr_states[x].first * 4 + curr_states[x].second]){
        dangerous_branch[x] = 1;
    }
    for (auto it : adj[x])
    {
        dfs_fill_sizes(it);
        sz[x] += sz[it];
        if(dangerous_branch[it]){
            dangerous_branch[x] = 1;
        }
    }
}
void CityPlanner::dfs_mark_valids(int x)
{
    allowedToMove[x] = 1;
    if (adj[x].size() == 0)
        return;
    int nxt = adj[x][0], mx = sz[adj[x][0]], mx_dpth = scc_city::comp_depth[scc_city::sccs[curr_states[adj[x][0]].first]], mx_is_turning = 0;
    if (old_paths[nxt].size() <= 1 || (old_paths[nxt][0].first == old_paths[nxt][1].first))
    {
        mx_is_turning = 1;
    }
    int sm = 0;
    bool flagNextIsPrioritized = false;
    for (auto it : adj[x])
    {
         if(dangerous_branch[it])
        // if (dangerousPlaceFree[curr_states[it].first] && bridge[curr_states[it].first * 4 + curr_states[it].second])
        {
            if (sz[it] > mx)
            {
                nxt = it;
                mx = sz[it];
            }
            flagNextIsPrioritized = true;
            // break;
        }
        // if(scc_game::comp_depth[scc_game::sccs[curr_states[it].first]] > mx_dpth){
        //     mx = sz[it];
        //     nxt = it;
        //     mx_dpth = scc_game::comp_depth[scc_game::sccs[curr_states[it].first]];
        // }
        // else if(scc_game::comp_depth[scc_game::sccs[curr_states[it].first]] == mx_dpth){
        // if(old_paths[it].size()<=1 || (old_paths[it][0].first == old_paths[it][1].first)){
        //     if(mx_is_turning){
        if(!flagNextIsPrioritized){
            if (sz[it] > mx)
            {
                nxt = it;
                mx = sz[it];
            }
        }
        //     }
        // }
        // else{
        //     if(mx_is_turning){
        //         mx_is_turning = 0;
        //         nxt = it;
        //         mx = sz[it];
        //     }
        //     else{
        //         if (sz[it] > mx)
        //         {
        //             nxt = it;
        //             mx = sz[it];
        //         }
        //     }
        // }
        // }
        // if (dangerousPlaceFree[curr_states[it].first])
        // {
        //     nxt = it;
        //     break;
        // }
    }
    for (auto it : adj[x])
    {
        sm += sz[it];
    }
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int location = curr_states[x].first;
    for (int k = 0; k < 4; ++k)
    {
        int ii = row[location] + dx[k];
        int jj = col[location] + dy[k];
        if( ii >= 0 && ii < env->rows && jj >=0 && jj < env->cols && env->map[ii*env->cols+jj]==0){
            if (nebors[freeCells[ii * env->cols + jj] * 4 + (k + 2) % 4] == location)
            {
                if (occupied[freeCells[ii * env->cols + jj]] && old_paths[occupied[freeCells[ii * env->cols + jj]] - 1][0].first == location)
                {
                    if (occupied[freeCells[ii * env->cols + jj]] - 1 != nxt)
                        costEdge[freeCells[ii * env->cols + jj] * 4 + (k + 2) % 4] = sm - sz[occupied[freeCells[ii * env->cols + jj]] - 1];
                }
                else
                {
                    costEdge[freeCells[ii * env->cols + jj] * 4 + (k + 2) % 4] = sm;
                }
                costEdgeVector.push_back(freeCells[ii * env->cols + jj] * 4 + (k + 2) % 4);
                // cout<<"Cost:"<<ii<<" "<<jj<<" "<<k<<" = "<<costEdge[freeCells[ii*env->cols+jj]*4+(k+2)%4]<<endl;
            }
        }
    }
    // for (auto it : adj[x])
    // {
    //     if (it != nxt)
    //     {
    //         old_paths[it].clear();
    //     }
    // }
    dfs_mark_valids(nxt);
}
bool CityPlanner::dfs_fill_cycles(int x, int pa)
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
void CityPlanner::markWhoAllowedToMove()
{
    for (auto it : costEdgeVector)
    {
        costEdge[it] = 0;
    }
    costEdgeVector.clear();
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
        int mx = -1, mx_id = i, mx_dpth = scc_city::comp_depth[scc_city::sccs[curr_states[i].first]], mx_is_turning = 0;
        if (old_paths[i].size() <= 1 || (old_paths[i][0].first == old_paths[i][1].first))
        {
            mx_is_turning = 1;
        }
        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            ++cnt[it - 1];
            dfs_fill_sizes(it - 1);
        }
        bool flagMxIsPrioritized = false;
        for (auto it : nxt_occupied[old_paths[i].front().first])
        {
            // if(curr_states[it-1].second%2){
            //     mx_id = it-1;
            //     break;
            // }

            if(dangerous_branch[it-1])
            // if (dangerousPlaceFree[curr_states[it - 1].first] && bridge[curr_states[it - 1].first * 4 + curr_states[it-1].second])
            {
                if (sz[it - 1] > mx)
                {
                    mx_id = it - 1;
                    mx = sz[it - 1];
                }
                // mx_id = it - 1;
                flagMxIsPrioritized = true;
                // break;
            }
            // if(scc_city::comp_depth[scc_city::sccs[curr_states[it-1].first]] > mx_dpth){
            //     mx_dpth = scc_city::comp_depth[scc_city::sccs[curr_states[it-1].first]];
            //     mx_id = it-1;
            //     mx = sz[it-1];
            // }
            // else if(scc_city::comp_depth[scc_city::sccs[curr_states[it-1].first]] == mx_dpth){
            // if(old_paths[it-1].size()<=1  (old_paths[it-1][0].first == old_paths[it-1][1].first)){
            //     if(mx_is_turning){
            //         if (sz[it - 1] > mx)
            //         {
            //             mx_id = it - 1;
            //             mx = sz[it - 1];
            //         }
            //     }
            // }
            // else{
            // if(mx_is_turning){
            //     mx_is_turning = 0;
            //     mx_id = it-1;
            //     mx = sz[it-1];
            // }
            // else{
            if(!flagMxIsPrioritized){
                if (sz[it - 1] > mx)
                {
                    mx_id = it - 1;
                    mx = sz[it - 1];
                }
            }
            // }
            // }

            // }
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
void CityPlanner::move_away(int id)
{
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    int cell = curr_states[id].first, dr = curr_states[id].second;
    deque<pair<int, int>> new_path;
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

void CityPlanner::writeMap()
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
    out << scc_city::cyc << endl;
    for (int i = 0; i < scc_city::cyc; ++i)
    {
        l = 1;
        for (int j = 0; j < env->rows; ++j)
        {
            for (int k = 0; k < env->cols; ++k)
            {
                if (env->map[j * env->cols + k] == 0 && scc_city::comp_depth[scc_city::sccs[freeCells[j * env->cols + k]]] == i)
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
                if (env->map[j * env->cols + k] == 0 && scc_city::comp_depth[scc_city::sccs[freeCells[j * env->cols + k]]] == i)
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
                if ((nebors[cell * 4 + k] != -1) && (!bridge[cell * 4 + k] || available[cell * 4 + k]))
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

void CityPlanner::writeGoals(){
    ofstream out;
    out.open("goals"+to_string(timeStep)+".txt");
    // for(auto it:safe_places_vector){
    //     int i = row[it];
    //     int j = col[it];
    //     out<<j<<" "<<env->rows-1-i<<endl;
    // }
    for(int ii=0; ii<env->num_of_agents; ++ii){
        int goal = goal_locations[ii];
        if(goal < 0)continue;
        int i = row[goal];
        int j = col[goal];
        out<<j<<" "<<env->rows-1-i<<endl;
    }
    out.close();
}
void CityPlanner::writePaths()
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

void CityPlanner::checkBridges()
{
    vector<int> dangerousCells;
    for (auto it : bridges)
    {
        int cell1 = it.first;
        int cell2 = nebors[it.first * 4 + it.second];
        int comp1 = scc_city::sccs[cell1];
        int comp2 = scc_city::sccs[cell2];
        if (scc_city::comp_depth[comp1] > scc_city::comp_depth[comp2])
        {
            bool l = dfsOccInBigDpth(comp1);
            if (l)
            {
                available[cell1 * 4 + it.second] = 1;
                available[cell2 * 4 + (it.second + 2) % 4] = 0;
                dangerousPlaceFree[cell2] = 1;
                dangerousCells.push_back(cell2);
                dangerousPlaceFree[cell1] = 0;
            }
            else
            {
                available[cell1 * 4 + it.second] = 0;
                available[cell2 * 4 + (it.second + 2) % 4] = 1;
                dangerousPlaceFree[cell1] = 1;
                dangerousCells.push_back(cell1);
                dangerousPlaceFree[cell2] = 0;
            }
        }
        else
        {
            bool l = dfsOccInBigDpth(comp2);
            if (l)
            {
                available[cell1 * 4 + it.second] = 0;
                available[cell2 * 4 + (it.second + 2) % 4] = 1;
                dangerousPlaceFree[cell1] = 1;
                dangerousCells.push_back(cell1);
                dangerousPlaceFree[cell2] = 0;
            }
            else
            {
                available[cell1 * 4 + it.second] = 1;
                available[cell2 * 4 + (it.second + 2) % 4] = 0;
                dangerousPlaceFree[cell2] = 1;
                dangerousCells.push_back(cell2);
                dangerousPlaceFree[cell1] = 0;
            }
        }
    }
    for(auto it:dangerousCells){
        dangerousPlaceFree[it] = 1;
    }
}
void CityPlanner::writePathsSteps()
{
    std::ofstream out;
    out.open("paths_steps-city-" + to_string(timeStep) + ".txt");
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

// bool CityPlanner::canGetToGoal(int i)
// {
//     int comp1 = scc_city::sccs[curr_states[i].first];
//     int comp2 = scc_city::sccs[goal_locations[i]];
//     if(comp1==comp2)return true;
//     if(scc_city::scc[comp1].size()<scc_city::scc[comp2].size()){
//         if(bridge[])
//     }
//     return false;
// }

bool CityPlanner::dfsOccInBigDpth(int comp)
{
    int dp = scc_city::comp_depth[comp];
    for (auto c : scc_city::scc[comp])
    {
        if (occupied[c])
        {
            return true;
        }
    }
    for (auto cmp : scc_city::comps[comp])
    {
        if (scc_city::comp_depth[cmp] <= dp)
            continue;
        if (dfsOccInBigDpth(cmp))
            return true;
    }
    return false;
}
void CityPlanner::assertBridges()
{
    for (auto it : bridges)
    {
        int cell1 = it.first;
        int cell2 = nebors[it.first * 4 + it.second];
        int comp1 = scc_city::sccs[cell1];
        int comp2 = scc_city::sccs[cell2];
        if (scc_city::comp_depth[comp1] > scc_city::comp_depth[comp2])
        {
            bool l = dfsOccInBigDpth(comp1);
            if (l)
            {
                assert(available[cell1 * 4 + it.second] == 1);
                assert(available[cell2 * 4 + (it.second + 2) % 4] == 0);
            }
            else
            {
                assert(available[cell1 * 4 + it.second] == 0);
                assert(available[cell2 * 4 + (it.second + 2) % 4] == 1);
            }
        }
        else
        {
            bool l = dfsOccInBigDpth(comp2);
            if (l)
            {
                assert(available[cell1 * 4 + it.second] == 0);
                assert(available[cell2 * 4 + (it.second + 2) % 4] == 1);
            }
            else
            {
                assert(available[cell1 * 4 + it.second] == 1);
                assert(available[cell2 * 4 + (it.second + 2) % 4] == 0);
            }
        }
    }
}

void CityPlanner::print_location(int loc){
    cout<<col[loc]<<" "<<env->rows-1-row[loc]<<endl;
}
void CityPlanner::print_agent_information(int ag){
    cout<<"Agent :"<<ag<<endl;
    cout<<"current location:";
    print_location(curr_states[ag].first);
    cout<<"Goal:";
    print_location(goal_locations[ag]);
    cout<<"Path size:"<<old_paths[ag].size()<<"::"<<endl;
    for(auto it:old_paths[ag]){
        print_location(it.first);
    }
    cout<<"was cleared: "<<wasCleared[ag]<<endl;
    cout<<"----"<<endl;
}
// plan using simple A* that ignores the time dimension
void CityPlanner::plan(int time_limit, vector<Action> &actions)
{
    start_plan_time = high_resolution_clock::now();
    ++timeStep;
    if (timeStep == 0)
    {
        wasCleared = new int[env->num_of_agents]();
        dangerous_branch = new int[env->num_of_agents]();
        old_paths = new deque<pair<int, int>>[env->num_of_agents]();
        tmp_paths = new deque<pair<int, int>>[env->num_of_agents]();
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
        for(int i=0; i<env->num_of_agents; ++i){
            wasCleared[i] = 0;
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        occupied[curr_states[i].first] = 0;
    }
    for(int i=0; i<env->num_of_agents; ++i){
        curr_states[i] = make_pair(freeCells[env->curr_states[i].location], env->curr_states[i].orientation);
        occupied[curr_states[i].first] = i + 1;
    }
    if (timeStep == 0)
    {
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            mapFree[curr_states[i].first] = 1;
        }
        checkBridges();
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
                assert(0);
                return;
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (allowedToMove[i] && old_paths[i].size() > 0)
        {
            --cellCost[old_paths[i].front().first];
            old_paths[i].pop_front();
        }
    }
    if (timeStep > 0)
    {
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            if (goal_locations[i] != -1)
            {
                isGoal[goal_locations[i]] = 0;
            }
        }
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            isGoal[freeCells[env->goal_locations[i].front().first]] = 1;
        }
    }

    map<int, int> mp_gaol_cnt;
    int pointer_to_safe_places = 0;
    int num_of_repeated_goals = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        ++mp_gaol_cnt[freeCells[env->goal_locations[i].front().first]];
        if(mp_gaol_cnt[freeCells[env->goal_locations[i].front().first]]>1)++num_of_repeated_goals;
    }

    mp_gaol_cnt.clear();
    for(int i=0; i<env->num_of_agents; ++i){
        ++mp_gaol_cnt[freeCells[env->goal_locations[i].front().first]];
        if (env->goal_locations[i].empty() || mp_gaol_cnt[freeCells[env->goal_locations[i].front().first]]>1)
        {
            goal_locations[i] = 0;//curr_states[i].first;
            while(pointer_to_safe_places < safe_places_vector.size()){
                if(!isGoal[safe_places_vector[pointer_to_safe_places]]){
                    goal_locations[i] = safe_places_vector[pointer_to_safe_places];
                    isGoal[goal_locations[i]] = 1;
                    pointer_to_safe_places += (safe_places_vector.size()-pointer_to_safe_places)/(num_of_repeated_goals+1);
                    --num_of_repeated_goals;
                    break;
                }
                ++pointer_to_safe_places;
            }
        }
        else
        {
            // if (goal_locations[i] != freeCells[env->goal_locations[i].front().first])
            // {
            //     solved[i] = false;
            // }
            // goal_locations[i] = 0;
            if(goal_locations[i] != freeCells[env->goal_locations[i].front().first]){
                for(auto it:old_paths[i]){
                    --cellCost[it.first];
                }
                old_paths[i].clear();
            }
            goal_locations[i] = freeCells[env->goal_locations[i].front().first];
            isGoal[goal_locations[i]] = 1;
        }
    }
    // int tmp = 0;
    // for(int i=0; i<env->num_of_agents; ++i){
    //     tmp += old_paths[i].size();
    // }
    // for(int i=0; i<numOfFreeCells; ++i){
    //     assert(cellCost[i]>=0);
    //     tmp -= cellCost[i];
    // }
    // assert(tmp==0);
    checkBridges();
    // writeGoals();
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W); 
    int cnt2 = 0, cnt1 = 0, cnt3 = 0, cnt4 = 0;
    thread threads[processor_count];
    for(int i=0; i<processor_count; ++i){

        if(i < processor_count-1){
            threads[i] = thread(&CityPlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents/processor_count*(i+1));
        }
        else{
            threads[i] = thread(&CityPlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents);
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
            ++cellCost[curr_states[i].first];
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
    // for(int i=0; i<numOfFreeCells; ++i){
    //     cellCost[i] = 0;
    // }
    // for(int i=0; i<env->num_of_agents; ++i){
    //     for(auto &it:old_paths[i]){
    //         ++cellCost[it.first];
    //     }
    // }
    cout<<cnt<<"///";
    for(int i=0; i<processor_count; ++i){
        cout<<shift_id[i]<<" ";
    }
    cout<<"/"<<env->num_of_agents<<endl;
    // for (int i = 0; i < env->num_of_agents; ++i)
    // {
    //     if (old_paths[i].size() > 0)
    //     {
    //         bool l = 1;
    //         if (old_paths[i][0].first != curr_states[i].first)
    //         {
    //             if (bridge[curr_states[i].first * 4 + curr_states[i].second] && !available[curr_states[i].first * 4 + curr_states[i].second])
    //             {
    //                 l = 0;
    //             }
    //             if (!(bridge[curr_states[i].first * 4 + curr_states[i].second] && available[curr_states[i].first * 4 + curr_states[i].second]) && dangerousPlaceFree[old_paths[i][0].first]){
    //                 l = 0;
    //             }
    //         }
    //         assert(l == 1);
    //     }
    // }
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
    markWhoAllowedToMove();
    // writeMap();
    // writePaths();
    cnt1 = 0, cnt2 = 0, cnt3 = 0;
    cnt4 = 0;
    int cnt5 = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        // assert(it.size()>0);
        if (it.size() > 0)
        {
            if (old_paths[i][0].first == curr_states[i].first)
            {
                ++cnt3;
                if (old_paths[i][0].second != curr_states[i].second)
                    ++cnt5;
            }
            if (old_paths[i].back().first == goal_locations[i])
            {
                ++cnt2;
            }
        }
        else
        {
            ++cnt4;
        }
        // else
        // {
        //     // cout<<i<<endl;
        //     // pair<int, int> tmp = it.front();
        //     // cout<<i<<" "<<col[curr_states[i].first]<<", "<<env->rows-1-row[curr_states[i].first]<<endl;
        //     // // cout<<i<<" "<<row[curr_states[i].first]<<" "<<col[curr_states[i].first]<<endl;
        //     // cout<<i<<" "<<col[tmp.first]<<", "<<env->rows-1-row[tmp.first]<<endl;
        //     // // cout<<i<<" "<<row[tmp.first]<<" "<<col[tmp.first]<<endl;
        //     // cout<<i<<" "<<col[goal_locations[i]]<<", "<<env->rows-1-row[goal_locations[i]]<<endl;
        //     // // cout<<i<<" "<<row[goal_locations[i]]<<" "<<col[goal_locations[i]]<<endl;
        // }
        if (allowedToMove[i])
        {
            ++cnt1;
            pair<int, int> tmp = it.front();
            if (tmp.first != curr_states[i].first)
            {
                lastActions[i] = actions[i] = Action::FW; // forward action
                // available[tmp.first * 4 + (curr_states[i].second + 2) % 4] = 1;
                // if(i==303){
                //     cout<<i<<endl;
                //     cout<<i<<" "<<col[curr_states[i].first]<<", "<<env->rows-1-row[curr_states[i].first]<<endl;
                //     cout<<i<<" "<<row[curr_states[i].first]<<" "<<col[curr_states[i].first]<<endl;
                //     cout<<i<<" "<<col[tmp.first]<<", "<<env->rows-1-row[tmp.first]<<endl;
                //     cout<<i<<" "<<row[tmp.first]<<" "<<col[tmp.first]<<endl;
                // }
                // if (bridge[curr_states[i].first * 4 + curr_states[i].second])
                //     assert(available[curr_states[i].first * 4 + curr_states[i].second] == 1);
                // available[curr_states[i].first * 4 + curr_states[i].second] = 0;
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
        // else{
        //     old_paths[i].clear();
        // }
    }
    cout << "number of allowed to move = " << cnt1 << endl;
    cout << "number to move to goal = " << cnt2 << endl;
    cout << "number of constant agents = (turning)" << cnt5 << "/ " << cnt3 << endl;
    cout << "number of cleared agents = " << cnt4 << endl;
    // if (cnt1 < 1000)
    // {
    //     writePathsSteps();
    //     // writeCosts();
    //     // exit(0);
    // }
    // int cnt = 0;
    // for(int i=0; i<env->num_of_agents; ++i){
    //     cnt += allowedToMove[i] && !ignore[i];
    // }
    // if(cnt==0){
    //     for(int i=0; i<env->num_of_agents; ++i){
    //         ignore[i]=0;
    //         old_paths[i].clear();
    //         solved[i] = 0;
    //     }
    //     // exit(0);
    //     dir_h = drh[dr];
    //     dir_v = drv[dr];
    //     dr = (dr+1)%4;
    //     genAllNeighors();
    // }
    // assertBridges();

    // assert(std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() < 1000000);
    return;
}
void CityPlanner::partial_plan(int thread_id, int start_id, int end_id)
{
    // cout<<thread_id<<", "<<start_id<<" "<<end_id<<endl;
    // auto start_plan_time = high_resolution_clock::now();
    vector<pair<int, int>> v;
    for (int i = start_id; i < end_id; ++i)
    {
        v.push_back({-scc_city::comp_depth[scc_city::sccs[curr_states[i].first]], i});
    }
    sort(v.begin(), v.end());
    for (int i = start_id; i < end_id; ++i)
    {
        bool l = 1;
        if (old_paths[i].size() > 0)
        {
            if (old_paths[i][0].first != curr_states[i].first)
            {
                if (bridge[curr_states[i].first * 4 + curr_states[i].second] && !available[curr_states[i].first * 4 + curr_states[i].second])
                {
                    l = 0;
                }
            }
            if (!l)
            {
                wasCleared[i] = 1; //curr_states[i].first * 4 + curr_states[i].second;
                
                mutex_cellCost.lock();
                for(auto it:old_paths[i]){
                    --cellCost[it.first];
                }
                mutex_cellCost.unlock();
                old_paths[i].clear();
            }
        }
    }
    for (int ii=0; ii<v.size(); ii++)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = v[ii].second;
        if (goal_locations[i] != -1 && old_paths[i].size() == 0)
        {
            allowedToMove[i]=1;
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i], wasCleared[i]);
            if(old_paths[i].size() > 0 &&  old_paths[i].back().first == goal_locations[i]){
                wasCleared[i] = 0;
            }

            mutex_cellCost.lock();
            for(auto it:old_paths[i]){
                ++cellCost[it.first];
            }
            mutex_cellCost.unlock();
        }
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
            // if(old_paths[i].size()>0 && old_paths[i].back().first==goal_locations[i])continue;
            auto tmp_path = old_paths[i];

            mutex_cellCost.lock();
            for(auto it:old_paths[i]){
                --cellCost[it.first];
            }
            mutex_cellCost.unlock();
            old_paths[i].clear();
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i], wasCleared[i]);
            // if(old_paths[i].size() > 0 &&  old_paths[i].back().first == goal_locations[i]){
            //     wasCleared[i] = 0;
            // }
            // else{
            //     old_paths[i] = tmp_path;
            // }
            mutex_cellCost.lock();
            for(auto it:old_paths[i]){
                ++cellCost[it.first];
            }
            mutex_cellCost.unlock();
        }
    }
}
void CityPlanner::single_agent_plan(int thread_id, int start, int start_direct, int end, deque<pair<int, int>> &path, bool wasCleared)
{
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, 0, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    AstarNode *ans_node = nullptr, *second_ans = nullptr;
    bool solved = false, solved_second = false;
    while (!open_list.empty())
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
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
        // if(safePlaceFree[curr->location] && !solved_second){
        //     second_ans = curr;
        //     solved_second = true;
        // }
        // if(start == 286){
        //     cout<<curr->g<<endl;
        //     cout<<col[curr->location]<<" "<<env->rows-1-row[curr->location]<<endl;
        // }
        if (curr->location != start && scc_city::sccs[curr->location] == scc_city::main_component && !dangerousPlaceFree[curr->location])// && safePlaceFree[curr->location])
        {
            if (solved_second)
            {
                if(occupied[second_ans->location])
                // if (getManhattanDistance(start, curr->location) > getManhattanDistance(start, second_ans->location))
                {
                    second_ans = curr;
                }
                else{
                    if(!occupied[curr->location] && getManhattanDistance(end, curr->location) < getManhattanDistance(end, second_ans->location)){
                        second_ans = curr;
                    }
                }
            }
            else
            {
                second_ans = curr;
                solved_second = true;
            }
        }
        closed_list[thread_id][id] = 1;
        // if(dangerousPlaceFree[curr->location] && curr->location!=start)continue;
        // if(occupied[curr->location]>0){
        //     if(curr_states[occupied[curr->location]].second != start_direct){
        //         cost += 4;
        //     }
        // }
        // cost += 10000 * int(cost>0 && curr->location==start);
        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction, curr->location == start, wasCleared);
        for (const pair<int, int> &neighbor : neighbors)
        {
            // if (occupied[neighbor.first] > 0 &&
            //     curr_states[occupied[neighbor.first]].second != curr->direction)
            // {
            //     AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
            //                                          cost + 100 + W2, W2 * getManhattanDistance(neighbor.first, end), curr);
            //     open_list.push(next_node);
            //     all_nodes.push_back(next_node);
            // }
            // else
            // {
            if (neighbor.first == curr->location)
            {
                AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                     cost + W1 + W2 * cellCost[neighbor.first], W1 * getManhattanDistance(neighbor.first, end), curr);
                open_list.push(next_node);
                all_nodes.push_back(next_node);
            }
            else
            {
                int cost_add = 0;
                if (occupied[neighbor.first] > 0)
                {
                    cost_add += W2;
                    // if (old_paths[occupied[neighbor.first] - 1][0].first == curr_states[occupied[neighbor.first] - 1].first)
                    // {
                    //     cost_add += W2;
                    // }
                } // + costEdge[curr->location * 4 + curr->direction]
                AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                     cost + W1 + W2 * cellCost[neighbor.first], W1 * getManhattanDistance(neighbor.first, end), curr);
                open_list.push(next_node);
                all_nodes.push_back(next_node);
            }
            // }
        }
        // int forward = emergency_nebors[curr->location*4+curr->direction];
        // if(forward == end && !occupied[end]){
        //     AstarNode *next_node = new AstarNode(forward, curr->direction,
        //                                          cost+W2, 0, curr);
        //     open_list.push(next_node);
        //     all_nodes.push_back(next_node);
        // }
    }
    if (solved)
    {
        while (ans_node->parent != NULL)
        {
            path.emplace_front(make_pair(ans_node->location, ans_node->direction));
            ans_node = ans_node->parent;
        }
    }
    else if (solved_second)
    {
        while (second_ans->parent != NULL)
        {
            path.emplace_front(make_pair(second_ans->location, second_ans->direction));
            second_ans = second_ans->parent;
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

int CityPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

list<pair<int, int>> CityPlanner::getNeighbors(int location, int direction, bool is_start, bool wasCleared)
{
    list<pair<int, int>> neighbors;
    // forward
    int forward = nebors[location * 4 + direction];
    if (forward != -1)
    {
        // if (is_start || wasCleared)
        // {
            // if(dangerousPlaceFree[location]){
            if (!bridge[location * 4 + direction] || available[location * 4 + direction])
            {
                neighbors.push_back({forward, direction});
            }
            // }
            // else{
            //     if (!dangerousPlaceFree[forward] && (!bridge[location * 4 + direction] || available[location * 4 + direction]))
            //     {
            //         neighbors.push_back({forward, direction});
            //     }
            // }
        // }
        // else
        // {
        //     neighbors.push_back({forward, direction});
        // }
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
