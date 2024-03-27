#include <RandomPlanner.h>
#include <random>
using namespace std::chrono;

struct AstarNode
{
    AstarNode(){}
    int location;
    int direction;
    int f, g, h;
    int parent;
    bool goalWasVisited;
    AstarNode(int _location, int _direction, int _g, int _h, int _parent, bool goalWasVisited) : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent), goalWasVisited(goalWasVisited) {}
};

struct cmp
{
    bool operator()(AstarNode a, AstarNode b)
    {
        if (a.f == b.f)
            return a.g <= b.g;
        else
            return a.f > b.f;
    }
};

void RandomPlanner::initialize(int preprocess_time_limit)
{
    shift_id = 0;
    shift_id1 = 0;
    max_time_limit = 970000;
    
    // if(env->num_of_agents > 100){

    // }
    safePlace = new bool[(env->cols) * (env->rows)]();
    dangerousPlace = new bool[(env->cols) * (env->rows)]();
    findSafePlaces();
    timeStep = -1;
    stopFlag = false;
    freeCells = new int [env->map.size()]();
    compressFreeCells();
    genAllNeighbors();
}


void RandomPlanner::dfs_one_cycle(int i, int j, int dir, bool ccw, vector<int> &mp, int id, int cnt, int i_st, int j_st, vector<int> &vis)
{
    int m = env->cols, n = env->rows;
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int cell = freeCells[i * m + j];
    dangerousPlace[i*m+j] = 1;
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
                // if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1){
                //     nebors[cell * 4 + kk] = freeCells[nxt];
                //     // dangerousPlace[cell * 4 + kk] = 1;
                // }
                lastEnd = true;
                // break;
            }
            if (mp[nxt])
            {
                if (!env->map[nxt])
                {
                    if (mp[nxt] == id)
                    {
                        // if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        // {
                        //     nebors[cell * 4 + kk] = freeCells[nxt];
                        //     // dangerousPlace[cell * 4 + kk] = 1;
                        // }
                    }
                    else
                    {
                        // if(id==4){
                        //     cout<<"###"<<cnt<<endl;
                        // }
                        // if (cnt % 2)
                        // {
                        //     if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        //     {
                        //         nebors[cell * 4 + kk] = freeCells[nxt];
                        //         // dangerousPlace[cell * 4 + kk] = 1;
                        //         // didConnect = 1;
                        //         ++cnt;
                        //     }
                        // }
                        // else
                        // {
                        //     if (nebors[cell * 4 + kk] == -1)
                        //     {
                        //         nebors[freeCells[nxt] * 4 + (kk + 2) % 4] = cell;
                        //         // dangerousPlace[freeCells[nxt] * 4 + (kk + 2) % 4] = 1;
                        //         // didConnect = 1;
                        //         ++cnt;
                        //     }
                        // }
                    }
                }
                continue;
            }
            // if(onTheEdge(ii, jj, kk, ccw, mp, id)){
            if (vis[nxt])
            {
                // nebors[cell * 4 + kk] = freeCells[nxt];
                onTheEnd = false;
                // assert(nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1);
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
                // if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1){
                //     nebors[cell * 4 + kk] = freeCells[nxt];
                //     // dangerousPlace[cell * 4 + kk] = 1;

                // }
                lastEnd = true;
                // break;
            }
            if (mp[nxt])
            {
                if (!env->map[nxt])
                {
                    if (mp[nxt] == id)
                    {
                        // if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        // {
                        //     nebors[cell * 4 + kk] = freeCells[nxt];
                        //     // dangerousPlace[cell * 4 + kk] = 1;
                        // }
                    }
                    else
                    {
                        // if(id==4){
                        //     cout<<"###"<<cnt<<endl;
                        // }
                        // if (cnt % 2)
                        // {
                        //     if (nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1)
                        //     {
                        //         nebors[cell * 4 + kk] = freeCells[nxt];
                        //         // dangerousPlace[cell * 4 + kk] = 1;
                        //         // didConnect = 1;
                        //         ++cnt;
                        //     }
                        // }
                        // else
                        // {
                        //     if (nebors[cell * 4 + kk] == -1)
                        //     {
                        //         nebors[freeCells[nxt] * 4 + (kk + 2) % 4] = cell;
                        //         // dangerousPlace[freeCells[nxt] * 4 + (kk + 2) % 4] = 1;
                        //         // didConnect = 1;
                        //         ++cnt;
                        //     }
                        // }
                    }
                }
                continue;
            }
            // cout<<ii<<",,,"<<jj<<endl;
            // if(onTheEdge(ii, jj, kk, ccw, mp, id)){
            if (vis[nxt])
            {
                // nebors[cell * 4 + kk] = freeCells[nxt];
                onTheEnd = false;
                // assert(nebors[freeCells[nxt] * 4 + (kk + 2) % 4] == -1);
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
                    // nebors[freeCells[i * m + j] * 4 + k] = freeCells[ii * m + jj];
                    // assert(mp[ii*m+jj]<id);
                    // assert(nebors[freeCells[ii*m+jj]*4+(k+2)%4]==-1);
                    // nebors[freeCells[ii * m + jj] * 4 + (2 + k) % 4] = -1;
                    // notOnTheEnd = true;
                    break;
                    // return true;
                }
            }
    }
    // return lastEnd || touchedTheEnd;
}
void RandomPlanner::bfs_outer_cycle()
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
    // for(int i=0; i<env->rows; ++i){
    //     for(int j=0; j<env->cols; ++j){
    //         if(mp[i*env->cols+j]){
    //             q.push({i,j});
    //         }
    //     }
    // }
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

void RandomPlanner::findSafePlaces()
{
    bfs_outer_cycle();
    for(int i=0; i<env->cols; ++i){
        dangerousPlace[13*env->cols+i]=1;
        dangerousPlace[14*env->cols+i]=1;
        dangerousPlace[15*env->cols+i]=1;
        dangerousPlace[16*env->cols+i]=1;
    }
    for(int i=0; i<env->rows; ++i){
        dangerousPlace[i*env->cols+13]=1;
        dangerousPlace[i*env->cols+14]=1;
        dangerousPlace[i*env->cols+15]=1;
        dangerousPlace[i*env->cols+16]=1;
    }
    // int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    //     int cnt = 0;
    //     for(int i=0; i<env->rows; ++i){
    //         for(int j=0; j<env->cols; ++j){
    //             if(env->map[i*env->cols+j]==0){
    //                 vector<int> v;
    //                 for(int k=0; k<8; ++k){
    //                     int l1 = -1;
    //                     int ii = i+dx[k];
    //                     int jj = j+dy[k];
    //                     if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //                         if(k%2==0)
    //                             l1 = 0;
    //                     }
    //                     else{
    //                         l1=1;
    //                     }
    //                     if(l1!=-1)
    //                         v.push_back(l1);
    //                 }
    //                 int tmp = 0;
    //                 for(int i=0;i<v.size();++i){
    //                     if(v[i]!=v[(i+1)%v.size()])++tmp;
    //                 }
    //                 if(tmp>=3){
    //                     dangerousPlace[i*env->cols+j]=1;
    //                 }
    //             }
    //         }
    //     }
    //     for(int i=0; i<env->rows; ++i){
    //         for(int j=0; j<env->cols; ++j){
    //             if(env->map[i*env->cols+j]==0){
    //                 int l = 0;
    //                 for(int k=0; k<8; k+=2){
    //                     bool l1, l2;
    //                     int ii = i+dx[k];
    //                     int jj = j+dy[k];
    //                     if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //                         // l1 = 0;
    //                     }
    //                     else{
    //                         ++l;
    //                         // l1=1;
    //                     }
    //                     // ii = i+dx[(k+2)%4];
    //                     // jj = j+dy[(k+2)%4];
    //                     // if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //                     //     l2 = 0;
    //                     // }
    //                     // else{
    //                     //     l2=1;
    //                     // }
    //                 }
    //                 if(l==3){
    //                     dangerousPlace[i*env->cols+j]=1;
    //                     // cout<<i*env->cols+j<<endl;
    //                     // ++cnt;
    //                     // break;
    //                 }
    //             }
    //         }
    //     }
    //     for(int i=0; i<env->rows; ++i){
    //         for(int j=0; j<env->cols; ++j){
    //             if(env->map[i*env->cols+j]==0){
    //                 int l = 0;
    //                 for(int k=0; k<8; ++k){
    //                     bool l1, l2;
    //                     int ii = i+dx[k];
    //                     int jj = j+dy[k];
    //                     if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0 && safePlace[ii*env->cols+jj]==0){
    //                         // l1 = 0;
    //                     }
    //                     else{
    //                         ++l;
    //                         // l+=safePlace[ii*env->cols+jj];
    //                         // l1=1;
    //                     }
    //                     // ii = i+dx[(k+2)%4];
    //                     // jj = j+dy[(k+2)%4];
    //                     // if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //                     //     l2 = 0;
    //                     // }
    //                     // else{
    //                     //     l2=1;
    //                     // }
    //                 }
    //                 if(l<=2 && dangerousPlace[i*env->cols+j]==0){
    //                     safePlace[i*env->cols+j]=1;
    //                     // assert(dangerousPlace[i*env->cols+j]==0);
    //                     // cout<<i*env->cols+j<<endl;
    //                     ++cnt;
    //                     // break;
    //                 }
    //             }
    //         }
    //     }
    //     // cout<<"Number of safe places = "<<cnt<<endl;
    
    // // exit(0);
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

void RandomPlanner::compressFreeCells(){
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows*env->cols; ++i)
    {
        if (!env->map[i])
        {
            freeCells[i] = numOfFreeCells;
            ++numOfFreeCells;
        }
    }
    mapFree = new int [numOfFreeCells]();
    std::fill(mapFree, mapFree+numOfFreeCells, INT_MAX);
    nebors = new int[numOfFreeCells*4]();
    map = new std::set<pair<int, int>>[numOfFreeCells]();
    row = new int [numOfFreeCells]();
    col = new int [numOfFreeCells]();
    isGoal = new int [numOfFreeCells]();
    safePlaceFree = new bool[numOfFreeCells]();
    dangerousPlaceFree = new bool[numOfFreeCells]();
    for (int i = 0; i < 4; ++i)
    {
        blocked_edges[i] = new std::set<int> [numOfFreeCells]();
    }
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows*env->cols; ++i)
    {
        if (!env->map[i])
        {
            row[numOfFreeCells] = i/env->cols;
            col[numOfFreeCells] = i%env->cols;
            if(safePlace[i]){
                safePlaceFree[numOfFreeCells]=1;
            }
            if(dangerousPlace[i]){
                dangerousPlaceFree[numOfFreeCells]=1;
            }
            ++numOfFreeCells;
        }
    }
}
void RandomPlanner::genAllNeighbors()
{
    int cnt = 0;
    int di[4] = {0, 1, 0, -1}, dj[4] = {1, 0, -1, 0};
    for (int i = 0; i < env->rows; ++i)
    {
        for (int j = 0; j < env->cols; ++j)
        {
            if (!env->map[cnt])
            {
                int cell = freeCells[cnt];
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    int id = cnt + dj[k] + di[k] * env->cols;
                    if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
                    {
                        if (!env->map[id])
                        {
                            nebors[cell*4+k] = freeCells[id];
                        }
                        else{
                            nebors[cell*4+k] = -1;
                        }
                    }
                    else{
                        nebors[cell*4+k] = -1;
                    }
                }
            }
            ++cnt;
        }
    }
    cnt = 0;
    // for (int i = 0; i < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    //         if (env->map[cnt])
    //         {
    //             int cell = freeCells[cnt];
    //             bool l = 1;
    //             for (int k = 0; k < 8; ++k)
    //             {
    //                 int ii = i + dx[k];
    //                 int jj = j + dy[k];
    //                 int id = cnt + dy[k] + dx[k] * env->cols;
    //                 if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
    //                 {
    //                     if (env->map[id])
    //                     {
    //                         l = 0;
    //                         break;
    //                     }
    //                 }
    //                 else{
    //                     l = 0;
    //                     break;
    //                 }
    //             }
    //             if(l){
    //                 int tmp, tmp1, tmp2;
    //                 if(nebors[freeCells[(i-1)*env->cols+j]*4+2]!=-1 && nebors[freeCells[(i-1)*env->cols+j+1]*4+2]!=-1 
    //                 && nebors[freeCells[(i)*env->cols+j-1]*4+1]!=-1 && nebors[freeCells[(i-1)*env->cols+j-1]*4+1]!=-1
    //                 && nebors[freeCells[(i)*env->cols+j+1]*4+3]!=-1 && nebors[freeCells[(i+1)*env->cols+j+1]*4+3]!=-1
    //                 ){
    //                     tmp = freeCells[(i-1)*env->cols+j-1];
    //                     tmp1 = freeCells[(i-1)*env->cols+j];
    //                     tmp2 = freeCells[(i-1)*env->cols+j+1];
    //                     nebors[tmp*4]=-1;
    //                     nebors[tmp1*4]=-1;
    //                     tmp = freeCells[(i+1)*env->cols+j-1];
    //                     tmp1 = freeCells[(i+1)*env->cols+j];
    //                     tmp2 = freeCells[(i+1)*env->cols+j+1];
    //                     nebors[tmp1*4+2]=-1;
    //                     nebors[tmp2*4+2]=-1;
    //                 // }
    //                 // if(nebors[freeCells[(i)*env->cols+j-1]*4+1]!=-1 && nebors[freeCells[(i-1)*env->cols+j-1]*4+1]!=-1){
    //                     tmp = freeCells[(i-1)*env->cols+j-1];
    //                     tmp1 = freeCells[(i)*env->cols+j-1];
    //                     tmp2 = freeCells[(i+1)*env->cols+j-1];
    //                     nebors[tmp1*4+3]=-1;
    //                     nebors[tmp2*4+3]=-1;
    //                     tmp = freeCells[(i-1)*env->cols+j+1];
    //                     tmp1 = freeCells[(i)*env->cols+j+1];
    //                     tmp2 = freeCells[(i+1)*env->cols+j+1];
    //                     nebors[tmp*4+1]=-1;
    //                     nebors[tmp1*4+1]=-1;
    //                 }
    //                 // else if(nebors[freeCells[(i-1)*env->cols+j]*4]!=-1 && nebors[freeCells[(i-1)*env->cols+j-1]*4]!=-1){
    //                 //     int tmp = freeCells[(i-1)*env->cols+j-1];
    //                 //     int tmp1 = freeCells[(i-1)*env->cols+j];
    //                 //     int tmp2 = freeCells[(i-1)*env->cols+j+1];
    //                 //     nebors[tmp1*4+2]=-1;
    //                 //     nebors[tmp2*4+2]=-1;
    //                 //     tmp = freeCells[(i+1)*env->cols+j-1];
    //                 //     tmp1 = freeCells[(i+1)*env->cols+j];
    //                 //     tmp2 = freeCells[(i+1)*env->cols+j+1];
    //                 //     nebors[tmp*4+2]=-1;
    //                 //     nebors[tmp1*4+2]=-1;
    //                 // }
    //             }
    //         }
    //         ++cnt;
    //     }
    // }
            
    // for (int i = 0; i+1 < env->rows; ++i)
    // {
    //     for (int j = 0; j < env->cols; ++j)
    //     {
    //         int tmp = j;
    //         while (tmp < env->cols && !env->map[i*env->cols+tmp] && !env->map[(i+1)*env->cols+tmp])
    //         {
    //             ++tmp;
    //         }
    //         for(int k=j+3; k+4<tmp; ++k){
    //             if(i%2){
    //                 int cell = freeCells[i*env->cols+k];
    //                 int cell1 = freeCells[(i+1)*env->cols+k+1];
    //                 nebors[cell*4+0]=-1;
    //                 nebors[cell1*4+2]=-1;
    //             }
    //             else{
    //                 int cell = freeCells[i*env->cols+k+1];
    //                 int cell1 = freeCells[(i+1)*env->cols+k];
    //                 nebors[cell*4+2]=-1;
    //                 nebors[cell1*4]=-1;
    //             }
    //         }
    //         j = max(j, tmp-1);
    //         //     int cell = freeCells[cnt];
    //         //     for (int k = 0; k < 4; ++k)
    //         //     {
    //         //         int ii = i + di[k];
    //         //         int jj = j + dj[k];
    //         //         int id = cnt + dj[k] + di[k] * env->cols;
    //         //         if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
    //         //         {
    //         //             if (!env->map[id])
    //         //             {
    //         //                 nebors[cell*4+k] = freeCells[id];
    //         //             }
    //         //             else{
    //         //                 nebors[cell*4+k] = -1;
    //         //             }
    //         //         }
    //         //         else{
    //         //             nebors[cell*4+k] = -1;
    //         //         }
    //         //     }
    //         // }
    //         // ++cnt;
    //     }
    // }
    // for (int j = 0; j+1 < env->cols; ++j)
    // {
    //     for (int i = 0; i < env->rows; ++i)
    //     {
    //         int tmp = i;
    //         while (tmp < env->rows && !env->map[tmp*env->cols+j] && !env->map[tmp*env->cols+j+1])
    //         {
    //             ++tmp;
    //         }
    //         for(int k=i+3; k+4<tmp; ++k){
    //             if(j%2){
    //                 int cell = freeCells[k*env->cols+j];
    //                 int cell1 = freeCells[(k+1)*env->cols+j+1];
    //                 nebors[cell*4+1]=-1;
    //                 nebors[cell1*4+3]=-1;
    //             }
    //             else{
    //                 int cell = freeCells[(k+1)*env->cols+j];
    //                 int cell1 = freeCells[(k)*env->cols+j+1];
    //                 nebors[cell*4+3]=-1;
    //                 nebors[cell1*4+1]=-1;
    //             }
    //         }
    //         i = max(i, tmp-1);
    //         //     int cell = freeCells[cnt];
    //         //     for (int k = 0; k < 4; ++k)
    //         //     {
    //         //         int ii = i + di[k];
    //         //         int jj = j + dj[k];
    //         //         int id = cnt + dj[k] + di[k] * env->cols;
    //         //         if (ii < env->rows && ii >= 0 && jj < env->cols && jj >= 0)
    //         //         {
    //         //             if (!env->map[id])
    //         //             {
    //         //                 nebors[cell*4+k] = freeCells[id];
    //         //             }
    //         //             else{
    //         //                 nebors[cell*4+k] = -1;
    //         //             }
    //         //         }
    //         //         else{
    //         //             nebors[cell*4+k] = -1;
    //         //         }
    //         //     }
    //         // }
    //         // ++cnt;
    //     }
    // }

    std::ofstream out;
    out.open("map.txt");
    bool l = 1;
    int m = env->cols;
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(env->map[i*m+j]){
                if(!l)out<<" ";l=0;
                out<<j;
            }
        }
    }
    out<<endl;
    l = 1;
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(env->map[i*m+j]){
                if(!l)out<<" ";l=0;
                out<<env->rows-1-i;
            }
        }
    }
    out<<endl;
    l = 1;
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(!env->map[i*m+j]){
                if(!l)out<<" ";l=0;
                out<<j;
            }
        }
    }
    out<<endl;
    l=1;
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(!env->map[i*m+j]){
                if(!l)
                    out<<" ";
                out<<env->rows-1-i;
                l = 0;
            }
        }
    }
    out<<endl;
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(env->map[i*env->cols+j])continue;
            int cell = freeCells[i*env->cols+j];
            int dx[4] = {0, 1, 0, -1}, dy[4]={1,0,-1,0};
            for(int k=0; k<4; ++k){
                if(nebors[cell*4+k]!=-1){
                    // if(dx[k]<0 || dy[k]<0)
                    //     out<<i+dx[k]<<" "<<j+dy[k]<<" "<<i<<" "<<j<<endl;
                    // else
                        out<<j<<" "<<env->rows-1-i<<" "<<j+dy[k]<<" "<<env->rows-1-i-dx[k]<<endl;
                }
            }
        }
    }
    out.close();
}

void RandomPlanner::writeSafePlaces(){
    std::ofstream out;
    out.open("safe_places.txt");
    // for(auto it:safe_places_vector){
    //     int i = row[it];
    //     int j = col[it];
    //     out<<j<<" "<<env->rows-1-i<<endl;
    // }
    for(int ii=0; ii<numOfFreeCells; ++ii){
        if(safePlaceFree[ii]){
            int i = row[ii];
            int j = col[ii];
            out<<j<<" "<<env->rows-1-i<<endl;
        }
    }
    out.close();
}

// void RandomPlanner::fillMap()
// {
//     map.width = env->cols;
//     map.height = env->rows;
//     int cnt = 0;
//     for(int i=0; i<env->rows; ++i){
//         for(int j=0; j<env->cols; ++j){
//             map.grid[i][j] = env->map[cnt];
//         }
//     }
//     map.generate_moves();
// }
// void RandomPlanner::fillTask()
// {
//     task.agents.clear();
//     for(int i=0; i<env->num_of_agents; ++i){
//         Agent a;
//         a.id = i;
//         a.start_i = env->curr_states[i].location / env->cols;
//         a.start_j = env->curr_states[i].location % env->cols;
//         a.goal_i = env->goal_locations[i].front().first / env->cols;
//         a.goal_j = env->goal_locations[i].front().first % env->cols;
//         task.agents.push_back(a);
//     }
// }
// plan using simple A* that ignores the time dimension
void RandomPlanner::plan(int time_limit, vector<Action> &actions)
{
    ++timeStep;
    start_plan_time = high_resolution_clock::now();
    if (timeStep == 0)
    {
        writeSafePlaces();
        old_paths = new list<pair<pair<int, int>, pair<int, int>>> [env->num_of_agents]();
        curr_states = new pair<int, int> [env->num_of_agents]();
        goal_locations = new int [env->num_of_agents]();
        // wasSearched = new bool [env->num_of_agents]();
    }
    if(timeStep>0){
        for(int i=0; i<env->num_of_agents; ++i){
            if(goal_locations[i]!=-1){
                isGoal[goal_locations[i]] = 0;
            }
        }
    }
    for(int i=0; i<env->num_of_agents; ++i){
        curr_states[i] = make_pair(freeCells[env->curr_states[i].location], env->curr_states[i].orientation);
        if(env->goal_locations[i].empty()){
            goal_locations[i] = -1;
        }
        else{
            goal_locations[i] = freeCells[env->goal_locations[i].front().first];
            isGoal[goal_locations[i]]++;
        }
    }
    if(timeStep == 0){
        // old_paths = vector<list<pair<pair<int, int>, pair<int, int>>>>(env->num_of_agents, list<pair<pair<int, int>,pair<int, int>>>());
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            mapFree[curr_states[i].first] = 0;
        }

    }
    for (int i=0; i<env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        if(it.size()>0){
            if((it.front().first.first != curr_states[i].first)||(it.front().first.second != curr_states[i].second)){
                actions = lastActions;
                --timeStep;
                return;
            }
        }
    }
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W);
    vector<pair<int, int>> v;
    for(int i=0; i<env->num_of_agents; ++i){
        if(isGoal[curr_states[i].first] || isGoal[old_paths[i].back().first.first]){
            v.emplace_back(-INT_MAX+abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first]), i);
            continue;
        }
        // if(mapFree[goal_locations[i]]!=INT_MAX)continue;
        if(timeStep+abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])>=mapFree[goal_locations[i]]){
            v.push_back({INT_MAX/2+rand()%(env->num_of_agents), i});
            continue;
        }
        // if(wasSearched[i])continue;
        bool goingToGoal = false;
        for(auto it:old_paths[i]){
            if(it.first.first == goal_locations[i]){
                goingToGoal = true;
                break;
            }
        }
        if(!goingToGoal){
            v.emplace_back(abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first]), i);
        }
        else{
            v.push_back({INT_MAX/2+rand()%(env->num_of_agents), i});
        }
    }
    if(v.size()>0)
        sort(v.begin(), v.end());
    int tmp_shift_id = shift_id1;
    for(int repeat = 0; repeat < 3; ++repeat){
        for (int ii=0; ii<v.size(); ++ii)
        {
            // int i = (ii+tmp_shift_id);
            // if(i >= (int)v.size()){
            //     i -= (int)v.size();
            // }
            if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
            {
                break;
            }
            shift_id1 = ii;
            int i = v[ii].second;
            list<pair<pair<int, int>, pair<int, int>>> path;
            if(old_paths[i].size()>0){
                mapFree[old_paths[i].back().first.first]=INT_MAX;
                del_reservations(old_paths[i]);
            }
            else{
                mapFree[curr_states[i].first] = INT_MAX;
            }
            
            single_agent_plan(curr_states[i].first,
                                        curr_states[i].second,
                                        goal_locations[i], path);
            // cout<<i<<" "<<path.size()<<endl;
            if (path.size() > 0)
            {
                mapFree[path.back().first.first] = path.back().second.first;
                old_paths[i] = path;
                reserve(path);
            }
            else
            {
                // wasSearched[i] = 1;
                if(old_paths[i].size()>0){
                    mapFree[old_paths[i].back().first.first] = old_paths[i].back().second.first;
                    reserve(old_paths[i]);
                }
                else{
                    mapFree[curr_states[i].first] = timeStep;
                }
            }
        }
    }
    while(1){
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        random_shuffle(v.begin(), v.end());
        for (int ii = 0; ii<env->num_of_agents; ii++)
        {
            int i = v[ii].second;
            if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
            {
                break;
            }
            // shift_id = i;
            list<pair<pair<int, int>, pair<int, int>>> path;
            bool l = 0;
            if(old_paths[i].size()>0){
                mapFree[old_paths[i].back().first.first]=INT_MAX;
                del_reservations(old_paths[i]);
            }
            else{
                mapFree[curr_states[i].first] = INT_MAX;
            }
            
            single_agent_plan(curr_states[i].first,
                                        curr_states[i].second,
                                        goal_locations[i], path);
            if (path.size() > 0)
            {
                mapFree[path.back().first.first] = path.back().second.first;
                old_paths[i] = path;
                reserve(path);
            }
            else
            {
                if(old_paths[i].size()>0){
                    mapFree[old_paths[i].back().first.first] = old_paths[i].back().second.first;
                    reserve(old_paths[i]);
                }
                else{
                    mapFree[curr_states[i].first] = timeStep;
                }
            }
        }
    }
    int cnt = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        bool l = 0;
        for(auto it:old_paths[i]){
            if(it.first.first == goal_locations[i]){
                ++cnt;
                l = 1;
                break;
            }
        }
        // if(!l){
        //     cout<<i<<" --> "<<row[goal_locations[i]]<<", "<<col[goal_locations[i]]<<endl;
        //     cout<<old_paths[i].size()<<endl;
        //     // int tmp = old_paths[i].back().first.first;
        //     int tmp = curr_states[i].first;
        //     cout<<row[tmp]<<", "<<col[tmp]<<endl;
        //     // exit(0);
        // }
        if (old_paths[i].size() > 0)
        {
            pair<int, int> tmp;
            if(old_paths[i].size()==1 || old_paths[i].front().second.first<old_paths[i].front().second.second){
                tmp = old_paths[i].front().first;
            }
            else{
                tmp = (++old_paths[i].begin())->first;
            }
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
            else{
                lastActions[i] = actions[i] = Action::W;
            }
            del_first_reservation(old_paths[i]);
            // if(old_paths[i].size()>0)
            //     assert(old_paths[i].front().second.first == timeStep+1);
            // ++cnt;
        }
        else{
            // assert(mapFree[curr_states[i].first]<=timeStep);
            lastActions[i] = actions[i] = Action::W;
        }
        // if(goal_locations[i]!=-1){
        //     ++cnt1;
        // }
        // if(!env->goal_locations[i].empty()){
        //     ++cnt2;
        // }
    }
    // checkReservations();
    // cout<<cnt<<"/"<<v.size()<<"//"<<env->num_of_agents<<endl;
    return;
}

void RandomPlanner::single_agent_plan(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path)
{
    // if(mapFree[end]){
    //     return;
    // }
    // assert(mapFree[start]==0);
    priority_queue<AstarNode, vector<AstarNode>, cmp> open_list;
    vector<AstarNode> all_nodes;
    AstarNode s = AstarNode(start, start_direct, timeStep, getManhattanDistance(start, end), -1, 0);
    open_list.push(s);
    AstarNode ans_node, second_ans;
    bool solved = false, solved_second = false, goalWasOnceVisited = false;
    int par;
    int goalCost = INT_MAX/2;
    std::set<int> *closed_list = new std::set<int>[numOfFreeCells * 8]();
    while (!open_list.empty())
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        AstarNode curr = open_list.top();
        open_list.pop();
        par = all_nodes.size();
        all_nodes.push_back(curr);
        int id = curr.location * 4 + curr.direction;
        bool goalWasVisited = curr.goalWasVisited;

        if (!closed_list[id*2+curr.goalWasVisited].empty())
        {
            auto it_node_prev = closed_list[id*2+curr.goalWasVisited].upper_bound(curr.g);
            if (it_node_prev != closed_list[id*2+curr.goalWasVisited].begin())
            {
                --it_node_prev; // node in closed_list which have time less or equal curr.g
                if (map[curr.location].empty())
                { // no block times at this location.
                    continue;
                }
                auto it_block_prev = map[curr.location].upper_bound({curr.g,curr.g});
                if (it_block_prev == map[curr.location].begin())
                { // no block time at this location less than curr.g
                    continue;
                }
                --it_block_prev; // block time less than curr.g
                if (*it_node_prev > it_block_prev->second)
                    continue;
            }
        }
        
        if(curr.location == end){
            if(!goalWasOnceVisited){
                // ans_node = curr;
                // solved = true;
                // break;
                goalWasOnceVisited = true;
                goalCost = min(goalCost, curr.g);
                goalWasVisited = true;
            }
        }
        if(!solved && mapFree[end] < curr.g){
            if(!isGoal[start] || solved_second)
                break;
        }
        if (!dangerousPlaceFree[curr.location] && mapFree[curr.location] == INT_MAX && (map[curr.location].empty() || map[curr.location].upper_bound({curr.g,curr.g}) == map[curr.location].end()))
        {
            if(goalWasVisited && !isGoal[curr.location]){
                ans_node = curr;
                solved = true;
                if(curr.g > goalCost+100)
                break;
            }
            // if(curr.location == end && !dangerousPlaceFree[curr.location] && isGoal[curr.location]==1){
            //     ans_node = curr;
            //     solved = true;
            //     break;
            // }
            // if(safePlaceFree[curr.location] && goalWasVisited && !isGoal[curr.location]){
            //     ans_node = curr;
            //     solved = true;
            //     break;
            // }
            if(!isGoal[curr.location]){
                if(!solved_second){
                    second_ans = curr;
                }
                else if(getManhattanDistance(curr.location, end)<getManhattanDistance(second_ans.location, end)){
                    second_ans = curr;
                }
                solved_second = true;
            }
            // if(!solved && !isGoal[curr.location] && !dangerousPlaceFree[curr.location] && safePlaceFree[curr.location]){
            //     ans_node = curr;
            //     solved = true;
            //     if(mapFree[end]){
            //         break;
            //     }
            // }
        }
        if(goalWasOnceVisited && !goalWasVisited && solved_second)continue;
        // Check visited
        
        closed_list[id*2+curr.goalWasVisited].insert(curr.g);
        list<pair<int, pair<int, int>>> neighbors = getNeighbors(curr.location, curr.direction, curr.g);
        for (const pair<int, pair<int, int>> &neighbor : neighbors)
        {
            open_list.push(AstarNode(neighbor.first, neighbor.second.first,
                                                 neighbor.second.second, getManhattanDistance(neighbor.first, end), par, goalWasVisited));
        }
    }
    if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() <= max_time_limit){
        if(solved){
            path.emplace_front(make_pair(make_pair(ans_node.location, ans_node.direction), make_pair(ans_node.g, ans_node.g)));
            while (ans_node.parent != -1)
            {
                auto pa = all_nodes[ans_node.parent];
                path.emplace_front(make_pair(make_pair(pa.location, pa.direction), make_pair(pa.g, ans_node.g-1)));
                ans_node = pa;
            }
        }
        else if(solved_second){
            path.emplace_front(make_pair(make_pair(second_ans.location, second_ans.direction), make_pair(second_ans.g, second_ans.g)));
            while (second_ans.parent != -1)
            {
                auto pa = all_nodes[second_ans.parent];
                path.emplace_front(make_pair(make_pair(pa.location, pa.direction), make_pair(pa.g, second_ans.g-1)));
                second_ans = pa;
            }
        }
    }
    delete []closed_list;
    // all_nodes.clear();
    return;
}

void RandomPlanner::reserve(const std::list<pair<pair<int, int>,pair<int, int>>> &path)
{
    if(path.size()==0)return;
    auto pre_loc = path.front().first;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
        // if (it != it_prev_end)
        map[it->first.first].insert(it->second);
        if (it->first.first != pre_loc.first)
        {
            blocked_edges[pre_loc.second][pre_loc.first].insert(it->second.first - 1);
        }
        pre_loc = it->first;
    }
    // }
}

void RandomPlanner::del_reservations(const std::list<pair<pair<int, int>, pair<int, int>>> &path)
{
    if(path.size()==0){
        return;
    }
    auto pre_loc = path.front().first;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
        // if (it != it_prev_end)
        map[it->first.first].erase(it->second);
        if (it->first.first != pre_loc.first)
        {
            blocked_edges[pre_loc.second][pre_loc.first].erase(it->second.first - 1);
        }
        pre_loc = it->first;
    }
}

void RandomPlanner::del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path)
{
    auto loc = path.front();
    // assert(loc.second.first==timeStep);
    map[loc.first.first].erase(map[loc.first.first].find(loc.second));
    path.pop_front();
    // if(path.size()==0){
    //     return;
    // }
    loc.second.first++;
    if(loc.second.first <= loc.second.second){
        path.push_front(loc);
        // assert(loc.second.first==timeStep+1);
        map[loc.first.first].insert(loc.second);
    }
    else if(path.size()>0){
        if(loc.first.first != path.front().first.first){
            blocked_edges[loc.first.second][loc.first.first].erase(timeStep);
        }
    }
    // }
}

int RandomPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool RandomPlanner::check(list<pair<int, int>> &path)
{
    int t = 1;
    for (auto it : path)
    {
        if (mapFree[it.first])
        {
            return false;
        }
        ++t;
        if (t == 2)
            break;
    }
    return true;
}

bool RandomPlanner::is_safe_place(int loc)
{
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    int x = row[loc];
    int y = col[loc];
    vector<int> v;
    for(int k=0; k<8; ++k){
        int l1 = -1;
        int ii = x+dx[k];
        int jj = y+dy[k];
        if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && (env->map[ii*env->cols+jj]==0 && mapFree[freeCells[ii*env->cols+jj]]==INT_MAX)){
            if(k%2==0)
                l1 = 0;
        }
        else{
            l1=1;
        }
        if(l1!=-1)
            v.push_back(l1);
    }
    int tmp = 0;
    for(int i=0;i<v.size();++i){
        if(v[i]!=v[(i+1)%v.size()])++tmp;
    }
    if(tmp>=3){
        return false;
    }
    return true;

    // for(auto it:safeValues){
    //     if ((x == it || x == env->rows - 1-it) && (y >= mx_edge_x+2 && y <= env->cols-1-mx_edge_x-2 && y !=env->cols/2))
    //     {
    //         return true;
    //     }
    //     if ((y == it || y == env->cols - 1-it) && (x >= mx_edge_y+2 && x <= env->rows-1-mx_edge_y-2 && x != env->rows/2))
    //     {
    //         return true;
    //     }
    // }
    return false;
}

list<pair<int, pair<int, int>>> RandomPlanner::getNeighbors(int location, int direction, int t)
{
    list<pair<int, pair<int, int>>> neighbors;
    // forward
    int forward =nebors[location*4+direction];
    int t_u = mapFree[location]-1;
    auto it = map[location].upper_bound({t,t});
    if (it != map[location].end())
    {
        t_u = min(t_u, it->first - 1);
    }
    if(forward!=-1)
    {
        if(mapFree[forward]>t+1){
            // if(static_map[forward]==0 && (t>=2 || env->map[forward]==0)){
            
            if (map[forward].size() > 0)
            {
                auto it1 = map[forward].upper_bound({t+1, 1e9});
                int t_prev = 0;
                if(it1!=map[forward].begin()){
                    --it1;t_prev = it1->second+1;
                    ++it1;
                }
                for (; it1 != map[forward].end(); ++it1)
                {
                    int t_l = max(t + 1, t_prev);
                    if(t_l >= mapFree[forward]){
                        break;
                    }
                    if (t_l < it1->first && t_l - 1 <= t_u)
                    {
                        if (blocked_edges[(direction + 2) % 4][forward].find(t_l - 1) == blocked_edges[(direction + 2) % 4][forward].end())
                        {
                            neighbors.push_back({forward, {direction, t_l}});
                        }
                    }
                    if (t_l - 1 > t_u)
                        break;
                    t_prev = it1->second + 1;
                }
                int t_l = max(t + 1, t_prev);
                if (t_l - 1 <= t_u && t_l < mapFree[forward])
                {
                    if (blocked_edges[(direction + 2) % 4][forward].find(t_l - 1) == blocked_edges[(direction + 2) % 4][forward].end())
                    {
                        neighbors.push_back({forward, {direction, t_l}});
                    }
                }
            }
            else
            {
                neighbors.push_back({forward, {direction, t + 1}});
            }
        }
    }
    int new_direction = direction;
    if (t + 1 <= t_u)
    {
        // turn left
        new_direction = direction - 1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.push_back({location, {new_direction, t + 1}});
        // turn right
        new_direction = direction + 1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.push_back({location, {new_direction, t + 1}});
    }
    return neighbors;
}

void RandomPlanner::checkReservations()
{
    int tmp = 0;
    for(int i=0; i<env->num_of_agents; ++i){
        tmp += old_paths[i].size();
    }
    cout<<"Number of reservation = "<<tmp<<endl;
    for(int i=0; i<numOfFreeCells; ++i){
        tmp -= map[i].size();
    }
    assert(tmp==0);
}