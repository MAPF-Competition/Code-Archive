#include <WarehousePlanner.h>
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

void WarehousePlanner::initialize(int preprocess_time_limit)
{
    W1 = 1;
    W2 = 10;
    max_time_limit = 900000;
    safeValues.push_back(4);
    safeValues.push_back(5);
    safeExtraLine = 7;
    mx_edge_x = 6;
    mx_edge_y = 8;
    safePlace = new bool[(env->cols) * (env->rows)]();
    dangerousPlace = new bool[(env->cols) * (env->rows)]();
    // findSafePlaces();
    timeStep = -1;
    freeCells = new int[env->map.size()]();
    compressFreeCells();
    genAllNeighors();
    dir_h = dir_v = dr = 0;
}

void WarehousePlanner::writeMap()
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
                if (nebors[cell * 4 + k] != -1)
                {
                    out << j << " " << env->rows - 1 - i << " " << j + dy[k] << " " << env->rows - 1 - i - dx[k] << endl;
                }
            }
        }
    }
    out.close();
}

void WarehousePlanner::compressFreeCells()
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
    cellCost = new int[numOfFreeCells]();
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
void WarehousePlanner::genAllNeighors()
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
        // if (env->map[i * env->cols + 5] == 0)
        // {
            tmp.push_back(i);
        // }
    }
    for (int i = 0; i < tmp.size(); ++i)
    {
        h[tmp[i]] = (i + dir_h) % 2;
    }
    tmp.clear();
    for (int i = 0; i < env->cols; ++i)
    {
        if (env->map[9 * env->cols + i] == 0)
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
    vector<int> start_up1;// = {4, 11,   25,        46,    60, 67,    81,        ,103, }
    vector<int> start_down1;// = {   18,    32, 39,     53,       74,     89, 96,    , 110,}
    int num = -3;
    vector<int> d1 = {2, 1, 1}, d2 = {1, 2, 1};
    cnt = 0;
    while(num<495){
        for(int j=0; j<d1[cnt]; ++j){
            num += 7;
            if(num>=495)break;
            start_up1.push_back(num);
        }
        for(int j=0; j<d2[cnt]; ++j){
            num += 7;
            if(num>=495)break;
            start_down1.push_back(num);
        }
        cnt = (cnt+1)%3;
    }
    for(auto it:start_up1){
        for(int j=0; j<4; ++j){
            if(j%2){
                int col = it+j;
                for(int i=0; i<10; ++i){
                    int cell = i*m+col, nxt = (i+1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+1]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+3]=-1;
                }
            }
            else{
                int col = it+j;
                for(int i=0; i<10; ++i){
                    if(i<3 && j==0){
                        for(int jj=col; jj<col+3; ++jj){
                            int cell = i*m+jj, nxt = i*m+jj+1;
                            if(env->map[nxt])break;
                            if(i%2==0){
                                nebors[freeCells[cell]*4+0]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+2]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+0]=-1;
                                nebors[freeCells[nxt]*4+2]=freeCells[cell];
                            }
                        }
                    }
                    int cell = i*m+col, nxt = (i+1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+1]=-1;
                    nebors[freeCells[nxt]*4+3]=freeCells[cell];
                }
            }
        }
    }
    for(auto it:start_down1){
        for(int j=0; j<4; ++j){
            if(j%2==0){
                int col = it+j;
                for(int i=0; i<10; ++i){
                    if(i<3 && j==0){
                        for(int jj=col; jj<col+3; ++jj){
                            int cell = i*m+jj, nxt = i*m+jj+1;
                            if(env->map[nxt])break;
                            if(i%2){
                                nebors[freeCells[cell]*4+0]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+2]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+0]=-1;
                                nebors[freeCells[nxt]*4+2]=freeCells[cell];
                            }
                        }
                    }
                    int cell = i*m+col, nxt = (i+1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+1]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+3]=-1;
                }
            }
            else{
                int col = it+j;
                for(int i=0; i<10; ++i){
                    int cell = i*m+col, nxt = (i+1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+1]=-1;
                    nebors[freeCells[nxt]*4+3]=freeCells[cell];
                }
            }
        }
    }

    vector<int> start_left1;// = {     123,    110,   96,   81,   68, }
    vector<int> start_right1;// = {131,    117,   103.    89,   75, }
    d1 = {1}, d2 = {1};
    num = 1;
    while(num<134){
        for(int j=0; j<d1[cnt]; ++j){
            num += 7;
            if(num>=138)break;
            start_right1.push_back(num);
        }
        for(int j=0; j<d2[cnt]; ++j){
            num += 7;
            if(num>=138)break;
            start_left1.push_back(num);
        }
        cnt = (cnt+1)%1;
    }
    for(auto it:start_right1){
        for(int i=0; i<4; ++i){
            if(i%2==0){
                int row = it+i;
                for(int j=env->cols-1; j>=env->cols-1-10; --j){
                    if(j>env->cols-4 && i==0){
                        for(int ii=row; ii<row+3; ++ii){
                            int cell = ii*m+j, nxt = (ii+1)*m+j;
                            if(env->map[nxt])break;
                            if((env->cols-1-j)%2==0){
                                nebors[freeCells[cell]*4+1]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+3]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+1]=-1;
                                nebors[freeCells[nxt]*4+3]=freeCells[cell];
                            }
                        }
                    }
                    int cell = row*m+j, nxt = row*m+j-1;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+2]=-1;
                    nebors[freeCells[nxt]*4+0]=freeCells[cell];
                }
            }
            else{
                int row = it+i;
                for(int j=env->cols-1; j>=env->cols-1-10; --j){
                    int cell = row*m+j, nxt = row*m+j-1;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+2]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+0]=-1;
                }
            }
        }
    }
    for(auto it:start_left1){
        for(int i=0; i<4; ++i){
            if(i%2){
                int row = it+i;
                for(int j=env->cols-1; j>=env->cols-1-10; --j){
                    int cell = row*m+j, nxt = row*m+j-1;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+2]=-1;
                    nebors[freeCells[nxt]*4+0]=freeCells[cell];
                }
            }
            else{
                int row = it+i;
                for(int j=env->cols-1; j>=env->cols-1-10; --j){
                    if(j>env->cols-4 && i==0){
                        for(int ii=row; ii<row+3; ++ii){
                            int cell = ii*m+j, nxt = (ii+1)*m+j;
                            if(env->map[nxt])break;
                            if((env->cols-1-j)%2){
                                nebors[freeCells[cell]*4+1]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+3]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+1]=-1;
                                nebors[freeCells[nxt]*4+3]=freeCells[cell];
                            }
                        }
                    }
                    int cell = row*m+j, nxt = row*m+j-1;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+2]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+0]=-1;
                }
            }
        }
    }
    vector<int> start_up2, start_down2;
    num = 2;
    while(num<124){
        num+=7;
        start_down2.push_back(num);
        if(num>=124)break;
        num+=7;
        start_up2.push_back(num);
    }
    for(auto it:start_up2){
        for(int j=0; j<3; ++j){
            for(int i=it; i<it+3; ++i){
                int cell = i*m+j, nxt = (i+1)*m+j;
                if(j%2){
                    nebors[freeCells[cell]*4+1]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+3]=-1;
                }
                else{
                    nebors[freeCells[cell]*4+1]=-1;
                    nebors[freeCells[nxt]*4+3]=freeCells[cell];
                }
            }
        }
    }
    for(auto it:start_down2){
        for(int j=0; j<3; ++j){
            for(int i=it; i<it+3; ++i){
                int cell = i*m+j, nxt = (i+1)*m+j;
                if(j%2==0){
                    nebors[freeCells[cell]*4+1]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+3]=-1;
                }
                else{
                    nebors[freeCells[cell]*4+1]=-1;
                    nebors[freeCells[nxt]*4+3]=freeCells[cell];
                }
            }
        }
    }

    vector<int> start_up3;
    vector<int> start_down3;
    num = 2;
    d1 = {1, 1, 2};
    d2 = {2, 1, 1};
    cnt = 0;
    while(num<495){
        for(int j=0; j<d1[cnt]; ++j){
            num += 7;
            if(num>=495)break;
            start_up3.push_back(num);
        }
        for(int j=0; j<d2[cnt]; ++j){
            num += 7;
            if(num>=495)break;
            start_down3.push_back(num);
        }
        cnt = (cnt+1)%3;
    }
    for(auto it:start_up3){
        for(int j=0; j<4; ++j){
            if(j%2){
                int col = it+j;
                for(int i=0; i<10; ++i){
                    int cell = (env->rows-1-i)*m+col, nxt = (env->rows-1-i-1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+3]=-1;
                    nebors[freeCells[nxt]*4+1]=freeCells[cell];
                }
            }
            else{
                int col = it+j;
                for(int i=0; i<10; ++i){
                    if(i<3 && j==0){
                        for(int jj=col; jj<col+3; ++jj){
                            int cell = (env->rows-1-i)*m+jj, nxt = (env->rows-1-i)*m+jj+1;
                            if(env->map[nxt])break;
                            if(i%2){
                                nebors[freeCells[cell]*4+0]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+2]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+0]=-1;
                                nebors[freeCells[nxt]*4+2]=freeCells[cell];
                            }
                        }
                    }
                    int cell = (env->rows-1-i)*m+col, nxt = (env->rows-1-i-1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+3]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+1]=-1;
                }
            }
        }
    }
    for(auto it:start_down3){
        for(int j=0; j<4; ++j){
            if(j%2==0){
                int col = it+j;
                for(int i=0; i<10; ++i){
                    if(i<3 && j==0){
                        for(int jj=col; jj<col+3; ++jj){
                            int cell = (env->rows-1-i)*m+jj, nxt = (env->rows-1-i)*m+jj+1;
                            if(env->map[nxt])break;
                            if(i%2==0){
                                nebors[freeCells[cell]*4+0]=freeCells[nxt];
                                nebors[freeCells[nxt]*4+2]=-1;
                            }
                            else{
                                nebors[freeCells[cell]*4+0]=-1;
                                nebors[freeCells[nxt]*4+2]=freeCells[cell];
                            }
                        }
                    }
                    int cell = (env->rows-1-i)*m+col, nxt = (env->rows-1-i-1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+3]=-1;
                    nebors[freeCells[nxt]*4+1]=freeCells[cell];
                }
            }
            else{
                int col = it+j;
                for(int i=0; i<10; ++i){
                    int cell = (env->rows-1-i)*m+col, nxt = (env->rows-1-i-1)*m+col;
                    if(env->map[nxt])break;
                    nebors[freeCells[cell]*4+3]=freeCells[nxt];
                    nebors[freeCells[nxt]*4+1]=-1;
                }
            }
        }
    }
}
void WarehousePlanner::dfs_fill_sizes(int x)
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
void WarehousePlanner::dfs_mark_valids(int x)
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
bool WarehousePlanner::dfs_fill_cycles(int x, int pa)
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
void WarehousePlanner::markWhoAllowedToMove()
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
void WarehousePlanner::sortAgents(vector<int> &agents_vector){
    const int partition = 4;
    vector<pair<pair<int, int>, int>> v[partition][partition];
    for(auto i:agents_vector){
        v[row[goal_locations[i]]/((env->rows)/partition)][col[goal_locations[i]]/((env->cols)/partition)].push_back({{
            // -abs(row[goal_locations[i]]-row[curr_states[i].first])-abs(col[goal_locations[i]]-col[curr_states[i].first]),
            min(min(row[goal_locations[i]], env->rows-1-row[goal_locations[i]]), min(col[goal_locations[i]], env->cols-1-col[goal_locations[i]])), 
            abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])},
            i});
    }
    for(int i=0; i<partition; ++i){
        for(int j=0; j<partition; ++j){
            sort(v[i][j].begin(), v[i][j].end());
            reverse(v[i][j].begin(), v[i][j].end());
        }
    }
    int cnt = 0;
    int num_of_agents = agents_vector.size();
    agents_vector.clear();
    while(agents_vector.size() < num_of_agents){
        int i = (cnt%(partition*partition))/partition;
        int j = (cnt%(partition*partition))%partition;
        if(v[i][j].size()>0){
            agents_vector.push_back(v[i][j].back().second);
            v[i][j].pop_back();
        }
        ++cnt;
    }
}
void WarehousePlanner::partial_plan(int thread_id, int start_id, int end_id)
{
    // cout<<thread_id<<", "<<start_id<<" "<<end_id<<endl;
    // auto start_plan_time = high_resolution_clock::now();
    vector<int> v;
    for (int i = start_id; i < end_id; ++i)
    {
        if (old_paths[i].size() == 0)
            v.push_back(i);
    }
    sortAgents(v);
    for (int ii=0; ii<v.size(); ii++)
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = v[ii];

            // mutex_cellCost.lock();
            // --cellCost[curr_states[i].first];
            // // for(int j=0; j<min((int)old_paths[i].size(), 2); ++j){
            // //     auto it = old_paths[i][j];
            // //     --cellCost[it.first];
            // // }
            // mutex_cellCost.unlock();
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i]);
            
            // mutex_cellCost.lock();
            // ++cellCost[curr_states[i].first];
            // // for(int j=0; j<min((int)old_paths[i].size(), 2); ++j){
            // //     auto it = old_paths[i][j];
            // //     ++cellCost[it.first];
            // // }
            // mutex_cellCost.unlock();
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
        // if (allowedToMove[i]==0 || old_paths[i].back().first != goal_locations[i])
        // {
            // auto tmp_path = old_paths[i];
            // mutex_cellCost.lock();
            // --cellCost[curr_states[i].first];
            // // for(int j=0; j<min((int)old_paths[i].size(), 2); ++j){
            // //     auto it = old_paths[i][j];
            // //     --cellCost[it.first];
            // // }
            // mutex_cellCost.unlock();
            old_paths[i].clear();
            single_agent_plan(thread_id, curr_states[i].first,
                              curr_states[i].second,
                              goal_locations[i], old_paths[i]);
           
            // mutex_cellCost.lock();
            // ++cellCost[curr_states[i].first];
            // // for(int j=0; j<min((int)old_paths[i].size(), 2); ++j){
            // //     auto it = old_paths[i][j];
            // //     ++cellCost[it.first];
            // // }
            // mutex_cellCost.unlock();
        // }
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

            // mutex_cellCost.lock();
            // ++cellCost[curr_states[i].first];
            // mutex_cellCost.unlock();
        }
    }
}

// plan using simple A* that ignores the time dimension
void WarehousePlanner::plan(int time_limit, vector<Action> &actions)
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
    // fill(cellCost, cellCost+numOfFreeCells, 0);
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (allowedToMove[i] && old_paths[i].size() > 0)
        {
            old_paths[i].pop_front();
        }
        // ++cellCost[curr_states[i].first];
        // for(int j=0; j<min((int)old_paths[i].size(), 2); ++j){
        //     auto it = old_paths[i][j];
        //     ++cellCost[it.first];
        // }
    }
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W);
    thread threads[processor_count];
    for(int i=0; i<processor_count; ++i){

        if(i < processor_count-1){
            threads[i] = thread(&WarehousePlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents/processor_count*(i+1));
        }
        else{
            threads[i] = thread(&WarehousePlanner::partial_plan, this, i, env->num_of_agents/processor_count*i, env->num_of_agents);
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
    cout<<cnt<<"///";
    for(int i=0; i<processor_count; ++i){
        cout<<shift_id[i]<<" ";
    }
    cout<<"/"<<env->num_of_agents<<endl;
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

void WarehousePlanner::single_agent_plan(int thread_id, int start, int start_direct, int end, deque<pair<int, int>> &path)
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
        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction, curr->location==start&&curr->direction==start_direct);
        for (const pair<int, int> &neighbor : neighbors)
        {
            AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
                                                 cost + W2, W2 * getManhattanDistance(neighbor.first, end), curr);
            open_list.push(next_node);
            all_nodes.push_back(next_node);
        }
        // for (const pair<int, int> &neighbor : neighbors)
        // {
        //     if (neighbor.first != curr->location)
        //     {
        //         AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
        //                                              curr->g + (int)(cellCost[neighbor.first]), curr->h + 1, curr);
        //         open_list.push(next_node);
        //         all_nodes.push_back(next_node);
        //     }
        //     else
        //     {
        //         AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second,
        //                                              curr->g + 1, curr->h + 1, curr);
        //         open_list.push(next_node);
        //         all_nodes.push_back(next_node);
        //     }
        // }
        // int forward = emergency_nebors[curr->location * 4 + curr->direction];
        // if (forward == end)
        // {
        //     AstarNode *next_node = new AstarNode(forward, curr->direction,
        //                                          cost + W2, 0, curr);
        //     open_list.push(next_node);
        //     all_nodes.push_back(next_node);
        // }
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

int WarehousePlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

list<pair<int, int>> WarehousePlanner::getNeighbors(int location, int direction, bool is_start)
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
