#include <CityPlanner2.h>
#include <random>
using namespace std::chrono;
using namespace std;

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

void CityPlanner2::initialize(int preprocess_time_limit)
{
    shift_id = 0;
    max_time_limit = 950000;
    
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

void CityPlanner2::findSafePlaces()
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
    //                 safePlace[i*env->cols+j]=0;
    //             }
    //         }
    //     }
    // }
}


void CityPlanner2::writeSafePlaces(){
    std::ofstream out;
    out.open("safe_places"+std::to_string(timeStep)+".txt");
    // for(auto it:safe_places_vector){
    //     int i = row[it];
    //     int j = col[it];
    //     out<<j<<" "<<env->rows-1-i<<endl;
    // }
    int cnt = 0;
    for(int ii=0; ii<numOfFreeCells; ++ii){
        if(mapFree[ii]<INT_MAX){
            int i = row[ii];
            int j = col[ii];
            out<<j<<" "<<env->rows-1-i<<endl;
            ++cnt;
        }
    }
    assert(cnt==env->num_of_agents);
    for(int ii=0; ii<env->num_of_agents; ++ii){
        int i = row[curr_states[ii].first];
        int j = col[curr_states[ii].first];
        out<<j<<" "<<env->rows-1-i<<endl;
    }
    for(int ii=0; ii<env->num_of_agents; ++ii){
        int i = row[goal_locations[ii]];
        int j = col[goal_locations[ii]];
        out<<j<<" "<<env->rows-1-i<<endl;
    }
    out.close();
}

void CityPlanner2::compressFreeCells(){
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
    // closed_list = new std::set<int>[numOfFreeCells * 8]();
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
void CityPlanner2::genAllNeighbors()
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
// void CityPlanner2::fillMap()
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
// void CityPlanner2::fillTask()
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

void CityPlanner2::print_location(int loc){
    cout<<col[loc]<<" "<<env->rows-1-row[loc]<<endl;
}
void CityPlanner2::print_agent_information(int ag){
    cout<<"Agent :"<<ag<<endl;
    cout<<"current location:";
    print_location(curr_states[ag].first);
    cout<<"Goal:";
    print_location(goal_locations[ag]);
    cout<<"Path size:"<<old_paths[ag].size()<<"::"<<endl;
    for(auto it:old_paths[ag]){
        print_location(it.first.first);
    }
    // cout<<"was cleared: "<<wasCleared[ag]<<endl;
    cout<<"----"<<endl;
}

// plan using simple A* that ignores the time dimension
void CityPlanner2::plan(int time_limit, vector<Action> &actions)
{
    ++timeStep;
    start_plan_time = high_resolution_clock::now();
    if (timeStep == 0)
    {
        old_paths = new list<pair<pair<int, int>, pair<int, int>>> [env->num_of_agents]();
        curr_states = new pair<int, int> [env->num_of_agents]();
        goal_locations = new int [env->num_of_agents]();
        wasSearched = new bool [env->num_of_agents]();
        bestCost = new int [env->num_of_agents]();
        sortCost = new int [env->num_of_agents]();
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
    // int num_of_safe_places = 0;
    // for(int i=0; i<numOfFreeCells; ++i){
    //     if(is_safe_place(i)){
    //         ++num_of_safe_places;
    //     }
    // }
    // cout<<"number of safe places;:"<<num_of_safe_places<<endl;
    // writeSafePlaces();
    lastActions = actions = std::vector<Action>(env->num_of_agents, Action::W);
    for (shift_id; shift_id<env->num_of_agents; ++shift_id)
    {
        int i = shift_id;
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        list<pair<pair<int, int>, pair<int, int>>> path;
        if(old_paths[i].size()>0){
            mapFree[old_paths[i].back().first.first]=INT_MAX;
            del_reservations(old_paths[i]);
        }
        else{
            mapFree[curr_states[i].first] = INT_MAX;
        }

        single_agent_plan_dummy(curr_states[i].first,
                                    curr_states[i].second,
                                    goal_locations[i], path);
        // cout<<i<<" "<<path.size()<<endl;             
        if (path.size() > 0)
        {
            mapFree[path.back().first.first] = path.back().second.first;
            old_paths[i] = path;
            reserve(path);
            sortCost[i]=0;
            for(auto it:path){
                if(it.first.first == goal_locations[i]){
                    // ++num_of_success;
                    // sortCost[i] = 0;
                    break;
                }
            }
        }
        else
        {
            // ++num_of_zeros;
            ++sortCost[i];
            wasSearched[i] = 1;
            if(old_paths[i].size()>0){
                mapFree[old_paths[i].back().first.first] = old_paths[i].back().second.first;
                reserve(old_paths[i]);
            }
            else{
                mapFree[curr_states[i].first] = timeStep;
            }
        }
    }
    vector<pair<pair<int, int>, int>> v;
    if(shift_id == env->num_of_agents){
        for(int i=0; i<env->num_of_agents; ++i){
            if(old_paths[i].size()>0 && old_paths[i].back().first.first != goal_locations[i] && (!is_safe_place(old_paths[i].back().first.first) || isGoal[old_paths[i].back().first.first])){
                v.push_back({{0, -INT_MAX+abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])}, i});
                continue;
            }
            if(old_paths[i].size() == 0 && curr_states[i].first != goal_locations[i] && (!is_safe_place(curr_states[i].first) || isGoal[curr_states[i].first])){
                v.push_back({{0, -INT_MAX+abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])}, i});
                continue;
            }
            // if(mapFree[goal_locations[i]]!=INT_MAX)continue;
            // if(wasSearched[i])continue;
            bool goingToGoal = false;
            for(auto it:old_paths[i]){
                if(it.first.first == goal_locations[i]){
                    goingToGoal = true;
                    break;
                }
            }
            if(!goingToGoal){
                if(timeStep+abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])>=mapFree[goal_locations[i]]){
                    // v.push_back({{sortCost[i], INT_MAX/2+rand()%(env->num_of_agents)}, i});
                    continue;
                }
                if(old_paths[i].size()==0){
                    v.push_back({{sortCost[i], abs(row[goal_locations[i]]-row[curr_states[i].first])+abs(col[goal_locations[i]]-col[curr_states[i].first])}, i});
                }
                else{
                    v.push_back({{sortCost[i], INT_MAX/4+rand()%(env->num_of_agents)}, i});
                }
            }
            else{
                v.push_back({{INT_MAX/2+rand()%(env->num_of_agents), INT_MAX/2+rand()%(env->num_of_agents)}, i});
            }
        }
        if(v.size()>0)
            sort(v.begin(), v.end());
        // writeSafePlaces();
        
        // int num_of_searched = 0, num_of_success = 0, num_of_zeros = 0;
        for (auto it:v)
        {
            int i = it.second;
            if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
            {
                break;
            }
            list<pair<pair<int, int>, pair<int, int>>> path;
            if(old_paths[i].size()>0){
                mapFree[old_paths[i].back().first.first]=INT_MAX;
                del_reservations(old_paths[i]);
            }
            else{
                mapFree[curr_states[i].first] = INT_MAX;
            }

            // ++num_of_searched;
            single_agent_plan(curr_states[i].first,
                                        curr_states[i].second,
                                        goal_locations[i], path);
            // cout<<i<<" "<<path.size()<<endl;             
            if (path.size() > 0)
            {
                mapFree[path.back().first.first] = path.back().second.first;
                old_paths[i] = path;
                reserve(path);
                sortCost[i]=0;
                // for(auto it:path){
                //     if(it.first.first == goal_locations[i]){
                //         ++num_of_success;
                //         // sortCost[i] = 0;
                //         break;
                //     }
                // }
            }
            else
            {
                // ++num_of_zeros;
                ++sortCost[i];
                wasSearched[i] = 1;
                if(old_paths[i].size()>0){
                    mapFree[old_paths[i].back().first.first] = old_paths[i].back().second.first;
                    reserve(old_paths[i]);
                }
                else{
                    mapFree[curr_states[i].first] = timeStep;
                }
            }
        }
        // cout<<"Searched: "<<num_of_searched<<", success: "<<num_of_success<<", zeros: "<<num_of_zeros<<endl;
    }
    int cnt = 0;
    // int tmp_shift_id = shift_id;
    // for (int ii = 0; ii<env->num_of_agents; ii++)
    // {
    //     int i = (ii+tmp_shift_id);
    //     if(i >= env->num_of_agents){
    //         i -= env->num_of_agents;
    //     }
    //     if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
    //     {
    //         break;
    //     }
    //     shift_id = i;
    //     list<pair<pair<int, int>, pair<int, int>>> path;
    //     bool l = 0;
    //     if(old_paths[i].size()>0){
    //         mapFree[old_paths[i].back().first.first]=INT_MAX;
    //         del_reservations(old_paths[i]);
    //     }
    //     else{
    //         mapFree[curr_states[i].first] = INT_MAX;
    //     }
        
    //     single_agent_plan(curr_states[i].first,
    //                                 curr_states[i].second,
    //                                 goal_locations[i], path);
    //     if (path.size() > 0)
    //     {
    //         mapFree[path.back().first.first] = path.back().second.first;
    //         old_paths[i] = path;
    //         reserve(path);
    //     }
    //     else
    //     {
    //         if(old_paths[i].size()>0){
    //             mapFree[old_paths[i].back().first.first] = old_paths[i].back().second.first;
    //             reserve(old_paths[i]);
    //         }
    //         else{
    //             mapFree[curr_states[i].first] = timeStep;
    //         }
    //     }
    // }
    cnt = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        // bool l = 0;
        // for(auto it:old_paths[i]){
        //     if(it.first.first == goal_locations[i]){
        //         ++cnt;
        //         l = 1;
        //         break;
        //     }
        // }
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

    // if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > 1000000){
    //     exit(0);
    // }
    // checkReservations();
    // if(shift_id == env->num_of_agents){
    //     cout<<cnt<<"/"<<v.size()<<"//"<<env->num_of_agents<<endl;
    // }
    // else{
    //     cout<<"going-to-safe---"<<cnt<<"/"<<shift_id<<"//"<<env->num_of_agents<<endl;
    // }
    return;
}

void CityPlanner2::single_agent_plan(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path)
{
    queue<pair<int, pair<int, int>>> q;
    for(int k=0; k<4; ++k)
        q.push({0,{end,k}});
    int *h_cost = new int [(numOfFreeCells)*4];
    fill(h_cost, h_cost+(numOfFreeCells)*4, -1);
    while(!q.empty()){
        auto tmp = q.front();
        q.pop();
        if(h_cost[tmp.second.first*4+tmp.second.second]!=-1)continue;
        h_cost[tmp.second.first*4+tmp.second.second]=tmp.first;
        int nxt = nebors[tmp.second.first*4+tmp.second.second];
        if(nxt != -1)
            q.push({tmp.first+1, {nxt, tmp.second.second}});
        int k = (tmp.second.second+1)%4;
        q.push({tmp.first+1, {tmp.second.first, k}});
        k = (tmp.second.second+3)%4;
        q.push({tmp.first+1, {tmp.second.first, k}});
    }
    for(int i=0; i<numOfFreeCells; ++i){
        swap(h_cost[i*4], h_cost[i*4+2]);
        swap(h_cost[i*4+1], h_cost[i*4+3]);
    }
    // if(mapFree[end]){
    //     return;
    // }
    // assert(mapFree[start]==0);
    priority_queue<AstarNode, vector<AstarNode>, cmp> open_list;
    vector<AstarNode> all_nodes;
    AstarNode s = AstarNode(start, start_direct, timeStep, h_cost[start*4+start_direct], -1, 0);
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
            // if(!goalWasOnceVisited){
                // ans_node = curr;
                // solved = true;
                // break;
                goalWasOnceVisited = true;
                goalCost = min(goalCost, curr.g);
                goalWasVisited = true;
            // }
        }
        if(!solved && mapFree[end] < curr.f){
            if(!isGoal[start] || solved_second)
                break;
        }
        if (is_safe_place(curr.location) && mapFree[curr.location] == INT_MAX && (map[curr.location].empty() || map[curr.location].upper_bound({curr.g,curr.g}) == map[curr.location].end()))
        {
            if(goalWasVisited && !isGoal[curr.location]){
                ans_node = curr;
                solved = true;
                // if(curr.g > goalCost+5)
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
                // else if(getManhattanDistance(curr.location, end)<getManhattanDistance(second_ans.location, end)){
                else if(h_cost[curr.location*4+curr.direction]<h_cost[second_ans.location*4+second_ans.direction]){
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
        //  if (mapFree[curr.location] == INT_MAX && (map[curr.location].empty() || map[curr.location].upper_bound({curr.g,curr.g}) == map[curr.location].end()))
        // {
        //     if(goalWasVisited && !isGoal[curr.location]){
        //         ans_node = curr;
        //         solved = true;
        //         // if(curr.g > goalCost+5)
        //     }
        //     // if(curr.location == end && !dangerousPlaceFree[curr.location] && isGoal[curr.location]==1){
        //     //     ans_node = curr;
        //     //     solved = true;
        //     //     break;
        //     // }
        //     // if(safePlaceFree[curr.location] && goalWasVisited && !isGoal[curr.location]){
        //     //     ans_node = curr;
        //     //     solved = true;
        //     //     break;
        //     // }
        //     if(!isGoal[curr.location]){
        //         if(!solved_second){
        //             second_ans = curr;
        //         }
        //         else if(getManhattanDistance(curr.location, end)<getManhattanDistance(second_ans.location, end)){
        //             second_ans = curr;
        //         }
        //         solved_second = true;
        //     }
        //     // if(!solved && !isGoal[curr.location] && !dangerousPlaceFree[curr.location] && safePlaceFree[curr.location]){
        //     //     ans_node = curr;
        //     //     solved = true;
        //     //     if(mapFree[end]){
        //     //         break;
        //     //     }
        //     // }
        // }
        if(goalWasOnceVisited && !goalWasVisited && solved_second)continue;
        // Check visited
        closed_list[id*2+curr.goalWasVisited].insert(curr.g);
        list<pair<int, pair<int, int>>> neighbors = getNeighbors(curr.location, curr.direction, curr.g);
        for (const pair<int, pair<int, int>> &neighbor : neighbors)
        {
            open_list.push(AstarNode(neighbor.first, neighbor.second.first,
                                                 neighbor.second.second, h_cost[neighbor.first*4+neighbor.second.first], par, goalWasVisited));
                                                //  neighbor.second.second, getManhattanDistance(neighbor.first, end), par, goalWasVisited));
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
    delete []h_cost;
    // all_nodes.clear();
    return;
}

    void CityPlanner2::single_agent_plan_dummy(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path)
{
    // queue<pair<int, pair<int, int>>> q;
    // for(int k=0; k<4; ++k)
    //     q.push({0,{end,k}});
    // int *h_cost = new int [(numOfFreeCells)*4];
    // fill(h_cost, h_cost+(numOfFreeCells)*4, -1);
    // while(!q.empty()){
    //     auto tmp = q.front();
    //     q.pop();
    //     if(h_cost[tmp.second.first*4+tmp.second.second]!=-1)continue;
    //     h_cost[tmp.second.first*4+tmp.second.second]=tmp.first;
    //     int nxt = nebor[tmp.second.first*4+tmp.second.second];
    //     if(nxt != -1)
    //         q.push({tmp.first+1, {nxt, tmp.second.second}});
    //     int k = (tmp.second.second+1)%4;
    //     q.push({tmp.first+1, {tmp.second.first, k}});
    //     k = (tmp.second.second+3)%4;
    //     q.push({tmp.first+1, {tmp.second.first, k}});
    // }
    // for(int i=0; i<numOfFreeCells; ++i){
    //     swap(h_cost[i*4], h_cost[i*4+2]);
    //     swap(h_cost[i*4+1], h_cost[i*4+3]);
    // }
    priority_queue<AstarNode, vector<AstarNode>, cmp> open_list;
    vector<AstarNode> all_nodes;
    AstarNode s = AstarNode(start, start_direct, timeStep, getManhattanDistance(start, end), -1, 0);
    open_list.push(s);
    AstarNode ans_node, second_ans;
    bool solved = false, solved_second = false, goalWasOnceVisited = false;
    int par;
    int goalCost = INT_MAX/2;
    std::set<int> *closed_list = new std::set<int>[numOfFreeCells * 4]();
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
        if (!closed_list[id].empty())
        {
            auto it_node_prev = closed_list[id].upper_bound(curr.g);
            if (it_node_prev != closed_list[id].begin())
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
        if (is_safe_place(curr.location) && mapFree[curr.location] == INT_MAX && (map[curr.location].empty() || map[curr.location].upper_bound({curr.g,curr.g}) == map[curr.location].end()))
        {
            if(!isGoal[curr.location]){
                solved_second = true;
                second_ans = curr;
                break;
            }
        }
        // Check visited
        closed_list[id].insert(curr.g);
        list<pair<int, pair<int, int>>> neighbors = getNeighbors(curr.location, curr.direction, curr.g);
        for (const pair<int, pair<int, int>> &neighbor : neighbors)
        {
            open_list.push(AstarNode(neighbor.first, neighbor.second.first,
                                                 neighbor.second.second, getManhattanDistance(neighbor.first, end), par, goalWasVisited));
        }
    }
    if(solved_second){
        path.emplace_front(make_pair(make_pair(second_ans.location, second_ans.direction), make_pair(second_ans.g, second_ans.g)));
        while (second_ans.parent != -1)
        {
            auto pa = all_nodes[second_ans.parent];
            path.emplace_front(make_pair(make_pair(pa.location, pa.direction), make_pair(pa.g, second_ans.g-1)));
            second_ans = pa;
        }
    }
    delete []closed_list;
    return;
}
void CityPlanner2::reserve(const std::list<pair<pair<int, int>,pair<int, int>>> &path)
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

void CityPlanner2::del_reservations(const std::list<pair<pair<int, int>, pair<int, int>>> &path)
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

void CityPlanner2::del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path)
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

int CityPlanner2::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool CityPlanner2::check(list<pair<int, int>> &path)
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

bool CityPlanner2::is_safe_place(int loc)
{
    // if(safePlaceFree[loc]){
    //     return true;
    // }
    int i = row[loc];
    int j = col[loc];
    
    // bool nebor_is_blocked = false;
    // for(int k=0; k<4; ++k){
    //     int ii = i+di[k];
    //     int jj = j+dj[k];
    //     if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
    //     }
    //     else{
    //         nebor_is_blocked = true;
    //     }
    // }
    // if(!nebor_is_blocked && !safePlaceFree[loc]){
    //     return false;
    // }

    vector<int> v, v2;
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    for(int k=0; k<8; ++k){
        int l1 = -1, l2 = -1;
        int ii = i+dx[k];
        int jj = j+dy[k];
        // cout<<" "<< env->map[ii*env->cols+jj] <<" "<<mapFree[freeCells[ii*env->cols+jj]]<<endl;
        if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
            if(k%2==0)
                l1 = 0;
            if(mapFree[freeCells[ii*env->cols+jj]]==INT_MAX){
                if(k%2==0){
                    l2 = 0;
                }
            }
            else{
                l2 = 1;
            }
        }
        else{
            l1 = 1;
            l2 = 1;
        }
        if(l1!=-1)
            v.push_back(l1);
        if(l2!=-1)
            v2.push_back(l2);
        // cout<<jj<<" "<<env->rows-1-ii<<" "<<l1<<endl;
    }
    int tmp = 0, tmp2 = 0;
    for(int i=0;i<v.size();++i){
        if(v[i]!=v[(i+1)%v.size()])++tmp;
        // cout<<v[i]<<" ";
    }
    for(int i=0;i<v2.size();++i){
        if(v2[i]!=v2[(i+1)%v2.size()])++tmp2;
    }
    // cout<<endl;
    if(tmp>=3 || tmp2>=3){
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
    // return false;
}

list<pair<int, pair<int, int>>> CityPlanner2::getNeighbors(int location, int direction, int t)
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

void CityPlanner2::checkReservations()
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