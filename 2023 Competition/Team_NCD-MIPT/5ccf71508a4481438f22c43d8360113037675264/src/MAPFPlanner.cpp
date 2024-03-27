#include <MAPFPlanner.h>
#include <random>
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
        if (a->f == b->f)
            return a->g <= b->g;
        else
            return a->f > b->f;
    }
};

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "planner initialize done" << endl;
    shift_id = 0;
    shift_id1 = 0;
    max_time_limit = 980000;
    if (isMapCity())
    {
        max_time_limit = 950000;
    }
    if(isMapSortation() || isMapWarehouse()){
        max_time_limit = 890000;
    }
    if(isMapGame()){
        max_time_limit = 890000;
    }

    if(isMapWarehouse()){
        flagMapisWarehouse = true;
        safeValues.push_back(4);
        safeValues.push_back(5);
        safeExtraLine = 7;
        // safeValues.push_back(7);
        mx_edge_x = 6;
        mx_edge_y = 8;
    }
    if(isMapSortation()){
        flagIsMapSortation = true;
        safeValues.push_back(2);
        safeValues.push_back(3);
        safeExtraLine = 5;
        // safeValues.push_back(5);
        mx_edge_x = 4;
        mx_edge_y = 6;
    }
    safePlace = new bool[(env->cols) * (env->rows)]();
    dangerousPlace = new bool[(env->cols) * (env->rows)]();
    findSafePlaces();
    timeStep = -1;
    stopFlag = false;
    freeCells = new int [env->map.size()]();
    compressFreeCells();
    genAllNeighors();
}

void MAPFPlanner::findSafePlaces()
{
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    if(!flagIsMapSortation && !flagMapisWarehouse){
        auto data = env->map;
        for(int i=0; i<env->rows; ++i){
            for(int j=0; j<env->cols; ++j){
                if(env->map[i*env->cols+j]){
                    for(int k=0; k<8; ++k){
                        int ii = i+dx[k];
                        int jj = j+dy[k];
                        if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols){
                            data[ii*env->cols+jj]=1;
                        }
                    }
                }
            }
        }
        Image img(env->cols, env->rows, data);
        ZhangSuen zs;
        Image mp = std::move(zs(img));
        for(int i=0; i<mp.height(); ++i){
            for(int j=0; j<mp.width(); ++j){
                if(mp(j, i)==1){
                    safePlace[i*(env->cols)+j]=1;
                }
            }
        }
    }
    if(isMapRandom()){
        for(int i=0; i<env->rows; ++i){
            for(int j=0; j<env->cols; ++j){
                if(env->map[i*env->cols+j]==0){
                    for(int k=0; k<8; k+=2){
                        bool l1, l2;
                        int ii = i+dx[k];
                        int jj = j+dy[k];
                        if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
                            l1 = 0;
                        }
                        else{
                            l1=1;
                        }
                        ii = i+dx[(k+2)%4];
                        jj = j+dy[(k+2)%4];
                        if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
                            l2 = 0;
                        }
                        else{
                            l2=1;
                        }
                        if(l1 && l2){
                            safePlace[i*env->cols+j]=1;
                            break;
                        }
                    }
                }
            }
        }
    }
    if(flagIsMapSortation || flagMapisWarehouse){
        for(int x=0; x<env->rows; ++x){
            for(int y=0; y<env->cols; ++y){
                bool l = 0;
                for(auto it:safeValues){
                    if ((x == it || x == env->rows - 1-it) && (y >= mx_edge_x+2 && y <= env->cols-1-mx_edge_x-2 && y !=env->cols/5&& y !=2*env->cols/5&& y !=3*env->cols/5&& y !=4*env->cols/5))
                    {
                        l = 1;
                        break;
                    }
                    if ((y == it || y == env->cols - 1-it) && (x >= mx_edge_y+2 && x <= env->rows-1-mx_edge_y-2 && x != env->rows/5 && x != 2*env->rows/5 && x != 3*env->rows/5 && x != 4*env->rows/5))
                    {
                        l = 1;
                        break;
                    }
                }
                if ((x == env->rows - 1-safeExtraLine) && (y >= mx_edge_x+2 && y <= env->cols-1-mx_edge_x-2 && y !=env->cols/5&& y !=2*env->cols/5&& y !=3*env->cols/5&& y !=4*env->cols/5))
                {
                    l = 1;
                }
                if(l){
                    safePlace[x*env->cols+y] = 1;
                }
            }
        }
    }
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(env->map[i*env->cols+j]==0){
                vector<int> v;
                for(int k=0; k<8; ++k){
                    int l1 = -1;
                    int ii = i+dx[k];
                    int jj = j+dy[k];
                    if(ii>=0 && ii<env->rows && jj>=0 && jj<env->cols && env->map[ii*env->cols+jj]==0){
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
                    dangerousPlace[i*env->cols+j]=1;
                }
            }
        }
    }
    std::ofstream out;
    out.open("map.txt");
    for(int i=0; i<env->rows; ++i){
        for(int j=0; j<env->cols; ++j){
            if(dangerousPlace[i*(env->cols)+j])
            out<<"+";
            else
            out<<safePlace[i*(env->cols)+j];
        }
        out<<endl;
    }
    out.close();
}

void MAPFPlanner::compressFreeCells(){
    numOfFreeCells = 0;
    for (int i = 0; i < env->rows*env->cols; ++i)
    {
        if (!env->map[i])
        {
            freeCells[i] = numOfFreeCells;
            ++numOfFreeCells;
        }
    }
    mapFree = new bool [numOfFreeCells]();
    nebors = new int[numOfFreeCells*4]();
    map = new std::set<pair<int, int>>[numOfFreeCells]();
    closed_list = new std::set<int>[numOfFreeCells * 4]();
    row = new int [numOfFreeCells]();
    col = new int [numOfFreeCells]();
    isGoal = new bool [numOfFreeCells]();
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
void MAPFPlanner::genAllNeighors()
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
}
// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit, vector<Action> &actions)
{
    ++timeStep;
    start_plan_time = high_resolution_clock::now();
    if (timeStep == 0)
    {
        old_paths = new list<pair<pair<int, int>, pair<int, int>>> [env->num_of_agents]();
        curr_states = new pair<int, int> [env->num_of_agents]();
        goal_locations = new int [env->num_of_agents]();
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
            isGoal[goal_locations[i]]=1;
        }
    }
    if(timeStep == 0){
        // old_paths = vector<list<pair<pair<int, int>, pair<int, int>>>>(env->num_of_agents, list<pair<pair<int, int>,pair<int, int>>>());
        for (int i = 0; i < env->num_of_agents; ++i)
        {
            mapFree[curr_states[i].first] = 1;
        }

    }
    actions = std::vector<Action>(env->num_of_agents, Action::W);
    // if(timeStep > 100)return;
    // if(stopFlag)return;
    vector<pair<int, int>> v;
    vector<int> v1;
    for (int i=0; i<env->num_of_agents; ++i)
    {
        auto &it = old_paths[i];
        // if(it.size()>0){
        //     // cout<<curr_states[i].first<<" "<<it.front().first.first<<endl;
        //     // cout<<curr_states[i].second<<" "<<it.front().first.second<<endl;
        //     assert(it.front().first.first == curr_states[i].first);
        //     assert(it.front().first.second == curr_states[i].second);
        //     assert(it.front().second.first == timeStep);
        //     // assert(mapFree[])
        // }
        if(it.size()==1){
            del_reservations(it);
            it.clear();
        }
        // checkReservations();
        if (it.size()>0) // it.size() > 1
        {
            // for(auto it:old_paths[i]){
            //     cout<<it.first.first<<":["<<it.second.first<<","<<it.second.second<<"], ";
            // }
            // cout<<endl;
            pair<int, int> tmp;
            if(it.front().second.first<it.front().second.second){
                tmp = it.front().first;
            }
            else{
                tmp = (++it.begin())->first;
            }
            // assert(mapFree[it.back().first.first] == 1);
            if (tmp.first != curr_states[i].first)
            {
                actions[i] = Action::FW; // forward action
            }
            else if (tmp.second != curr_states[i].second)
            {
                int incr = tmp.second - curr_states[i].second;
                if (incr == 1 || incr == -3)
                {
                    actions[i] = Action::CR; // C--counter clockwise rotate
                }
                else if (incr == -1 || incr == 3)
                {
                    actions[i] = Action::CCR; // CCR--clockwise rotate
                }
            }
            v1.push_back(i);
        }
        else{
            int loc_x = row[curr_states[i].first];
            int loc_y = col[curr_states[i].first];
            v.push_back(make_pair(min(min(max(0, loc_x-5), max(0, loc_y-5)), min(max(0, env->rows-5-loc_x), max(0, env->cols-5-loc_y))), i));
        }
    }
    // if(flagMapisWarehouse){
    //     sort(v.begin(), v.end());
    // }
    if(v.size()>0){
        shift_id %= v.size();
    }
    int tmp_shift_id = shift_id;
    for (int ii = 0; ii < v.size(); ii++)
    {
        // if(goal_locations[i]!=-1){
        //     assert(goal_locations[ii]!=curr_states[ii].first);
        // }
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = (ii+tmp_shift_id);
        if(i >= v.size()){
            i -= v.size();
        }
        i = v[i].second;
        if (goal_locations[i]!=-1 && old_paths[i].size() == 0)
        {
            list<pair<pair<int, int>, pair<int, int>>> path;
            // assert(mapFree[curr_states[i].first] == 1);
            mapFree[curr_states[i].first] = 0;
            single_agent_plan(curr_states[i].first,
                                     curr_states[i].second,
                                     goal_locations[i], path);
            if (path.size() > 0)
            {
                // assert(path.front().second.first == timeStep);
                // assert(path.back().first.first == goal_locations[i]);
                mapFree[path.back().first.first] = 1;
                old_paths[i] = path;
                // for(auto it:old_paths[i]){
                //     cout<<it.first.first<<":["<<it.second.first<<","<<it.second.second<<"], ";
                // }
                // cout<<endl;
                reserve(path);
            }
            else
            {
                // if(isGoal[curr_states[i].first]){
                // if(flagMapisWarehouse || flagIsMapSortation){
                //     single_agent_plan_away(curr_states[i].first,
                //                      curr_states[i].second,
                //                      goal_locations[i], path);
                //     if (path.size() > 0)
                //     {
                //         // assert(path.front().second.first == timeStep);
                //         mapFree[path.back().first.first] = 1;
                //         old_paths[i] = path;
                //         reserve(path);
                //     }
                //     else{

                //         mapFree[curr_states[i].first] = 1;
                //     }
                // }
                // else{
                // if(isGoal[curr_states[i].first]){
                //     single_agent_plan_just_away(curr_states[i].first,
                //                      curr_states[i].second,
                //                      goal_locations[i], path);
                //     if (path.size() > 0)
                //     {
                //         // assert(path.front().second.first == timeStep);
                //         mapFree[path.back().first.first] = 1;
                //         old_paths[i] = path;
                //         reserve(path);
                //     }
                //     else{

                //         mapFree[curr_states[i].first] = 1;
                //     }
                // }
                // else{
                    mapFree[curr_states[i].first] = 1;
                // }
            }
            // cout<<safePlaceFree[curr_states[i].first]<<"::"<<mapFree[goal_locations[i]]<<"::"<<isGoal[curr_states[i].first]<<" "<<path.size()<<endl;
            // if (path.size() > 1 || (path.size()>0 && path.front().second.first<path.front().second.second))
            // {
            if(path.size() > 0){
                pair<int, int> tmp;
                if(path.size()==1 || path.front().second.first<path.front().second.second){
                    tmp = path.front().first;
                }
                else{
                    tmp = (++path.begin())->first;
                }
                if (tmp.first != curr_states[i].first)
                {
                    actions[i] = Action::FW; // forward action
                }
                else if (tmp.second != curr_states[i].second)
                {
                    int incr = tmp.second - curr_states[i].second;
                    if (incr == 1 || incr == -3)
                    {
                        actions[i] = Action::CR; // C--counter clockwise rotate
                    }
                    else if (incr == -1 || incr == 3)
                    {
                        actions[i] = Action::CCR; // CCR--clockwise rotate
                    }
                }
                else{
                    actions[i] = Action::W;
                }
            }
        }
        shift_id = ii;
        // cout<<ii<<"/"<<v.size()<<endl;
    }
    shift_id += tmp_shift_id;
    if(shift_id>=v.size()){
        shift_id -= v.size();
    }
    if(v1.size()>0){
        shift_id1 %= v1.size();
    }
    tmp_shift_id = shift_id1;
    for (int ii = 0; ii < v1.size(); ii++)
    {
        // if(goal_locations[i]!=-1){
        //     assert(goal_locations[ii]!=curr_states[ii].first);
        // }
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        int i = (ii+tmp_shift_id);
        if(i >= v1.size()){
            i -= v1.size();
        }
        i = v1[i];
        if (goal_locations[i]!=-1)
        {
            list<pair<pair<int, int>, pair<int, int>>> path;
            // assert(mapFree[old_paths[i].back().first.first] == 1);
            mapFree[old_paths[i].back().first.first] = 0;
            del_reservations(old_paths[i]);
            single_agent_plan(curr_states[i].first,
                                     curr_states[i].second,
                                     goal_locations[i], path);
            if (path.size() > 0 && path.back().second.first < old_paths[i].back().second.first)
            {
                // assert(path.front().second.first == timeStep);
                // assert(path.back().first.first == goal_locations[i]);
                mapFree[path.back().first.first] = 1;
                old_paths[i] = path;
                // for(auto it:old_paths[i]){
                //     cout<<it.first.first<<":["<<it.second.first<<","<<it.second.second<<"], ";
                // }
                // cout<<endl;
                reserve(path);
                pair<int, int> tmp;
                if(path.size()==1 || path.front().second.first<path.front().second.second){
                    tmp = path.front().first;
                }
                else{
                    tmp = (++path.begin())->first;
                }
                if (tmp.first != curr_states[i].first)
                {
                    actions[i] = Action::FW; // forward action
                }
                else if (tmp.second != curr_states[i].second)
                {
                    int incr = tmp.second - curr_states[i].second;
                    if (incr == 1 || incr == -3)
                    {
                        actions[i] = Action::CR; // C--counter clockwise rotate
                    }
                    else if (incr == -1 || incr == 3)
                    {
                        actions[i] = Action::CCR; // CCR--clockwise rotate
                    }
                }
                else{
                    actions[i] = Action::W;
                }
            }
            else
            {
                // if(isGoal[curr_states[i].first]){
                // if(flagMapisWarehouse || flagIsMapSortation){
                //     single_agent_plan_away(curr_states[i].first,
                //                      curr_states[i].second,
                //                      goal_locations[i], path);
                //     if (path.size() > 0)
                //     {
                //         // assert(path.front().second.first == timeStep);
                //         mapFree[path.back().first.first] = 1;
                //         old_paths[i] = path;
                //         reserve(path);
                //     }
                //     else{

                //         mapFree[curr_states[i].first] = 1;
                //     }
                // }
                // else{
                // if(isGoal[curr_states[i].first]){
                //     single_agent_plan_just_away(curr_states[i].first,
                //                      curr_states[i].second,
                //                      goal_locations[i], path);
                //     if (path.size() > 0)
                //     {
                //         // assert(path.front().second.first == timeStep);
                //         mapFree[path.back().first.first] = 1;
                //         old_paths[i] = path;
                //         reserve(path);
                //     }
                //     else{

                //         mapFree[curr_states[i].first] = 1;
                //     }
                // }
                // else{
                    mapFree[old_paths[i].back().first.first] = 1;
                    reserve(old_paths[i]);
                // }
            }
            // cout<<safePlaceFree[curr_states[i].first]<<"::"<<mapFree[goal_locations[i]]<<"::"<<isGoal[curr_states[i].first]<<" "<<path.size()<<endl;
            // if (path.size() > 1 || (path.size()>0 && path.front().second.first<path.front().second.second))
            // {
            // if(path.size() > 0){
                
            // }
        }
        shift_id1 = ii;
        // cout<<ii<<"/"<<v.size()<<endl;
    }
    shift_id1 += tmp_shift_id;
    if(shift_id1>=v1.size()){
        shift_id1 -= v1.size();
    }
    // int cnt = 0, cnt1=  0, cnt2 = 0;
    for (int i = 0; i < env->num_of_agents; ++i)
    {
        if (old_paths[i].size() > 0)
        {
            // assert(mapFree[old_paths[i].back().first.first]==1);
            del_first_reservation(old_paths[i]);
            // if(old_paths[i].size()>0)
            //     assert(old_paths[i].front().second.first == timeStep+1);
            // ++cnt;
        }
        // else{
        //     assert(mapFree[curr_states[i].first]==1);
        // }
        // if(goal_locations[i]!=-1){
        //     ++cnt1;
        // }
        // if(!env->goal_locations[i].empty()){
        //     ++cnt2;
        // }
    }
    // cout<<"cnt = "<<cnt<<"/"<<cnt1<<" "<<cnt2<<endl;
    // checkReservations();
    // cout<<"Time elapsed="<<std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count()<<endl;
    // assert(std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count()<1000000);
    return;
}

void MAPFPlanner::single_agent_plan(int start, int start_direct, int end, list<pair<pair<int, int>,pair<int, int>>> &path)
{
    // if(mapFree[end]){
    //     return;
    // }
    // assert(mapFree[start]==0);
    priority_queue<AstarNode *, vector<AstarNode *>, cmp> open_list;
    vector<AstarNode *> all_nodes;
    AstarNode *s = new AstarNode(start, start_direct, timeStep, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes.push_back(s);
    AstarNode * ans_node = nullptr, * second_ans = nullptr;
    bool solved = false, solved_second = false;
    while (!open_list.empty())
    {
        if (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_plan_time).count() > max_time_limit)
        {
            break;
        }
        AstarNode *curr = open_list.top();
        open_list.pop();
        int id = curr->location * 4 + curr->direction;
        if ((map[curr->location].empty() || map[curr->location].upper_bound({curr->g,curr->g}) == map[curr->location].end()))
        {
            if(curr->location == end){
                ans_node = curr;
                solved = true;
                break;
            }
            if(!solved_second && !isGoal[curr->location] && !dangerousPlaceFree[curr->location]){
                solved_second = true;
                second_ans = curr;
            }
            if(!solved && !isGoal[curr->location] && !dangerousPlaceFree[curr->location] && safePlaceFree[curr->location]){
                ans_node = curr;
                solved = true;
                if(mapFree[end]){
                    break;
                }
            }
        }

        // Check visited
        if (!closed_list[id].empty())
        {
            auto it_node_prev = closed_list[id].upper_bound(curr->g);
            if (it_node_prev != closed_list[id].begin())
            {
                --it_node_prev; // node in closed_list which have time less or equal curr->g
                if (map[curr->location].empty())
                { // no block times at this location.
                    continue;
                }
                auto it_block_prev = map[curr->location].upper_bound({curr->g,curr->g});
                if (it_block_prev == map[curr->location].begin())
                { // no block time at this location less than curr->g
                    continue;
                }
                --it_block_prev; // block time less than curr->g
                if (*it_node_prev > it_block_prev->second)
                    continue;
            }
        }
        closed_list[id].insert(curr->g);
        list<pair<int, pair<int, int>>> neighbors = getNeighbors(curr->location, curr->direction, curr->g);
        for (const pair<int, pair<int, int>> &neighbor : neighbors)
        {
            AstarNode *next_node = new AstarNode(neighbor.first, neighbor.second.first,
                                                 neighbor.second.second, getManhattanDistance(neighbor.first, end), curr);
            open_list.push(next_node);
            all_nodes.push_back(next_node);
        }
    }
    if(solved){
        path.emplace_front(make_pair(make_pair(ans_node->location, ans_node->direction), make_pair(ans_node->g, ans_node->g)));
        while (ans_node->parent != NULL)
        {
            auto pa = ans_node->parent;
            path.emplace_front(make_pair(make_pair(pa->location, pa->direction), make_pair(pa->g, ans_node->g-1)));
            ans_node = pa;
        }
    }
    else if(solved_second){
        path.emplace_front(make_pair(make_pair(second_ans->location, second_ans->direction), make_pair(second_ans->g, second_ans->g)));
        while (second_ans->parent != NULL)
        {
            auto pa = second_ans->parent;
            path.emplace_front(make_pair(make_pair(pa->location, pa->direction), make_pair(pa->g, second_ans->g-1)));
            second_ans = pa;
        }
    }
    for (auto n : all_nodes)
    {
        closed_list[n->location * 4 + n->direction].clear();
        delete n;
    }
    all_nodes.clear();
    return;
}
void MAPFPlanner::reserve(const std::list<pair<pair<int, int>,pair<int, int>>> &path)
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

void MAPFPlanner::del_reservations(const std::list<pair<pair<int, int>, pair<int, int>>> &path)
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

void MAPFPlanner::del_first_reservation(std::list<pair<pair<int, int>, pair<int, int>>> &path)
{
    if(path.size()==0){
        return;
    }
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

int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = row[loc1], loc1_y = col[loc1];
    int loc2_x = row[loc2], loc2_y = col[loc2];
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::check(list<pair<int, int>> &path)
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

bool MAPFPlanner::is_safe_place(int loc)
{
    int x = row[loc];
    int y = col[loc];
    for(auto it:safeValues){
        if ((x == it || x == env->rows - 1-it) && (y >= mx_edge_x+2 && y <= env->cols-1-mx_edge_x-2 && y !=env->cols/2))
        {
            return true;
        }
        if ((y == it || y == env->cols - 1-it) && (x >= mx_edge_y+2 && x <= env->rows-1-mx_edge_y-2 && x != env->rows/2))
        {
            return true;
        }
    }
    return false;
}

list<pair<int, pair<int, int>>> MAPFPlanner::getNeighbors(int location, int direction, int t)
{
    list<pair<int, pair<int, int>>> neighbors;
    // forward
    int forward =nebors[location*4+direction];
    int t_u = 1e9;
    auto it = map[location].upper_bound({t,t});
    if (it != map[location].end())
    {
        t_u = it->first - 1;
    }
    if(forward!=-1)
    {
        if(!mapFree[forward]){
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
                if (t_l - 1 <= t_u)
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

void MAPFPlanner::checkReservations()
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

bool MAPFPlanner::isMapWarehouse()
{
    auto name = env->map_name;
    if(name[0]=='w' || name[0]=='W')return true;
    return false;
}

bool MAPFPlanner::isMapSortation()
{
    auto name = env->map_name;
    if(name[0]=='s' || name[0]=='S')return true;
    return false;
}

bool MAPFPlanner::isMapCity(){
    auto name = env->map_name;
    if(name[0]=='C' || name[0]=='c')return true;
    if(name[0]=='P' || name[0]=='p')return true;
    if(env->cols==256 && env->rows==256)return true;
    return false;
}

bool MAPFPlanner::isMapRandom(){
    auto name = env->map_name;
    if(name[0]=='R' || name[0]=='r')return true;
    if(env->cols==32 && env->rows==32)return true;
    return false;
}

bool MAPFPlanner::isMapGame()
{
    auto name = env->map_name;
    if(name[0]=='B' || name[0]=='b')return true;
    if(name[0]=='g' || name[0]=='G')return true;
    if(env->cols==530 && env->rows==481)return true;
    return false;
}
