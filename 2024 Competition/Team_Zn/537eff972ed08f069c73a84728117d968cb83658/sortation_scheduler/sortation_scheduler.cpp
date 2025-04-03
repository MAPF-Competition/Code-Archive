#include "sortation_scheduler.h"
#include "amapfd_soc_planner.h"
#include <set>
#include <fstream>

namespace SortationScheduler
{
    std::mt19937 mt;
    std::unordered_map<int, bool> undesired_locations;
    long long number_of_free_cells;
    bool candidate_tasks[500000];
    int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    int16_t *distancesBetweenCells;

    int manhattanCost(int cell1, int cell2, const SharedEnvironment *env)
    {
        int r1 = cell1 / env->cols, c1 = cell1 % env->cols;
        int r2 = cell2 / env->cols, c2 = cell2 % env->cols;
        return abs(r1 - r2) + abs(c1 - c2) + (r1 != r2 && c1 != c2);
    }
    int manhattanCost(Task &task, const SharedEnvironment *env)
    {
        return manhattanCost(task.locations[0], task.locations[1], env);
    }
    int realCost(long long cell1, long long cell2)
    {
        cell1 = AMAPFDSOCPlanner::free_cells[cell1];
        cell2 = AMAPFDSOCPlanner::free_cells[cell2];
        if (cell1 > cell2)
            return distancesBetweenCells[cell2 * number_of_free_cells - (cell2 * (cell2 - 1) / 2) + cell1 - cell2];
        return distancesBetweenCells[cell1 * number_of_free_cells - (cell1 * (cell1 - 1) / 2) + cell2 - cell1];
    }
    int taskCost(Task &task)
    {
        int cost = 0;
        for (int i = 0; i + 1 < task.locations.size(); ++i)
        {
            cost += realCost(task.locations[i], task.locations[i + 1]);
        }
        return cost;
    }
    bool undesiredLocation(int location, const SharedEnvironment *env)
    {
        int col = location % env->cols;
        int row = location / env->cols;
        int cnt = 0;
        for (int k = 0; k < 4; ++k)
        {
            int x = col + dx[k];
            int y = row + dy[k];
            int id = y * env->rows + x;
            if (x < 0 || y < 0 || x >= env->cols || y >= env->rows || env->map[id] == 1)
            {
                ++cnt;
            }
        }
        return cnt >= 3;
    }
    void storeUndesiredLocations(const SharedEnvironment *env)
    {
        for (int i = 0; i < env->map.size(); ++i)
        {
            if (undesiredLocation(i, env))
            {
                undesired_locations[i] = 1;
            }
            else
            {
                undesired_locations[i] = 0;
            }
        }
    }

    void calculateRealDistances(SharedEnvironment *env)
    {
        // std::ofstream out;
        // out.open("../pre/"+env->map_name+"_dists");
        // if(!out.good()){
        //     std::cerr<<"Couldn't open the storage file"<<std::endl;
        //     exit(0);
        // }
        distancesBetweenCells = new int16_t[number_of_free_cells * (number_of_free_cells + 1) / 2];
        for (long long i = 0; i < number_of_free_cells * (number_of_free_cells + 1) / 2; ++i)
        {
            distancesBetweenCells[i] = -1;
        }
        for (long long cell1 = 0; cell1 < number_of_free_cells; ++cell1)
        {
            vector<bool> vis(number_of_free_cells * 4, 0);
            std::queue<pair<int, int>> q;
            q.push(make_pair(0, cell1 * 4));
            q.push(make_pair(0, cell1 * 4 + 1));
            q.push(make_pair(0, cell1 * 4 + 2));
            q.push(make_pair(0, cell1 * 4 + 3));
            while (!q.empty())
            {
                auto tmp = q.front();
                q.pop();
                if (vis[tmp.second])
                    continue;
                vis[tmp.second] = true;
                long long cell2 = tmp.second / 4, dir = tmp.second % 4, cost = tmp.first;
                if (cell2 >= cell1 && distancesBetweenCells[cell1 * number_of_free_cells - (cell1 * (cell1 - 1) / 2) + cell2 - cell1] == -1)
                    distancesBetweenCells[cell1 * number_of_free_cells - (cell1 * (cell1 - 1) / 2) + cell2 - cell1] = cost;
                q.push({cost + 1, cell2 * 4 + (dir + 3) % 4});
                q.push({cost + 1, cell2 * 4 + (dir + 1) % 4});
                int cell = AMAPFDSOCPlanner::inverse_free_cells[cell2];
                int x = cell / env->cols, y = cell % env->cols;
                int xx = x + dx[dir], yy = y + dy[dir];
                int id = xx * env->cols + yy;
                if (xx < env->rows && xx >= 0 && yy < env->cols && yy >= 0)
                {
                    if (!env->map[id])
                    {
                        q.push({cost + 1, AMAPFDSOCPlanner::free_cells[id] * 4 + dir});
                    }
                }
            }
            // for(long long i=cell1; i<number_of_free_cells; ++i){
            //     out<<distancesBetweenCells[cell1*number_of_free_cells-(cell1*(cell1-1)/2)+i-cell1]<<" ";
            // }
            // out<<"\n";
        }
        // out.close();
    }
    void readDistances(SharedEnvironment *env)
    {
        distancesBetweenCells = new int16_t[number_of_free_cells * (number_of_free_cells + 1) / 2];
        auto file_path = env->file_storage_path + "/" + env->map_name + "_dists-";
        std::ifstream in;
        in.open(file_path.c_str());
        if (!in.good())
        {
            std::cerr << "Can't read the file\n";
            std::cerr << "There is no file named: " << file_path << "\n";
            calculateRealDistances(env);
            return;
        }
        else
        {
            std::ios_base::sync_with_stdio(false);
            int cnt = 0;
            for (long long cell1 = 0; cell1 < number_of_free_cells; ++cell1)
            {
                for (long long cell2 = cell1; cell2 < number_of_free_cells; ++cell2)
                {
                    in >> distancesBetweenCells[cnt++];
                }
            }
            std::cerr << "Stored file was read successfully!\n";
        }
        in.close();
    }
    void getNewInputs(vector<int> &free_agents, vector<int> &free_tasks, SharedEnvironment *env)
    {
        vector<bool> near_agents(env->map.size(), 0);
        int sz = env->num_of_agents;
        vector<int> agents;
        for (int i = 0; i < sz; ++i)
        {
            if (env->curr_task_schedule[i] == -1)
            {
                free_agents.push_back(i);
            }
            // else if(env->task_pool[env->curr_task_schedule[i]].idx_next_loc == 0){
            //     agents.push_back(i);
            // }
        }
        for (auto task : env->task_pool)
        {
            if (task.second.agent_assigned == -1)// && candidate_tasks[task.first])
            { // && assigned_tasks[task.second.get_next_loc()] < num_of_allowed_repetions_for_the_same_goal){
                free_tasks.push_back(task.first);
                // assigned_tasks[task.second.get_next_loc()]++;
            }
        }
        // std::queue<pair<int, int>> q;
        // vector<bool> vis(number_of_free_cells * 4, 0);
        // vector<int> dist(number_of_free_cells, 1e9);
        // for(auto agent:free_agents){
        //     q.push({0, AMAPFDSOCPlanner::free_cells[env->curr_states[agent].location]*4+env->curr_states[agent].orientation});
        // }
        // while(!q.empty()){
        //     auto tmp = q.front();
        //     q.pop();
        //     if(vis[tmp.second])continue;
        //     vis[tmp.second]=1;
        //     int cell = tmp.second / 4, dir = tmp.second % 4, cost = tmp.first;
        //     dist[cell]=min(dist[cell], tmp.first);
        //     q.push({cost + 1, cell * 4 + (dir + 3) % 4});
        //     q.push({cost + 1, cell * 4 + (dir + 1) % 4});
        //     cell = AMAPFDSOCPlanner::inverse_free_cells[cell];
        //     int x = cell / env->cols, y = cell % env->cols;
        //     int xx = x + dx[dir], yy = y + dy[dir];
        //     int id = xx * env->cols + yy;
        //     if (xx < env->rows && xx >= 0 && yy < env->cols && yy >= 0)
        //     {
        //         if (!env->map[id] && !vis[AMAPFDSOCPlanner::free_cells[id] * 4 + dir])
        //         {
        //             q.push({cost + 1, AMAPFDSOCPlanner::free_cells[id] * 4 + dir});
        //         }
        //     }
        // }
        // for(auto agent:agents){
        //     int task = env->curr_task_schedule[agent];
        //     int task_location = env->task_pool[task].get_next_loc();
        //     int cost = realCost(env->curr_states[agent].location, task_location);
        //     if(dist[AMAPFDSOCPlanner::free_cells[task_location]] + 5 < cost){
        //         free_agents.push_back(agent);
        //         free_tasks.push_back(task);
        //     }
        // }
        // cout<<free_agents.size()<<" < "<<free_tasks.size()<<endl;
    }

    void refineTaskPool(SharedEnvironment* &env){
        int cnt[2][2] = {0};
        int sz = env->task_pool.size();
        sz /= 4;
        for(int i=0; i<2; ++i){
            for(int j=0; j<2; ++j){
                cnt[i][j] = sz+200;
            }
        }
        vector<pair<int, int>> tasks_with_costs;
        for(auto it:env->task_pool){
            int location = it.second.locations.back();
            int col = location%env->cols/((env->cols+1)/2);
            int row = location/env->cols/((env->rows+1)/2);
            if(it.second.agent_assigned != -1){
                --cnt[row][col];
                continue;
            }
            int cost = taskCost(it.second);
            tasks_with_costs.push_back({cost, it.first});
        }
        sort(tasks_with_costs.begin(), tasks_with_costs.end());
        for(auto it:tasks_with_costs){
            int location = env->task_pool[it.second].locations.back();
            int col = location%env->cols/((env->cols+1)/2);
            int row = location/env->cols/((env->rows+1)/2);
            if(cnt[row][col]>0){
                candidate_tasks[it.second] = true;
                --cnt[row][col];
            }
            else{
                candidate_tasks[it.second] = false;
            }
        }
    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        // if (env->new_freeagents.size() == 0)
        // {
        //     return;
        // }
        clock_t start = clock();
        vector<int> free_agents, free_tasks, free_agents_location;
        vector<pair<int, int>> free_tasks_locations_and_costs;
        // refineTaskPool(env);
        getNewInputs(free_agents, free_tasks, env);
        for (auto agent : free_agents)
        {
            free_agents_location.push_back(env->curr_states[agent].location);
        }
        unordered_map<int, vector<pair<int, int>>> location_to_task;
        int j = 0;
        const int factor = 1;
        const int limit = 4;
        int minimum_cost = INT_MAX;
        int mx_number_of_repetitions_of_a_task_location = 0;
        unordered_map<int, int> number_of_times_task_location_is_assigned;
        unordered_map<int, int> number_of_times_task_location_is_available;
        for (auto task : env->task_pool)
        {
            int location = task.second.get_next_loc();
            if(task.second.idx_next_loc == 0){
                number_of_times_task_location_is_available[location]++;
                mx_number_of_repetitions_of_a_task_location = max(mx_number_of_repetitions_of_a_task_location, number_of_times_task_location_is_available[location]);
            // }
            if (task.second.agent_assigned != -1)
            {
                number_of_times_task_location_is_assigned[location]++;
                // if(number_of_times_task_location_is_assigned[location]>limit){
                //     exit(0);
                // }
                continue;
            }
            }
        }
        for (auto task : free_tasks)
        {
            int location = env->task_pool[task].locations[0];
            if (env->task_pool[task].agent_assigned != -1){
                number_of_times_task_location_is_assigned[location]--;
            }
            int new_cost = taskCost(env->task_pool[task]);
            location_to_task[location].push_back(make_pair(new_cost, task));
            minimum_cost = min(minimum_cost, new_cost);
        }
        for (auto &location_task_vector : location_to_task)
        {
            sort(location_task_vector.second.begin(), location_task_vector.second.end());
        }
        // unordered_map<int, int> free_tasks_locations_count;
        // for (auto task : free_tasks)
        // {
        //     ++free_tasks_locations_count[env->task_pool[task].get_next_loc()];
        // }
        if(free_agents_location.size()<40){
            for (auto location_and_vector_of_id_cost_task : location_to_task)
            {
                for (int i = 0; i < min((int)location_and_vector_of_id_cost_task.second.size(), limit-number_of_times_task_location_is_assigned[location_and_vector_of_id_cost_task.first]); ++i)
                {
                    free_tasks_locations_and_costs.push_back(make_pair(location_and_vector_of_id_cost_task.first, (location_to_task[location_and_vector_of_id_cost_task.first][i].first - minimum_cost)/5)); // {location, i-th minimum cost}
                    // free_tasks_locations_and_costs.push_back(make_pair(location_and_vector_of_id_cost_task.first, (mx_number_of_repetitions_of_a_task_location-number_of_times_task_location_is_available[location_and_vector_of_id_cost_task.first]+i)*factor)); // {location, i-th minimum cost}
                }
            }
        }
        else{
            for (auto location_and_vector_of_id_cost_task : location_to_task)
            {
                for (int i = 0; i < min((int)location_and_vector_of_id_cost_task.second.size(), limit-number_of_times_task_location_is_assigned[location_and_vector_of_id_cost_task.first]); ++i)
                {
                    free_tasks_locations_and_costs.push_back(make_pair(location_and_vector_of_id_cost_task.first, (location_to_task[location_and_vector_of_id_cost_task.first][i].first - minimum_cost)/20)); // {location, i-th minimum cost}
                    // free_tasks_locations_and_costs.push_back(make_pair(location_and_vector_of_id_cost_task.first, (mx_number_of_repetitions_of_a_task_location-number_of_times_task_location_is_available[location_and_vector_of_id_cost_task.first]+i)*factor)); // {location, i-th minimum cost}
                }
            }
        }
        // if(free_tasks_locations_and_costs.size() < 1.2 * free_agents_location.size()){
        //     for(auto &it:free_tasks_locations_and_costs){
        //         it.second = 0;
        //     }
        // }
        AMAPFDSOCPlanner::dynInit(free_tasks_locations_and_costs.size());
        AMAPFDSOCPlanner::fillInputs(free_agents_location, free_tasks_locations_and_costs);
        AMAPFDSOCPlanner::dynGenAllNeighbors(free_tasks_locations_and_costs.size());
        int max_number_of_assignments =  min((int)(free_tasks_locations_and_costs.size()/1.2), (int)(free_agents_location.size()));
        // cout<<free_agents.size()<<" "<<free_tasks_locations_and_costs.size()<<"::"<<max_number_of_assignments<<endl;
        AMAPFDSOCPlanner::solve(max_number_of_assignments);
        vector<int> new_assignments; // new_assignment: map_size -> map_size. It gives the assignment of the start location to the goal location. We later convert the locations to tasks.
        AMAPFDSOCPlanner::getAssignments(new_assignments);
        AMAPFDSOCPlanner::dynReset();
        
        for (auto &location_task_vector : location_to_task)
        {
            reverse(location_task_vector.second.begin(), location_task_vector.second.end());
        }
        for (int i = 0; i < free_agents.size(); ++i)
        {
            if (new_assignments[free_agents_location[i]] == -1)
            {
                proposed_schedule[free_agents[i]] = -1;
                // exit(0);
            }
            else
            {
                proposed_schedule[free_agents[i]] = location_to_task[new_assignments[free_agents_location[i]]].back().second;
                location_to_task[new_assignments[free_agents_location[i]]].pop_back();
            }
        }
        return;
    }
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        AMAPFDSOCPlanner::readMap(env);
        AMAPFDSOCPlanner::compressFreeCells();
        AMAPFDSOCPlanner::genAllNeighbors();
        number_of_free_cells = AMAPFDSOCPlanner::number_of_free_cells;
        readDistances(env);
        return;
    }
}
