// #include "MyPlanner.h"

// namespace MyPlanner{

// std::mt19937 mt;
// std::unordered_set<int> free_agents;
// std::unordered_set<int> free_tasks;


// static const int ORIENTATIONS = 4; // 0:east,1:south,2:west,3:north

// // 存储 (loc, dir) → cost
// struct HeuristicTableRot {
//     std::vector<int> htable;  // 大小 = map.size() * ORIENTATIONS
//     std::deque<HNode> open;   // 存放 BFS 节点
//     bool empty() const {
//         return htable.empty();
//     }
// };

// // 全局带方向的启发式表向量
// std::vector<HeuristicTableRot> global_heuristic_rot;

// //heuristics.cpp
// //存储从某个目标位置出发到地图其他位置的距离（或代价）
// std::vector<HeuristicTable> global_heuristictable;
// Neighbors global_neighbors;

// void init_heuristic_rot(HeuristicTableRot& ht, SharedEnvironment* env,
//                         int goal_loc, int goal_dir)
// {
//     // htable 大小 = map_size * ORIENTATIONS
//     ht.htable.clear();
//     ht.htable.resize(env->map.size() * ORIENTATIONS, MAX_TIMESTEP);
//     ht.open.clear();

//     // 构造一个根节点 root( location=goal_loc, direction=goal_dir, cost=0 )
//     HNode root(goal_loc, goal_dir, 0);
//     // 在 htable 中记录下 (goal_loc, goal_dir) = 0
//     int idx = goal_loc * ORIENTATIONS + goal_dir;
//     ht.htable[idx] = 0;

//     // 加入到 open
//     ht.open.push_back(root);
// }

// int get_heuristic_rot(HeuristicTableRot& ht,
//                       SharedEnvironment* env,
//                       int source_loc, int source_dir,
//                       Neighbors* ns)
// {
//     // 若已经有值
//     int idx_src = source_loc * ORIENTATIONS + source_dir;
//     if (ht.htable[idx_src] < MAX_TIMESTEP) {
//         return ht.htable[idx_src];
//     }

//     // BFS 扩展
//     while (!ht.open.empty())
//     {
//         HNode curr = ht.open.front();
//         ht.open.pop_front();

//         int curr_idx = curr.location * ORIENTATIONS + curr.direction;
//         // BFS 当前 cost
//         int curr_cost = ht.htable[curr_idx];

//         // 若找到目标
//         if (curr.location == source_loc){//&& curr.direction == source_dir
//             return curr_cost;
//         }

//         // 1) 前进
//         //   如果朝向是 0(east)，前进到 loc+1； 1(south)-> loc+cols 等
//         //   也可通过 global_neighbors 实现通用
//         for (int next_loc : ns->at(curr.location)) {
//             // 只有当 next_loc 是在当前方向正前方时，才视为“前进”
//             // 示例: direction = 0(east)，那么 next_loc应该 == curr.location +1
//             // 这里简单做个判断:
//             if ( MyPlanner::isForward(curr.location, next_loc, curr.direction, env->cols) ) {
//                 int nxt_idx = next_loc * ORIENTATIONS + curr.direction;
//                 int new_cost = curr_cost + 1; // 前进耗费1
//                 if (new_cost < ht.htable[nxt_idx]){
//                     ht.htable[nxt_idx] = new_cost;
//                     ht.open.emplace_back(next_loc, curr.direction, new_cost);
//                 }
//             }
//         }

//         // 2) 左转、右转：不改变 location，但 direction 改变
//         {
//             int left_dir = (curr.direction + 3) % ORIENTATIONS;  //左转
//             int right_dir = (curr.direction + 1) % ORIENTATIONS; //右转
//             int cost_turn = curr_cost + 1; // 转向也当做1步

//             int idxL = curr.location * ORIENTATIONS + left_dir;
//             if (cost_turn < ht.htable[idxL]) {
//                 ht.htable[idxL] = cost_turn;
//                 ht.open.emplace_back(curr.location, left_dir, cost_turn);
//             }

//             int idxR = curr.location * ORIENTATIONS + right_dir;
//             if (cost_turn < ht.htable[idxR]) {
//                 ht.htable[idxR] = cost_turn;
//                 ht.open.emplace_back(curr.location, right_dir, cost_turn);
//             }
//         }

//         // 3) 等待: 不变 loc, 不变 dir, cost + 1 (可根据需求决定是否需要)
//         // {
//         //     int wait_idx = curr_idx; // same loc, dir
//         //     int cost_wait = curr_cost + 1;
//         //     if (cost_wait < ht.htable[wait_idx]) {
//         //         ht.htable[wait_idx] = cost_wait;
//         //         ht.open.emplace_back(curr.location, curr.direction, cost_wait);
//         //     }
//         // }
//     }

//     // 若整个 BFS 都没找到 (source_loc, source_dir)，说明不可达
//     return MAX_TIMESTEP;
// }

// // 判断 next_loc 是否为当前 direction 的正前方
// bool isForward(int curr_loc, int next_loc, int direction, int cols)
// {
//     switch(direction) {
//         case 0: // east
//             return next_loc == curr_loc + 1;
//         case 1: // south
//             return next_loc == curr_loc + cols;
//         case 2: // west
//             return next_loc == curr_loc - 1;
//         case 3: // north
//             return next_loc == curr_loc - cols;
//         default:
//             return false;
//     }
// }

// int get_h_rot(SharedEnvironment* env,
//               int source_loc, int source_dir,
//               int target_loc, int target_dir)
// {
//     // 如果全局向量还没初始化，就初始化
//     if (global_heuristic_rot.empty()) {
//         global_heuristic_rot.resize(env->map.size() * ORIENTATIONS); 
//         // 并初始化邻接
//         init_neighbor(env);
//     }

//     // 计算全局索引: (target_loc, target_dir)
//     int t_idx = target_loc * ORIENTATIONS + target_dir;
//     // 若 global_heuristic_rot[t_idx] 还没初始化 (htable 为空)
//     if (global_heuristic_rot[t_idx].htable.empty()) {
//         init_heuristic_rot(global_heuristic_rot[t_idx], env, target_loc, target_dir);
//     }

//     // 调用 BFS 来获取 source 的距离
//     return get_heuristic_rot(global_heuristic_rot[t_idx], env,
//                              source_loc, source_dir, &global_neighbors);
// }


// void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
// {
//     // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
//     MyPlanner::init_heuristics(env);
//     mt.seed(0);
//     return;
// }

// void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
// {
//     //use at most half of time_limit to compute schedule, -10 for timing error tolerance
//     //so that the remainning time are left for path planner
//     TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
//     // cout<<"schedule plan limit" << time_limit <<endl;

//     // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
//     free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
//     free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

//     // 输出检查这两个集合
//     std::cout << "Free Agents Size: " << free_agents.size() << std::endl;
//     std::cout << "Free Tasks Size: " << free_tasks.size() << std::endl;

//     // if (free_agents.size() < 2) {
//     //     return;
//     // }

//     int min_task_i, min_task_makespan, dist, c_loc, count;
//     int t_id, best_agent, best_makespan, existenceTime, dynamic_threshold;
//     int c_dir;
//     clock_t start = clock();

//     // 反过来任务选智能体
//     // 为每个任务设置一个随时间动态增长的最大makespan阈值，只有最优的智能体可以满足该阈值才分配执行

//     // // iterate over the free tasks to decide which agent to assign each task
//     // std::unordered_set<int>::iterator task_it = free_tasks.begin();
//     // while (task_it != free_tasks.end())
//     // {
//     //     // Check for timeout before processing each task
//     //     if (std::chrono::steady_clock::now() > endtime)
//     //     {
//     //         break;
//     //     }
        
//     //     t_id = *task_it;
        
//     //     best_agent = -1;
//     //     best_makespan = INT_MAX;
//     //     count = 0;
        
//     //     existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 0);
//     //     dynamic_threshold = std::min(existenceTime + 5, 25);

//     //     // Iterate over all free agents to find the one with minimum makespan for task t_id
//     //     for (int i : free_agents)
//     //     {
//     //         //check for timeout every 10 task evaluations
//     //         if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
//     //         {
//     //             break;
//     //         }
//     //         // For each free agent, calculate the makespan for completing task t_id
//     //         int makespan = 0;
//     //         int current_loc = env->curr_states.at(i).location;
            
//     //         // Iterate over the locations (errands) of task t_id to compute the makespan
//     //         for (int loc : env->task_pool[t_id].locations) {
//     //             makespan += MyPlanner::get_h(env, current_loc, loc);
//     //             current_loc = loc;
//     //             break;  // 考虑第一个点位即可
//     //         }
            
//     //         if (makespan < best_makespan) {
//     //             best_makespan = makespan;
//     //             best_agent = i;
//     //         }
//     //         count++;
//     //     }
        
//     //     // If a suitable agent is found, assign task t_id to that agent
//     //     if (best_agent != -1) {
//     //         // proposed_schedule[best_agent] = t_id;
//     //         // // Remove the selected agent from free_agents so it is not assigned another task
//     //         // free_agents.erase(best_agent);
//     //         // // Erase the current task from free_tasks
//     //         // task_it = free_tasks.erase(task_it);
            
//     //         // // Output best_makespan, best_agent, and t_id for checking
//     //         // std::cout << "Best Makespan: " << best_makespan << ", Best Agent: " << best_agent << ", Task ID: " << t_id << std::endl;
//     //         if (best_makespan <= dynamic_threshold)
//     //         {
//     //             proposed_schedule[best_agent] = t_id;
//     //             // 从空闲集合移除该智能体，以防再次分配
//     //             free_agents.erase(best_agent);
//     //             // 从任务集合移除该任务
//     //             task_it = free_tasks.erase(task_it);

//     //             // 输出用于调试或观察
//     //             // std::cout << "[Assign] Task " << t_id 
//     //             //           << " bestAgent=" << best_agent 
//     //             //           << ", bestMakespan=" << best_makespan
//     //             //           << ", threshold=" << dynamic_threshold
//     //             //           << std::endl;
//     //         }
//     //     }
//     //     else {
//     //         // 如果没有找到合适的智能体，则保留任务（或将其分配为 -1 表示未分配），然后继续下一个任务
//     //         task_it++;
//     //     }
//     // }

//     // 智能体选任务 动态阈值决定最大可接受的makespan
//     // iterate over the free agents to decide which task to assign to each of them
//     std::unordered_set<int>::iterator it = free_agents.begin();
//     while (it != free_agents.end())
//     {
//         //keep assigning until timeout
//         if (std::chrono::steady_clock::now() > endtime)
//         {
//             break;
//         }
//         int i = *it;

//         assert(env->curr_task_schedule[i] == -1);
            
//         min_task_i = -1;
//         min_task_makespan = INT_MAX;
//         count = 0;


//         // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
//         for (int t_id : free_tasks)
//         {
//             // existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 5);
//             // dynamic_threshold = std::min(existenceTime, 10);
//             // dynamic_threshold = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, 20);
//             //根据地图规模和智能体数量设置区间
//             int x = env->rows * env->cols / env->num_of_agents;
//             // std::cout << "x: " << x << std::endl;
//             // random_32_32_20_100.json x=10
//             // warehouse_large_5000.json x=14
//             // sortation_large_2000.json x=35
//             existenceTime = std::max(env->curr_timestep - env->task_pool[t_id].t_revealed, x/2);
//             dynamic_threshold = std::min(existenceTime, x);


//             //check for timeout every 10 task evaluations
//             if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
//             {
//                 std::cout << "Timeout" << std::endl;
//                 break;
//             }   //每 10 次评估后检查当前时间是否超过截止时间 endtime，如果超时则退出当前任务遍历。
//             dist = 0;
//             c_loc = env->curr_states.at(i).location;
//             c_dir = env->curr_states.at(i).orientation;

//             // iterate over the locations (errands) of the task to compute the makespan to finish the task
//             // makespan: the time for the agent to complete all the errands of the task t_id in order
//             for (int loc : env->task_pool[t_id].locations){
//                 // dist += MyPlanner::get_h(env, c_loc, loc);
//                 dist += get_h_rot(env, c_loc, c_dir, loc, c_dir);
//                 c_loc = loc;
//                 break;  // 考虑第一个点位即可
//             }

//             // update the new minimum makespan
//             if (dist < dynamic_threshold and dist < min_task_makespan){
//             // if (dist < min_task_makespan){
//                 min_task_i = t_id;
//                 min_task_makespan = dist;
//             }
//             count++;            
//         }

//         // assign the best free task to the agent i (assuming one exists)
//         if (min_task_i != -1){
//             proposed_schedule[i] = min_task_i;
//             it = free_agents.erase(it);
//             free_tasks.erase(min_task_i);
//             // 输出用于调试或观察
//             std::cout << "[Assign] Agent " << i 
//                   << " assigned to Task " << min_task_i 
//                   << " with Makespan " << min_task_makespan 
//                   << std::endl;
//         }
//         // nothing to assign
//         else{
//             proposed_schedule[i] = -1;
//             it++;
//         }
//     }

//     #ifndef NDEBUG
//     cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
//     cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
//     cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
//     #endif
//     return;
// }

// //heuristics.cpp
// void init_neighbor(SharedEnvironment* env){
// 	global_neighbors.resize(env->rows * env->cols);
// 	for (int row=0; row<env->rows; row++){
// 		for (int col=0; col<env->cols; col++){
// 			int loc = row*env->cols+col;
// 			if (env->map[loc]==0){
// 				if (row>0 && env->map[loc-env->cols]==0){
// 					global_neighbors[loc].push_back(loc-env->cols);
// 				}
// 				if (row<env->rows-1 && env->map[loc+env->cols]==0){
// 					global_neighbors[loc].push_back(loc+env->cols);
// 				}
// 				if (col>0 && env->map[loc-1]==0){
// 					global_neighbors[loc].push_back(loc-1);
// 				}
// 				if (col<env->cols-1 && env->map[loc+1]==0){
// 					global_neighbors[loc].push_back(loc+1);
// 				}
// 			}
// 		}
// 	}
// };

// void init_heuristics(SharedEnvironment* env){
// 	if (global_heuristictable.size()==0){
// 		global_heuristictable.resize(env->map.size());
// 		init_neighbor(env);
// 	}

// }

// void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
// 	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
// 	ht.htable.clear();
// 	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
// 	ht.open.clear();
// 	// generate a open that can save nodes (and a open_handle)
// 	// 构造一个根节点 root，表示在 goal_location 处距离为 0
// 	HNode root(goal_location,0, 0);
// 	ht.htable[goal_location] = 0;
// 	ht.open.push_back(root);  // add root to open
// }


// int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
// 		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

// 		std::vector<int> neighbors;
// 		int cost, diff;	//BFS
// 		while (!ht.open.empty())
// 		{
// 			HNode curr = ht.open.front();
// 			ht.open.pop_front();

			
// 			getNeighborLocs(ns,neighbors,curr.location);

			
// 			for (int next : neighbors)
// 			{
// 				cost = curr.value + 1;
// 				diff = curr.location - next;
				
// 				assert(next >= 0 && next < env->map.size());
// 				//set current cost for reversed direction

// 				if (cost >= ht.htable[next] )
// 					continue;

// 				ht.open.emplace_back(next,0, cost);
// 				ht.htable[next] = cost;
				
// 			}

// 			if (source == curr.location)
// 				return curr.value;
// 		}


// 		return MAX_TIMESTEP;
// }

// //获取地图上两个位置 source、target 间的启发式距离
// int get_h(SharedEnvironment* env, int source, int target){
// 	if (global_heuristictable.empty()){
// 		init_heuristics(env);
// 	}

// 	if (global_heuristictable.at(target).empty()){
// 		init_heuristic(global_heuristictable.at(target),env,target);
// 	}

// 	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
// }



// void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
// 	if (dp.dist2path.empty())
// 		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
// 	dp.open.clear();
// 	dp.label++;

//     int togo = 0;
//     for(int i = path.size()-1; i>=0; i--){
//         auto p = path[i];
// 		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
// 		dp.open.emplace_back(dp.label,p,0,togo);
// 		dp.dist2path[p] = {dp.label,p,0,togo};
// 		togo++;
//     }

// }
// //计算某一个普通位置到给定路径的距离
// std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
// {
// 	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
// 		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

// 		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
// 	}

	
// 	std::vector<int> neighbors;
// 	int cost;

// 	while (!dp.open.empty())// 多源BFS
// 	{
// 		d2p curr = dp.open.front();
// 		dp.open.pop_front();



// 		getNeighborLocs(ns,neighbors,curr.id);

// 		for (int next_location : neighbors)
// 		{

// 			cost = curr.cost + 1;

// 			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
// 				continue;
// 			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
// 			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
// 		}
// 		if (source == curr.id){
// 			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
// 			return std::make_pair(curr.cost, curr.togo);
// 		}
// 	}

// 	return std::make_pair(MAX_TIMESTEP,0);
// }
// //计算地图上任意位置到给定路径的最短距离和剩余步数
// int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
// {

// 	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

// 	return dists.first + dists.second;
// }

// }
