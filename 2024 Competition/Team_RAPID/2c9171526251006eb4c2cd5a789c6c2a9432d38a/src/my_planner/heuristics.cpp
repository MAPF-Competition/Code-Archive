
#include "my_planner/heuristics.h"
#include <queue>

namespace MyPlanner{
//存储从某个目标位置出发到地图其他位置的距离（或代价）
std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;

static const int ORIENTATIONS = 4; // 0:east,1:south,2:west,3:north


// 全局带方向的启发式表向量
std::vector<HeuristicTableRot> global_heuristic_rot;

void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (global_heuristictable.size()==0){
		global_heuristictable.resize(env->map.size());
		init_neighbor(env);
	}

}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	// 构造一个根节点 root，表示在 goal_location 处距离为 0
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

		std::vector<int> neighbors;
		int cost, diff;	//BFS
		while (!ht.open.empty())
		{
			HNode curr = ht.open.front();
			ht.open.pop_front();

			
			getNeighborLocs(ns,neighbors,curr.location);

			
			for (int next : neighbors)
			{
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				if (cost >= ht.htable[next] )
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable[next] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}

//获取地图上两个位置 source、target 间的启发式距离
int get_h(SharedEnvironment* env, int source, int target){
	if (global_heuristictable.empty()){
		init_heuristics(env);
	}

	if (global_heuristictable.at(target).empty()){
		init_heuristic(global_heuristictable.at(target),env,target);
	}

	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
}



void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	if (dp.dist2path.empty())
		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
	dp.open.clear();
	dp.label++;

    int togo = 0;
    for(int i = path.size()-1; i>=0; i--){
        auto p = path[i];
		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
		dp.open.emplace_back(dp.label,p,0,togo);
		dp.dist2path[p] = {dp.label,p,0,togo};
		togo++;
    }

}
//计算某一个普通位置到给定路径的距离
std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())// 多源BFS
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();



		getNeighborLocs(ns,neighbors,curr.id);

		for (int next_location : neighbors)
		{

			cost = curr.cost + 1;

			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
				continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
		}
		if (source == curr.id){
			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
			return std::make_pair(curr.cost, curr.togo);
		}
	}

	return std::make_pair(MAX_TIMESTEP,0);
}
//计算地图上任意位置到给定路径的最短距离和剩余步数
int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}


void init_heuristic_rot(HeuristicTableRot& ht, SharedEnvironment* env,
                        int goal_loc, int goal_dir)
{
    // htable 大小 = map_size * ORIENTATIONS
    ht.htable.clear();
    ht.htable.resize(env->map.size() * ORIENTATIONS, MAX_TIMESTEP);
    ht.open.clear();

    // 构造一个根节点 root( location=goal_loc, direction=goal_dir, cost=0 )
    HNode root(goal_loc, goal_dir, 0);
    // 在 htable 中记录下 (goal_loc, goal_dir) = 0
    int idx = goal_loc * ORIENTATIONS + goal_dir;
    ht.htable[idx] = 0;

    // 加入到 open
    ht.open.push_back(root);
}

int get_heuristic_rot(HeuristicTableRot& ht,
                      SharedEnvironment* env,
                      int source_loc, int source_dir,
                      Neighbors* ns)
{
    // 若已经有值
    int idx_src = source_loc * ORIENTATIONS + source_dir;
    if (ht.htable[idx_src] < MAX_TIMESTEP) {
        return ht.htable[idx_src];
    }

    // BFS 扩展
    while (!ht.open.empty())
    {
        HNode curr = ht.open.front();
        ht.open.pop_front();

        int curr_idx = curr.location * ORIENTATIONS + curr.direction;
        // BFS 当前 cost
        int curr_cost = ht.htable[curr_idx];

        // 若找到目标
        if (curr.location == source_loc){//&& curr.direction == source_dir
            return curr_cost;
        }

        // 1) 前进
        //   如果朝向是 0(east)，前进到 loc+1； 1(south)-> loc+cols 等
        //   也可通过 global_neighbors 实现通用
        for (int next_loc : ns->at(curr.location)) {
            // 只有当 next_loc 是在当前方向正前方时，才视为“前进”
            // 示例: direction = 0(east)，那么 next_loc应该 == curr.location +1
            // 这里简单做个判断:
            if ( MyPlanner::isForward(curr.location, next_loc, curr.direction, env->cols) ) {
                int nxt_idx = next_loc * ORIENTATIONS + curr.direction;
                int new_cost = curr_cost + 1; // 前进耗费1
                if (new_cost < ht.htable[nxt_idx]){
                    ht.htable[nxt_idx] = new_cost;
                    ht.open.emplace_back(next_loc, curr.direction, new_cost);
                }
            }
        }

        // 2) 左转、右转：不改变 location，但 direction 改变
        {
            int left_dir = (curr.direction + 3) % ORIENTATIONS;  //左转
            int right_dir = (curr.direction + 1) % ORIENTATIONS; //右转
            int cost_turn = curr_cost + 1; // 转向也当做1步

            int idxL = curr.location * ORIENTATIONS + left_dir;
            if (cost_turn < ht.htable[idxL]) {
                ht.htable[idxL] = cost_turn;
                ht.open.emplace_back(curr.location, left_dir, cost_turn);
            }

            int idxR = curr.location * ORIENTATIONS + right_dir;
            if (cost_turn < ht.htable[idxR]) {
                ht.htable[idxR] = cost_turn;
                ht.open.emplace_back(curr.location, right_dir, cost_turn);
            }
        }

        // 3) 等待: 不变 loc, 不变 dir, cost + 1 (可根据需求决定是否需要)
        // {
        //     int wait_idx = curr_idx; // same loc, dir
        //     int cost_wait = curr_cost + 1;
        //     if (cost_wait < ht.htable[wait_idx]) {
        //         ht.htable[wait_idx] = cost_wait;
        //         ht.open.emplace_back(curr.location, curr.direction, cost_wait);
        //     }
        // }
    }

    // 若整个 BFS 都没找到 (source_loc, source_dir)，说明不可达
    return MAX_TIMESTEP;
}

// 判断 next_loc 是否为当前 direction 的正前方
bool isForward(int curr_loc, int next_loc, int direction, int cols)
{
    switch(direction) {
        case 0: // east
            return next_loc == curr_loc + 1;
        case 1: // south
            return next_loc == curr_loc + cols;
        case 2: // west
            return next_loc == curr_loc - 1;
        case 3: // north
            return next_loc == curr_loc - cols;
        default:
            return false;
    }
}

int get_h_rot(SharedEnvironment* env,
              int source_loc, int source_dir,
              int target_loc, int target_dir)
{
    // 如果全局向量还没初始化，就初始化
    if (global_heuristic_rot.empty()) {
        global_heuristic_rot.resize(env->map.size() * ORIENTATIONS); 
        // 并初始化邻接
        init_neighbor(env);
    }

    // 计算全局索引: (target_loc, target_dir)
    int t_idx = target_loc * ORIENTATIONS + target_dir;
    // 若 global_heuristic_rot[t_idx] 还没初始化 (htable 为空)
    if (global_heuristic_rot[t_idx].htable.empty()) {
        init_heuristic_rot(global_heuristic_rot[t_idx], env, target_loc, target_dir);
    }

    // 调用 BFS 来获取 source 的距离
    return get_heuristic_rot(global_heuristic_rot[t_idx], env,
                             source_loc, source_dir, &global_neighbors);
}


}
