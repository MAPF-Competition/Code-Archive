
#include "heuristics.h"
#include <queue>

namespace DefaultPlanner{

std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;


/**
* @brief: 根据地图（env->map）的结构初始化全局邻居信息表（global_neighbors），为每个节点记录其可到达的邻居节点。
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return void
 * comment: 这里的可到达没有考虑动态障碍物(other agent)
 */
void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){ // map[loc] == 0: 当前位置可通行。map[loc] == 1: 当前位置为障碍物，不可通行。
                // 为当前节点 loc 添加其上方邻居节点。
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
                // 下方邻居
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
                // 左侧邻居
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
                // 右侧邻居
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

/**
* @brief: 初始化每个地图点的邻居
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return void
 */
void init_heuristics(SharedEnvironment* env){
    // 只有在启发式表未初始化时才需要执行初始化逻辑，避免重复计算。
	if (global_heuristictable.size()==0){
        // 将 global_heuristictable 的大小调整为地图中节点的数量（env->map.size()）。
		global_heuristictable.resize(env->map.size());
        // 调用 init_neighbor 函数，为地图中的每个节点计算其邻居节点。
		init_neighbor(env);
	}

}

/**
* @brief: 初始化地图中每个点和目标点的启发式距离
 * @param HeuristicTable& ht: 地图所有点到所有点的启发式距离
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param int goal_location: 目标点。
 * @return void
 */
void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear(); // 地图中每个点和目标点的启发式距离
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	ht.open.clear();
	// generate an open that can save nodes (and a open_handle)
    // root居然是goal location
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}

/**
* @brief: 通过启发式搜索计算从给定源位置 (source) 到目标位置的启发式代价（heuristic）
* @param HeuristicTable& ht: 存储地图两点间的启发式距离。实际代入的参数是global_heuristictable, 因此一对位置只计算一次。
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param int source: 源位置（source），即从这个位置开始计算启发式值。
 * @param Neighbors* ns: 提供某个位置的邻居信息（即可以直接到达的相邻位置）。实际上是只考虑静态障碍物的global_neighbors.
 * @return curr.value: 给定源位置 (source) 到目标位置的启发式代价（heuristic）
 */
int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
    // 如果 htable[source] 的值已经被计算过（小于 MAX_TIMESTEP，表示有效值），直接返回，不需要重新计算。
		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

        // 2. 初始化邻居节点和扩展队列
		std::vector<int> neighbors;
		int cost, diff;
        // 3. 开始扩展搜索
		while (!ht.open.empty())
		{
            // 取出队列中的第一个节点 curr，进行处理。
			HNode curr = ht.open.front();
			ht.open.pop_front();

            // 5. 获取邻居节点
			getNeighborLocs(ns,neighbors,curr.location);
            // 6. 遍历所有邻居节点
			for (int next : neighbors)
			{
                // 7. 计算邻居节点的代价
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

                // 8. 检查邻居节点是否需要更新
				if (cost >= ht.htable[next] )
					continue;

                // 9. 更新邻居节点的代价
				ht.open.emplace_back(next,0, cost);
				ht.htable[next] = cost;
			}

            // 10. 检查是否到达源位置
			if (source == curr.location)
				return curr.value;
		}

        // 11. 返回默认值
		return MAX_TIMESTEP;
}

/**
* @brief: hash中有就读取, 没有就计算
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param int source: 源位置（source），即从这个位置开始计算启发式值。
 * @param int target: 目标位置位置（target），即从这个位置开始计算启发式值。
 * @return int distance: 给定源位置 (source) 到目标位置的启发式代价（heuristic）
 */
int get_h(SharedEnvironment* env, int source, int target){
	if (global_heuristictable.empty()){
        // 调用 init_neighbor 函数，为地图中的每个节点计算其邻居节点。
		init_heuristics(env);
	}

	if (global_heuristictable.at(target).empty()){
        // 把目标节点作为根节点, 为计算地图中每个点到目标点的位置做准备
		init_heuristic(global_heuristictable.at(target),env,target);
	}

    // source到dest的启发式距离, 只考虑静态障碍物
	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
}


/**
* @brief: 初始化地图中每个点到路径末尾的距离, 计算对象路径中每个点到路径末尾的距离。
 * @param Dist2Path& dp: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param Traj& path: 对象路径。
 * @return void
 */
void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	if (dp.dist2path.empty())
		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
	dp.open.clear();
	dp.label++; // 每次初始化后，将 label 自增，这样就能标记不同阶段的路径更新，避免重复标记或更新旧数据。

    int togo = 0;
    // 从路径末尾遍历到路径开端
    for(int i = path.size()-1; i>=0; i--){
        auto p = path[i];
		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
		dp.open.emplace_back(dp.label,p,0,togo);
		dp.dist2path[p] = {dp.label,p,0,togo};
		togo++;
    }
}

/**
* @brief: 从某个起始点（source）找到该点到预定义路径终点的距离信息
 * @param Dist2Path& dp: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param int source: 源位置（source），即从这个位置开始计算启发式值。
 * @param Neighbors* ns: 提供获取当前点的相邻点（邻居）的功能。
 * @return std::pair<int, int>: 距离代价（cost）, 剩余步数（togo）。
 */
std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
    // 检查 source 点是否已经被当前路径计算版本（dp.label）标记
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	std::vector<int> neighbors;
	int cost;

    // BFS 计算距离
	while (!dp.open.empty())
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();

        // 4. 获取当前点的邻居点
		getNeighborLocs(ns,neighbors,curr.id);

        // 5. 更新邻居点的距离信息
		for (int next_location : neighbors)
		{
			cost = curr.cost + 1;

            // 如果邻居点的 label 是当前路径版本（dp.label），并且新计算的 cost 不优于已有的值，则跳过。
			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
				continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
		}

        // 6. 检查是否找到起始点
		if (source == curr.id){
			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
			return std::make_pair(curr.cost, curr.togo);
		}
	}

	return std::make_pair(MAX_TIMESTEP,0);
}

/**
* @brief: 计算地图上某个点 source 到目标路径终点的 总距离 = 到路径最近点的代价 (cost) + 从路径最近点到路径终点的剩余步数 (togo)。
 * @param Dist2Path& dp: 路径距离信息的结构体，包含了所有地图点到路径的距离数据。
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @param int source: 源位置（source），即从这个位置开始计算启发式值。
 * @param Neighbors* ns: 提供获取当前点的相邻点（邻居）的功能。
 * @return int: 点 source 到目标路径终点的 总距离
 * comment: 有点像highway模型了, 不知道能不能用起来
 */
int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second; // source到路径最近点的距离 + 从最近点到路径终点的剩余距离。
}



}
