
#include "heuristics.h"
#include <queue>
#include <Logger.h>
namespace MyPlanner{

std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;
Logger* logger = nullptr;


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
		//computeGlobalHeuristics(env);
		computeAllHeuristics(env);
	}

}
void computeGlobalHeuristics(SharedEnvironment* env) {
	
    for (int target = 0; target < env->map.size(); ++target) {
        if (env->map[target] == 0) { // 检查目标位置是否可通行
            init_heuristic(global_heuristictable[target], env, target);
            // 计算启发式表
			cout<<"calculate "<<target<<endl;
			auto start = std::chrono::high_resolution_clock::now(); // 开始计时
            global_heuristictable[target] = get_heuristic_table(env, target, &global_neighbors);
			auto end = std::chrono::high_resolution_clock::now(); // 结束计时
			std::chrono::duration<double, std::milli> duration = end - start; // 计算时间间隔
			logger->log_info(" calculate "+ std::to_string(target) +" heuristics table time usage: " + std::to_string(duration.count()) + "ms", env->curr_timestep);
        }
    }
}
void computeAllHeuristics(SharedEnvironment* env) {
	for (int target = 0; target < env->map.size(); ++target) {
        if (env->map[target] == 0) { // 检查目标位置是否可通行
			init_heuristic(global_heuristictable[target], env, target);
			cout<<"calculate "<<target<<endl;
		}
	}
	
    for (int target = 0; target < env->map.size(); ++target) {
        if (env->map[target] == 0) { // 检查目标位置是否可通行
			//init_heuristic(global_heuristictable[target], env, target);
			cout<<"calculate "<<target<<endl;
			auto start = std::chrono::high_resolution_clock::now(); // 开始计时
            for (int source = 0; source < env->map.size(); ++source) {
				if (env->map[source] == 0) { // 检查源位置是否可通行
				    if (source != target) { // 确保源位置和目标位置不同且可通行
                    // 检查是否已经计算过对称位置的启发式值
                    	// if (global_heuristictable[source].htable[target] != MAX_TIMESTEP) {
                        // 	global_heuristictable[target].htable[source] = global_heuristictable[source].htable[target];
                    	// } else {
                        // 	int heuristicValue = get_heuristic(global_heuristictable[target], env, source, &global_neighbors);
                        // 	global_heuristictable[target].htable[source] = heuristicValue;
                        // 	//global_heuristictable[source].htable[target] = heuristicValue; // 复用启发式值
                    	// }
						int heuristicValue = get_heuristic(global_heuristictable[target], env, source, &global_neighbors);
                        global_heuristictable[target].htable[source] = heuristicValue;
                	}
				}  
            }
			auto end = std::chrono::high_resolution_clock::now(); // 结束计时
			std::chrono::duration<double, std::milli> duration = end - start; // 计算时间间隔
			logger->log_info(" calculate "+ std::to_string(target) +" heuristics table time usage: " + std::to_string(duration.count()) + "ms", env->curr_timestep);
        }
    }
}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	//ht.open.clear();
	ht.open = std::priority_queue<HNode, std::vector<HNode>, CompareHNode>();

	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	//ht.open.push_back(root);  // add root to open
	ht.open.push(root);  // add root to open

}

HeuristicTable get_heuristic_table(SharedEnvironment* env, int source, Neighbors* ns){
    HeuristicTable ht;
    ht.htable.resize(env->map.size(), MAX_TIMESTEP);
    //ht.open.clear();
	ht.open = std::priority_queue<HNode, std::vector<HNode>, CompareHNode>();

    HNode root(source, -1, 0);//设置source点方向为-1
    ht.htable[source] = 0;
    //ht.open.push_back(root);  // add root to open
	ht.open.push(root);
    std::vector<int> neighbors;
    int cost, diff;
    while (!ht.open.empty()){
        //HNode curr = ht.open.front();
		HNode curr = ht.open.top();

        //ht.open.pop_front();
		ht.open.pop();


        getNeighborLocs(ns, neighbors, curr.location);

        for (int next : neighbors){
            if (next == -1){
                continue;
            }
			if(next < source && global_heuristictable[next].htable[source] != MAX_TIMESTEP){
				ht.htable[next] = global_heuristictable[next].htable[source];
				continue;
			}
            diff = next - curr.location;
            if (curr.direction == 0 && diff == -env->cols) // 向上，正前方
                cost = curr.value + 1;
            else if (curr.direction == 1 && diff == 1) // 向右，正前方
                cost = curr.value + 1;
            else if (curr.direction == 2 && diff == env->cols) // 向下，正前方
                cost = curr.value + 1;
            else if (curr.direction == 3 && diff == -1) // 向左，正前方
                cost = curr.value + 1;
            else if ((curr.direction == 0 && (diff == 1 || diff == -1)) || // 向上，上下或左右
                     (curr.direction == 1 && (diff == -env->cols || diff == env->cols)) ||
                     (curr.direction == 2 && (diff == 1 || diff == -1)) ||
                     (curr.direction == 3 && (diff == -env->cols || diff == env->cols)))
                cost = curr.value + 2;
            else // 方向相反
                cost = curr.value + 3;

			if (curr.location == source && curr.direction == -1) {
                cost += 1;
            }
            assert(next >= 0 && next < env->map.size());

            if (cost >= ht.htable[next] )
                continue;

            int next_direction; // 计算下一个节点的方向
            if (diff == -env->cols) // 向上
                next_direction = 0;
            else if (diff == 1) // 向右
                next_direction = 1;
            else if (diff == env->cols) // 向下
                next_direction = 2;
            else if (diff == -1) // 向左
                next_direction = 3;
            else
                assert(false); // 应该不会到达这里
            //ht.open.emplace_back(next, next_direction, cost);
			ht.open.emplace(next, next_direction, cost);

            ht.htable[next] = cost;
        }
    }

    return ht;
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

		std::vector<int> neighbors;
		int cost, diff;
		while (!ht.open.empty())
		{
			//HNode curr = ht.open.front();
			//ht.open.pop_front();
			HNode curr = ht.open.top();
			ht.open.pop();

			
			getNeighborLocs(ns,neighbors,curr.location);

			
			for (int next : neighbors)
			{
				if (next == -1){
                	continue;
            	}
				//cost = curr.value + 1;
				//diff = curr.location - next;
				diff = next-curr.location;
				//cout<<diff<<" "<<curr.direction<<endl;
				// 根据当前方向和diff计算成本
            	if (curr.direction == 0 && diff == -env->cols) // 向上，正前方
                	cost = curr.value + 1;
            	else if (curr.direction == 1 && diff == 1) // 向右，正前方
                	cost = curr.value + 1;
            	else if (curr.direction == 2 && diff == env->cols) // 向下，正前方
                	cost = curr.value + 1;
            	else if (curr.direction == 3 && diff == -1) // 向左，正前方
                	cost = curr.value + 1;
            	else if ((curr.direction == 0 && (diff == 1 || diff == -1)) || // 向上，上下或左右
                     (curr.direction == 1 && (diff == -env->cols || diff == env->cols)) ||
                     (curr.direction == 2 && (diff == 1 || diff == -1)) ||
                     (curr.direction == 3 && (diff == -env->cols || diff == env->cols)))
                	cost = curr.value + 2;
            	else // 方向相反
                	cost = curr.value + 3;

				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				if (cost >= ht.htable[next] )
					continue;

				int next_direction; // 计算下一个节点的方向
            	if (diff == -env->cols) // 向上
                	next_direction = 0;
            	else if (diff == 1) // 向右
                	next_direction = 1;
            	else if (diff == env->cols) // 向下
                	next_direction = 2;
            	else if (diff == -1) // 向左
                	next_direction = 3;
            	else
                	assert(false); // 应该不会到达这里
				//ht.open.emplace_back(next,next_direction, cost);
				ht.open.emplace(next, next_direction, cost);

				ht.htable[next] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
				//cout<<"value"<<curr.value<<endl;
		}


		return MAX_TIMESTEP;
}

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

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())
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

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}



}
