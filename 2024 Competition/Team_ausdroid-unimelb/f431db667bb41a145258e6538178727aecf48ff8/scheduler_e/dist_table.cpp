
#include "heuristics.h"
#include <queue>

namespace SchedulerE{

std::vector<HeuristicTable> dt_global_heuristictable;
std::unordered_map<int, int> valid_locs;
Neighbors dt_global_neighbors;



void dt_init_neighbor(SharedEnvironment* env){
	dt_global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					dt_global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					dt_global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					dt_global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					dt_global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void dt_init_heuristics(SharedEnvironment* env){
	if (dt_global_heuristictable.size()==0){
		
        int count = 0;
        for (int i = 0; i < env->map.size(); i++) {
            if (env->map[i] == 0) valid_locs[i] = count++;
        }

        dt_global_heuristictable.resize(valid_locs.size());

        // for (int i = 0; i < valid_locs.size(); i++) {
        //     dt_global_heuristictable[i] = HeuristicTable_D();
        //     // dt_global_heuristictable[loc].htable = unordered_map<int, int>();
        // }



		dt_init_neighbor(env);


        
	}

}

void dt_init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(valid_locs.size(),MAX_TIMESTEP);
    // for (int inner_loc : valid_locs) {
    //             ht.htable[inner_loc] = MAX_TIMESTEP;
    // }
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location,0, 0);
	ht.htable[valid_locs[goal_location]] = 0;
	ht.open.push_back(root);  // add root to open
}


int dt_get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		if (ht.htable[valid_locs[source]] < MAX_TIMESTEP) return ht.htable[valid_locs[source]];

		std::vector<int> neighbors;
		int cost, diff;
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

				if (cost >= ht.htable[valid_locs[next]] )
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable[valid_locs[next]] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}

int dt_get_h(SharedEnvironment* env, int source, int target){
	if (dt_global_heuristictable.empty()){
		dt_init_heuristics(env);
	}

	if (dt_global_heuristictable[valid_locs[target]].empty()){
		dt_init_heuristic(dt_global_heuristictable[valid_locs[target]],env,target);
	}

	return dt_get_heuristic(dt_global_heuristictable[valid_locs[target]], env, source, &dt_global_neighbors);
}

int dt_get_hh(SharedEnvironment* env, int source, int target) {
	return dt_global_heuristictable[valid_locs[target]].htable[valid_locs[source]];
}



void dt_init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
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

std::pair<int,int> dt_get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
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

int dt_get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = dt_get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}



}
