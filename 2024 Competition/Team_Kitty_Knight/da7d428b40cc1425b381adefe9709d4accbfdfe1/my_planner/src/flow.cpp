

#include "flow.h"


#include <random>
#include <unordered_set>

namespace MyPlanner{

std::mt19937 g(0);


//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent){
    lns.soc -= lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d, to;

    to = lns.trajs[agent].size();

    for (int j = 1; j < to; j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);


        lns.flow[prev_loc].d[d] -= 1;

    }
}

void add_traj(TrajLNS& lns, int agent){

    //update last replan time for agent
    lns.fw_metrics[agent].last_replan_t = lns.env->curr_timestep;

    lns.soc += lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d;
    for (int j = 1; j < lns.trajs[agent].size(); j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] += 1;

    }
}


void get_deviation(TrajLNS& lns){
    lns.deviation_agents.clear();
    for  (int i=0; i< lns.env->num_of_agents;i++){
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;

        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));

        if (dists.first > 0){
            lns.deviation_agents.emplace_back(dists.first, i);
        }
    }

    std::sort(lns.deviation_agents.begin(), lns.deviation_agents.end(),
        [](std::pair<int,int>& a, std::pair<int,int>& b)
		{
            return a.first > b.first;
        });
    return;
}

void update_fw_metrics(TrajLNS& lns){
    for  (int i=0; i< lns.env->num_of_agents;i++){
        lns.fw_metrics[i].id = i;
        lns.fw_metrics[i].rand = rand();
        lns.fw_metrics[i].deviation = 0;
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;
        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));
        assert(dists.first >= 0 );
        lns.fw_metrics[i].deviation = dists.first;
    }
    return;
}


void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit){
    update_fw_metrics(lns);
    std::vector<FW_Metric> replan_order = lns.fw_metrics;
    assert(replan_order.size() == lns.env->num_of_agents);

    std::sort(replan_order.begin(), replan_order.end(),
    [](FW_Metric& a, FW_Metric& b)
    {
        if (a.deviation > b.deviation)
            return true;
        else if ( a.deviation < b.deviation)
            return false;
        
        if (a.last_replan_t < b.last_replan_t)
            return true;
        else if (a.last_replan_t > b.last_replan_t)
            return false;

        return a.rand > b.rand;
    });

    int count=0;
    int a, index;
    while (std::chrono::steady_clock::now() < timelimit){
        index = count%lns.env->num_of_agents;
        a = replan_order[index].id;
        count++;
        if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
            continue;
        }
        remove_traj(lns,a);
        update_traj(lns,a);
        
    }
    return;

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



void update_dist_2_path(TrajLNS& lns, int i){
    init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
}

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount){

    int count = 0;
    for (int i=0 ; i <lns.env->num_of_agents; i++){
                // std::cout<<i<<";";
        if (count >= amount){
            break;
        }
        // TODO(rivers): what does this if do? I just comment it out...
        // if(!lns.trajs[i].empty() && lns.trajs[i].size() 
        //     == 
        //     // get_heuristic(lns.heuristics[lns.trajs[i].back()], lns.env,lns.trajs[i].front(),&(lns.neighbors))
        //     (int)get_h(lns.env,lns.trajs[i].front(),lns.trajs[i].back())
        // )
        //     continue;
        if(!lns.trajs[i].empty() && lns.traj_dists[i].empty()){
            init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
            count++;
            lns.dist2path_inited++;
        }

    }
}

//update traj and distance table for agent i
void update_traj(TrajLNS& lns, int i){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];
    lns.goal_nodes[i] = astar(lns.env,lns.flow,lns.trajs[i],lns.mem,start,goal, &(lns.neighbors));
    add_traj(lns,i);
    update_dist_2_path(lns,i);
}

}
