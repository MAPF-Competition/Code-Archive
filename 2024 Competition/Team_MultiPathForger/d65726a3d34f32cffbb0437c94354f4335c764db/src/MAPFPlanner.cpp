#include <random>
#include <Entry.h>

//default planner includes
// #include "planner.h"
#include "const.h"
#include <unordered_map>
#include <boost/log/trivial.hpp>
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    ///////////////////////////////////////////////////////////////////////////
	/// load the instance
    //////////////////////////////////////////////////////////////////////

    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    // DefaultPlanner::initialize(limit, env);
    return;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
	// DefaultPlanner::plan(limit, actions, env);
		// if (planner_thread && planner_thread->joinable()) {
		// 	planner_thread->join();
		// 	planner_thread.reset();
		// }
	// if (!check) {
	// 	MAPFPlanner::LACAMplan(limit);
	// 	// check = true;
	// }
 	if (adg_queue.empty()) {
        // Solve with ECBS and create a new ADG
        // MAPFPlanner::ECBSplan(limit, nullptr);
		MAPFPlanner::LACAMplan(limit);
    } else {
        std::shared_ptr<ADG_namespace::ADG> adg = adg_queue.front();
        if (adg->is_finished) {
            // Pop the finished ADG and run getActions with the new ADG
			// std::lock_guard<std::mutex> lock(queue_mutex);
            adg_queue.pop();
            if (!adg_queue.empty()) {
				if (commit_cut_ptr) commit_cut_ptr.reset();
                adg = adg_queue.front();
                getActions(actions, adg);
            }
        } else {
            // Keep executing the current ADG
            getActions(actions, adg);
        }
    }
    return;
}


std::optional<int> getSecondLocation(const SharedEnvironment* env, int agent_id) {
    int task_id = env->curr_task_schedule[agent_id];
    if (task_id != -1 && env->task_pool.find(task_id) != env->task_pool.end()) {
        const Task& task = env->task_pool.at(task_id);
        if (task.locations.size() > 1) {
            return task.locations[task.idx_next_loc+1];
        }
    }
    return std::nullopt;
}
void MAPFPlanner::ECBSplan(int limit,std::shared_ptr<std::vector<std::shared_ptr<ADG_namespace::Node>>> commit_cut) {

		
		high_level_solver_type s = high_level_solver_type::EES;

		EECBS_PREFIX_heuristics_type h = EECBS_PREFIX_heuristics_type::WDG;


		EECBS_PREFIX_heuristics_type h_hat = EECBS_PREFIX_heuristics_type::GLOBAL; // inadmissible heuristics


		conflict_selection conflict = conflict_selection::EARLIEST;
		node_selection n = node_selection::NODE_CONFLICTPAIRS;


		///////////////////////////////////////////////////////////////////////////
		EECBS_PREFIX_Instance instance;

		instance.num_of_cols = env->cols;
		instance.num_of_rows = env->rows;
		instance.map_size = instance.num_of_cols*instance.num_of_rows;
		instance.my_map = env->map;

		instance.num_of_agents = 0;
		std::unordered_map<int, int> goal_location_exists;


		if (!commit_cut) {
			for (int i = 0; i < env->num_of_agents; i++) {
				instance.num_of_agents++;
				instance.start_locations.push_back(env->curr_states.at(i).location);
				if (env->goal_locations[i].size() == 0 || goal_location_exists.count(env->goal_locations[i].front().first) > 0) {
					instance.goal_locations.push_back(env->curr_states.at(i).location);
					continue;
				}
				instance.goal_locations.push_back(env->goal_locations[i].front().first);
				goal_location_exists[env->goal_locations[i].front().first] = 1;
				
			}
		} else {
			for (int i = 0; i < env->num_of_agents; i++) {
				instance.num_of_agents++;
				int start = (*adg_queue.front())[i].back()->goal.first * env->cols + (*adg_queue.front())[i].back()->goal.second; 
				if (env->goal_locations[i].size() == 0 || goal_location_exists.count(env->goal_locations[i].front().first) > 0) {
					instance.goal_locations.push_back(env->curr_states.at(i).location);
					continue;
				}
				int goal = env->goal_locations[i].front().first;
				if ((*commit_cut)[i])  {
					start = (*commit_cut)[i]->start.first * env->cols + (*commit_cut)[i]->start.second;
					goal = *getSecondLocation(env, i);
				}
				instance.start_locations.push_back(start);
				instance.goal_locations.push_back(goal);
				goal_location_exists[goal] = 1;

			}
		}
		instance_num_agents = instance.num_of_agents;
		int restart = 0;
		int runs = 1 + restart;
		//////////////////////////////////////////////////////////////////////
		// initialize the solver

		ECBS ecbs(instance, true, 2);
		ecbs.setPrioritizeConflicts(true);
		ecbs.setDisjointSplitting(true);
		ecbs.setBypass(true);
		ecbs.setRectangleReasoning(true);
		ecbs.setCorridorReasoning(true);
		ecbs.setHeuristicType(h, h_hat);
		ecbs.setTargetReasoning(true);
		ecbs.setMutexReasoning(false);
		ecbs.setConflictSelectionRule(conflict);
		ecbs.setNodeSelectionRule(n);
		ecbs.setSavingStats(false);
		ecbs.setHighLevelSolver(s, 1.2);
		//////////////////////////////////////////////////////////////////////
		// run
		double runtime = 0;
		int lowerbound = 0;
		int cutoffTime = 60;

		try
{		for (int i = 0; i < runs; i++)
		{
			ecbs.clear();
			ecbs.solve(cutoffTime / runs, lowerbound);
			runtime += ecbs.runtime;
			if (ecbs.solution_found) {
			paths = std::vector< std::list<std::pair<int,int>>>(instance.num_of_agents);
			agent_timesteps = std::vector<std::list<std::pair<int, int>>::iterator>(instance.num_of_agents);
			for (int i = 0; i < instance.num_of_agents; i++) {
				for (int j = 0; j < ecbs.paths[i]->size(); j++) {
					auto t = ecbs.paths[i]->at(j);
					// std::cout << "getting actions" << std::endl;


					paths[i].emplace_back(
					t.location / env->cols,t.location - ((t.location / env->cols)*env->cols)
					);
					// std::cout << "path " << t.location / env->rows << t.location - ((t.location / env->rows)*env->rows) << std::endl;
				}
				agent_timesteps[i] = paths[i].begin();
			}				
				break;
			}
			lowerbound = ecbs.getLowerBound();
			ecbs.randomRoot = true;
			cout << "Failed to find solutions in Run " << i << endl;
		}
		ecbs.runtime = runtime;

		if (ecbs.solution_found) {
			ecbs.savePaths("paths.txt");
		}
			

		ecbs.clearSearchEngines();
		auto new_adg = std::make_shared<ADG_namespace::ADG>(paths);
		std::lock_guard<std::mutex> lock(queue_mutex);
		adg_queue.push(new_adg);
		new_adg->print();


		check = true;


		}
			catch(const std::exception& e){
			std::cout<<"this is causing issues" << std::endl;
		}


}

void MAPFPlanner::LACAMplan(int limit) {
	const std::string map_name = "default_map.txt";      
	const std::string scen_name = "";                    
	const std::string num_str = "1";                 
	const std::string seed_str = "0";                    
	const std::string verbose_str = "0";                   
	const std::string time_limit_sec_str = "3";            
	const std::string output_name = "./build/result.txt";  
	const bool log_short = false; 

	// solver parameters defaults
	const bool flg_no_all = false;
	const bool flg_no_star = false;
	const std::string random_insert_prob1_str = "0.001";
	const std::string random_insert_prob2_str = "0.01";
	const bool random_insert_init_node = false;
	const bool flg_no_swap = false;
	const bool flg_no_multi_thread = false;
	const std::string pibt_num_str = "10";
	const bool flg_no_scatter = false;
	const std::string scatter_margin_str = "10";
	const bool flg_no_refiner = false;
	const std::string refiner_num_str = "4";
	const std::string recursive_rate_str = "0.2";
	const std::string recursive_time_limit_str = "1";
	const std::string checkpoints_duration_str = "5";

	// Convert string parameters to numeric types
	const auto verbose = std::stoi(verbose_str);
	const auto time_limit_sec = std::stoi(time_limit_sec_str);
	const auto seed = std::stoi(seed_str);
	const auto N = env->num_of_agents;


	// Set solver parameters based on defaults
	LACAM_PREFIX_Planner::FLG_SWAP = !flg_no_swap && !flg_no_all;
	LACAM_PREFIX_Planner::FLG_STAR = !flg_no_star && !flg_no_all;
	LACAM_PREFIX_Planner::FLG_MULTI_THREAD = !flg_no_multi_thread && !flg_no_all;
	LACAM_PREFIX_Planner::PIBT_NUM = flg_no_all ? 1 : std::stoi(pibt_num_str);
	LACAM_PREFIX_Planner::FLG_REFINER = !flg_no_refiner && !flg_no_all;
	LACAM_PREFIX_Planner::REFINER_NUM = std::stoi(refiner_num_str);
	LACAM_PREFIX_Planner::FLG_SCATTER = !flg_no_scatter && !flg_no_all;
	LACAM_PREFIX_Planner::SCATTER_MARGIN = std::stoi(scatter_margin_str);
	LACAM_PREFIX_Planner::RANDOM_INSERT_PROB1 = flg_no_all ? 0 : std::stof(random_insert_prob1_str);
	LACAM_PREFIX_Planner::RANDOM_INSERT_PROB2 = flg_no_all ? 0 : std::stof(random_insert_prob2_str);
	LACAM_PREFIX_Planner::FLG_RANDOM_INSERT_INIT_NODE = random_insert_init_node && !flg_no_all;
	LACAM_PREFIX_Planner::RECURSIVE_RATE = flg_no_all ? 0 : std::stof(recursive_rate_str);
	LACAM_PREFIX_Planner::RECURSIVE_TIME_LIMIT = flg_no_all ? 0 : std::stof(recursive_time_limit_str) * 1000;
	LACAM_PREFIX_Planner::CHECKPOINTS_DURATION = std::stof(checkpoints_duration_str) * 1000;

	// Solve the problem

	if  (!ins)
		ins = std::make_shared<LACAM_PREFIX_Instance>(env->map, env->cols, env->rows, N);
	// const auto V_size = 
	ins->starts.clear();
	ins->goals.clear();
	for (int i = 0; i < env->num_of_agents; i++) {

		auto start = std::find_if(ins->G->V.begin(), ins->G->V.end(),
        [&](const LACAM_PREFIX_Vertex* n) {
            return n->index == env->curr_states.at(i).location;
        });
		if (start == ins->G->V.end()) {
			std::cerr << "Error: Start vertex not found for agent " << i << std::endl;
			std::cerr << "Start Location: " << env->curr_states.at(i).location << std::endl;
			continue; // Skip this agent
		}
		ins->starts.push_back(*start);

		if (env->goal_locations[i].size() == 0) {
			ins->goals.push_back(*start);
			continue;
		}
		auto goal = std::find_if(ins->G->V.begin(), ins->G->V.end(),
        [&](const LACAM_PREFIX_Vertex* n) {
            return n->index == env->goal_locations.at(i).front().first;
        });;
		    if (goal == ins->G->V.end()) {
        std::cerr << "Error: Goal vertex not found for agent " << i << std::endl;
		std::cerr << "Goal Location: " << env->goal_locations.at(i).front().first << std::endl;
        continue; // Skip this agent
    }
		
		ins->goals.push_back(*goal);
	}
	if (!ins->is_valid(1)) return;

	// Solve the problem
	const auto deadline = Deadline(time_limit_sec * 1000);
	const auto solution = solve(*ins, verbose - 1, &deadline, seed);
	const auto comp_time_ms = deadline.elapsed_ms();




	// failure
	if (solution.empty()) {
		check = false;
		info(1, verbose, "failed to solve");
	} else {
		check = true;
	}
		

	// check feasibility
	if (!is_feasible_solution(*ins, solution, verbose)) {
	info(0, verbose, "invalid solution");
	}

	// Post processing: print stats and write log
	print_stats(verbose, &deadline, *ins, solution, comp_time_ms);
	make_log(*ins, solution, output_name, comp_time_ms, map_name, seed, log_short);
	paths = std::vector< std::list<std::pair<int,int>>>(N);
	for (const auto& s : solution) {
		for (int i = 0; i < s.size(); i++) {
			auto t = s[i];
			paths[i].emplace_back(
				t->index / env->cols, t->index % env->cols
			);
		}
	}
	auto new_adg = std::make_shared<ADG_namespace::ADG>(paths);
	// std::lock_guard<std::mutex> lock(queue_mutex);
	if (check)
		adg_queue.push(new_adg);
	// new_adg->print();
}

void MAPFPlanner::getActions(vector<Action>& actions, std::shared_ptr<ADG_namespace::ADG> adg)
{
	auto get_x = [&](int k) { return k % env->cols; };
  	auto get_y = [&](int k) { return k / env->cols; };
    actions = vector<Action>(env->curr_states.size(), Action::NA);
	std::cout << "num of agents:" << env->num_of_agents<<std::endl;
	int num_finished = 0;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (adg->past_locs[i] >= (*adg)[i].size()) {
			actions[i] = Action::W;
			num_finished++;
            continue;
        }
		if (lineariseCoords(env->curr_states[i].location) != (*adg)[i][adg->past_locs[i]]->start) {
			std::cout << "Agent " << i << " real location doesn't match" << std::endl;
			std::cout << "Agent " << i << " real location" << lineariseCoords(env->curr_states[i].location).first << "," << lineariseCoords(env->curr_states[i].location).second << std::endl;
			std::cout << "Agent " << i << " adg location " << (*adg)[i][adg->past_locs[i]]->start.first << "," << (*adg)[i][adg->past_locs[i]]->start.second << std::endl;
			adg->print();
			adg->past_locs[i]--;
			(*adg)[i][adg->past_locs[i]]->status = ADG_namespace::node_status::STAGED;
		}

        auto& plan = (*adg)[i];
        std::shared_ptr<ADG_namespace::Node> node = plan[adg->past_locs[i]];
        Action action = determineAction(node->start, node->goal, env->curr_states[i].orientation);

        bool waiting_for_dependencies = false;
        for (auto dependency : node->incoming_edges) {

            if (dependency->id != i && dependency->status != ADG_namespace::node_status::COMPLETED) {
                    action = Action::W;
                    waiting_for_dependencies = true;
                	break;
            }
        }

        //all dependencies are non-conflicting(non ongoing), and we can stay at the next location(which is current one)
        if (!waiting_for_dependencies) {
			node->status = ADG_namespace::node_status::ONGOING;
			if (validateAction(action, node, i)) {
			node->status = ADG_namespace::node_status::COMPLETED;
			}
        } 


        if (node->status == ADG_namespace::node_status::COMPLETED) {
            adg->past_locs[i]++;
			if ((*adg)[i].size() - adg->past_locs[i] == 3 && !commit_cut_ptr) {
				std::cout << "Happened at " << node->id << " start: " << node->start.first << "," << node->start.second << std::endl;

				// commit_cut_ptr = std::make_shared<std::vector<std::shared_ptr<ADG_namespace::Node>>>(adg->computeCommitCut());


				// planner_thread = std::make_unique<std::thread>(&MAPFPlanner::ECBSplan, this, 60, commit_cut_ptr);
			}
        }


        actions[i] = action;
    }

	if (num_finished == env->num_of_agents) {
		adg->is_finished = true;
		// adg_queue.pop();
	}
}
bool MAPFPlanner::validateAction(Action action, std::shared_ptr<ADG_namespace::Node> node, int agent_id) {
	const auto& t = env->curr_states[agent_id]; 
	std::pair<int,int> real_loc = std::make_pair(t.location / env->cols,t.location - ((t.location / env->cols)*env->cols));
	if (real_loc != node->start) {
		// std::cout << "Agent " << agent_id << " real location" << real_loc.first << real_loc.second << " " << node->start.first << node->start.second << std::endl;
		// std::cout << "Agent " << agent_id << " real location doesn't match" << std::endl;
		return false;
	}

	if (node->start == node->goal && action == Action::W) return true;

	int orientation = determineOrientation(node->goal.first -  real_loc.first, node->goal.second -  real_loc.second);

	if (node->start != node->goal && action == Action::FW && t.orientation == orientation) return true;

	return false;
}
int MAPFPlanner::determineOrientation(int ydiff, int xdiff) {
	int orientation = 0;
	if (xdiff == 1 && ydiff == 0)
	{
		orientation = 0;
	}
	else if (xdiff == 0 && ydiff == 1)
	{
		orientation = 1;
	}
	else if (xdiff == -1 && ydiff == 0)
	{
		orientation = 2;
	}
	else if (xdiff == 0 && ydiff == -1)
	{
		orientation = 3;
	}
	return orientation;
}
Action MAPFPlanner::determineAction(std::pair<int,int> curr, std::pair<int,int> next, int curr_orientation) {
	Action action = Action::W;

	if (curr == next) return action;

	int ay = curr.first;
	int ax = curr.second;

	int by = next.first;
	int bx = next.second;

	// BOOST_LOG_TRIVIAL(debug) << "Agent " << i << " current location: (" << ax << ", " << ay << "), "
	//                             << env->map[env->curr_states.at(i).location];
	// BOOST_LOG_TRIVIAL(debug) << "Agent " << i << " next location: (" << bx << ", " << by << "), "
	//                             << env->map[by * 32 + bx];

	int xdiff = bx - ax;
	int ydiff = by - ay;
	int orientation = 5; // 0:east, 1:south, 2:west, 3:north
	// BOOST_LOG_TRIVIAL(debug) << "xdiff = " << xdiff;
	// BOOST_LOG_TRIVIAL(debug) << "Agent orientation: " << env->curr_states[i].orientation;

	 orientation = determineOrientation(ydiff, xdiff);
	int orientationDiff = orientation - curr_orientation;
	// int target_location = by * env->rows + bx;
	// BOOST_LOG_TRIVIAL(debug) << "Orientation difference: " << orientationDiff;

	if (orientationDiff == 0)
	{
		// BOOST_LOG_TRIVIAL(debug) << "Action: Move Forward";
		action = Action::FW;
	}
	else if (orientationDiff == 1 || orientationDiff == -3)
	{
		// BOOST_LOG_TRIVIAL(debug) << "Action: Rotate Clockwise";
		action = Action::CR; // Clockwise rotation
	}
	else if (orientationDiff == -1 || orientationDiff == 3)
	{
		// BOOST_LOG_TRIVIAL(debug) << "Action: Rotate Counter-Clockwise";
		action = Action::CCR; // Counter-clockwise rotation
	}
	else
	{
		// BOOST_LOG_TRIVIAL(debug) << "Action: Rotate Counter-Clockwise Twice";
		action = Action::CCR; // Counter-clockwise rotation
	}


	return action;
}