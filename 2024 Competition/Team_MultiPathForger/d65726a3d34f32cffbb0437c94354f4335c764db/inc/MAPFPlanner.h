#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "ECBS.h"
#include <lacam.hpp>
#include "ADG.h"

#include <mutex>
#include <queue>
#include <thread>
class MAPFPlanner
{
public:
    SharedEnvironment* env;
	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner() {delete env;};
	void getActions(vector<Action> & actions, int num_agents);

    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);
	// void CBSplan(int limit, std::vector<Action> & actions);
	void LACAMplan(int limit);
	void ECBSplan(int limit, std::shared_ptr<std::vector<std::shared_ptr<ADG_namespace::Node>>> commit_cut = nullptr);
	// void fillInstance()
	Action determineAction(std::pair<int,int> curr, std::pair<int,int> next, int curr_orientation);
	int determineOrientation(int y, int x);
	std::pair<int, int> lineariseCoords(int coord) const {
		return {coord / env->cols, coord % env->cols};
	}
	void getActions(vector<Action> & actions, std::shared_ptr<ADG_namespace::ADG> adg);
	bool validateAction(Action action, std::shared_ptr<ADG_namespace::Node> node, int agent_id);
	bool check = false;

	int CBS_TIMESTEP = 0;
	// CBS cbs;
	int instance_num_agents = 0;
	// std::shared_ptr<ADG_namespace::ADG> adg = nullptr;
	std::shared_ptr<std::vector<std::shared_ptr<ADG_namespace::Node>>> commit_cut_ptr = nullptr;
	vector<std::list<std::pair<int, int>>::iterator> agent_timesteps;
	vector<list<std::pair<int, int>>> paths;
	vector<std::pair<int, Action>> past_locs;
	std::unordered_map<int,int> vertex_occupied;
	std::mutex queue_mutex;
	std::unique_ptr<std::thread> planner_thread;
	std::queue<std::shared_ptr<ADG_namespace::ADG>> adg_queue;
	std::shared_ptr<LACAM_PREFIX_Instance> ins;
};