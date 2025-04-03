#ifndef SCHEDULER
#define SCHEDULER

#include "SharedEnv.h"
#include "util/HeuristicTable.h"
#include <random>
#include <unordered_set>
#include "planner.h"
#include <set>
#include <unordered_map>

namespace MyPlanner {

class MyScheduler {
public:
    
    std::mt19937 mt;
    std::set<int> free_agents;
    std::set<int> free_tasks;
    std::shared_ptr<MyPlanner::WPPLPlanner> wppl_planner;
    std::string scheduler_name;

    std::unordered_map<int, int> endpoints_loc_to_id_mapping;
    std::vector<int> endpoint_locs;

    MyScheduler(std::shared_ptr<MyPlanner::WPPLPlanner> _wppl_planner): mt(0), wppl_planner(_wppl_planner) {}

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

    void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);


    void schedule_plan_default(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_default2(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_greedy_matching2(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_greedy_matching3(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_greedy_matching4(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
    void schedule_plan_greedy_matching_warehouse(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

    bool skip_matching_if_no_free_agents=true;
    bool no_matching_for_disable_agents=true;

};

} // namespace WPPL

#endif