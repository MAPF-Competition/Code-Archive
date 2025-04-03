#pragma once
#include <queue>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <execution>
#include <omp.h>
#include "util/Timer.h"

namespace MyPlanner {

using COST_TYPE = float;

class Endpoints {
public:
    int id;
    std::vector<std::tuple<COST_TYPE, int> > tasks; // to be sorted
    std::vector<std::tuple<COST_TYPE, int> > agents; // to be sorted

    int min_task_idx=0; // NOTE, idx is for array, which might be different from id
    int min_agent_idx=0;

    inline void push_task(int task_id, COST_TYPE cost) {
        tasks.emplace_back(cost, task_id);
    }

    inline void push_agent(int agent_id, COST_TYPE cost) {
        agents.emplace_back(cost, agent_id);
    }

    inline void sort_tasks() {
        std::sort(tasks.begin(), tasks.end());
    }

    inline void sort_agents() {
        std::sort(agents.begin(), agents.end());
    }

    inline bool empty() {
        return min_task_idx>=tasks.size() || min_agent_idx>=agents.size();
    }

    void update(std::vector<bool> & matched_agents, bool matched_task) {
        while (min_agent_idx<agents.size()) {
            auto & min_agent = agents[min_agent_idx];
            if (matched_agents[std::get<1>(min_agent)]) {
                ++min_agent_idx;
            } else {
                break;
            }
        }

        if (matched_task) {
            ++min_task_idx;
        }
    }

    std::tuple<COST_TYPE, int, int, int> pop_min(std::vector<bool> & matched_agents) {
        // NOTE: we don't check empty() here, so make sure to call empty() before calling this function
        auto & min_task = tasks[min_task_idx];
        COST_TYPE min_task_cost = std::get<0>(min_task);
        int min_task_id = std::get<1>(min_task);

        auto & min_agent = agents[min_agent_idx];
        COST_TYPE min_agent_cost = std::get<0>(min_agent);
        int min_agent_id = std::get<1>(min_agent);

        return {min_task_cost+min_agent_cost, min_agent_id, min_task_id, id};
    }

};


class WarehouseMatchingPriorityQueue {
public: 
    int num_agents;
    int num_endpoints;
    std::vector<bool> matched_agents;
    std::vector<Endpoints> endpoints;

    WarehouseMatchingPriorityQueue(
        int _num_agents,
        int _num_endpoints
    ): 
        num_agents(_num_agents), 
        num_endpoints(_num_endpoints),
        matched_agents(_num_agents),
        endpoints(_num_endpoints) {
        for (int i=0;i<num_endpoints;++i) {
            endpoints[i].id=i;
        }
    }

    inline void push_task(int endpoint_id, int task_id, COST_TYPE cost) {
        endpoints[endpoint_id].push_task(task_id, cost);
    }

    inline void push_agent(int endpoint_id, int agent_id, COST_TYPE cost) {
        endpoints[endpoint_id].push_agent(agent_id, cost);
    }

    void sort_all() {
        #pragma omp parallel for schedule(dynamic,1)
        for (size_t i=0;i<endpoints.size();++i) {
            endpoints[i].sort_tasks();
            endpoints[i].sort_agents();
        }
    }

    void _update_all(int matched_task_endpoint_id) {
        // NOT SURE why it slows down a lot if parallelized
        // #pragma omp parallel for schedule(dynamic,1)
        for (size_t i=0;i<endpoints.size();++i) {
            bool matched_task = (i==matched_task_endpoint_id);
            endpoints[i].update(matched_agents, matched_task);
        }
    }

    tuple<COST_TYPE, int, int, int> _pop() {

        std::vector<tuple<COST_TYPE, int, int, int> > costs;

        for (int i=0;i<endpoints.size();++i) {
            if (!endpoints[i].empty()) {
                costs.push_back(endpoints[i].pop_min(matched_agents));
            }
        }

        auto iterator = std::min_element(costs.begin(), costs.end());

        return *iterator;
    }


    std::unordered_map<int,int> get_assignment() {
        std::unordered_map<int,int> assignment;

        COST_TYPE cost;
        int agent_id, task_id, endpoint_id;

        int num_matched=0;
        while (num_matched<num_agents) {
            // g_timer.record_p("_pop_s");
            std::tie(cost, agent_id, task_id, endpoint_id)=_pop();
            // g_timer.record_d("_pop_s","_pop");

            // std::cout<<"pop: "<<cost<<" "<<agent_id<<" "<<task_id<<" "<<endpoint_id<<std::endl;
            matched_agents[agent_id]=true;
            assignment[agent_id]=task_id;   
            ++num_matched;
            
            // g_timer.record_p("_update_all_s");
            _update_all(endpoint_id);
            // g_timer.record_d("_update_all_s","_update_all");

            // std::cout<<"updated"<<std::endl;
        }
        
        return assignment;

    }

};

}; // namespace MyPlanner