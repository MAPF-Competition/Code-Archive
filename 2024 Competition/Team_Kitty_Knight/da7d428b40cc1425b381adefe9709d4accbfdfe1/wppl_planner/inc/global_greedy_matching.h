#pragma once
#include <unordered_map>
#include <iostream>
#include <queue>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace MyPlanner {

using COST_TYPE = float;

class GlobalGreedyMatching {
public:
    py::detail::unchecked_reference<COST_TYPE,2> sorted_costs;
    py::detail::unchecked_reference<COST_TYPE,2> sorted_task_indices;
    size_t num_agents;
    size_t num_tasks;

    std::priority_queue<std::tuple<COST_TYPE, size_t, size_t> > gpq;
    std::vector<size_t> pq_indices;

    std::vector<bool> matched_agents;
    std::vector<bool> matched_tasks;
    
    GlobalGreedyMatching(
        const py::array_t<COST_TYPE> & _sorted_costs,
        const py::array_t<COST_TYPE> & _sorted_task_indices
    ): sorted_costs(_sorted_costs.unchecked<2>()),
       sorted_task_indices(_sorted_task_indices.unchecked<2>()) {
        
        num_agents=sorted_costs.shape(0);
        num_tasks=sorted_task_indices.shape(1);

        matched_agents.resize(num_agents, false);
        matched_tasks.resize(num_tasks, false);
        pq_indices.resize(num_agents, 0);
    }

    bool _push_to_gpq_from_pq(size_t i_agent) {
        size_t pq_idx = pq_indices[i_agent];
        if (pq_idx>=num_tasks) return false;

        gpq.emplace(
            -sorted_costs(i_agent, pq_idx),
            i_agent,
            sorted_task_indices(i_agent, pq_idx)
        );

        ++pq_indices[i_agent];
       
        // if (indexes[i_agent]>=edges[i_agent].size()) return false;
        // auto & e=edges[i_agent][indexes[i_agent]];
        // gpq.emplace(-std::get<0>(e), std::get<1>(e), std::get<2>(e));
        // ++indexes[i_agent];
        return true;
    }

    void _init() {
        for (size_t i=0;i<num_agents;++i) {
            if (!_push_to_gpq_from_pq(i)) {
                std::cout<<"error: we always ensure more tasks than agents. place 1"<<std::endl;
                exit(-1);
            }
        }
    }

    std::tuple<COST_TYPE,int,int> _pop() {
        COST_TYPE dist;
        int i_agent, j_task;
        while (true) {
            std::tie(dist, i_agent, j_task) = gpq.top();
            gpq.pop();
            if (!matched_agents[i_agent]) {
                if (!matched_tasks[j_task]) {
                    matched_agents[i_agent] = true;
                    matched_tasks[j_task] = true;
                    return {-dist, i_agent, j_task};
                } else {
                    if (!_push_to_gpq_from_pq(i_agent)) {
                        std::cout<<"error: we always ensure more tasks than agents. place 2"<<std::endl;
                        exit(-1);
                    }
                }
            }
        }
        return {-dist, i_agent, j_task};
    }

    // TODO: include timing into the loop
    std::unordered_map<int,int> get_assignment() {
        _init();
        std::unordered_map<int,int> assignment;
        COST_TYPE dist;
        int i_agent, j_task;
        int num_matched=0;
        while (num_matched<num_agents) {
            std::tie(dist, i_agent, j_task)=_pop();
            assignment[i_agent]=j_task;
            ++num_matched;
        }
        return assignment;
    }

};

}; // namespace MyPlanner