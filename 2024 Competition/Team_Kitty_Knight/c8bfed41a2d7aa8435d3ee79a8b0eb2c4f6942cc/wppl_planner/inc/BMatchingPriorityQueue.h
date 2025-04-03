#pragma once
#include <queue>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <execution>
#include <iostream>

namespace MyPlanner {

using COST_TYPE = float;
class BMatchingPriorityQueue {
public: 
    BMatchingPriorityQueue(int _num_agents, int _num_endpoints): 
        num_agents(_num_agents),
        num_endpoints(_num_endpoints),
        agents_endpoints(num_agents),
        endpoints_tasks(num_endpoints),
        ae_indexes(num_agents, 0),
        et_indexes(num_endpoints, 0) {
        
        for (size_t i=0;i<agents_endpoints.size();++i) {
            agents_endpoints[i].resize(num_endpoints);
        }

    }

    int num_agents;
    int num_endpoints;

    std::vector<std::vector<std::tuple<COST_TYPE, int> > > agents_endpoints; // cost, endpoint_id
    std::vector<std::vector<int> > endpoints_tasks; // task_id
    std::vector<size_t> ae_indexes;
    std::vector<size_t> et_indexes;

    std::priority_queue<std::tuple<COST_TYPE, int, int> > gpq;

    void push_agent_endpoint(int i_agent, int j_endpoint, COST_TYPE cost) {
        agents_endpoints[i_agent][j_endpoint]={cost, j_endpoint};
    }

    // NOTE: we assume it is ordered directly by task costs when pushed
    void push_endpoint_task(int j_endpoint, int k_task) {
        endpoints_tasks[j_endpoint].push_back(k_task);
    }

    void sort(int i_agent) {
        // if (i_agent==599) {
        //     std::cout<<"i_agent: "<<i_agent<<std::endl;

        //         for (size_t j=0;j<agents_endpoints[i_agent].size();++j) {
        //             std::cout<<std::get<0>(agents_endpoints[i_agent][j])<<","<<std::get<1>(agents_endpoints[i_agent][j])<<" ";
        //         }
        //         std::cout<<std::endl;

        // }

        std::sort(agents_endpoints[i_agent].begin(), agents_endpoints[i_agent].end());
    
        // if (i_agent==599) {
        //     std::cout<<"i_agent: "<<i_agent<<std::endl;
        
        //         for (size_t j=0;j<agents_endpoints[i_agent].size();++j) {
        //             std::cout<<std::get<0>(agents_endpoints[i_agent][j])<<","<<std::get<1>(agents_endpoints[i_agent][j])<<" ";
        //         }
        //         std::cout<<std::endl;

        // }
    }

    void _push_ae_to_gpq(int i_agent) {      
        // no check here! make sure check before calling this function
        auto & e=agents_endpoints[i_agent][ae_indexes[i_agent]];
        gpq.emplace(-std::get<0>(e), i_agent, std::get<1>(e));
    }

    void _init() { // should be called 
        for (int i=0;i<num_agents;++i) {
            _push_ae_to_gpq(i);
        }
    }

    // NOTE: this implementation is dangerous. The two while loops wiil stop
    // because we have the assumption that there will always be more tasks than agents.
    // otherwise, we need to do more checking.
    // return {dist from agent to endpoint, i_agent, j_endpoint, k_task}
    std::tuple<COST_TYPE,int,int,int> _pop() {
        COST_TYPE dist=1;
        int i_agent=-1, k_task=-1, j_endpoint=-1;
        while (true) {
            std::tie(dist, i_agent, j_endpoint) = gpq.top();
            gpq.pop();
            auto & et_index=et_indexes[j_endpoint];

            if (et_index<endpoints_tasks[j_endpoint].size()) {
                // if the currently closest endpoint still has tasks, retrieve one
                k_task=endpoints_tasks[j_endpoint][et_index];
                ++et_index;
                return {-dist, i_agent, j_endpoint, k_task};
            } else {
                auto & ae_index=ae_indexes[i_agent];
                while (true) {
                    // if the currently closest endpoint doesn't have any tasks, move to the next closest endpoint
                    ++ae_index;    
                    int next_endpoint=std::get<1>(agents_endpoints[i_agent][ae_index]);
                    // if the next endpoint still has tasks, break
                    if (et_indexes[next_endpoint]<endpoints_tasks[next_endpoint].size()) {
                        break;
                    }
                } 

                // push a task from the currently closest endpoint to the gpq
                _push_ae_to_gpq(i_agent);
            }
        }
        return {-1,-1,-1,-1}; // should not reach here
    }

    // TODO: include timing into the loop
    std::unordered_map<int,int> get_assignment() {
        _init();
        std::unordered_map<int,int> assignment;
        COST_TYPE dist;
        int i_agent, j_endpoint, k_task;
        int num_matched=0;
        while (num_matched<num_agents) {
            std::tie(dist, i_agent, j_endpoint, k_task)=_pop();
            std::cout<<"i_agent: "<<i_agent<<" j_endpoint: "<<j_endpoint<<" k_task: "<<k_task<<" dist: "<<dist<<std::endl;
            assignment[i_agent]=k_task;
            ++num_matched;
        }
        return assignment;
    }

};

}; // namespace MyPlanner