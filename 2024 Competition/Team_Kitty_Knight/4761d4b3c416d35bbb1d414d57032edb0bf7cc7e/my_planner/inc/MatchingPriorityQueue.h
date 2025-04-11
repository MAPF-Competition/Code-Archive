#pragma once
#include <queue>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <execution>

class MatchingPriorityQueue {
public: 
    MatchingPriorityQueue(int num_agents, int num_tasks): 
        num_agents(num_agents),
        pqs(num_agents),
        edges(num_agents, std::vector<std::tuple<int, int, int> >(num_tasks)),
        indexes(num_agents, 0),
        matched_agents(num_agents, false),
        matched_tasks(num_tasks, false) {

    }

    int num_agents;

    // NOTE: pq is by default a max heap
    // TODO: we can customize the comparator
    std::vector<std::priority_queue<std::tuple<int, int, int> > > pqs;
    std::vector<std::vector<std::tuple<int, int, int> > > edges;
    std::vector<size_t> indexes;

    std::vector<bool> matched_agents;
    std::vector<bool> matched_tasks;

    std::priority_queue<std::tuple<int, int, int> > gpq;

    void push(int i_agent, int j_task, int dist) {
        pqs[i_agent].emplace(-dist, i_agent, j_task);
        // edges[i_agent][j_task]={dist, i_agent, j_task};
    }

    void sort(int i_agent) {
        // std::sort(std::execution::unseq, edges[i_agent].begin(), edges[i_agent].end());
    }

    bool _push_to_gpq_from_pq(int i_agent) {
        // TODO: ideally we need to protect if pq is empty if the bipartite graph has full edges.
        if (pqs[i_agent].empty()) return false;
        gpq.push(pqs[i_agent].top());
        pqs[i_agent].pop();
       
        // if (indexes[i_agent]>=edges[i_agent].size()) return false;
        // auto & e=edges[i_agent][indexes[i_agent]];
        // gpq.emplace(-std::get<0>(e), std::get<1>(e), std::get<2>(e));
        // ++indexes[i_agent];
        return true;
    }

    void _init() { // should be called 
        for (int i=0;i<num_agents;++i) {
            if (!_push_to_gpq_from_pq(i)) {
                std::cout<<"error: we always ensure more tasks than agents. place 1"<<std::endl;
                exit(-1);
            }
        }
    }

    std::tuple<int,int,int> _pop() {
        int dist, i_agent, j_task;
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
        int dist, i_agent, j_task;
        int num_matched=0;
        while (num_matched<num_agents) {
            std::tie(dist, i_agent, j_task)=_pop();
            assignment[i_agent]=j_task;
            ++num_matched;
        }
        return assignment;
    }

};