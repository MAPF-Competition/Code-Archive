#pragma once
#include <utility>
#include <list>
#include <vector>
#include <memory>
#include <unordered_set>
#include <iostream>
#include <queue>
namespace ADG_namespace {
    enum class node_status { STAGED, ONGOING, COMPLETED };

    struct Node {
        int id = 0;
        int timestep = 0;
        node_status status = node_status::STAGED;

        std::pair<int, int> start;
        std::pair<int, int> goal;
        std::vector<std::shared_ptr<Node>> incoming_edges;
        std::vector<std::shared_ptr<Node>> edges; // Pointers to next nodes in the dependency graph
    };

    class ADG {
    public:
        ADG(const std::vector<std::list<std::pair<int, int>>>& paths) {
            adJoin(paths);
        }

        void adJoin (const std::vector<std::list<std::pair<int, int>>>& paths, bool adjoin = false)  {
            // Step 1: Add ADG vertices and Type 1 dependencies
            size_t N = paths.size();
            if (!adjoin) {
                past_locs = std::vector<int>(N, 0);
                plans.resize(N);
            }

            for (size_t i = 0; i < N; ++i) {
                auto it = paths[i].begin();
                std::pair<int, int> p = *it;
                auto v = std::make_shared<Node>();
                v->id = i;
                v->start = p;
                v->status = node_status::STAGED;
                std::shared_ptr<Node> v_prev = nullptr;
            
                ++it; // Move to the next point in the path
                for (size_t k = 1; it != paths[i].end(); ++it, ++k) {
                    std::pair<int, int> p_k = *it;
                    v->goal = p_k;
                
                    plans[i].push_back(v);
                    if (v_prev) {
                        v_prev->edges.push_back(v); // Add Type 1 dependency
                    }
                    v_prev = v;

                    p = p_k;
                    v = std::make_shared<Node>();
                    v->id = i;
                    v->start = p;
                    v->status = node_status::STAGED;
                    v->timestep =  v_prev->timestep + 1;
                    v->incoming_edges.push_back(v_prev);
                }
            }

            // Step 2: Add Type 2 dependencies
            for (size_t i = 0; i < N; ++i) {
                for (size_t k = 0; k < plans[i].size(); ++k) {
                    for (size_t j = 0; j < N; ++j) {
                        if (i != j) {
                            for (size_t l = 0; l < plans[j].size(); ++l) {
                                int o_k = k, o_l = l; //offsets 
                                o_k += adjoin ? past_locs[i] + 1 : 0;
                                o_l += adjoin ? past_locs[j] + 1 : 0;
                                if (plans[i][o_k]->start == plans[j][o_l]->goal &&
                                    plans[i][o_k]->timestep <= plans[j][o_l]->timestep) {
                                        plans[i][o_k]->edges.push_back(plans[j][o_l]);
                                        plans[j][o_l]->incoming_edges.push_back(plans[i][o_k]);
                                    break;
                                }
                            }
                        }
                    }
                }
            }            
        }

        void print() {
            for (const auto& plan : plans) {
                for (const auto& n: plan) {
                    std::cout << '(' << n->id << ' ' << n->timestep 
                    << ", (" << n->start.first << "," << n->start.second << "), (" << n->goal.first << "," << n->goal.second << ")) ";
                }
                std::cout << std::endl;

            }

            for (const auto& plan : plans) {
                for (const auto& n: plan) {
                    std::cout  << "incoming edges: ";
                    for (const auto& edge: n->incoming_edges) {
                    std::cout << '(' << edge->id << ' '  << edge->timestep 
                    << ", (" << edge->start.first << "," << edge->start.second << "), (" << edge->goal.first << "," << edge->goal.second << ")) ";

                    }
                    std::cout << '(' << n->id << ' ' << n->timestep 
                    << ", (" << n->start.first << "," << n->start.second << "), (" << n->goal.first << "," << n->goal.second << ")) ";

                    std::cout  << "edges: ";
                    for (const auto& edge: n->edges) {
                    std::cout << '(' << edge->id << ' '  << edge->timestep 
                    << ", (" << edge->start.first << "," << edge->start.second << "), (" << edge->goal.first << "," << edge->goal.second << ")) ";

                    }

                    std::cout << std::endl;
                }
                std::cout << std::endl;

            }
        }

        // Overload the subscript operator to access plans
        std::vector<std::shared_ptr<Node>>& operator[](size_t index) {
            return plans[index];
        }

        // Const version of the subscript operator
        const std::vector<std::shared_ptr<Node>>& operator[](size_t index) const {
            return plans[index];
        }

        void clear() {
            plans.clear();
            visited.clear();
        }

        std::queue<std::shared_ptr<Node>> computeDesiredSet() {
            std::vector<int> start;

            std::queue<std::shared_ptr<Node>> desired;
            for (int i = 0; i < plans.size(); i++) {
                int counter = 1;
                // if (past_locs[i] == plans[i].size()) continue;
                for (int j = past_locs[i]; j < plans[i].size(); j++) {
                    if (counter == 3) {
                    desired.push(plans[i][j]);
                    break;
                    }
                    counter++;
                }
            }
            return desired;
        }

        std::vector<std::shared_ptr<Node>> computeCommitCut() {
            std::vector<std::shared_ptr<Node>> commit_cut; 
            auto desired = computeDesiredSet();

            std::unordered_set<std::shared_ptr<Node>> reachable;
            while (!desired.empty()) {
                auto p = desired.front();
                desired.pop();
                reachable.insert(p);
                
                for (auto u : p->incoming_edges) {
                    if (reachable.find(u) == reachable.end()) {
                        desired.push(u);
                    }
                }
            }

            for (int j = 0; j < plans.size(); j++) {
                std::shared_ptr<Node> latest = nullptr;
                for (auto p: reachable) {
                    // std::cout << "reachable id " << p->id << std::endl;
                    if (p->id != j) continue;
                    if (!latest) { 
                        latest = p;
                        continue;
                    }
                    if (latest->timestep <= p->timestep) latest = p;

                }
                // if (latest)
                commit_cut.push_back(latest);
                
            }

            return commit_cut;
        }
        bool is_finished = false;
        std::vector<int> past_locs;
    private:
        std::vector<std::vector<std::shared_ptr<Node>>> plans;
        std::unordered_set<std::shared_ptr<Node>> visited;
        bool createsCycle(const std::shared_ptr<Node>& from, const std::shared_ptr<Node>& to) {

            return hasPath(to, from, visited);
        }

        // Recursive function to check if there is a path from 'current' to 'target'
        bool hasPath(const std::shared_ptr<Node>& current, const std::shared_ptr<Node>& target, std::unordered_set<std::shared_ptr<Node>>& visited) {
            if (current == target) {
                return true;
            }

            if (visited.find(current) != visited.end()) {
                return false;
            }

            visited.insert(current);
            for (const auto& neighbor : current->edges) {
                if (hasPath(neighbor, target, visited)) {
                    return true;
                }
            }

            return false;
        }
    };
}