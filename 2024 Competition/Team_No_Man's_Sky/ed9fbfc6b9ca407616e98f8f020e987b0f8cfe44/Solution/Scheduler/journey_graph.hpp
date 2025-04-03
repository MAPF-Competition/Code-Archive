#pragma once

#include "Objects/Basic/assert.hpp"
#include "Objects/Basic/position.hpp"
#include "Objects/Environment/graph.hpp"
#include "Objects/Environment/heuristic_matrix.hpp"
#include "Scheduler/hungarian.hpp"
#include "SharedEnv.h"
#include <Objects/Basic/time.hpp>
#include <set>
#include <vector>


namespace JG {


    void make_unique(std::vector<size_t> &vec);

    int get_dist(const uint32_t &from, const uint32_t to);

    struct EdgeDefinition {
        // edge defined by two values:
        size_t from;// first task
        size_t to;  // second task

        // Define equality comparison for MyStruct
        bool operator==(const EdgeDefinition &other) const {
            return from == other.to && from == other.to;
        }
    };

    // Custom hash function for MyStruct
    struct EdgeDefinitionHasher {
        size_t operator()(const EdgeDefinition &key) const {
            return std::hash<size_t>()(key.from) ^ (std::hash<size_t>()(key.to) << 1);
        }
    };

    class PathSegment;

    struct Edge {
        size_t get_to_path_size;// path_size to get to the start of the task
        size_t task_path_size;  // path_size to complete task
        size_t task_id;
        size_t node_from;
        size_t node_to;

        PathSegment *path_segment;

        size_t get_full_path_size() const {
            return task_path_size + get_to_path_size;
        }
    };

    enum class NODE_TYPE {
        start,
        end
    };

    struct Node {
        size_t task_id;
        uint32_t pos;// WITH + 1
        NODE_TYPE type;
        size_t paired_id;// id of second node (nodes adds in pairs)
        size_t edge = -1;// every end node is a task, every task is should be completed somewhen
        bool is_deleted = false;
        std::vector<size_t> edges;// out edges only (NOT USED)
    };

    enum class TASK_STATUS {
        planned,
        running,
        completed,
        free
    };


    const size_t NOT_FIRST_SEGMENT = -1;
    const size_t HANGING_PATH = -2;

    class JourneyGraph;

    struct PathSegment {
        size_t edge_id;
        PathSegment *next;
        PathSegment *prev;
        size_t path_id;// only set for first element in path

        bool IsTerminal() {
            return next == nullptr;
        }

        bool IsStart() {
            if (prev == nullptr) {
                assert(path_id != NOT_FIRST_SEGMENT);
            } else {
                assert(path_id == NOT_FIRST_SEGMENT);
            }
            // assert((prev != nullptr) == (path_id == NOT_FIRST_SEGMENT || path_id == HANGING_PATH));
            return prev == nullptr;
        }
        bool IsHanging() {
            return path_id == HANGING_PATH;
        }

        size_t GetPathId() {
            assert(IsStart());
            return path_id;
        }

        void Hang() {
            // std::cout << "HANG " << edge_id << std::endl;
            if (IsStart()) {
                path_id = HANGING_PATH;
                return;
            }
            path_id = HANGING_PATH;
            prev->next = nullptr;
            prev = nullptr;
        }

        void HangNext() {
            if (next) {
                next->Hang();
            }
        }

        bool TryToFindCycle(PathSegment *rhs) {
            if (rhs == this) {
                return true;
            }
            while (rhs->next) {
                rhs = rhs->next;
                if (rhs == this) {
                    return true;
                }
            }
            return false;
        }

        void Connect(PathSegment *rhs) {
            assert(IsTerminal());
            assert(rhs);
            assert(rhs->IsHanging());
            if (TryToFindCycle(rhs)) {
                assert(false);
            }
            next = rhs;
            rhs->path_id = NOT_FIRST_SEGMENT;
            rhs->prev = this;
        }


        PathSegment(size_t edge_id, size_t path_id, JourneyGraph &jg, PathSegment *prev = nullptr, PathSegment *next = nullptr);
    };


    struct Path {
        size_t full_length = 0;
        PathSegment *start;
        PathSegment *end;// useless
        size_t taken_by = -1;
        bool is_deleted = false;// dont need(?)

        Path(PathSegment *start_segment, JourneyGraph &jg);

        void Delete() {
            assert(!is_deleted);
            is_deleted = true;
            // if (!IsEmpty()){
            //     start->path_id = -1;
            // }
        }
        void Pop() {
            PathSegment *new_start = nullptr;
            if (!start->IsTerminal()) {
                new_start = start->next;
                start->HangNext();
                new_start->path_id = start->path_id;
            }
            delete start;// TODO ADD BACK
            start = new_start;
        }

        bool IsEmpty() {
            return (start == nullptr);
        }
    };

    const int CLOSEST_AMOUNT = 15;
    const int MAX_PATH_SIZE = 2;


    class JourneyGraph {
    public:
        SharedEnvironment *env;
        std::vector<Node> nodes;
        std::vector<Path> paths;
        std::vector<Edge> edges;
        std::unordered_map<size_t, std::pair<size_t, size_t>> task_to_nodes;// get start and end nodes ids by task id
        std::unordered_map<size_t, TASK_STATUS> task_statuses;              // ~?
        std::vector<size_t> assigned_path;                                  // ? TO WHOM - TO TASK;
        int total_paths = 0;
        size_t total_robots = 0;
        // std::unordered_map<EdgeDefinition, size_t, EdgeDefinitionHasher> edge_to_num; // edge defenition to edge_id

        std::queue<size_t> optimization_q;

        JourneyGraph(){};
        JourneyGraph(SharedEnvironment *env) : env(env), total_robots(env->num_of_agents) {
            assigned_path.resize(env->num_of_agents, -1);
        };

        std::vector<size_t> FindClosest(const uint32_t pos, NODE_TYPE type) {
            std::priority_queue<pair<size_t, size_t>> closest;
            for (size_t n_id = 0; n_id < nodes.size(); ++n_id) {
                auto &node = nodes[n_id];
                if (node.type == type && !node.is_deleted) {
                    closest.push({get_dist(pos, node.pos), n_id});
                    if (closest.size() > CLOSEST_AMOUNT) {
                        closest.pop();
                    }
                }
            }
            std::vector<size_t> closest_tasks;
            while (!closest.empty()) {
                closest_tasks.push_back(closest.top().second);
                closest.pop();
            }

            return closest_tasks;
        }

        void AddTask(const size_t t, const Task &task) {
            auto [start_n_id, end_n_id] = AddTaskNodes(t, task);
            task_to_nodes[t] = {start_n_id, end_n_id};
            size_t edge_id = AddEdge(start_n_id, end_n_id);
            CreateSimplePath(edge_id);

            optimization_q.push(start_n_id);
            optimization_q.push(end_n_id);

            // for every node from nearest find next node (by path, if there is any)
        }

        void ProccessOptimizationQ(size_t time_limit) {
            ETimer timer;
            PRINT(Printer() << "Current optimization_q size: " << optimization_q.size() << '\n';);
            size_t completed_tasks = 0;
            while (!optimization_q.empty()) {
                size_t node_id = optimization_q.front();
                optimization_q.pop();
                RunOptimize(node_id);
                completed_tasks++;
                if (timer.get_ms() >= time_limit * 0.9) break;
                ;
            }
            PRINT(Printer() << "Optimization tasks completed: " << completed_tasks << " tasks left: " << optimization_q.size() << '\n';);
        }

        void Claim(size_t r, size_t t) {// Robots claims task;
            auto start_node = task_to_nodes[t].first;
            auto segment = GetNodePathSegment(start_node);
            assert(segment->IsStart());
            DeleteNode(start_node);
            if (assigned_path[r] == segment->path_id) return;
            assert(assigned_path[r] == -1);
            assigned_path[r] = segment->path_id;
            paths[segment->path_id].taken_by = r;
        }

        bool IsTaskFree(size_t t) {// Robots claims task;
            auto start_node = task_to_nodes[t].first;
            auto segment = GetNodePathSegment(start_node);
            assert(segment->IsStart());
            return paths[segment->path_id].taken_by == -1;
        }

        size_t GetPerfomingTask(size_t r) {
            if (assigned_path[r] == -1) return -1;
            return edges[paths[assigned_path[r]].start->edge_id].task_id;
        }

        void Complete(size_t r) {
            size_t task_id = GetPerfomingTask(r);
            if (task_id == -1) return;
            Compelete(r, task_id);
            auto path_id = assigned_path[r];
            paths[path_id].Pop();
            if (paths[path_id].IsEmpty()) {
                // std::cout << "Deleting " <<  path_id << " as completed fully" << std::endl;
                assigned_path[r] = -1;
                DeletePath(path_id);
            } else {
                assert(paths[path_id].start->path_id == path_id);
                Claim(r, edges[paths[path_id].start->edge_id].task_id);
            }
        }

        std::vector<size_t> GetAvailableStarts() {
            std::vector<size_t> result;
            // std::unordered_map<size_t, size_t> kkk;
            for (size_t i = 0; i < paths.size(); ++i) {
                if (paths[i].is_deleted || paths[i].taken_by != -1) continue;
                result.push_back(nodes[edges[paths[i].start->edge_id].node_from].task_id);
                // if (kkk.find(result.back()) != kkk.end()){
                //     assert(false);
                // }
                // kkk[result.back()] = i;
            }
            std::sort(result.begin(), result.end());
            return result;
        }

        std::vector<size_t> GetFreeRobots() {
            std::vector<size_t> free_robots;
            for (size_t i = 0; i < assigned_path.size(); i++) {
                if (assigned_path[i] == -1) {
                    free_robots.push_back(i);
                }
            }
            return free_robots;
        }

        std::vector<int> GetDoneProposedSchedule() {
            std::vector<int> done_proposed_schedule(assigned_path.size(), -1);
            for (size_t i = 0; i < done_proposed_schedule.size(); ++i) {
                if (assigned_path[i] != -1) {
                    done_proposed_schedule[i] = edges[paths[assigned_path[i]].start->edge_id].task_id;
                }
            }
            return done_proposed_schedule;
        }

        PathSegment *findLastSegmentInPath(PathSegment *segment) {
            PathSegment *curr_segment = segment;
            size_t iters = 0;
            while (curr_segment->next) {
                curr_segment = curr_segment->next;
                iters++;
                if (iters > 10'000) {
                    std::cerr << "INFINITE CYCLE" << std::endl;
                    exit(1);
                }
            }
            return curr_segment;
        }

        void OutDBG() {
            size_t total_paths = 0;
            size_t total_assigned_paths = 0;
            std::map<size_t, size_t> path_sizes;
            for (size_t i = 0; i < paths.size(); ++i) {
                if (!paths[i].is_deleted) {
                    total_paths++;
                    if (paths[i].taken_by) {
                        total_assigned_paths++;
                    }
                    size_t len = GetSegmentIndex(paths[i].start) + 1;
                    path_sizes[len]++;
                }
            }
            std::cout << "----------------" << std::endl;
            std::cout << "total_paths: " << total_paths << std::endl;
            std::cout << "total_assigned_paths: " << total_assigned_paths << std::endl;
            for (const auto [len, cnt]: path_sizes) {
                std::cout << "len: " << len << " cnt: " << cnt << std::endl;
            }
            std::cout << "----------------" << std::endl;
        }

    private:
        size_t CreateSimplePath(size_t edge_id) {
            const size_t path_id = paths.size();
            auto *segment = new PathSegment(edge_id, path_id, *this);
            Path path(segment, *this);
            paths.push_back(path);
            total_paths++;

            return path_id;
        }

        size_t CreatePath(PathSegment *segment) {
            assert(segment->prev == nullptr);
            // std::cout << "Creating plan with segment of edge: " << segment->edge_id << std::endl;
            const size_t path_id = paths.size();
            segment->path_id = path_id;
            Path path(segment, *this);
            paths.push_back(path);
            return path_id;
        }

        std::pair<size_t, size_t> AddTaskNodes(const size_t t, const Task &task) {
            size_t start_n_id = nodes.size();
            size_t end_n_id = nodes.size() + 1;

            nodes.push_back({.task_id = t, .pos = static_cast<uint32_t>(task.locations.front() + 1), .type = NODE_TYPE::start, .paired_id = end_n_id});
            nodes.push_back({.task_id = t, .pos = static_cast<uint32_t>(task.locations.back() + 1), .type = NODE_TYPE::end, .paired_id = start_n_id});

            return {start_n_id, end_n_id};
        }

        // Adds edge from one NODE_TYPE::end to another NODE_TYPE::end. Using paired node to calc distance
        size_t AddEdge(const size_t node_from, const size_t node_to) {
            size_t task_to = nodes[node_to].task_id;
            assert(nodes[node_to].type == NODE_TYPE::end);
            size_t get_to_path_size = get_dist(nodes[node_from].pos, nodes[nodes[node_to].paired_id].pos);
            size_t task_path_size = 0;

            auto &task = env->task_pool[nodes[node_to].task_id];
            for (size_t i = 0; i + 1 < task.locations.size(); i++) {
                task_path_size += get_dist(task.locations[i] + 1, task.locations[i + 1] + 1);
            }
            Edge edge{.get_to_path_size = get_to_path_size, .task_path_size = task_path_size, .task_id = task_to, .node_from = node_from, .node_to = node_to};
            size_t edge_id = edges.size();
            edges.push_back(edge);

            nodes[node_to].edge = edge_id;
            nodes[node_from].edge = edge_id;

            // edge_to_num[{.from = nodes[node_from].n, .to = nodes[node_to].task_id}] = edge_id; // ???
            return edge_id;
        }

        size_t GetSegmentIndex(PathSegment *segment) {//
            PathSegment *curr_segment = segment;
            size_t iters = 0;
            while (curr_segment->next) {
                curr_segment = curr_segment->next;
                iters++;
                if (iters > 10'000) {
                    std::cerr << "INFINITE CYCLE" << std::endl;
                    exit(1);
                }
            }
            return iters;
        }

        bool SegmentsOnSamePath(PathSegment *f, PathSegment *s) {
            return findLastSegmentInPath(f) == findLastSegmentInPath(s);
        }

        size_t GetSegmentIndexLeft(PathSegment *segment) {// Oposite way
            PathSegment *curr_segment = segment;
            size_t iters = 0;
            while (curr_segment->prev) {
                curr_segment = curr_segment->prev;
                iters++;
                if (iters > 10'000) {
                    std::cerr << "INFINITE CYCLE" << std::endl;
                    exit(1);
                }
            }
            return iters;
        }

        void DeletePath(size_t path_id) {
            assert(path_id < paths.size());
            for (size_t i = 0; i < assigned_path.size(); ++i) {
                if (assigned_path[i] == path_id) {
                    // std::cout << "Deleting claimed path" << std::endl;
                    assert(false);
                }
            }
            // std::cout << "Delete path (" << path_id << ")" << std::endl;
            paths[path_id].Delete();
        }

        void DeleteNode(size_t node_id) {
            assert(node_id < nodes.size());
            // std::cout << "Delete node (" << node_id << ")" << std::endl;
            nodes[node_id].is_deleted = true;
            // paths[path_id].is_deleted = true;
        }


        PathSegment *GetNodePathSegment(size_t node_id) {
            if (nodes[node_id].type == NODE_TYPE::start) {
                node_id = nodes[node_id].paired_id;
            }
            assert(nodes[node_id].edge >= 0 && nodes[node_id].edge < edges.size());
            return edges[nodes[node_id].edge].path_segment;
        }


        void RunOptimize(size_t node_id) {
            const auto nearest_starts = FindClosest(nodes[node_id].pos, NODE_TYPE::start);// ближайшие началы к старту этой задаче
            const auto nearest_finishes = FindClosest(nodes[node_id].pos, NODE_TYPE::end);// ближайшие концы к старту этой задаче


            std::vector<size_t> right_half;// Node ids of starts;
            std::vector<size_t> left_half; // node ids of finishes;
            size_t dead_ends = 0;
            for (const auto near_finish: nearest_finishes) {
                auto segment = GetNodePathSegment(near_finish);
                assert(segment != nullptr);
                left_half.push_back(near_finish);
                if (!segment->IsTerminal()) {
                    const auto node_candidate = edges[segment->next->edge_id].node_from;
                    if (!nodes[node_candidate].is_deleted) {
                        right_half.push_back(node_candidate);
                    }
                } else {
                    dead_ends++;
                }
            }

            for (const auto near_start: nearest_starts) {
                auto segment = GetNodePathSegment(near_start);
                right_half.push_back(near_start);
                // std::cout << "ACCCESING SEGMENT OF " << near_start << " " << nodes[near_start].paired_id << std::endl;
                if (!segment->IsStart()) {
                    const auto node_candidate = edges[segment->prev->edge_id].node_to;
                    if (!nodes[node_candidate].is_deleted) {
                        left_half.push_back(node_candidate);
                    }
                }
            }
            make_unique(left_half);
            make_unique(right_half);
            for (size_t i = 0; i < dead_ends; ++i) {
                right_half.push_back(-1);
            }
            assert(left_half.size() <= right_half.size());
            Optimize(left_half, right_half);
        }

        void Optimize(const std::vector<size_t> &left_half, const std::vector<size_t> &right_half) {
            // right_half can contain -1;
            std::unordered_map<size_t, size_t> right_half_value_to_index;
            for (size_t i = 0; i < right_half.size(); i++) {
                if (right_half[i] != -1) {
                    right_half_value_to_index[right_half[i]] = i;
                }
            }
            size_t rb = std::min(left_half.size(), right_half.size());
            std::vector<std::vector<int>> dist_matrix(rb + 1, std::vector<int>(right_half.size() + 1, 0));
            // FIND SAME PATH
            for (int i = 0; i < rb; i++) {
                assert(left_half[i] != -1);
                for (int g = 0; g < right_half.size(); g++) {
                    // check if its the same path;
                    if (right_half[g] == -1) {
                        dist_matrix[i + 1][g + 1] = Hungarian::UNWANTED;
                    } else {
                        auto left_segment = GetNodePathSegment(left_half[i]);
                        auto right_segment = GetNodePathSegment(right_half[g]);
                        int weight_to_set = (int) get_dist(nodes[left_half[i]].pos, nodes[right_half[g]].pos);
                        if (SegmentsOnSamePath(left_segment, right_segment)) {
                            if (GetSegmentIndex(left_segment) <= GetSegmentIndex(right_segment)) {
                                weight_to_set = Hungarian::FORBID;
                            }
                        }
                        size_t len_will_be = GetSegmentIndexLeft(left_segment) + 1 + GetSegmentIndex(right_segment) + 1;
                        if (len_will_be > MAX_PATH_SIZE) {
                            weight_to_set = Hungarian::UNWANTED;
                        }
                        dist_matrix[i + 1][g + 1] = weight_to_set;
                    }
                }
            }
            auto ans = Hungarian::DoHungarian(dist_matrix);
            if (!Hungarian::CheckIfAssigmentCorrect(dist_matrix, ans)) {
                for (size_t i = 0; i < dist_matrix.size(); i++) {
                    for (size_t g = 0; g < dist_matrix.size(); g++) {
                        std::cout << dist_matrix[i][g] << " ";
                    }
                    std::cout << endl;
                }
                std::cout << endl;
                for (size_t i = 0; i < ans.size(); i++) {
                    std::cout << ans[i] << " ";
                }
                std::cout << endl;
                std::cerr << "Wrong Assigment!" << std::endl;
                return;
            }
            std::vector<bool> newly_hung(right_half.size(), false);
            for (size_t i = 1; i < ans.size(); ++i) {
                auto left_segment = GetNodePathSegment(left_half[i - 1]);
                if (right_half[ans[i] - 1] == -1) {
                    if (!left_segment->IsTerminal()) {
                        size_t right_ind = right_half_value_to_index[edges[left_segment->next->edge_id].node_from];
                        newly_hung[right_ind] = true;
                        left_segment->HangNext();
                    }
                } else {
                    if (nodes[right_half[ans[i] - 1]].is_deleted) {
                        assert(false);
                    }
                    auto right_segment = GetNodePathSegment(right_half[ans[i] - 1]);
                    size_t len_will_be = GetSegmentIndexLeft(left_segment) + 1 + GetSegmentIndex(right_segment) + 1;
                    if (len_will_be > MAX_PATH_SIZE) {
                        continue;
                    }
                    if (SegmentsOnSamePath(left_segment, right_segment) && (GetSegmentIndex(left_segment) <= GetSegmentIndex(right_segment))) {
                        continue;
                    }

                    if (left_segment->next == right_segment) {
                        // nothing changed
                        continue;
                    }
                    if (total_paths <= total_robots && right_segment->IsStart()) {
                        std::cout << "REJECTED CAUSE WILL ELIMINATE PATH " << total_paths << " " << total_robots << std::endl;
                        ;
                        continue;
                    }

                    if (right_segment->IsStart()) {
                        //std::cout << "DELETING: " << right_segment->GetPathId() << std::endl;
                        if (!right_segment->IsHanging()) {
                            DeletePath(right_segment->GetPathId());
                            right_segment->Hang();// Already hanging, but here we just setting path_id to hanging

                            // TODO: DONT NEED? LIKELY
                            size_t right_ind = right_half_value_to_index[edges[right_segment->edge_id].node_from];
                            newly_hung[right_ind] = false;
                        } else {
                            size_t right_ind = right_half_value_to_index[edges[right_segment->edge_id].node_from];
                            newly_hung[right_ind] = false;
                        }
                    } else {
                        right_segment->Hang();
                        total_paths++;
                    }

                    if (!left_segment->IsTerminal()) {
                        size_t right_ind = right_half_value_to_index[edges[left_segment->next->edge_id].node_from];
                        newly_hung[right_ind] = true;
                        left_segment->HangNext();
                    }

                    std::cout << SegmentsOnSamePath(left_segment, right_segment) << " | " << GetSegmentIndex(left_segment) << " <?" << GetSegmentIndex(right_segment) << std::endl;

                    left_segment->Connect(right_segment);
                    total_paths--;
                }
            }
            // create paths for hanging
            for (size_t i = 0; i < newly_hung.size(); ++i) {
                if (newly_hung[i]) {
                    // std::cout << "CREATING NEWLY HUNG" << std::endl;
                    CreatePath(GetNodePathSegment(right_half[i]));
                }
            }
        }

        void Compelete(size_t r, size_t t) {// Robots claims task;
            auto start_node = task_to_nodes[t].first;
            auto segment = GetNodePathSegment(start_node);
            assert(segment->IsStart());
            assert(assigned_path[r] == segment->path_id);
            assigned_path[r] = segment->path_id;
            // DeleteNode(start_node);
            DeleteNode(task_to_nodes[t].second);
            assert(nodes[task_to_nodes[t].second].is_deleted);
            DeleteNode(task_to_nodes[t].second);
        }
    };

};// namespace JG

JG::JourneyGraph &get_jg();
