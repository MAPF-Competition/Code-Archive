
#include "my_heuristics.h"
#include <queue>
#include "utils.h"

namespace MyPlanner {

    std::vector<HeuristicTable> global_heuristictable;
    std::vector<Neighbor> global_neighbors;


    void init_neighbor(SharedEnvironment *env) {
        global_neighbors.resize(env->rows * env->cols);
        for (int row = 0; row < env->rows; row++) {
            for (int col = 0; col < env->cols; col++) {
                int loc = row * env->cols + col;
                if (env->map[loc] == 0) {
                    if (row > 0 && env->map[loc - env->cols] == 0) {
                        global_neighbors[loc].neighbor_location.push_back(loc - env->cols);
                        global_neighbors[loc].neighbor_orientation.push_back(3);  // north
                    }
                    if (row < env->rows - 1 && env->map[loc + env->cols] == 0) {
                        global_neighbors[loc].neighbor_location.push_back(loc + env->cols);
                        global_neighbors[loc].neighbor_orientation.push_back(1);  // south
                    }
                    if (col > 0 && env->map[loc - 1] == 0) {
                        global_neighbors[loc].neighbor_location.push_back(loc - 1);
                        global_neighbors[loc].neighbor_orientation.push_back(2);  // west
                    }
                    if (col < env->cols - 1 && env->map[loc + 1] == 0) {
                        global_neighbors[loc].neighbor_location.push_back(loc + 1);
                        global_neighbors[loc].neighbor_orientation.push_back(0);  // east
                    }
                }
            }
        }
    };

    void init_heuristics(SharedEnvironment *env) {
        if (global_heuristictable.size() == 0) {
            global_heuristictable.resize(env->map.size());
            init_neighbor(env);
        }

    }

    void init_heuristic(HeuristicTable &ht, SharedEnvironment *env, int goal_location) {
        // initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
        ht.htable.clear();
        ht.htable.resize(env->map.size(), MAX_TIMESTEP);
        // ht.open.clear();
        // generate a open that can save nodes (and a open_handle)
        // 0:east, 1:south, 2:west, 3:north
        HNode root_east(goal_location, 0, 0);
        HNode root_south(goal_location, 1, 0);
        HNode root_west(goal_location, 2, 0);
        HNode root_north(goal_location, 3, 0);
        ht.htable[goal_location] = 0;
        ht.open.push(root_east);  // add root to open
        ht.open.push(root_south);
        ht.open.push(root_west);
        ht.open.push(root_north);
    }

    Neighbor getNeighbor(std::vector<Neighbor> *ns, int location) {
        Neighbor neighbor;
        //forward
        assert(location >= 0 && location < ns->size());
        neighbor = ns->at(location);
        return neighbor;
    }

    int getStepCost(int current_direction, int goal_direction) {
        // mod 4 difference
        int diff = (goal_direction - current_direction) % 4;
        if (diff < 0) diff += 4;  // ensure 0..3

        // Now diff is 0,1,2,or 3
        // 0 => same direction
        // 1 => turn right 90 (cost 1)
        // 2 => turn 180 (cost 2)
        // 3 => turn left 90  (cost 1)
        if (diff == 0) return 1;
        else if (diff == 2) return 3;
        else return 2; // diff == 1 or diff == 3
    }


    int get_heuristic(HeuristicTable &ht, SharedEnvironment *env, int source, std::vector<Neighbor> *ns) {
        if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

        // int cost, diff;
        while (!ht.open.empty()) {
            HNode curr = ht.open.top();
            ht.open.pop();

            // If the cost we popped is no longer the best cost for that location,
            // it means we found a cheaper route already => skip
            if (curr.value > ht.htable[curr.location])
                continue;

            // If we've reached 'source' (meaning we now have the best cost for it),
            // return it immediately
            if (curr.location == source) {
                return curr.value;
            }

            //-----------
            // Expand
            //-----------
            // Get neighbors for 'curr.location'
            Neighbor neighbor = getNeighbor(ns, curr.location);
            auto &nLocs = neighbor.neighbor_location;
            auto &nOris = neighbor.neighbor_orientation;

            for (size_t i = 0; i < nLocs.size(); i++) {
                int nextLoc = nLocs[i];
                int nextOri = nOris[i]; // orientation you'd face if you move to that neighbor

                // cost depends on orientation change
                int stepCost = getStepCost(curr.direction, nextOri);
                int newCost = curr.value + stepCost;

                // boundary check
                assert(nextLoc >= 0 && nextLoc < (int) env->map.size());

                // If we found a cheaper way to get to 'nextLoc' (regardless of orientation),
                // then update htable & bestDir, push the new state to open
                if (newCost < ht.htable[nextLoc]) {
                    ht.htable[nextLoc] = newCost;
                    // ht.bestDir[nextLoc] = nextOri;

                    // Keep searching from that new state
                    ht.open.push(HNode(nextLoc, nextOri, newCost));
                }
            }
        }

        return MAX_TIMESTEP;
    }

    int get_h(SharedEnvironment *env, int source, int target) {
        // if (global_heuristictable.empty()) {
        //     init_heuristics(env);
        // }

        if (global_heuristictable.at(target).empty()) {
            init_heuristic(global_heuristictable.at(target), env, target);
        }

        return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
    }

    void export_heuristic_table(const std::string &filename="my_data.txt"){
        std::ofstream ofs(filename);
        if (!ofs) {
            std::cerr << "Error: Could not open file " << filename << " for writing.\n";
            return;
        }
        //--- A) Write global_cost_table ---
        // 1) Number of CostTables (outer dimension)
        ofs << global_heuristictable.size() << "\n";
        // 2) For each CostTable ...
        for (const auto &costTable : global_heuristictable) {
            // 3) For each Cost (the innermost int)
            for (auto cost : costTable.htable) {
                if (cost == 1073741823){
                    cost = -1;
                }
                    ofs << " " << cost;
                }
                ofs << "\n";
            }
        ofs.close();
        std::cout << "Data exported to " << filename << "\n";
    }

}