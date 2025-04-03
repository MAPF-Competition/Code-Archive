
#ifndef search_hpp
#define search_hpp

#include "Types.h"
#include "utils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.h"

#include <vector>
#include <mutex>




namespace DefaultPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow
struct my_node
{   
    std::vector<int> paths;
    int location;
    int timesteps;
    int cost;
    int orientation;
    vector<Action>  actions;

    my_node(int loc, 
            int ts, 
            int c, 
            const std::vector<int>& p,
            int o,
            const vector<Action>& act)
        : location(loc),
          timesteps(ts),
          cost(c),
          paths(p),
          orientation(o),
          actions(act){}

    my_node()
        : paths(), 
          location(0), 
          timesteps(0), 
          cost(0){}

    bool operator<(const my_node& other) const {
        if (cost + timesteps == other.cost + other.timesteps){
            if (timesteps == other.timesteps)
                return rand() % 2;
            else
                return timesteps < other.timesteps;
        }
        else{
            return cost + timesteps > other.cost + other.timesteps; 
        }
    }
};
class MCTSNode {
public:
    MCTSNode(std::vector<std::vector<int>> paths) 
        : paths(paths), constraint({}), num_visited(0), cost(0), num_cof(0), children(), depth(1) {}

    MCTSNode(MCTSNode* node) {
        this->paths = node->paths;
        this->constraint = node->constraint;
        this->num_visited = node->num_visited;
        this->cost = node->cost;
        this->num_cof = node->num_cof;
        this->V_Constraints = node->V_Constraints;
        this->E_Constraints = node->E_Constraints;
        this->depth = 1;
        this->children = node->children;
        this->closest_timestpes = node->closest_timestpes;
        this->parent = node;
    }

    std::vector<std::vector<int>> paths;
    std::vector<std::vector<int>> constraint;
    std::vector<std::vector<int>> V_Constraints;
    std::vector<std::vector<int>> E_Constraints;
    int num_visited = 0;
    int cost = 0;
    int num_cof = 0;
    int depth = 1;
    int closest_timestpes = 20;
    MCTSNode* parent = nullptr;
    std::vector<MCTSNode*> children;  
    double uctValue(double explorationWeight) const {
        return (-num_cof + 2*explorationWeight* std::sqrt(2*std::log(parent->depth) / depth));
    }
    bool operator<(const MCTSNode* other) const {
    //     if (num_cof + 2*1.4* std::sqrt(2*std::log(parent->depth) / depth) == other->num_cof + 2*1.4* std::sqrt(2*std::log(other->parent->depth) / other->depth)) {
    //         // if (num_cof == other->num_cof) {
    //         //     if(cost == other->cost){
    //         //         if(closest_timestpes == other->closest_timestpes){
    //         //             return rand() %2;
    //         //         }
    //         //         return closest_timestpes > other->closest_timestpes;
    //         //     }
    //         //     return cost > other->cost;  
    //         // }
    //         // return num_cof > other->num_cof;  
    //         return rand()%2;
    //     }
    //     return num_cof + 2*1.4* std::sqrt(2*std::log(parent->depth) / depth) > other->num_cof  + 2*1.4* std::sqrt(2*std::log(other->parent->depth) / other->depth);
    // }

        double uctThis = uctValue(1.0);  
        double uctOther = other->uctValue(1.0); 

        return uctThis > uctOther;
    }
};

struct a_node
{
    std::vector<my_node> Paths;
    std::vector<std::vector<int>> V_Constraints;
    std::vector<std::vector<int>> E_Constraints;
    int num_cof;
    int cost;
    int max_steps;
    int far_timestpes = 0;
    std::priority_queue<a_node> next_open;


    a_node() : num_cof(0), cost(0), max_steps(0){}

    a_node(const a_node& a)
        : Paths(a.Paths),
          V_Constraints(a.V_Constraints),
          E_Constraints(a.E_Constraints),
          num_cof(a.num_cof),
          cost(a.cost),
          max_steps(a.max_steps),
          far_timestpes(a.far_timestpes){}

    bool operator<(const a_node& other) const {
        if(num_cof == other.num_cof){
            if (far_timestpes == other.far_timestpes){
                if (cost == other.cost){
                    return rand() % 2;
                }
                
                else{
                    return cost > other.cost;
                }
            }
            else{
                return far_timestpes < other.far_timestpes;
            }
        }
        else{
           return num_cof > other.num_cof; 
        }
    }
    
};


void h3(MCTSNode* node); 

int expand(MCTSNode* root);

std::vector<int> backtrack(s_node *H);

bool isRepeated(const std::vector<std::vector<int>>& bigVec,
                    const std::vector<int>& smallVec);

my_node my_Astar(int agent_id, SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht);

my_node my_Astar_constraint(int agent_id ,SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
                                    std::vector<std::vector<int>>& V_Containts,
                                    std::vector<std::vector<int>>& E_Containts);

s_node Astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns);

std::vector<int> Astar_constraint(int i, SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns,  
    std::vector<std::vector<int>>& V_Containts,
    std::vector<std::vector<int>>& E_Containts);



std::vector<int> validate(a_node& trajs);

}
#endif