
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

class MCTSNode {
public:
    MCTSNode(std::vector<std::vector<int>> paths, std::vector<int> priorities, std::vector<Int4> flow) 
        : paths(paths), constraint({}), num_visited(0), cost(0), num_cof(0), children(), priority(priorities), flow(flow), far_timestpes(0), depth(0){}

    MCTSNode(const MCTSNode& node) {
        this->paths = node.paths;
        this->constraint = node.constraint;
        this->num_visited = node.num_visited;
        this->cost = node.cost;
        this->num_cof = node.num_cof;
        this->V_Constraints = node.V_Constraints;
        this->E_Constraints = node.E_Constraints;
        this->depth = node.depth;
        this->children = node.children; 
        // this->parent = node.parent; 
        this->priority = node.priority; 
        this->far_timestpes = node.far_timestpes;
        this->flow = node.flow;
    }

    // MCTSNode(const MCTSNode& node, MCTSNode* parent) {
    //     this->paths = node.paths;
    //     this->constraint = node.constraint;
    //     this->num_visited = node.num_visited;
    //     this->cost = node.cost;
    //     this->num_cof = node.num_cof;
    //     this->V_Constraints = node.V_Constraints;
    //     this->E_Constraints = node.E_Constraints;
    //     this->depth = node.depth;
    //     this->children = node.children;  
    //     this->parent = parent;
    // }



    std::vector<std::vector<int>> paths;
    std::vector<std::vector<int>> constraint;
    std::vector<std::vector<int>> V_Constraints;
    std::vector<std::vector<int>> E_Constraints;
    int num_visited = 0;
    int cost = 0;
    int num_cof = 0;
    int depth = 0;
    std::set<MCTSNode*> children = {nullptr}; 
    MCTSNode* parent = nullptr; 
    std::vector<int> priority;
    int far_timestpes = 0;
    std::vector<Int4> flow;
    // int num_completed_task = 0;

    // bool operator<(const MCTSNode* other) const {
    //     if (num_cof + cost == other->num_cof + other->cost) {
    //         if (cost == other->cost) {
    //             return num_cof > other->num_cof;  
    //         }
    //         return cost > other->cost;  
    //     }
    //     return num_cof + cost > other->num_cof + other->cost;
    // }
};

struct a_node
{
    std::vector<std::vector<int>> Paths;
    std::vector<std::vector<int>> V_Constraints;
    std::vector<std::vector<int>> E_Constraints;
    int num_cof;
    int cost;
    int max_steps;
    int far_timestpes = 0;
    std::vector<Int4> flow;


    a_node() : num_cof(0), cost(0), max_steps(0){}

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

// struct my_node
// {   
//     std::vector<int> paths;
//     int location;
//     int timesteps;
//     int cost;

//     my_node(int loc, 
//             int ts, 
//             int c, 
//             const std::vector<int>& p)
//         : location(loc),
//           timesteps(ts),
//           cost(c),
//           paths(p){}

//     my_node()
//         : paths(), 
//           location(0), 
//           timesteps(0), 
//           cost(0){}

//     bool operator<(const my_node& other) const {
//         if (cost + timesteps == other.cost + other.timesteps){
//             if (timesteps == other.timesteps)
//                 return rand() % 2;
//             else
//                 return timesteps < other.timesteps;
//         }
//         else{
//             return cost + timesteps > other.cost + other.timesteps; 
//         }
//     }
// };

struct my_node
{   
    std::vector<int> paths;
    int location;
    int timesteps;
    int cost;
    double all_vertex_flow = 0.0;
    double op_flow = 0.0;
    double tie_breaker = 0.0;

    

    my_node(int loc, 
            int ts, 
            int c, 
            const std::vector<int>& p, 
            double avf = 0.0, 
            double of = 0.0, 
            double tb = 0.0)
        : location(loc),
          timesteps(ts),
          cost(c),
          paths(p),
          all_vertex_flow(avf), 
          op_flow(of), 
          tie_breaker(tb){}

    my_node()
        : paths(), 
          location(0), 
          timesteps(0), 
          cost(0), 
          all_vertex_flow(0.0), 
          op_flow(0.0), 
          tie_breaker(0.0){}

    // bool operator<(const my_node& other) const {
    //     if (op_flow + all_vertex_flow + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == other.op_flow + other.all_vertex_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //         if (static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //             if (timesteps == other.timesteps)
    //                 if (tie_breaker  == other.tie_breaker)
    //                     return rand() % 2;
    //                 else
    //                     return tie_breaker > other.tie_breaker;
                    
    //             else
    //                 return timesteps < other.timesteps;
    //         }
    //         else{
    //             return static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker; 
    //         }
    //     }
    //     else{
    //         return op_flow + all_vertex_flow + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > other.op_flow + other.all_vertex_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker;
    //     }
    // }

    // bool operator<(const my_node& other) const {
    //     if (op_flow  + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == other.op_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //         if (static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //             if (timesteps == other.timesteps)
    //                 if (tie_breaker  == other.tie_breaker)
    //                     return rand() % 2;
    //                 else
    //                     return tie_breaker > other.tie_breaker;
                    
    //             else
    //                 return timesteps < other.timesteps;
    //         }
    //         else{
    //             return static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker; 
    //         }
    //     }
    //     else{
    //         return op_flow  + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > other.op_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker;
    //     }
    // }

    bool operator<(const my_node& other) const {
        if (static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
            
            if (timesteps == other.timesteps)
                if (tie_breaker  == other.tie_breaker)
                    return rand() % 2;
                else
                    return tie_breaker > other.tie_breaker;
                
            else
                return timesteps < other.timesteps;
        }
        else{
            return static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker;
        }
    }

    // bool operator<(const my_node& other) const {
    //     if (op_flow  + static_cast<double>(cost) + static_cast<double>(timesteps) == other.op_flow  + static_cast<double>(other.cost) + static_cast<double>(other.timesteps)){
    //         if (static_cast<double>(cost) + static_cast<double>(timesteps)== static_cast<double>(other.cost) + static_cast<double>(other.timesteps)){
    //             if (timesteps == other.timesteps)
    //                 // if (cost  == other.cost)
    //                 //     return rand() % 2;
    //                 // else
    //                 //     return cost < other.cost;
    //                 return rand() % 2;
                    
    //             else
    //                 return timesteps < other.timesteps;
    //         }
    //         else{
    //             return static_cast<double>(cost) + static_cast<double>(timesteps) > static_cast<double>(other.cost) + static_cast<double>(other.timesteps);
    //         }
    //     }
    //     else{
    //         return op_flow + static_cast<double>(cost) + static_cast<double>(timesteps) > other.op_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps);
    //     }
    // }

    // bool operator<(const my_node& other) const {
    //     if (op_flow  + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == other.op_flow  + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //         if (static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker == static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker){
    //             if (timesteps == other.timesteps)
    //                 if (cost  == other.cost)
    //                     // if (tie_breaker == other.tie_breaker)
    //                     //     return rand() % 2;
    //                     // else
    //                     //     return tie_breaker > other.tie_breaker;
    //                     return rand() % 2;
    //                 else
    //                     return cost < other.cost;
                    
    //             else
    //                 return timesteps < other.timesteps;
    //         }
    //         else{
    //             return static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker;
    //         }
    //     }
    //     else{
    //         return op_flow + static_cast<double>(cost) + static_cast<double>(timesteps) + tie_breaker > other.op_flow + static_cast<double>(other.cost) + static_cast<double>(other.timesteps) + other.tie_breaker;
    //     }
    // }



    // bool operator<(const my_node& other) const {
    //     if (cost + timesteps == other.cost + other.timesteps){
    //         if (timesteps == other.timesteps)
    //             return rand() % 2;
    //         else
    //             return timesteps < other.timesteps;
    //     }
    //     else{
    //         return cost + timesteps > other.cost + other.timesteps; 
    //     }
    // }
};

int h3(MCTSNode& node, Neighbors* ns); 

int expand(MCTSNode* root);

std::vector<int> backtrack(s_node *H);

bool isRepeated(const std::vector<std::vector<int>>& bigVec,
                    const std::vector<int>& smallVec);

std::vector<int> my_Astar(SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
                            std::vector<Int4> flow);

std::vector<int> my_Astar_constraint(int agent_id ,SharedEnvironment* env,int start, int goal, Neighbors* ns,HeuristicTable& ht,
                                    std::vector<Int4> flow,
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