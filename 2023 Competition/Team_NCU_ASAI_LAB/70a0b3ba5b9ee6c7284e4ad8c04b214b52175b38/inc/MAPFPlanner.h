#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <atomic>
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
struct Node;
struct TreeNode;

typedef typename boost::heap::d_ary_heap<TreeNode,boost::heap::arity<2>,boost::heap::mutable_<true>>high_open;
typedef typename high_open::handle_type high_handle;
struct State_my
{
    int location;
    int timestep;
    int orientation;
    State_my(int location,int timestep,int orientation) :timestep(timestep),location(location),orientation(orientation){}

    bool operator==(const State_my&other) const{
        return timestep==other.timestep&&location==other.location;
    }
    bool equalExceptTime(const State_my&other) const{return location==other.location&&timestep==other.timestep;}
};
struct Constraint_my {
    int location;
    int agentid;
    int time;
    Constraint_my(int _location,int _agentid,int _time) :
        location(_location),agentid(_agentid),time(_time){ }
    bool operator==(const Constraint_my&c)const{
        if(this->location==c.location&&this->agentid==c.agentid&&this->time==c.time) return true;
        return false;
    }
    
};
struct Conflict_my{
    std::pair<int,int>conflict_agent;
    int location1;
    int location2;
    int time;
    Conflict_my(int first_agent,int second_agent,int _location1,int _location2,int _time) :
            location1(_location1),location2(_location2),time(_time){
                conflict_agent=std::make_pair(first_agent,second_agent);
            }
};
struct TreeNode
{
    SharedEnvironment node_env;
    std::vector<int>deadlock;
    int cost=0;
    int focal;
    std::vector<int>path_empty;
    int path_flag=0;
    high_handle handle;
    std::vector<std::vector<std::pair<int,int>>>solution;
    std::vector<Constraint_my>constraints;
    std::vector<pair<int,int>> single_agent_plan(int start,int start_direct,int end,int agentid);
    //int getManhattanDistance(int loc1, int loc2);
    std::vector<State_my> getNeighbors(State_my &current);
    bool validateMove(int loc, int loc2);
    int getManhattanDistance(int loc1, int loc2);
    bool update_solution();
    void update_cost();
    void add_constraint(Constraint_my &constraint);
    bool is_constraint(int agentid,int location,int time,std::vector<Constraint_my>&constraints);
    int vertex_conflict(State_my&s,int agentid);
    int edge_conflict(State_my&s1,State_my&s2,int agentid);

    TreeNode();
    TreeNode(std::vector<Constraint_my>&constraints):constraints(constraints){};
    TreeNode(SharedEnvironment env):node_env(env){};
    /*
    TreeNode operator=(const TreeNode &node)const{
        TreeNode t(node.node_env);
        t.constraints=node.constraints;
        t.solution=node.solution;
        t.cost=node.cost;
        //cout<<"node.cost   t.cost: "<<node.cost<<"   "<<t.cost<<endl;
        return t;
    }
    */
    bool operator==(const TreeNode &node)const{
        if(node.constraints==this->constraints&&node.solution==this->solution&&node.cost==this->cost) return true;
        else return false;
    }
    bool operator<(const TreeNode&other)const{
        return cost>other.cost;
    }
    //TreeNode(std::vector<Constraint_my>&constraints);
    ~TreeNode();
    
};
struct compareFocal_high
{
    bool operator()(const high_handle &a,const high_handle &b)const{
        if((*a).focal!=(*b).focal){
            return (*a).focal>(*b).focal;
        }
        return (*a).cost>(*b).cost;
    }
};
typedef typename boost::heap::d_ary_heap<high_handle,boost::heap::arity<2>,boost::heap::mutable_<true>,boost::heap::compare<compareFocal_high>> high_focal;
class MAPFPlanner
{
public:
    std::unordered_map<string,int>deadlock;
    SharedEnvironment* env;
    int idx=0;
    int finish=0;
    int change=0;
    int conflict_finish=1;
    int big_map=0;
    std::vector<std::vector<int>>mid_target;
    high_open high_open_;
    high_focal high_focal_;
    std::atomic<bool> lock1;
    bool start=true;
    std::vector<std::vector<std::pair<int,int>>>ret;
    std::vector<std::vector<std::pair<int,int>>>ret1;
	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){
        env = new SharedEnvironment();
        //root=NULL;
        };
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);
    std::vector<Action> thread_plan();
    int getMinCost();
    std::vector<TreeNode> remove_node(TreeNode&p);
    void get_mid_target(int i);
    // Start kit dummy implementation
    //std::vector<pair<int,int>>single_agent_plan(int start,int start_direct, int end,int agentid);
    //bool is_constraint(int agentid,int x,int y,int time,std::vector<Constraint_my>&constraints);
    //int getManhattanDistance(int loc1, int loc2);
    //std::list<pair<int,int>> getNeighbors(int location, int direction,int agentid,int time);
    //bool validateMove(int loc,int loc2);
    //void update_solution();
    //void add_constraint(Constraint_my & constraint);
    //void update_cost();
    TreeNode find_best_node();
    bool hasconflict(TreeNode&node);
    bool hasconflict(std::vector<pair<int,int>>&a,std::vector<pair<int,int>>&b);
    bool hasEdgeConflict(TreeNode&node);
    bool hasEdgeConflict(std::vector<pair<int,int>>&a,std::vector<pair<int,int>>&b);
    Conflict_my getFirstConflict(TreeNode&node);
    int focal_heuristic(vector<vector<pair<int,int>>>&solutions);
    
};
typedef typename boost::heap::d_ary_heap<Node,boost::heap::arity<2>,boost::heap::mutable_<true>>open_t;
typedef typename open_t::handle_type heap_handle_t;
struct Node
{
    Node(const State_my&state,int fscore,int gscore,int focal):state(state),fscore(fscore),gscore(gscore),focal(focal){}
    
    bool operator<(const Node &other)const{
        if(fscore!=other.fscore) return fscore>other.fscore;
        else return gscore<other.gscore;
    }
    
    State_my state;
    int fscore;
    int gscore;
    int focal;
    heap_handle_t handle;
    
};
/*
struct compareFscore
{
    bool operator()(const Node*a,const Node*b) const
    {
        if((*a).fscore!=(*b).fscore)
            return a->fscore>b->fscore;
        else return a->gscore<b->gscore;
    }
};
*/




struct compareFocal
{
    bool operator()(const heap_handle_t&a,const heap_handle_t&b) const
    {
        if((*a).focal!=(*b).focal){
            return (*a).focal>(*b).focal;
        }else if((*a).fscore!=(*b).fscore){
            return (*a).fscore>(*b).fscore;
        }else return (*a).gscore<(*b).gscore;
    }
};

typedef typename boost::heap::d_ary_heap<heap_handle_t,boost::heap::arity<2>,boost::heap::mutable_<true>,boost::heap::compare<compareFocal>> focal_t;


namespace std {
template <>
struct hash<State_my> {
  size_t operator()(const State_my& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.location);
    boost::hash_combine(seed, s.timestep);
    boost::hash_combine(seed,s.orientation);
    return seed;
  }
};
}


