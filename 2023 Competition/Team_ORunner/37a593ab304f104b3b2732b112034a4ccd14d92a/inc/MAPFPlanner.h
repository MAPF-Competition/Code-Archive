#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <string>
#include "Grid.h"
#include "States.h"
#include "Logger.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;
	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};
    
    virtual void initialize(int preprocess_time_limit);


    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::pair<std::list<pair<int,int>>,int> single_agent_plan(int start,int start_direct, int end, int iterationLimit, int maxPathSize, int start_manhattan_distance, int agent_idx);
    int getManhattanDistance(int loc1, int loc2, int direction);
    std::list<pair<int,int>> getNeighbors(int location, int direction, int curt, int agent_idx);
    bool validateMove(int loc,int loc2, int curt);

    // Variables for new method
    std::vector<list<pair<int,int>>> vectorOfPaths;                                     // Vector that stores the path of each robot
    std::vector<boost::unordered_map<int, int>> all_vertex_occupied_vector;             // Vector storing occupied vertices in each epoch
    std::vector<boost::unordered_map<pair<int, int>, int>> all_edge_occupied_vector;    // Vector storing occupied edges in each epoch
    int time_steps;                                                                     // Number of time steps 
    float path_length_factor;
    int minimum_iteration_limit_astar;
    int minimum_path_length;
    int time_limit_milliseconds;

    
    // Functions for new method
    void printPath(list<pair<int,int>> path);
    void printPositionFromLocation(int loc);
    void deletePath(int agent_idx, int num_epochs, int path_block_limit);
    Action giveAction(int agentIndex);


    std::vector<int> is_valid(const vector<State>& prev, const vector<Action> & actions);
    vector<State> result_states(const vector<State>& prev, const vector<Action> & action, int cols){
        vector<State> next(prev.size());
        for (size_t i = 0 ; i < prev.size(); i ++){
            next[i] = result_state(prev[i], action[i], cols);
        }
        return next;
    };

protected:
    State result_state(const State & prev, Action action, int cols)
    {
        vector<int> moves{1, cols, -1, -cols};
        int new_location = prev.location;
        int new_orientation = prev.orientation;
        if (action == Action::FW)
        {
            new_location = new_location += moves[prev.orientation];
        }
        else if (action == Action::CR)
        {
            new_orientation = (prev.orientation + 1) % 4;
      
        }
        else if (action == Action::CCR)
        {
            new_orientation = (prev.orientation - 1) % 4;
            if (new_orientation == -1)
                new_orientation = 3;
        }

        return State(new_location, prev.timestep + 1, new_orientation);
    }

};
