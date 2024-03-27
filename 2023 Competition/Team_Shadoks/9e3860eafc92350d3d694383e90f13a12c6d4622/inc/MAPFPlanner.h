#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

/**
 * Position of an agent, defined by 
 * - a location: an integer encoding a pixel in the grid
 * - a direction: (0:east, 1:south, 2:west, 3:north)
 */
struct Position
{
    int location;
    int direction;
    int time;
    Position(int _location, int _direction, int _time=0):
        location(_location), direction(_direction), time(_time) {}

    Action action(const Position &prev) {
        if (location != prev.location)
            return Action::FW;
        if (direction == prev.direction) {
            return Action::W;
        }
        int incr = direction - prev.direction;
        if (incr == 1 || incr == -3) // a rotation is encoded as an integer: (0:east, 1:south, 2:west, 3:north)
            return Action::CR;
        else
            return Action::CCR;
    }
    // An integer encoding the Position
    int encoding() const {
        return location*4 + direction;
    }
    
    void print() const {
        std::cout << "{location: " << location << ", direction: " << direction << ", time: " << time << "}" << std::endl;
    }
};


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
    int getManhattanDistance(int loc1, int loc2) const;

    void plan_path(int agent, const Position &start, int end);
    std::list<Position> neighbors(int agent, const Position &position) const;
    bool collision_free(int agent, const Position &cur_pos, const Position &next) const;
    void reset_paths();
    void print_path(int i) const;

private:
    std::vector<std::vector<Position>> m_paths; // current computed paths for each agent
};
