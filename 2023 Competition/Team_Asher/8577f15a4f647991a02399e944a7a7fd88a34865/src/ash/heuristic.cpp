#include "ash/heuristic.hpp"


ash::WeakHeuristic::WeakHeuristic(const SharedEnvironment& env) :
    env(&env)
{
}

int ash::WeakHeuristic::operator()(const AgentPosition& src, int dst) const
{
    auto[x1,y1] = unravel(*env, src.location);
    auto[x2,y2] = unravel(*env, dst);
    auto[ori_dx,ori_dy] = orientation_to_vector(src.orientation); 
    int dx = x2 - x1;
    int dy = y2 - y1;
    int manhattan_dist = std::abs(dx) + std::abs(dy);
    int min_num_of_turns = (dx!=0 && dy!=0) + (ori_dx*dx<0 || ori_dy*dy<0);
    return manhattan_dist + min_num_of_turns;
}

ash::StrongHeuristic::StrongHeuristic(const Preprocessor& preprocessor) :
    preprocessor(&preprocessor)
{
}

int ash::StrongHeuristic::operator()(const AgentPosition& src, int dst) const
{
    return preprocessor->get_shortest_distance(src, dst);
}

