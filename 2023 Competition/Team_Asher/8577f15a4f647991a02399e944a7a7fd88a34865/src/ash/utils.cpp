#include "ash/utils.hpp"

#include <sstream>

int ash::get_neighbor(const SharedEnvironment& env, int location,
        Orientation orientation, int stride)
{
    auto[x,y] = unravel(env, location);
    auto[dx,dy] = orientation_to_vector(orientation);
    int x_neigh = x + stride*dx;
    int y_neigh = y + stride*dy;
    int new_location = ravel(env, x_neigh, y_neigh);
    if (is_between(x_neigh, 0, env.cols-1) &&
        is_between(y_neigh, 0, env.rows-1) && !env.map[new_location])
    {
        return new_location; }
    return -1;
}

ash::AgentPosition ash::get_neighbor(
        const SharedEnvironment& env, const AgentPosition& position,
        Action action)
{
    switch (action)
    {
        case FW:
        {
            int new_location = get_neighbor(env, position.location,
                    position.orientation);
            return AgentPosition{new_location, position.orientation};
        }
        case CR:
        {
            return AgentPosition{position.location,
                next_orientation(position.orientation)};
        }
        case CCR:
        {
            return AgentPosition{position.location,
                previous_orientation(position.orientation)};
        }
        default: // W and NA
        {
            return position;
        }
    }
}

ash::TimedAgentPosition ash::get_neighbor(const SharedEnvironment& env,
        const TimedAgentPosition& tpos, Action action)
{
    return TimedAgentPosition{
        get_neighbor(env,tpos.position,action), tpos.time+1};
}

ash::AgentPosition ash::get_reverse_neighbor(
        const SharedEnvironment& env, const AgentPosition& position,
        Action action)
{
    switch (action)
    {
        case FW:
        {
            int new_location = get_neighbor(env, position.location,
                    position.orientation, -1);
            return AgentPosition{new_location, position.orientation};
        }
        case CR:
        {
            return AgentPosition{position.location,
                previous_orientation(position.orientation)};
        }
        case CCR:
        {
            return AgentPosition{position.location,
                next_orientation(position.orientation)};
        }
        default: // W and NA
        {
            return position;
        }
    }
}

std::vector<std::pair<ash::Orientation,int>> ash::get_neighbors(
        const SharedEnvironment& env, int location) {
    std::vector<std::pair<ash::Orientation,int>> neighbors;
    for (auto orientation : ActionTypeTrait<int>::ALL_ACTIONS) {
        auto neighbor = get_neighbor(env, location, orientation);
        if (neighbor >= 0) {
            neighbors.emplace_back(orientation, neighbor);
        }
    }
    return neighbors;
}

std::vector<std::pair<Action,ash::AgentPosition>> ash::get_neighbors(
        const SharedEnvironment& env, const AgentPosition& position)
{
    std::vector<std::pair<Action,AgentPosition>> neighbors;
    for (auto action : ActionTypeTrait<AgentPosition>::ALL_ACTIONS)
    {
        auto neighbor = get_neighbor(env, position, action);
        if (is_valid_state(neighbor))
        {
            neighbors.emplace_back(action, neighbor);
        }
    }
    return neighbors;
}

std::vector<std::pair<Action,ash::TimedAgentPosition>> ash::get_neighbors(
        const SharedEnvironment& env,
        const TimedAgentPosition& tpos)
{
    std::vector<std::pair<Action,TimedAgentPosition>> neighbors;
    for (auto action : ActionTypeTrait<TimedAgentPosition>::ALL_ACTIONS)
    {
        auto neighbor = get_neighbor(env, tpos, action);
        if (is_valid_state(neighbor))
        {
            neighbors.emplace_back(action, neighbor);
        }
    }
    return neighbors;

}

std::vector<std::pair<Action,ash::AgentPosition>>
ash::get_reverse_neighbors(const SharedEnvironment& env,
        const AgentPosition& position)
{
    std::vector<std::pair<Action,AgentPosition>> neighbors;
    for (auto action : ActionTypeTrait<AgentPosition>::ALL_ACTIONS)
    {
        auto neighbor = get_reverse_neighbor(env, position, action);
        if (is_valid_state(neighbor))
        {
            neighbors.emplace_back(action, neighbor);
        }
    }
    return neighbors;
}

std::string ash::to_string(Orientation orientation)
{
    static const char* ORIENTATION_LABELS[] = {
        "EAST", "SOUTH", "WEST", "NORTH"
    };
    return ORIENTATION_LABELS[static_cast<int>(orientation)];
}

std::string ash::to_string(Action action)
{
    static const char* ACTION_LABELS[] = {"FW", "CR", "CCR", "W", "NA"};
    return ACTION_LABELS[static_cast<int>(action)];
}

template<class S>
std::string ash::to_string(const Plan<S>& plan)
{
    std::ostringstream out;
    out << "{start=" << plan.front().state << ", plan=[";
    for (size_t i = 1; i < plan.size(); ++i)
    {
        if (i != 1)
        {
            out << ", ";
        }
        out << to_string(plan[i].action);
    }
    out << "]}";
    return out.str();
}

template std::string ash::to_string(const Plan<int>& plan);
template std::string ash::to_string(const Plan<AgentPosition>& plan);
template std::string ash::to_string(const Plan<TimedAgentPosition>& plan);

std::ostream& ash::operator<<(std::ostream& out, const AgentPosition& position)
{
    return out << "{location=" << position.location << ", orientation="
               << to_string(position.orientation) << '}';
}

std::ostream& ash::operator<<(std::ostream& out, const TimedAgentPosition& tpos)
{
    return out << "{position=" << tpos.position << ", timestep="
               << tpos.time << '}';
}

