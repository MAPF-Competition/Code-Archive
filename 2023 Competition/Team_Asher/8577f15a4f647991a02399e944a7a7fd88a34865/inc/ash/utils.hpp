#pragma once

#include "ActionModel.h"
#include "SharedEnv.h"

#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include <ostream>

namespace ash {

typedef std::pair<int,int> Vec2i;

enum class Orientation : int { EAST, SOUTH, WEST, NORTH };

struct AgentPosition
{
    int location;
    Orientation orientation;

    int to_flat_index() const noexcept
    {
        return location*4 + static_cast<int>(orientation);
    }

    bool operator==(const AgentPosition& other) const noexcept
    {
        return location==other.location && orientation==other.orientation;
    }

    bool operator!=(const AgentPosition& other) const noexcept
    {
        return location!=other.location || orientation!=other.orientation;
    }
};

struct TimedAgentPosition
{
    AgentPosition position;
    int time;

    bool operator==(const TimedAgentPosition& other) const noexcept
    {
        return position==other.position && time==other.time;
    }

    bool operator!=(const TimedAgentPosition& other) const noexcept
    {
        return position!=other.position || time!=other.time;
    }
};

template<class S>
bool is_valid_state(const S& state)
{
	static_assert(std::is_same_v<S,int>||
				  std::is_same_v<S,AgentPosition>||
				  std::is_same_v<S,TimedAgentPosition>);
	if constexpr(std::is_same_v<S,int>)
	{
		return state >= 0;
	}
	if constexpr(std::is_same_v<S,AgentPosition>)
	{
		return state.location >= 0;
	}
	if constexpr(std::is_same_v<S,TimedAgentPosition>)
	{
		return state.position.location >= 0;
	}
}



template <class State>
struct ActionTypeTrait;

template <>
struct ActionTypeTrait<int>
{
    typedef Orientation Value;
    static constexpr Orientation ALL_ACTIONS[] = {
        Orientation::EAST,
        Orientation::SOUTH,
        Orientation::WEST,
        Orientation::NORTH
    };
    static constexpr int NUMBER_OF_ACTIONS = sizeof(ALL_ACTIONS)/sizeof(Value);
};

template <>
struct ActionTypeTrait<AgentPosition>
{
    typedef Action Value;
    static constexpr Action ALL_ACTIONS[] = {FW, CR, CCR};
    static constexpr int NUMBER_OF_ACTIONS = sizeof(ALL_ACTIONS)/sizeof(Value);
};

template <>
struct ActionTypeTrait<TimedAgentPosition>
{
    typedef Action Value;
    static constexpr Action ALL_ACTIONS[] = {FW, CR, CCR, W};
    static constexpr int NUMBER_OF_ACTIONS = sizeof(ALL_ACTIONS)/sizeof(Value);
};

template<class S>
using ActionType = typename ActionTypeTrait<S>::Value;

template<class S>
struct ActionStatePair
{
    ActionType<S> action;
    S state;
};


template<class S>
using Plan = std::vector<ActionStatePair<S>>;

constexpr int INF = std::numeric_limits<int>::max();

constexpr Orientation ALL_ORIENTATIONS[] = {
    Orientation::EAST,
    Orientation::SOUTH,
    Orientation::WEST,
    Orientation::NORTH
};

constexpr Vec2i ORIENTATION_VECTORS[] = {
    Vec2i(1, 0),
    Vec2i(0, 1),
    Vec2i(-1, 0),
    Vec2i(0, -1)
};

inline Orientation next_orientation(Orientation orientation)
{
    return static_cast<Orientation>((static_cast<int>(orientation)+1)%4);
}

inline Orientation previous_orientation(Orientation orientation)
{
    return static_cast<Orientation>((static_cast<int>(orientation)+3)%4);
}

inline int number_of_turns(Orientation dir1, Orientation dir2)
{
    int ans = static_cast<int>(dir2) - static_cast<int>(dir1);
    ans += 4*((ans<=-2) - (ans>2));
    return ans;
}

inline int move_cost(Orientation facing_dir, Orientation move_dir)
{
    return 1 + std::abs(number_of_turns(facing_dir, move_dir));
}

inline const Vec2i& orientation_to_vector(Orientation orientation)
{
    return ORIENTATION_VECTORS[static_cast<int>(orientation)];
}

inline int ravel(const SharedEnvironment& env, int x, int y)
{
    return y*env.cols + x;
}

inline Vec2i unravel(const SharedEnvironment& env, int location)
{
    return Vec2i(location%env.cols, location/env.cols);
}

inline bool is_between(int x, int lo, int hi)
{
    return lo <= x && x <= hi;
}

int get_neighbor(const SharedEnvironment& env, int location,
        Orientation orientation, int stride = 1);

AgentPosition get_neighbor(const SharedEnvironment& env,
        const AgentPosition& position, Action action);

TimedAgentPosition get_neighbor(const SharedEnvironment& env,
        const TimedAgentPosition& tpos, Action action);

AgentPosition get_reverse_neighbor(const SharedEnvironment& env,
        const AgentPosition& position, Action action);

std::vector<std::pair<Orientation,int>> get_neighbors(
        const SharedEnvironment& env, int location);

std::vector<std::pair<Action,AgentPosition>> get_neighbors(
        const SharedEnvironment& env, const AgentPosition& tpos);

std::vector<std::pair<Action,TimedAgentPosition>> get_neighbors(
        const SharedEnvironment& env,
        const TimedAgentPosition& position);

std::vector<std::pair<Action,AgentPosition>> get_reverse_neighbors(
        const SharedEnvironment& env, const AgentPosition& position);

void simulate(std::vector<TimedAgentPosition>& state,
        const std::vector<Action>& actions);

std::string to_string(Action action);

std::string to_string(Orientation orientation);

template<class S>
std::string to_string(const Plan<S>& plan);

std::ostream& operator<<(std::ostream& out, const AgentPosition& position);

std::ostream& operator<<(std::ostream& out, const TimedAgentPosition& tpos);

} // namespace ash


namespace std {

template<>
struct hash<ash::AgentPosition>
{
    size_t operator()(const ash::AgentPosition& position) const noexcept
    {
        return position.to_flat_index();
    }
};

template<>
struct hash<ash::TimedAgentPosition>
{
    size_t operator()(const ash::TimedAgentPosition& tpos) const noexcept
    {
        size_t seed = tpos.position.to_flat_index();
        seed = seed*45'131 + tpos.time;
        return seed;
    }
};

} // namespace std

