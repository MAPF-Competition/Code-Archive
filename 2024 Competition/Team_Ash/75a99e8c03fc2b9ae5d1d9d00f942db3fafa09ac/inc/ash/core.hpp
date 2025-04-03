#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"

#include <functional>
#include <optional>
#include <ostream>

namespace ash
{
	
constexpr const char* kActionTags[] = {"FW", "CR", "CCR", "W", "NA"};
constexpr const char* kOrientationTags[] = {"EAST", "SOUTH", "WEST", "NORTH"};
	
constexpr int kEast = 0;
constexpr int kSouth = 1;
constexpr int kWest = 2;
constexpr int kNorth = 3;
	
template<class ActionType, class StateType, size_t Capacity>
class SuccessorList
{
	public:
		typedef std::pair<ActionType,StateType> Value;
		typedef Value* iterator;
		typedef const Value* const_iterator;
		
		SuccessorList() : size(0)
		{
		}
		
		iterator begin()
		{
			return &successors[0];
		}
		
		iterator end()
		{
			return &successors[size];
		}
		
		const_iterator begin() const
		{
			return &successors[0];
		}
		
		const_iterator end() const
		{
			return &successors[size];
		}
		
		Value& operator[](size_t i)
		{
			return successors[i];
		}
		
		const Value& operator[](size_t i) const
		{
			return successors[i];
		}
		
		size_t getSize() const
		{
			return size;
		}
		
		void appendSuccessor(ActionType action, const StateType& state)
		{
			successors[size++] = Value(action, state);
		}
	
	private:
		std::array<Value,Capacity> successors;
		size_t size;
};

int getTraversableNeighbor(const SharedEnvironment& env, int location, int orientation);

struct Position
{
	int location;
	int orientation;
    
    Position(int location = -1, int orientation = kEast) :
		location(location), orientation(orientation)
    {
	}
	
	size_t flatIndex() const
	{
		return location*4 + orientation;
	}
    
    bool operator==(const Position& other) const
    {
		return location == other.location &&
		       orientation == other.orientation;
	}
	
	bool operator!=(const Position& other) const
	{
		return !(*this == other);
	}
	
	struct Hasher
    {
        size_t operator()(const Position& state) const
        {
			return state.flatIndex();
        }
    };
};

std::ostream& operator<<(std::ostream& out, const Position& position);

std::string toString(const Position& position);

template<class Generator, class ActionType, class StateType, size_t numberOfActions>
SuccessorList<ActionType,StateType,numberOfActions> generateSuccessorsHelper
(
	const Generator& generator, 
	const StateType& state,
	const std::array<ActionType,numberOfActions>& availableActions
)
{
	SuccessorList<ActionType,StateType,numberOfActions> successors;
	for (auto action : availableActions)
	{
		if (auto successor = generator(state, action))
		{
			successors.appendSuccessor(action, successor.value());
		}
	}
	return successors;
}

class SuccessorGenerator
{
	public:
	
		static constexpr std::array<Action,3> availableActions{Action::FW, Action::CR, Action::CCR};
		
		SuccessorGenerator(const SharedEnvironment& env) : env(&env)
		{
		}
		
		std::optional<Position> operator()(const Position& state, Action action) const;
		
		SuccessorList<Action,Position,3> operator()(const Position& state) const
		{
			return generateSuccessorsHelper(*this, state, availableActions);
		}
	
	private:
		const SharedEnvironment* env;
};

class ParentGenerator
{
	public:
	
		static constexpr std::array<Action,3> availableActions{Action::FW, Action::CR, Action::CCR};
		
		ParentGenerator(const SharedEnvironment& env) : env(&env)
		{
		}
		
		std::optional<Position> operator()(const Position& state, Action action) const;
		
		SuccessorList<Action,Position,3> operator()(const Position& state) const
		{
			return generateSuccessorsHelper(*this, state, availableActions);
		}
	
	private:
		const SharedEnvironment* env;
};

typedef State TimedPosition;

std::string toString(const TimedPosition& timedPosition);

class TimedSuccessorGenerator
{
	public:
	
		static constexpr std::array<Action,4> availableActions{Action::FW, Action::CR, Action::CCR, Action::W};
	
		TimedSuccessorGenerator(const SharedEnvironment& env) : env(&env)
		{
		}
		
		std::optional<State> operator()(const State& state, Action action) const;
		
		SuccessorList<Action,TimedPosition,4> operator()(const TimedPosition& state) const
		{
			return generateSuccessorsHelper(*this, state, availableActions);
		}
	
	private:
		const SharedEnvironment* env;
};



} // end namespace ash
