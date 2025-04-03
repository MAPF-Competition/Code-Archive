#include "ash/core.hpp"

#include <sstream>

int ash::getTraversableNeighbor(const SharedEnvironment& env, int location, int orientation)
{
	int x = location%env.cols;
	int y = location/env.cols;
	switch (orientation)
	{
		case kEast:
			x += 1;
			if (x >= env.cols)
			{
				return -1;
			}
			break;
		case kSouth:
			y += 1;
			if (y >= env.rows)
			{
				return -1;
			}
			break;
		case kWest:
			x -= 1;
			if (x < 0)
			{
				return -1;
			}
			break;
		case kNorth:
			y -= 1;
			if (y < 0)
			{
				return -1;
			}
			break;
		default:
			return -1;
	}
	int newLocation = y*env.cols + x;
	return env.map[newLocation]? -1 : newLocation;
}

std::ostream& ash::operator<<(std::ostream& out, const Position& position)
{
	return out << '(' << position.location << ',' << kOrientationTags[position.orientation] << ')';
}

std::string ash::toString(const Position& position)
{
	std::ostringstream oss;
	oss << position;
	return oss.str();
}

std::optional<ash::Position> ash::SuccessorGenerator::operator()(const Position& state, Action action) const
{
	Position successor = state;
	switch (action)
	{
		case Action::FW:
			successor.location = getTraversableNeighbor(*env, successor.location, successor.orientation);
			if (successor.location == -1)
			{
				return {};
			}
			break;
		case Action::CR:
			successor.orientation = (successor.orientation+1)%4;
			break;
		case Action::CCR:
			successor.orientation = (successor.orientation+3)%4;
			break;
		default:
			return {};
	}
	return successor;
}

std::optional<ash::Position> ash::ParentGenerator::operator()(const Position& state, Action action) const
{
	Position successor = state;
	switch (action)
	{
		case Action::FW:
			successor.location = getTraversableNeighbor(*env, successor.location, (successor.orientation+2)%4);
			if (successor.location == -1)
			{
				return {};
			}
			break;
		case Action::CR:
			successor.orientation = (successor.orientation+3)%4;
			break;
		case Action::CCR:
			successor.orientation = (successor.orientation+1)%4;
			break;
		default:
			return {};
	}
	return successor;
}

std::string ash::toString(const TimedPosition& position)
{
	std::ostringstream oss;
	oss << '(' << position.location << ',' << kOrientationTags[position.orientation] << ',' << position.timestep << ')';
	return oss.str();
}

std::optional<State> ash::TimedSuccessorGenerator::operator()(const State& state, Action action) const
{
	State successor = state;
	switch (action)
	{
		case Action::FW:
			successor.location = getTraversableNeighbor(*env, successor.location, successor.orientation);
			if (successor.location == -1)
			{
				return {};
			}
			break;
		case Action::CR:
			successor.orientation = (successor.orientation+1)%4;
			break;
		case Action::CCR:
			successor.orientation = (successor.orientation+3)%4;
			break;
		case Action::W:
			break;
		default:
			return {};
	}
	++successor.timestep;
	return successor;
}
