#include "ash/core.hpp"

int ash::getTraversableNeighbor(const SharedEnvironment& env, int location, int orientation, int inc)
{
	int x = location%env.cols;
	int y = location/env.cols;
	switch (orientation)
	{
		case kEast:
			x += inc;
			if (x >= env.cols)
			{
				return -1;
			}
			break;
		case kSouth:
			y += inc;
			if (y >= env.rows)
			{
				return -1;
			}
			break;
		case kWest:
			x -= inc;
			if (x < 0)
			{
				return -1;
			}
			break;
		case kNorth:
			y -= inc;
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
		case Action::W:
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
			successor.location = getTraversableNeighbor(*env, successor.location, successor.orientation, -1);
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
		case Action::W:
			break;
		default:
			return {};
	}
	return successor;
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
			successor.orientation = (successor.orientation+3)%4;
			break;
		case Action::CCR:
			successor.orientation = (successor.orientation+1)%4;
			break;
		case Action::W:
			break;
		default:
			return {};
	}
	++successor.timestep;
	return successor;
}
