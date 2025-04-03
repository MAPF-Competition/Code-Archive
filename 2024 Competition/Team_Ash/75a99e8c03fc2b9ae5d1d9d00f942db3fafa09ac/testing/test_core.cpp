#include "ash/core.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>

namespace Catch {
	
template<>
struct StringMaker<std::pair<Action,ash::Position>>
{
	static std::string convert(const std::pair<Action,ash::Position>& value ) {
		std::ostringstream oss;
		oss << '<' << ash::kActionTags[(int)(value.first)] << ',' << value.second << '>';
		return oss.str();
	}
};

template<>
struct StringMaker<std::pair<Action,ash::TimedPosition>>
{
	static std::string convert(const std::pair<Action,ash::TimedPosition>& value ) {
		std::ostringstream oss;
		oss << '<' << ash::kActionTags[(int)(value.first)] << ',' << ash::toString(value.second) << '>';
		return oss.str();
	}
};

}

void populateEnvironment(SharedEnvironment& env)
{
	// .....
	// ...#.
	// .....
	// ...#.
	env.rows = 4;
	env.cols = 5;
	env.map = std::vector<int>{
		0,0,0,0,0,
		0,0,0,1,0,
		0,0,0,0,0,
		0,0,0,1,0
	};
}

TEST_CASE( "SuccessorList works as intended", "[SuccessorList]" )
{
	
	ash::SuccessorList<Action,State,4> successors;
	REQUIRE( successors.getSize() == 0 );
	REQUIRE( successors.end()-successors.begin() == 0 );
	
	SECTION( "testing adding new successors" )
	{
		successors.appendSuccessor(W, State(0, 0, 1));
		successors.appendSuccessor(CR, State(0, 1, 1));
		successors.appendSuccessor(CCR, State(0, 3, 1));
		successors.appendSuccessor(FW, State(1, 0, 1));
		REQUIRE( successors.getSize() == 4 );
		REQUIRE( successors[0].first == W );
		REQUIRE( successors[0].second == State(0, 0, 1) );
		REQUIRE( successors[1].first == CR );
		REQUIRE( successors[1].second == State(0, 1, 1) );
		REQUIRE( successors[2].first == CCR );
		REQUIRE( successors[2].second == State(0, 3, 1) );
		REQUIRE( successors[3].first == FW );
		REQUIRE( successors[3].second == State(1, 0, 1) );
	}
	
	SECTION( "testing iterators" )
	{
		successors.appendSuccessor(W, State(0, 0, 1));
		successors.appendSuccessor(CR, State(0, 1, 1));
		successors.appendSuccessor(CCR, State(0, 3, 1));
		successors.appendSuccessor(FW, State(1, 0, 1));
		int idx = 0;
		for (const auto&[action, state] : successors)
		{
			REQUIRE( successors[idx].first == action );
			REQUIRE( successors[idx].second == state );
			++idx;
		}
	}
	
}

TEST_CASE("getTraversableNeighbor in various scenarios", "[getTraversableNeighbor]")
{
	SharedEnvironment env;
	populateEnvironment(env);
	SECTION( "all four neighbors available" )
	{
		REQUIRE( ash::getTraversableNeighbor(env, 12, ash::kEast) == 13 );
		REQUIRE( ash::getTraversableNeighbor(env, 12, ash::kNorth) == 7 );
		REQUIRE( ash::getTraversableNeighbor(env, 12, ash::kWest) == 11 );
		REQUIRE( ash::getTraversableNeighbor(env, 12, ash::kSouth) == 17 );
	}
	SECTION( "some neighbors blocked by obstacles" )
	{
		REQUIRE( ash::getTraversableNeighbor(env, 13, ash::kEast) == 14 );
		REQUIRE( ash::getTraversableNeighbor(env, 13, ash::kNorth) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 13, ash::kWest) == 12 );
		REQUIRE( ash::getTraversableNeighbor(env, 13, ash::kSouth) < 0 );
	}
	SECTION( "in each of the 4 edges (but not a corner)" )
	{
		REQUIRE( ash::getTraversableNeighbor(env, 14, ash::kEast) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 14, ash::kNorth) == 9 );
		REQUIRE( ash::getTraversableNeighbor(env, 14, ash::kWest) == 13 );
		REQUIRE( ash::getTraversableNeighbor(env, 14, ash::kSouth) == 19 );
		REQUIRE( ash::getTraversableNeighbor(env, 10, ash::kEast) == 11 );
		REQUIRE( ash::getTraversableNeighbor(env, 10, ash::kNorth) == 5 );
		REQUIRE( ash::getTraversableNeighbor(env, 10, ash::kWest) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 10, ash::kSouth) == 15 );
		REQUIRE( ash::getTraversableNeighbor(env, 2, ash::kEast) == 3 );
		REQUIRE( ash::getTraversableNeighbor(env, 2, ash::kNorth) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 2, ash::kWest) == 1 );
		REQUIRE( ash::getTraversableNeighbor(env, 2, ash::kSouth) == 7 );
		REQUIRE( ash::getTraversableNeighbor(env, 16, ash::kEast) == 17 );
		REQUIRE( ash::getTraversableNeighbor(env, 16, ash::kNorth) == 11 );
		REQUIRE( ash::getTraversableNeighbor(env, 16, ash::kWest) == 15 );
		REQUIRE( ash::getTraversableNeighbor(env, 16, ash::kSouth) < 0 );
	}
	SECTION( "in each of the 4 corners" )
	{
		REQUIRE( ash::getTraversableNeighbor(env, 0, ash::kEast) == 1 );
		REQUIRE( ash::getTraversableNeighbor(env, 0, ash::kNorth) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 0, ash::kWest) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 0, ash::kSouth) == 5 );
		REQUIRE( ash::getTraversableNeighbor(env, 4, ash::kEast) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 4, ash::kNorth) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 4, ash::kWest) == 3 );
		REQUIRE( ash::getTraversableNeighbor(env, 4, ash::kSouth) == 9 );
		REQUIRE( ash::getTraversableNeighbor(env, 15, ash::kEast) == 16 );
		REQUIRE( ash::getTraversableNeighbor(env, 15, ash::kNorth) == 10 );
		REQUIRE( ash::getTraversableNeighbor(env, 15, ash::kWest) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 15, ash::kSouth) < 5 );
		REQUIRE( ash::getTraversableNeighbor(env, 19, ash::kEast) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 19, ash::kNorth) == 14 );
		REQUIRE( ash::getTraversableNeighbor(env, 19, ash::kWest) < 0 );
		REQUIRE( ash::getTraversableNeighbor(env, 19, ash::kSouth) < 0 );
	}
}

TEST_CASE("Testing generation of successors", "[SuccessorGenerator]")
{
	using Catch::Matchers::UnorderedRangeEquals;
	
	SharedEnvironment env;
	populateEnvironment(env);
	ash::SuccessorGenerator successorGenerator(env);
	
	SECTION( "all successors available (facing east)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(7, ash::kEast)},
			{CR, ash::Position(6, ash::kSouth)},
			{CCR, ash::Position(6, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing south)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(11, ash::kSouth)},
			{CR, ash::Position(6, ash::kWest)},
			{CCR, ash::Position(6, ash::kEast)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing west)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(5, ash::kWest)},
			{CR, ash::Position(6, ash::kNorth)},
			{CCR, ash::Position(6, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing north)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(1, ash::kNorth)},
			{CR, ash::Position(6, ash::kEast)},
			{CCR, ash::Position(6, ash::kWest)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing north)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(13, ash::kEast)},
			{CCR, ash::Position(13, ash::kWest)}
		};
		auto successors = successorGenerator(ash::Position(13, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing east)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(4, ash::kSouth)},
			{CCR, ash::Position(4, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::Position(4, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing south)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(17, ash::kWest)},
			{CCR, ash::Position(17, ash::kEast)}
		};
		auto successors = successorGenerator(ash::Position(17, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing west)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(15, ash::kNorth)},
			{CCR, ash::Position(15, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::Position(15, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
}

TEST_CASE("Testing generation of parents", "[ParentGenerator]")
{
	using Catch::Matchers::UnorderedRangeEquals;
	
	SharedEnvironment env;
	populateEnvironment(env);
	ash::ParentGenerator successorGenerator(env);
	
	SECTION( "all parents available (facing west)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(7, ash::kWest)},
			{CR, ash::Position(6, ash::kSouth)},
			{CCR, ash::Position(6, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all parents available (facing north)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(11, ash::kNorth)},
			{CR, ash::Position(6, ash::kWest)},
			{CCR, ash::Position(6, ash::kEast)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all parents available (facing east)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(5, ash::kEast)},
			{CR, ash::Position(6, ash::kNorth)},
			{CCR, ash::Position(6, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all parents available (facing south)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{FW, ash::Position(1, ash::kSouth)},
			{CR, ash::Position(6, ash::kEast)},
			{CCR, ash::Position(6, ash::kWest)}
		};
		auto successors = successorGenerator(ash::Position(6, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing south)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(13, ash::kEast)},
			{CCR, ash::Position(13, ash::kWest)}
		};
		auto successors = successorGenerator(ash::Position(13, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing west)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(4, ash::kSouth)},
			{CCR, ash::Position(4, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::Position(4, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing north)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(17, ash::kWest)},
			{CCR, ash::Position(17, ash::kEast)}
		};
		auto successors = successorGenerator(ash::Position(17, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing east)" )
	{
		std::vector<std::pair<Action,ash::Position>> expected{
			{CR, ash::Position(15, ash::kNorth)},
			{CCR, ash::Position(15, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::Position(15, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
}

TEST_CASE("Testing generation of successors with timed positions", "[TimedSuccessorGenerator]")
{
	using Catch::Matchers::UnorderedRangeEquals;
	
	SharedEnvironment env;
	populateEnvironment(env);
	ash::TimedSuccessorGenerator successorGenerator(env);
	
	SECTION( "all successors available (facing east)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{FW, ash::TimedPosition(7, 2, ash::kEast)},
			{W, ash::TimedPosition(6, 2, ash::kEast)},
			{CR, ash::TimedPosition(6, 2, ash::kSouth)},
			{CCR, ash::TimedPosition(6, 2, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::TimedPosition(6, 1, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing south)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{FW, ash::TimedPosition(11, 2, ash::kSouth)},
			{W, ash::TimedPosition(6, 2, ash::kSouth)},
			{CR, ash::TimedPosition(6, 2, ash::kWest)},
			{CCR, ash::TimedPosition(6, 2, ash::kEast)}
		};
		auto successors = successorGenerator(ash::TimedPosition(6, 1, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing west)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{FW, ash::TimedPosition(5, 2, ash::kWest)},
			{W, ash::TimedPosition(6, 2, ash::kWest)},
			{CR, ash::TimedPosition(6, 2, ash::kNorth)},
			{CCR, ash::TimedPosition(6, 2, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::TimedPosition(6, 1, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "all successors available (facing north)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{FW, ash::TimedPosition(1, 2, ash::kNorth)},
			{W, ash::TimedPosition(6, 2, ash::kNorth)},
			{CR, ash::TimedPosition(6, 2, ash::kEast)},
			{CCR, ash::TimedPosition(6, 2, ash::kWest)}
		};
		auto successors = successorGenerator(ash::TimedPosition(6, 1, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing north)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{CR, ash::TimedPosition(13, 2, ash::kEast)},
			{CCR, ash::TimedPosition(13, 2, ash::kWest)},
			{W, ash::TimedPosition(13, 2, ash::kNorth)}
		};
		auto successors = successorGenerator(ash::TimedPosition(13, 1, ash::kNorth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing east)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{CR, ash::TimedPosition(4, 2, ash::kSouth)},
			{CCR, ash::TimedPosition(4, 2, ash::kNorth)},
			{W, ash::TimedPosition(4, 2, ash::kEast)}
		};
		auto successors = successorGenerator(ash::TimedPosition(4, 1, ash::kEast));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing south)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{CR, ash::TimedPosition(17, 2, ash::kWest)},
			{CCR, ash::TimedPosition(17, 2, ash::kEast)},
			{W, ash::TimedPosition(17, 2, ash::kSouth)}
		};
		auto successors = successorGenerator(ash::TimedPosition(17, 1, ash::kSouth));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
	
	SECTION( "FW not available (facing west)" )
	{
		std::vector<std::pair<Action,ash::TimedPosition>> expected{
			{CR, ash::TimedPosition(15, 2, ash::kNorth)},
			{CCR, ash::TimedPosition(15, 2, ash::kSouth)},
			{W, ash::TimedPosition(15, 2, ash::kWest)}
		};
		auto successors = successorGenerator(ash::TimedPosition(15, 1, ash::kWest));
		REQUIRE_THAT(successors, UnorderedRangeEquals(expected));
	}
}
