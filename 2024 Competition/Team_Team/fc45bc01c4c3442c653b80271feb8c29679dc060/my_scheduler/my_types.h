#pragma once
#include <limits.h>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>


#define MAX_TIMESTEP INT_MAX/2

#include "SharedEnv.h"
#include "ActionModel.h"

namespace MyPlanner{

	struct HNode
		{
			int label;
			int location;
			int direction;
			int value;
			int other;

			unsigned int priority;
			unsigned int get_priority() const { return priority; }
    		void set_priority(unsigned int p) { priority = p; }


			HNode() = default;
			HNode(int location,int direction, int value) : location(location), direction(direction), value(value) {}
			// the following is used to compare nodes in the OPEN list
			struct compare_node
			{
				bool operator()(const HNode& n1, const HNode& n2) const
				{
					return n1.value > n2.value;
				}
			};  // used by OPEN (open) to compare nodes (top of the open has min value)
		};

	struct HeuristicTable{
		std::vector<int> htable;
        std::priority_queue<HNode, std::vector<HNode>, HNode::compare_node> open;
		
		bool empty(){
			return htable.empty();
		}
	};

    struct Neighbor {
        std::vector<int> neighbor_location;
        std::vector<int> neighbor_orientation;
    };
}







