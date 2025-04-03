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

namespace DefaultPlanner{

	
	enum DONE{
		NOT_DONE,
		DONE
	};

	struct Int4{
		int d[4];
	};

	struct Int2{
		int d[4];
	};

	struct DCR{
		int loc;						
		int state;
	};

	typedef std::vector<int> Traj;

	struct PIBT_C{
		int location;
		int heuristic;
		int orientation;  // 0:east, 1:south, 2:west, 3:north
		int tie_breaker;

		//constructor
		PIBT_C(int location, int heuristic, int orientation, int tie_breaker):
			location(location), heuristic(heuristic), orientation(orientation), tie_breaker(tie_breaker) {};
	};

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
					return n1.value < n2.value;
				}
			};  // used by OPEN (open) to compare nodes (top of the open has min f-val, and then highest g-val)
		};

	struct HeuristicTable{
		std::vector<int> htable; 		// goal locations
		std::deque<HNode> open;	 		// OPEN path, the list of nodes to be expanded 
		
		bool empty(){
			return htable.empty();
		}
	};



	struct d2p{
		int label = 0;					// 	
		int id = -1;					//	location id
		int cost = -1;					// 	cost from source to nearest path vertice
		int togo = -1;					//	cost from nearest path vertice path to goal

		d2p(int label, int id, int cost, int togo):label(label), id(id), cost(cost), togo(togo){};
	};

	struct Dist2Path{
		int label = 0;
		std::vector<d2p> dist2path; 	// distance to path
		std::deque<d2p> open;			// open path nodes to guided path
		
		bool empty(){
			return dist2path.empty();
		}

	};

	typedef std::vector<std::vector<int>> Neighbors;

}







