#pragma once
#include "ReservationTable.h"
#include "Instance.h"
#include "SingleAgentSolver.h"

// enum corridor_strategy { NC, C, DISJOINTC };

class CorridorReasoning
{
public:
	// corridor_strategy strategy;
	double accumulated_runtime = 0;

	CorridorReasoning(const vector<SingleAgentSolver*>& search_engines,
		const vector<ConstraintTable>& initial_constraints):
		search_engines(search_engines), initial_constraints(initial_constraints) {}
	
	shared_ptr<EECBS_PREFIX_Conflict> run(const shared_ptr<EECBS_PREFIX_Conflict>& conflict,
		const vector<EECBS_PREFIX_Path*>& paths, const HLNode& node);

private:
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;

	shared_ptr<EECBS_PREFIX_Conflict> findCorridorConflict(const shared_ptr<EECBS_PREFIX_Conflict>& conflict,
		const vector<EECBS_PREFIX_Path*>& paths, const HLNode& node);
	int findCorridor(const shared_ptr<EECBS_PREFIX_Conflict>& conflict,
		const vector<EECBS_PREFIX_Path*>& paths, int endpoints[], int endpoints_time[]); // return the length of the corridor 
	int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t);
	int getExitingTime(const std::vector<PathEntry>& path, int t);
	int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


	// int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size);
	// int getBypassLengthByAStar(int start, int end, std::pair<int, int> blocked,
	//	const ConstraintTable& constraint_table, int upper_bound);
	// int getBypassLengthBySIPP(int start, int end, std::pair<int, int> blocked,
	//	 ReservationTable& reservation_table, int upper_bound);

	bool blocked(const EECBS_PREFIX_Path& path, const EECBS_PREFIX_Constraint& constraint);

};


