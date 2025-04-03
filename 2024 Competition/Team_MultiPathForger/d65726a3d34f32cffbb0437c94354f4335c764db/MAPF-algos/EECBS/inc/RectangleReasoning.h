#pragma once
#include "MDD.h"

//enum rectangle_strategy { NR, R, RM, DISJOINTR };

class RectangleReasoning
{
public:
	//rectangle_strategy strategy;
	double accumulated_runtime = 0;

	RectangleReasoning(const EECBS_PREFIX_Instance& instance) : instance(instance) {}

	shared_ptr<EECBS_PREFIX_Conflict> run(const vector<EECBS_PREFIX_Path*>& paths, int timestep, 
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);


private:
	const EECBS_PREFIX_Instance& instance;
	shared_ptr<EECBS_PREFIX_Conflict> findRectangleConflictByRM(const vector<EECBS_PREFIX_Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);
	shared_ptr<EECBS_PREFIX_Conflict> findRectangleConflictByGR(const vector<EECBS_PREFIX_Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);

	bool ExtractBarriers(const MDD& mdd, int loc, int timestep, int dir, int dir2, int start, int goal, int start_time, list<EECBS_PREFIX_Constraint>& B);
	bool isEntryBarrier(const EECBS_PREFIX_Constraint& b1, const EECBS_PREFIX_Constraint& b2, int dir1);
	bool isExitBarrier(const EECBS_PREFIX_Constraint& b1, const EECBS_PREFIX_Constraint& b2, int dir1);
	pair<int, int> getIntersection(const EECBS_PREFIX_Constraint& b1, const EECBS_PREFIX_Constraint& b2);
	bool blockedNodes(const vector<PathEntry>& path,
		const pair<int, int>& Rs, const pair<int, int>& Rg, int Rg_t, int dir);
	bool isCut(const EECBS_PREFIX_Constraint& b, const pair<int, int>& Rs, const pair<int, int>& Rg);

	void generalizedRectangle(const vector<PathEntry>& path1, const vector<PathEntry>& path2, const MDD& mdd1, const MDD& mdd2,
		const list<EECBS_PREFIX_Constraint>& B1, const list<EECBS_PREFIX_Constraint>& B2, int timestep,
		int& best_type, pair<int, int>& best_Rs, pair<int, int>& best_Rg);

	//Identify rectangle conflicts
	bool isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
		const pair<int, int>& g1, const pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
	bool isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2) const;// for RM

	//Classify rectangle conflicts
	int classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
		const pair<int, int>& g1, const pair<int, int>& g2);// for CR and R
	int classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2, const pair<int, int>& Rg);// for RM

	 //Compute rectangle corners
	pair<int, int> getRg(const pair<int, int>& s1, const pair<int, int>& g1, const pair<int, int>& g2);
	pair<int, int> getRs(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1);

	//Compute start and goal candidates for RM
	list<int> getStartCandidates(const EECBS_PREFIX_Path& path, const MDD& mdd, int timestep);
	list<int> getGoalCandidates(const EECBS_PREFIX_Path& path, const MDD& mdd, int timestep);
	//Compute start and goal candidates for GR
	int getStartCandidate(const EECBS_PREFIX_Path& path, int dir1, int dir2, int timestep);
	int getGoalCandidate(const EECBS_PREFIX_Path& path, int dir1, int dir2, int timestep);


	// int getRectangleTime(const EECBS_PREFIX_Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);
	bool hasNodeOnBarrier(const MDD* mdd, int y_start, int y_end, int x, int t_min, bool horizontal) const;

	bool addModifiedBarrierConstraints(int a1, int a2, const pair<int, int>& Rs, const pair<int, int>& Rg,
		const pair<int, int>& s1, const pair<int, int>& s2, int Rg_t,
		const MDD* mdd1, const MDD* mdd2,
		list<EECBS_PREFIX_Constraint>& constraint1, list<EECBS_PREFIX_Constraint>& constraint2); // for RM

	// add a horizontal modified barrier constraint
	bool addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
		int Ri_y, int Rg_y, int Rg_t, list<EECBS_PREFIX_Constraint>& constraints);

	// add a vertival modified barrier constraint
	bool addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
		int Ri_x, int Rg_x, int Rg_t, list<EECBS_PREFIX_Constraint>& constraints);

	bool blocked(const EECBS_PREFIX_Path& path, const list<EECBS_PREFIX_Constraint>& constraints);
	bool traverse(const EECBS_PREFIX_Path& path, int loc, int t);

};

