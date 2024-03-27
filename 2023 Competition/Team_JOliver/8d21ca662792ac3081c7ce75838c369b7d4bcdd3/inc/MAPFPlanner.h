#pragma once
#include <ctime>
#include <chrono>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;
    struct NormalNode
    {
        int location;
        std::set<int> directions;
        std::set<int> lockedTimeSteps;
        int agentNumber;
        bool isAgentOnNode;
        bool isDeadEnd;
        bool isOneWay;
        NormalNode* rightNode;
        NormalNode* bottomNode;
        NormalNode* leftNode;
        NormalNode* topNode;
        NormalNode(int _location):
            location(_location){}
        NormalNode(int _location, int _direction):
            location(_location), directions(std::set<int>{_direction}){}
        NormalNode(int _location, std::set<int> _directions):
            location(_location), directions(_directions){}
    };
    
    struct Cluster
    {
        int clusterId;
        int row;
        int col;
        int horizontalOrigin;
        int verticalOrigin;
        int width;
        int height;
        vector<int> nodes;
        vector<int> parents;

        Cluster(int _id, int _row, int _col, int _hOrigin, int _vOrigin, int _width, int _height):
            clusterId(_id), row(_row), col(_col), horizontalOrigin(_hOrigin), verticalOrigin(_vOrigin), width(_width), height(_height) {}

        void addParent(int n) {parents.push_back(n);}
        int getNode(int n) {return nodes[n];}
    };

    struct Entrance
    {
        int id;
        int center1Row;
        int center1Col;
        int cluster1Id;
        int cluster2Id;
        int center1Id;
        int center2Id;
        int row;
        int col;
        int length;
        int orientation; // 0 -> HORIZONTAL, 1 -> VERTICAL

        Entrance(int _id, int _center1Row, int _center1Col, int _cl1Id, int _cl2Id, int _center1Id, int _center2Id, int _row, int _col, int _length, int _orientation):
            id(_id), center1Row(_center1Row), center1Col(_center1Col), cluster1Id(_cl1Id), cluster2Id(_cl2Id), center1Id(_center1Id), center2Id(_center2Id),
            row(_row), col(_col), length(_length), orientation(_orientation) {}
    };


	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};

    
    virtual void initialize(int preprocess_time_limit);

    //static bool agentOrder(int& i, int& j);
    void sortAgents();
    void printMap();
    void printRunningMap();
    void printReducedMap();
    void printReducedMapWaypoints();

    void initNormalNodesMap();
    void connectNeighborNodes();
    void limitedOneWayWarehouseSetup();
    void switchingLimitedOneWayWarehouseSetup();
    void fullOneWay();

    int getLocation(int row, int column);
    int getNode(int row, int column);

    void reduceMapStart();
    void reduceMapUpdate();
    bool reduceMap(bool keepSalient);
    void reduceMapWaypointsStart();
    bool reduceReduceMapWaypoints(int distance);
    bool areNeighboursTraversable(int location);
    bool isNorthEastEmpty(int location, int x, int y);
    bool isSouthEastEmpty(int location, int x, int y);
    bool isNorthWestEmpty(int location, int x, int y);
    bool isSouthWestEmpty(int location, int x, int y);
    int countReducedVerticalNeighbours(int location);
    int countReducedHorizontalNeighbours(int location);



    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    //multiple plannings
    void planWithSingleIteration(int time_limit, std::vector<Action> & plan);
    void planWithMultipleIterationConflictChecks(int time_limit, std::vector<Action> & plan);
    void planWithMultipleIterationConflictChecksOnReducedMap(int time_limit, std::vector<Action> & plan);
    void planWithMultipleIterationConflictChecksOnReducedMapWaypoints(int time_limit, std::vector<Action> & actions);
    void planWithMultipleIterationConflictChecksOnReducedMapWaypointsImprovedConflictResolution(int time_limit, std::vector<Action> & actions);

    // Start kit dummy implementation
    std::list<pair<int,pair<int,int>>>single_agent_plan(int start,int start_direct, int end);
    std::list<pair<int,pair<int,int>>>spaceTimeSingleAgentAstar(int start,int start_direct, int end, std::set<std::tuple<int,int,int>> & reservation);
    list<pair<int,pair<int,int>>> spaceTimeSingleAgentAstarWithProximity(int start, int start_direct, int end, std::set<std::tuple<int,int,int>> &reservation, int proximity);
    std::list<int> reducedMapAStar(int start, int end, int step);
    std::list<int> reducedMapWaypointsAStar(int start, int end, int step);
    int BFStoReducedMap(int start);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction, bool isRestricted);
    std::list<pair<int,int>> getNeighborsWithRestriction(int location, int direction, int g);
    std::list<int> getPlainNeighbors(int location);
    std::list<int> getPlainNeighborsOnReducedMap(int location);
    pair<int, std::set<int> >getNodeNeighborCountWithAvailableDirections(int location);
    bool validateMove(int loc,int loc2);
    bool validateMoveOnReducedMap(int loc1, int loc2);

    NormalNode* getNormalNodeByLocation(int location);
    pair<bool, int> isOtherAgentTakenPlace(unordered_map<int,int> &locationTakenByAgent, int location);
    void clearPath(list<pair<int,pair<int,int>>> &path);
    void debug(int debugTimeStep, int debugLocation);
    pair<bool, std::vector<bool>> findConflicts();
    std::tuple<bool, std::unordered_map<int,bool>, std::set<int>> findConflicts(std::set<int> agentsToCheck);
    void updateAgentMap();

    NormalNode* normalNode;
    vector<int> heatMap;
    vector<int> reducedMap;
    vector<int> l0reducedMap;
    vector<int> l1reducedMap;
    vector<int> l2reducedMap;
    vector<unordered_map<int,int>> reducedMapWaypoints;
    vector<unordered_map<int,int>> l1reducedMapWaypoints;
    vector<unordered_map<int,int>> l2reducedMapWaypoints;
    unordered_map<int,NormalNode*> allNormalNodes;
    unordered_map<int,int> agentMap;
    std::set<std::tuple<int,int,int>> reservation;
    vector< vector<pair<int, int> > > waypointDestinations;
    vector<int> simpleWaypointDestinations;
    vector<int> agentPriority;
    unordered_map<int,int> agentEmptySpaceMap;

    //HPA
    vector<Cluster> clusters;
    vector<Entrance> entrances;

    vector<list<int>> waypoints;

    int selfTime;

    //for each agent
    // .first -> TimeStep
    // .second.first -> location
    // .second.second -> orinetation
    std::vector<list<pair<int,pair<int,int>>>> paths ;
    //std::vector<int> currentGoalLocations;

};
