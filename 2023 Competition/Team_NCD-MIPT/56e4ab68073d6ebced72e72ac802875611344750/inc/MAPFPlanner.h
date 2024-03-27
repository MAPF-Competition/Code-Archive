#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "skeleton.h"
#include "ActionModel.h"
#include "CityPlanner.h"
#include "CityPlanner2.h"
#include "GamePlanner.h"
#include "WarehousePlanner.h"
#include "SortationPlanner.h"
#include "RandomPlanner.h"
#include "RandomPlannerSIPP.h"
#include "RandomPlanner2.h"
using namespace std;

class MAPFPlanner
{
public:
    SharedEnvironment *env;

    MAPFPlanner(SharedEnvironment *env) : env(env){};
    MAPFPlanner() { env = new SharedEnvironment(); };
    CityPlanner *cityPlanner;
    CityPlanner2 *cityPlanner2;
    WarehousePlanner *warehousePlanner;
    SortationPlanner *sortationPlanner;
    RandomPlanner *randomPlanner;
    RandomPlannerSIPP *randomPlannerSIPP;
    RandomPlanner2 *randomPlanner2;
    GamePlanner *gamePlanner;

    virtual ~MAPFPlanner() {
        if(isMapCity()){
            if(env->num_of_agents > 12000){
                delete cityPlanner;
            }
            else{
                delete cityPlanner2;
            }
        }
        else if(isMapGame()){
            // delete cityPlanner2;
            delete gamePlanner;
        }
        else if(isMapWarehouse()){
            delete warehousePlanner;
        }
        else if(isMapSortation()){
            delete sortationPlanner;
        }
        else if(isMapRandom()){
            if(env->num_of_agents < 400){
                delete randomPlanner;
                // delete randomPlannerSIPP;
            }
            else{
                delete randomPlanner2;
            }
        }
        delete env;
    };

    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<Action> &plan);
    bool isMapWarehouse();
    bool isMapSortation();
    bool isMapCity();
    bool isMapRandom();
    bool isMapGame();
    bool flagMapisWarehouse, flagIsMapSortation, flagMapIsRandom, flagMapIsGame, flagMapIsCity;
};
