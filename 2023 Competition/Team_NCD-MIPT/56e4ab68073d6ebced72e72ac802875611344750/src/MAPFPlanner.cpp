#include <MAPFPlanner.h>
using namespace std::chrono;

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    flagIsMapSortation = 0;
    flagMapIsCity = 0;
    flagMapIsGame = 0;
    flagMapIsRandom = 0;
    flagMapisWarehouse = 0;
    if (isMapCity())
    {
        flagMapIsCity = true;
        if(env->num_of_agents > 12000){
            cityPlanner = new CityPlanner(env);
            cityPlanner->initialize(preprocess_time_limit);
        }
        else{
            cityPlanner2 = new CityPlanner2(env);
            cityPlanner2->initialize(preprocess_time_limit);
        }
    }
    else if (isMapSortation())
    {
        flagIsMapSortation = true;
        sortationPlanner = new SortationPlanner(env);
        sortationPlanner->initialize(preprocess_time_limit);
    }
    else if (isMapWarehouse())
    {
        flagMapisWarehouse = true;
        warehousePlanner = new WarehousePlanner(env);
        warehousePlanner->initialize(preprocess_time_limit);
    }
    else if (isMapGame())
    {
        flagMapIsGame = true;
        gamePlanner = new GamePlanner(env);
        gamePlanner->initialize(preprocess_time_limit);
        // cityPlanner2 = new CityPlanner2(env);
        // cityPlanner2->initialize(preprocess_time_limit);
    }
    else if (isMapRandom())
    {
        flagMapIsRandom = true;
        if(env->num_of_agents < 400){
            randomPlanner = new RandomPlanner(env);
            randomPlanner->initialize(preprocess_time_limit);
            // randomPlannerSIPP = new RandomPlannerSIPP(env);
            // randomPlannerSIPP->initialize(preprocess_time_limit);
        }
        else{
            randomPlanner2 = new RandomPlanner2(env);
            randomPlanner2->initialize(preprocess_time_limit);
        }
    }
}

void MAPFPlanner::plan(int time_limit, vector<Action> &actions)
{
    // if((!flagMapIsGame && !flagMapIsCity) || (flagMapIsCity && env->num_of_agents < 3000)|| (flagMapIsRandom && env->num_of_agents > 200)){
    //     return;
    // }
    // if(!flagMapIsRandom || (flagMapIsRandom && (env->num_of_agents > 600 || env->num_of_agents < 400)))
    // {
    //     return;
    // }
    // if(!flagIsMapSortation){
    //     actions = std::vector<Action>(env->num_of_agents, Action::W);
    //     return;
    // }
    if (flagIsMapSortation)
    {
        sortationPlanner->plan(time_limit, actions);
        // cityPlanner2->plan(time_limit, actions);
    }
    else if (flagMapIsCity)
    {
        if(env->num_of_agents > 12000){
            cityPlanner->plan(time_limit, actions);
        }
        else{
            cityPlanner2->plan(time_limit, actions);
        }
    }
    else if (flagMapIsGame)
    {
        gamePlanner->plan(time_limit, actions);
        // cityPlanner2->plan(time_limit, actions);
    }
    else if (flagMapisWarehouse)
    {
        warehousePlanner->plan(time_limit, actions);
        // cityPlanner2->plan(time_limit, actions);
    }
    else if (flagMapIsRandom)
    {
        if(env->num_of_agents < 400){
            randomPlanner->plan(time_limit, actions);
            // randomPlannerSIPP->plan(time_limit, actions);
        }
        else{
            randomPlanner2->plan(time_limit, actions);
        }
    }
    else
    {
        assert(0);
    }
}
bool MAPFPlanner::isMapWarehouse()
{
    auto name = env->map_name;
    if (name[0] == 'w' || name[0] == 'W')
        return true;
    return false;
}

bool MAPFPlanner::isMapSortation()
{
    auto name = env->map_name;
    if (name[0] == 's' || name[0] == 'S')
        return true;
    return false;
}

bool MAPFPlanner::isMapCity()
{
    auto name = env->map_name;
    if (name[0] == 'C' || name[0] == 'c')
        return true;
    if (name[0] == 'P' || name[0] == 'p')
        return true;
    if (env->cols == 256 && env->rows == 256)
        return true;
    return false;
}

bool MAPFPlanner::isMapRandom()
{
    auto name = env->map_name;
    if (name[0] == 'R' || name[0] == 'r')
        return true;
    if (env->cols == 32 && env->rows == 32)
        return true;
    return false;
}

bool MAPFPlanner::isMapGame()
{
    auto name = env->map_name;
    if (name[0] == 'B' || name[0] == 'b')
        return true;
    if (name[0] == 'g' || name[0] == 'G')
        return true;
    if (env->cols == 530 && env->rows == 481)
        return true;
    return false;
}
