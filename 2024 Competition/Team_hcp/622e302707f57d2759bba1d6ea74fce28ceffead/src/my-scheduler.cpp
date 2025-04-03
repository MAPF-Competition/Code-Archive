#include "my-scheduler.h"
#include <map>
#include <algorithm>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <cmath>
namespace MyScheduler{
struct ProcessedTask
{
    int task_id = -1;
    double temperature = 0;
    bool second_class = false;
    bool game_left = false;
    bool game_right = false;
    bool city_middle_north = false;
    bool city_middle_south = false;
    bool city_downright_north = false;
    bool city_downright_south = false;
    int makespan = 0;
    int offset;
    int starter_makespan = 0;
    int task_front = -1;
    int started = 0;
    int started_makespan = 0;
    std::vector<int> locations = {};
    std::vector<int> stays = {};
};
struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};
struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};
const int BUFFER_SIZE = 30;
const int MAP_PRINT_WIDTH = 300;
const int MAP_PRINT_OFFSET = 0;
const int INCREMENT = 0;
const int EAST = 0;
const int SOUTH = 1;
const int WEST = 2;
const int NORTH = 3;
const int MAX_STEPS = 6000;
const int MAX_HEATMAPSIZE = 300000;
const int MAX_MAPSIZE = 262144;
const int MAX_PATH = 262144;
static std::unordered_map<int,ProcessedTask> waiting_tasks;
static std::set<int> free_agents;
static std::unordered_map<int,ProcessedTask> running_tasks;
static std::deque< std::pair <int,int> > waitingTasksSorted;
static std::queue<Task> queued_tasks;
static std::vector<int> freeAgentMap;
static std::vector<int> workingAgentMap;
static std::vector<int> bases;
static std::vector<int> agentMap;
static std::vector<std::vector<int>> taskMap;
static std::vector<int> search_path = {};
static std::vector<int> second_class_histogram = {};
static int current_step=-1;
static int heat_map_cols;
static int heat_map_rows;
static int heat_map_size;
static int heat_map_capacity_percentage = 50;
static std::vector<int> heatMap;
static std::vector<int> heatAgentMap;
static std::vector<int> heatMapCapacity;
static std::vector<double> heatMapWP;
static std::vector<std::vector<int>> heatTimeMap;
static std::vector<std::unordered_map<int,int>*> heatFlowMap;
static bool heat_is_used = false;
static bool fullheat_is_used = false;
static bool heatavg_is_used = false;
static bool heat_time_is_used = false;
static bool heat_filter_is_used = false;
static bool heatflow_is_used = false;
static bool heatflow_filter_is_used = false;
static int heat_filter_divident = 2;
static bool multi_criteria_is_used = false;
static bool agent_heat_is_used = false;
static bool agent_heat_filter_is_used = false;
static bool heat_limit_filter_is_used = false;
static bool heatflow_limit_filter_is_used = false;
static bool pibt_is_used = false;
static bool offset_is_used = false;
static int saturation_limit=0;
static int heat_limit;
static int heatflow_limit;
static int heat_limit_increase = 0;
static int dynamic_heat_limit;
static double heat_coeff = 0.5;
static double heatMap_coeff = 1;
static double heatAgentMap_coeff = 0;
static double measured_heat_coeff = 0;
static double makespan_coeff = 1;
static int heatgridsize = 10;
static int heatselection = 3;
double avg_makespan = -1;
double total_makespan = 0;
static int makespan_bid = 0;
static double avg_heat = 0;
static int total_heat = 0;
static int heat_bid = 0;
static std::vector<int> reducedMap;
static std::vector<std::unordered_map<int,int>> reducedMapWaypoints;
static std::vector<int> widthMap;
static std::vector<int> prev_task_schedule;
static int curr_update_timestep, prev_update_timestep;
static int starter_agents = 0;
static int previous_starter_agents = 0;
static int starter_tasks = 0;
static int working_agents = 0;
static int bottleneck_drops = 0;
static int starter_agent_limit = 0;
static int time_limit_correction = 0;
static int bfstotask_extra = 0;
static bool bfswithturn = false;
static bool singleagent = false;
static int waypoint_step = 24;
static int gridsize = 24;
static bool manhattan = false;
static bool random_map = false;
static int random_long_makespan = 500;
static bool city_map = false;
static int city_long_makespan = 150;
static bool game_map = false;
static int game_long_makespan = 500;
static int game_long_count = 0;
static int game_long_limit = 0;
static int game_long_starter_correction = 0;
static int long_makespan = 0;
static int origo;
static bool warehouse_map = false;
static bool warehouse_vertical = false;
static int warehouse_long_makespan = 1000;
static bool starter_is_used = true;
static bool filter_bottleneck = false;
static bool filter_bottleneck_with_heat = false;
static bool filter_timed_bottleneck = false;
static int timed_window = 10;
static bool second_class_is_used = false;
static int second_class_counter = 0;
static int second_class_limit = 100;
static int second_class_cycle_counter = 0;
static int second_class_cycle_limit = 10;
static int second_class_cycle_period = 20;
static int second_class_semafor = 0;
static int task_counter = 0;
static int task_second_counter = 0;
static int task_second_finished = 0;
static int counter_100 = 0;
static int counter_200 = 0;
static int counter_300 = 0;
static int counter_400 = 0;
static int counter_500 = 0;
static int counter_600 = 0;
static int counter_700 = 0;
static int counter_800 = 0;
static int counter_900 = 0;
static int counter_1000 = 0;
static int last_checked_agent = 0;
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    cout<<"schedule initialise limit " << preprocess_time_limit<<endl;
    free_agents.clear();
    prev_task_schedule.clear();
    prev_task_schedule.resize(env->num_of_agents, -1);
    while (!queued_tasks.empty()) queued_tasks.pop();
    curr_update_timestep = -1;
    prev_update_timestep = -1;
    for (int i=0; i<MAX_PATH; i++) second_class_histogram.push_back(0);
    freeAgentMap.clear();
    freeAgentMap.resize(env->map.size(), -1);
    taskMap.clear();
    taskMap.resize(env->map.size(), std::vector<int>());
    heatMapWP.clear();
    heatMapWP.resize(env->map.size(), 0);
    avg_makespan = (env->cols + env->rows)/2;
    starter_agent_limit = (3 * env->num_of_agents)/4;
    waypoint_step = 512;
    reduceMapStart(env);
    widthMap = reducedMap;
    waypoint_step = 4;
    reduceMapStart(env);
    bfstotask_extra = (env->cols + env->rows)/100;
    if (bfstotask_extra < 5) bfstotask_extra = 5;
    if (env->map_name == "Paris_1_256.map")
    {
        city_map = true;
        origo = env->cols*env->rows/2 + env->cols/2;
        time_limit_correction = 0;
        second_class_is_used = false;
        second_class_cycle_limit = 40;
        if (env->num_of_agents < 2100) {second_class_cycle_limit = 35;}
        else {second_class_cycle_limit = 40;};
        city_long_makespan = 180;
        heat_is_used = true;
        heat_filter_is_used = false;
        heatMap_coeff = 1;
        heatflow_is_used = false;
        heat_coeff = 1;
        makespan_coeff = 1;
        heatflow_filter_is_used = false;
        filter_bottleneck_with_heat = false;
        heat_filter_divident = 4;
        agent_heat_is_used = false;
        agent_heat_filter_is_used = false;
        heatAgentMap_coeff = 0;
        heat_limit_filter_is_used = false;
        if (env->num_of_agents < 2000) {heat_limit = 280;}
        else {heat_limit = 600;};
        dynamic_heat_limit = heat_limit;
        heat_limit_increase = 1000;
        filter_bottleneck = true;
        filter_timed_bottleneck = false;
        timed_window = 100;
        if (env->num_of_agents < 2000) {heat_map_capacity_percentage = 3700;}
        else {heat_map_capacity_percentage = 6400;};
        heatgridsize = 8;
        createHeatMap(env, heatgridsize);
        heatselection = 2;
        starter_is_used = true;
        starter_agent_limit = (3 * env->num_of_agents)/4;
        if (env->num_of_agents < 2000) {bfstotask_extra = 19;}
        else {bfstotask_extra = 15;};
        singleagent = true;
    }
    if (env->map_name == "brc202d.map")
    {
        game_map = true;
        time_limit_correction = 0;
        starter_is_used = true;
        starter_agent_limit = (3 * env->num_of_agents)/4;
        pibt_is_used = false;
        game_long_makespan = 650;
        game_long_starter_correction = 400;
        second_class_is_used = false;
        second_class_limit = 720;
        second_class_cycle_limit = 1000;
        second_class_cycle_period = 1;
        origo = env->cols*env->rows/2 + env->cols/2;
        heat_is_used = false;
        heat_time_is_used = false;
        heat_filter_is_used = false;
        heatMap_coeff = 1;
        heatflow_is_used = false;
        heat_coeff = 1;
        makespan_coeff = 0;
        heatflow_filter_is_used = false;
        filter_bottleneck_with_heat = false;
        heat_filter_divident = 2;
        agent_heat_is_used = false;
        agent_heat_filter_is_used = false;
        heatAgentMap_coeff = 0;
        heat_limit_filter_is_used = false;
        heat_limit = 1700;
        dynamic_heat_limit = heat_limit;
        heat_limit_increase = 1000;
        filter_bottleneck = false;
        filter_timed_bottleneck = false;
        timed_window = 100;
        heat_map_capacity_percentage = 2000;
        heatgridsize = 10;
        createHeatMap(env, heatgridsize);
        heatselection = 2;
        bfstotask_extra = 40;
        singleagent = true;
    }
    if (env->map_name == "random-32-32-20.map")
    {
        manhattan = false;
        random_map = true;
        origo = env->cols*env->rows/2 + env->cols/2;
        if (env->num_of_agents > 5000)
        {
            random_long_makespan = 0;
            starter_is_used = true;
            starter_agent_limit = (3 * env->num_of_agents)/4;
            second_class_is_used = false;
            pibt_is_used = true;
            if (env->num_of_agents < 150) {saturation_limit = 250;}
            else if (env->num_of_agents < 250) {saturation_limit = 300;}
            else if (env->num_of_agents < 450) {saturation_limit = 450;}
            else if (env->num_of_agents < 650) {saturation_limit = 500;}
            else {saturation_limit = 500;};
            heat_is_used = true;
            heatavg_is_used = false;
            heat_filter_is_used = false;
            heat_limit_filter_is_used = true;
            if (env->num_of_agents < 150) {heat_limit = 160;}
            else if (env->num_of_agents < 250) {heat_limit = 200;}
            else if (env->num_of_agents < 450) {heat_limit = 350;}
            else if (env->num_of_agents < 650) {heat_limit = 500;}
            else {heat_limit = 600;};
            dynamic_heat_limit = heat_limit;
            heat_limit_increase = 0;
            heatflow_is_used = false;
            heatflow_filter_is_used = false;
            heatflow_limit_filter_is_used = false;
            if (env->num_of_agents == 100) {heatflow_limit = 160;}
            else if (env->num_of_agents == 200) {heatflow_limit = 200;}
            else if (env->num_of_agents == 400) {heatflow_limit = 350;}
            else if (env->num_of_agents == 600) {heatflow_limit = 650;}
            else if (env->num_of_agents == 800) {heatflow_limit = 550;};
            filter_bottleneck_with_heat = false;
            heat_filter_divident = 2;
            agent_heat_is_used = false;
            if (env->num_of_agents == 100) {heat_coeff = 0.2;}
            else if (env->num_of_agents == 200) {heat_coeff = 0.4;}
            else if (env->num_of_agents == 400) {heat_coeff = 0.6;}
            else if (env->num_of_agents == 600) {heat_coeff = 0.8;}
            else if (env->num_of_agents == 800) {heat_coeff = 1.0;};
            makespan_coeff = 1;
            heatMap_coeff = 1;
            heatAgentMap_coeff = 0;
            heatgridsize = 1;
            filter_bottleneck = false;
            filter_timed_bottleneck = false;
            timed_window = 80;
            heat_map_capacity_percentage = 1200;
            if (env->num_of_agents < 400) {bfstotask_extra = 3;}
            else {bfstotask_extra = 5;}
        }
        if (env->num_of_agents >= 0 && env->num_of_agents < 5000)
        {
            if (env->num_of_agents < 150) {random_long_makespan = 9;}
            else {random_long_makespan = 7;}
            starter_is_used = false;
            starter_agent_limit = (3 * env->num_of_agents)/4;
            second_class_is_used = false;
            pibt_is_used = false;
            if (env->num_of_agents == 100) {saturation_limit = 500;}
            else if (env->num_of_agents == 200) {saturation_limit = 500;}
            else if (env->num_of_agents == 400) {saturation_limit = 500;}
            else if (env->num_of_agents == 600) {saturation_limit = 500;}
            else if (env->num_of_agents == 800) {saturation_limit = 550;};
            heat_is_used = true;
            fullheat_is_used = false;
            heatavg_is_used = false;
            heat_time_is_used = false;
            heat_filter_is_used = false;
            heat_limit_filter_is_used = false;
            if (env->num_of_agents == 100) {heat_limit = 160;}
            else if (env->num_of_agents == 200) {heat_limit = 200;}
            else if (env->num_of_agents == 400) {heat_limit = 350;}
            else if (env->num_of_agents == 600) {heat_limit = 500;}
            else if (env->num_of_agents == 800) {heat_limit = 550;};
            dynamic_heat_limit = heat_limit;
            heat_limit_increase = 20;
            heatflow_is_used = false;
            heatflow_filter_is_used = false;
            heatflow_limit_filter_is_used = false;
            heatflow_limit = 12;
            filter_bottleneck_with_heat = false;
            heat_filter_divident = 4;
            agent_heat_is_used = false;
            heat_coeff = 1;
            makespan_coeff = 1;
            heatMap_coeff = 1;
            heatAgentMap_coeff = 0;
            heatgridsize = 4;
            filter_bottleneck = true;
            filter_timed_bottleneck = false;
            timed_window = 12;
            heat_map_capacity_percentage = 300;
            if (env->num_of_agents < 400) {bfstotask_extra = 4;}
            else {bfstotask_extra = 3;}
        }
        if (env->num_of_agents < 0)
        {
            if (env->num_of_agents < 150) {random_long_makespan = 9;}
            else {random_long_makespan = 7;}
            starter_is_used = false;
            starter_agent_limit = 100000;
            second_class_is_used = false;
            pibt_is_used = true;
            if (env->num_of_agents == 100) {saturation_limit = 500;}
            else if (env->num_of_agents == 200) {saturation_limit = 500;}
            else if (env->num_of_agents == 400) {saturation_limit = 500;}
            else if (env->num_of_agents == 600) {saturation_limit = 500;}
            else if (env->num_of_agents == 800) {saturation_limit = 550;};
            heat_is_used = false;
            heatavg_is_used = false;
            heat_time_is_used = false;
            heat_filter_is_used = false;
            heat_limit_filter_is_used = false;
            if (env->num_of_agents == 100) {heat_limit = 160;}
            else if (env->num_of_agents == 200) {heat_limit = 200;}
            else if (env->num_of_agents == 400) {heat_limit = 350;}
            else if (env->num_of_agents == 600) {heat_limit = 500;}
            else if (env->num_of_agents == 800) {heat_limit = 550;};
            dynamic_heat_limit = heat_limit;
            heat_limit_increase = 20;
            heatflow_is_used = false;
            heatflow_filter_is_used = false;
            heatflow_limit_filter_is_used = false;
            heatflow_limit = 12;
            filter_bottleneck_with_heat = false;
            heat_filter_divident = 4;
            agent_heat_is_used = false;
            heat_coeff = 4;
            makespan_coeff = 1;
            heatMap_coeff = 1;
            heatAgentMap_coeff = 0;
            heatgridsize = 1;
            filter_bottleneck = false;
            filter_timed_bottleneck = false;
            timed_window = 12;
            heat_map_capacity_percentage = 100*timed_window;
            if (env->num_of_agents < 400) {bfstotask_extra = 4;}
            else {bfstotask_extra = 3;}
        }
        if (env->num_of_agents < 150) {heat_map_capacity_percentage = 590;}
        else if (env->num_of_agents < 250) {heat_map_capacity_percentage = 1050;}
        else if (env->num_of_agents < 450) {heat_map_capacity_percentage = 1800;}
        else if (env->num_of_agents < 650) {heat_map_capacity_percentage = 2000;}
        else {heat_map_capacity_percentage = 1800;};
        createHeatMap(env, heatgridsize);
        heatselection = 2;
        singleagent = true;
    }
    if (env->map_name == "sortation_large.map")
    {
        long_makespan = 70;
        manhattan = false;
        warehouse_map = true;
        warehouse_vertical = true;
        origo = env->cols*env->rows/2 + env->cols/2;
        time_limit_correction = 0;
        second_class_is_used = false;
        second_class_limit = 10000;
        second_class_cycle_limit = 10000;
        second_class_cycle_period = 1;
        offset_is_used = false;
        heat_is_used = true;
        heatavg_is_used = false;
        heat_time_is_used = false;
        heat_filter_is_used = false;
        heatMap_coeff = 1;
        heatflow_is_used = false;
        heat_coeff = 1;
        makespan_coeff = 300;
        heatflow_filter_is_used = false;
        filter_bottleneck_with_heat = false;
        heat_filter_divident = 30;
        agent_heat_is_used = false;
        agent_heat_filter_is_used = false;
        heatAgentMap_coeff = 0;
        heat_limit_filter_is_used = false;
        heat_limit = 190000;
        dynamic_heat_limit = heat_limit;
        heat_limit_increase = 1000;
        filter_bottleneck = false;
        filter_timed_bottleneck = false;
        timed_window = 40;
        heat_map_capacity_percentage = 700;
        heatgridsize = 8;
        createHeatMap(env, heatgridsize);
        heatselection = 2;
        bfstotask_extra = 40;
        starter_is_used = true;
        pibt_is_used = false;
        starter_agent_limit = (3 * env->num_of_agents)/4;
        singleagent = true;
    }
    if (env->map_name == "warehouse_large.map")
    {
        long_makespan = 70;
        manhattan = true;
        warehouse_map = true;
        warehouse_vertical = true;
        origo = env->cols*env->rows/2 + env->cols/2;
        time_limit_correction = 0;
        second_class_is_used = false;
        second_class_limit = 10000;
        second_class_cycle_limit = 10000;
        second_class_cycle_period = 1;
        offset_is_used = false;
        heat_is_used = true;
        heatavg_is_used = false;
        heat_time_is_used = false;
        heat_filter_is_used = false;
        heatMap_coeff = 5;
        heatflow_is_used = false;
        heat_coeff = 1;
        makespan_coeff = 1;
        heatflow_filter_is_used = false;
        filter_bottleneck_with_heat = false;
        heat_filter_divident = 30;
        agent_heat_is_used = false;
        agent_heat_filter_is_used = false;
        heatAgentMap_coeff = 0;
        heat_limit_filter_is_used = false;
        heat_limit = 190000;
        dynamic_heat_limit = heat_limit;
        heat_limit_increase = 1000;
        filter_bottleneck = false;
        filter_timed_bottleneck = false;
        timed_window = 40;
        heat_map_capacity_percentage = 700;
        heatgridsize = 1;
        createHeatMap(env, heatgridsize);
        heatselection = 2;
        bfstotask_extra = 25;
        starter_is_used = true;
        pibt_is_used = false;
        starter_agent_limit = (3 * env->num_of_agents)/4;
        singleagent = true;
    }
}
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    current_step++;
    if (current_step == 0)
    {
        bases.clear();
        bases.resize(env->num_of_agents,-1);
        for (int i=0; i<env->num_of_agents; i++) bases[i] = env->curr_states[i].location;
    }
    if (game_map && current_step > 30)
    {
        int sum = 0;
        for (int i=0; i<MAX_PATH; i++)
        {
            sum = sum + second_class_histogram[i];
            if (sum >= second_class_limit)
            {
                game_long_makespan = i + (second_class_limit - second_class_counter);
                break;
            }
        }
    }
    if (game_map && current_step == 1000) second_class_limit = second_class_limit - 20;
    if (game_map && current_step == 1600) second_class_limit = second_class_limit - 20;
    if (game_map && current_step == 2200) second_class_limit = second_class_limit - 20;
    if (game_map && current_step == 2800) second_class_limit = second_class_limit - 20;
    cout << "singleagent= " << singleagent << " pibt is used= " << pibt_is_used << "starter is used= " << starter_is_used << " filter_bottleneck= " << filter_bottleneck << " filter_timed_bottleneck= " << filter_timed_bottleneck << " warehouse map= " << warehouse_map << " second_class_cycle_counter= " << second_class_cycle_counter << " game_long_makespan= " << game_long_makespan << " avg_makespan= " << avg_makespan << " avg_heat= " << avg_heat << endl;
    if (current_step%second_class_cycle_period == 0) second_class_cycle_counter=0;
    if (!starter_is_used)
    {
        schedule_plan_from_awm(time_limit - time_limit_correction, proposed_schedule, env);
        return;
    }
    if (starter_agents < starter_agent_limit)
    {
        if (pibt_is_used) schedule_plan_pibt_starter(time_limit - time_limit_correction, proposed_schedule, env);
        else schedule_plan_from_twm(time_limit - time_limit_correction, proposed_schedule, env);
        return;
    }
    else
    {
        schedule_plan_from_awm(time_limit - time_limit_correction, proposed_schedule, env);
        return;
    }
}
void schedule_plan_from_a(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    proposed_schedule.resize(env->num_of_agents, -1);
    proposed_schedule = env->curr_task_schedule;
    clock_t start = clock();
    curr_update_timestep = env->curr_timestep;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (env->curr_task_schedule[i] == -1 && prev_task_schedule[i] != -1) {
            task_finished(env, i);
            working_agents--;
        }
    }
    for (int i_task=0 ; i_task < env->task_pool.size();i_task++)
    {
        if (env->task_pool[i_task].t_revealed > prev_update_timestep)
        {
            process_new_task(env, env->task_pool[i_task]);
        }
    }
    prev_update_timestep = curr_update_timestep;
    int min_task_id, min_task_id2, min_task_id3, min_task_id4, min_task_makespan, min_task_makespan2, min_task_makespan3, min_task_makespan4, dist, c_loc;
    std::vector< std::pair <int,int> > tasksSorted;
    std::vector< std::pair <double,int> > tasksSortedByHeat;
    for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
    {
        if (env->curr_task_schedule[i] == -1)
        {
            min_task_id = -1;
            min_task_id2 = -1;
            min_task_id3 = -1;
            min_task_id4 = -1;
            min_task_makespan = INT_MAX;
            min_task_makespan2 = INT_MAX;
            min_task_makespan3 = INT_MAX;
            min_task_makespan4 = INT_MAX;
            std::vector<int> min_path = {};
            std::vector<int> path = {};
            int start, end;
            std::unordered_map<int,ProcessedTask>::iterator i_task;
            for (i_task = waiting_tasks.begin(); i_task != waiting_tasks.end() && std::chrono::steady_clock::now() < endtime; ++i_task)
            {
                dist = getManhattanDistance(env,env->curr_states.at(i).location, i_task->second.locations[0]);
                if (dist < min_task_makespan){
                    min_task_id4 = min_task_id3;
                    min_task_id3 = min_task_id2;
                    min_task_id2 = min_task_id;
                    min_task_id = i_task->first;
                    min_task_makespan4 = min_task_makespan3;
                    min_task_makespan3 = min_task_makespan2;
                    min_task_makespan2 = min_task_makespan;
                    min_task_makespan = dist;
                    min_path = path;
                }
                else if (dist < min_task_makespan2) {
                    min_task_id4 = min_task_id3;
                    min_task_id3 = min_task_id2;
                    min_task_id2 = i_task->first;
                    min_task_makespan4 = min_task_makespan3;
                    min_task_makespan3 = min_task_makespan2;
                    min_task_makespan2 = dist;
                    min_path = path;
                } else if (dist < min_task_makespan3) {
                    min_task_id4 = min_task_id3;
                    min_task_id3 = i_task->first;
                    min_task_makespan4 = min_task_makespan3;
                    min_task_makespan3 = dist;
                    min_path = path;
                } else if (dist < min_task_makespan4) {
                    min_task_id4 = i_task->first;
                    min_task_makespan4 = dist;
                    min_path = path;
                }
            }
            if (min_task_id != -1){
                min_task_makespan = min_task_makespan + waiting_tasks[min_task_id].makespan;
                if (min_task_id2 == -1) min_task_makespan2 = INT_MAX;
                else min_task_makespan2 = min_task_makespan2 + waiting_tasks[min_task_id2].makespan;
                if (min_task_id3 == -1) min_task_makespan3 = INT_MAX;
                else min_task_makespan3 = min_task_makespan3 + waiting_tasks[min_task_id3].makespan;
                if (min_task_id4 == -1) min_task_makespan4 = INT_MAX;
                else min_task_makespan4 = min_task_makespan4 + waiting_tasks[min_task_id4].makespan;
                tasksSorted.clear();
                tasksSorted.push_back(make_pair(min_task_makespan,min_task_id));
                tasksSorted.push_back(make_pair(min_task_makespan2,min_task_id2));
                tasksSorted.push_back(make_pair(min_task_makespan3,min_task_id3));
                tasksSorted.push_back(make_pair(min_task_makespan3,min_task_id4));
                std::sort(tasksSorted.begin(), tasksSorted.end());
                tasksSortedByHeat.clear();
                for (int i_heat=0; i_heat < 3 && i_heat < tasksSorted.size(); i_heat++)
                    tasksSortedByHeat.push_back(make_pair(getFromHeatMapWP(env, waiting_tasks[tasksSorted[i_heat].second].locations),tasksSorted[i_heat].second));
                std::sort(tasksSortedByHeat.begin(), tasksSortedByHeat.end());
                proposed_schedule[i] = tasksSortedByHeat[0].second;
                addToHeatMapWP(env, waiting_tasks[tasksSortedByHeat[0].second].locations);
                waiting_tasks.erase(tasksSortedByHeat[0].second);
                working_agents++;
            }
            else{
                proposed_schedule[i] = -1;
            }
        }
        else
        {
            proposed_schedule[i] = env->curr_task_schedule[i];
        }
    }
    prev_task_schedule = proposed_schedule;
}
void schedule_plan_from_awm(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    TimePoint process_endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-100);
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    proposed_schedule.resize(env->num_of_agents, -1);
    proposed_schedule = env->curr_task_schedule;
    clock_t start = clock();
    if (current_step%100 == 0) dynamic_heat_limit = dynamic_heat_limit + heat_limit_increase;
    cout << "scheduling from agents with map ... working agents = " << working_agents << " or " << env->num_of_agents - free_agents.size() << "  bottleneck drops = " << bottleneck_drops << " total heat = " << total_heat << " task_counter= " << task_counter << " task_second_counter= " << task_second_counter << " task_second_finished= " << task_second_finished << " second_class_counter= " << second_class_counter << endl;
    curr_update_timestep = env->curr_timestep;
    int new_tasks_size = env->new_tasks.size();
    if (new_tasks_size >= BUFFER_SIZE)
    {
        for (int i_task=0 ; i_task < new_tasks_size;i_task++)
        {
            queued_tasks.push(env->task_pool[env->new_tasks[i_task]]);
    }
    }
    int new_free_agents_size = env->new_freeagents.size();
    for (int i = 0; i < new_free_agents_size; i++)
    {
        free_agents.insert(env->new_freeagents[i]);
        if (env->curr_task_schedule[env->new_freeagents[i]] == -1 && prev_task_schedule[env->new_freeagents[i]] != -1) {
            task_finished(env, env->new_freeagents[i]);
            working_agents--;
        }
    }
    if (new_tasks_size < BUFFER_SIZE)
    {
        for (int i_task=0 ; i_task < new_tasks_size;i_task++)
        {
            taskMap[env->task_pool[env->new_tasks[i_task]].locations[0]].push_back(env->task_pool[env->new_tasks[i_task]].task_id);
            process_new_task(env, env->task_pool[env->new_tasks[i_task]]);
        }
    }
    if (agent_heat_is_used || multi_criteria_is_used) {
        addAgentsToHeatMap(env, env->curr_states);
    }
    while (!queued_tasks.empty() && std::chrono::steady_clock::now() < process_endtime)
    {
        taskMap[queued_tasks.front().locations[0]].push_back(queued_tasks.front().task_id);
        process_new_task(env, queued_tasks.front());
        queued_tasks.pop();
    }
    prev_update_timestep = curr_update_timestep;
    int min_task_id, min_task_id2, min_task_id3, min_task_id4, min_task_makespan, min_task_makespan2, min_task_makespan3, min_task_makespan4, dist, c_loc;
    double makespan_performance, heat_performance;
    std::vector< std::pair <int,int> > tasksSorted;
    std::vector< std::pair <double,std::pair<int,int>> > tasksSortedByHeat;
    std::deque< std::pair< int, std::pair <int,int> > > bidsSorted;
    std::deque< std::pair< int, std::pair <int,int> > > bidsToKeep;
    std::deque< std::pair< int, std::pair <int,int> > > bidsToKeep2;
    bool secondclass = false;
    std::vector<int> v1 = {};
    std::vector<int> v2 = {};
    std::set<int>::iterator itr;
    std::vector<int> agentstocheck = {};
    std::vector<int> agentstocheck2 = {};
    for (itr = free_agents.begin(); itr != free_agents.end() && std::chrono::steady_clock::now() < endtime; itr++)
    {
        if (*itr > last_checked_agent) {agentstocheck.push_back(*itr);}
        else {agentstocheck2.push_back(*itr);}
    }
    agentstocheck.insert(agentstocheck.end(), agentstocheck2.begin(), agentstocheck2.end());
    bidsSorted.clear();
    for (int i = 0; i < agentstocheck.size() && std::chrono::steady_clock::now() < endtime; i++)
    {
        int agent_id = agentstocheck[i];
        last_checked_agent = agent_id;
        std::vector<std::pair<int,int>> tasks = {};
        if (bfswithturn) { BFStoTWithTurn(env, endtime, env->curr_states.at(agent_id).location, env->curr_states.at(agent_id).orientation, tasks); }
        else { BFStoT(env, endtime, env->curr_states.at(agent_id).location, env->curr_states.at(agent_id).orientation, tasks); };
        if ( !tasks.empty())
        {
            if (heat_filter_is_used || agent_heat_filter_is_used || heatflow_filter_is_used)
            {
                tasksSortedByHeat.clear();
                if (heat_filter_is_used || agent_heat_filter_is_used) {
                    for (int i_heat=0; i_heat < tasks.size(); i_heat++)
                        tasksSortedByHeat.push_back(make_pair(getFromHeatMap(env, waiting_tasks[tasks[i_heat].second].locations),tasks[i_heat]));
                }
                if (heatflow_filter_is_used) {
                    for (int i_heat=0; i_heat < tasks.size(); i_heat++)
                        tasksSortedByHeat.push_back(make_pair(getFromHeatFlowMap(env, waiting_tasks[tasks[i_heat].second].locations),tasks[i_heat]));
                }
                std::sort(tasksSortedByHeat.begin(), tasksSortedByHeat.end());
                if (heat_limit_filter_is_used)
                {
                    for (int index = 0; index < tasksSortedByHeat.size(); index++)
                    {
                        if (tasksSortedByHeat[index].first > dynamic_heat_limit)
                        {
                            bottleneck_drops = bottleneck_drops + tasksSortedByHeat.size() - index;
                            break;
                        }
                        dist = tasksSortedByHeat[index].second.first + waiting_tasks[tasksSortedByHeat[index].second.second].makespan;
                        bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasksSortedByHeat[index].second.second)));
                    }
                }
                else
                if (heatflow_limit_filter_is_used)
                {
                    for (int index = 0; index < tasksSortedByHeat.size(); index++)
                    {
                        if (tasksSortedByHeat[index].first > heatflow_limit)
                        {
                            bottleneck_drops = bottleneck_drops + tasksSortedByHeat.size() - index;
                            break;
                        }
                        dist = tasksSortedByHeat[index].second.first + waiting_tasks[tasksSortedByHeat[index].second.second].makespan;
                        bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasksSortedByHeat[index].second.second)));
                    }
                }
                else
                {
                    dist = tasksSortedByHeat[0].second.first + waiting_tasks[tasksSortedByHeat[0].second.second].makespan;
                    bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasksSortedByHeat[0].second.second)));
                    for (int index = 1; index < tasksSortedByHeat.size()/heat_filter_divident; index++)
                    {
                        dist = tasksSortedByHeat[index].second.first + waiting_tasks[tasksSortedByHeat[index].second.second].makespan;
                        bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasksSortedByHeat[index].second.second)));
                    }
                };
            }
            else
            if (heat_is_used || heat_time_is_used || agent_heat_is_used || fullheat_is_used)
            {
                for (int index = 0; index < tasks.size(); index++)
                {
                    waiting_tasks[tasks[index].second].started_makespan = tasks[index].first + waiting_tasks[tasks[index].second].makespan;
                    int makespan = waiting_tasks[tasks[index].second].makespan;
                    if (heat_time_is_used)
                        dist = makespan_coeff*(tasks[index].first + makespan) +
                            heat_coeff * getFromHeatTimeMap(env, waiting_tasks[tasks[index].second].locations, waiting_tasks[tasks[index].second].stays);
                    else
                    {
                        dist = makespan_coeff*(tasks[index].first + makespan) + heat_coeff * getFromHeatMap(env, waiting_tasks[tasks[index].second].locations);
                    }
                    bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasks[index].second)));
                };
            }
            else
            if (heatflow_is_used)
            {
                for (int index = 0; index < tasks.size(); index++)
                {
                    waiting_tasks[tasks[index].second].started_makespan = tasks[index].first + waiting_tasks[tasks[index].second].makespan;
                    dist = makespan_coeff*(tasks[index].first + waiting_tasks[tasks[index].second].makespan) + heat_coeff * getFromHeatFlowMap(env, waiting_tasks[tasks[index].second].locations);
                    bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasks[index].second)));
                };
            }
            else
            {
                for (int index = 0; index < tasks.size(); index++)
                {
                    int makespan = waiting_tasks[tasks[index].second].makespan;
                    dist = tasks[index].first + makespan;
                    bidsSorted.push_back(make_pair(dist,make_pair(agent_id,tasks[index].second)));
                };
            }
        }
    }
    std::sort(bidsSorted.begin(), bidsSorted.end());
    int best_agent_id, best_task_id;
    int number_of_bids = bidsSorted.size();
    while (number_of_bids > 0)
    {
        best_agent_id = bidsSorted[0].second.first;
        best_task_id = bidsSorted[0].second.second;
        if (game_map && waiting_tasks[best_task_id].second_class && second_class_counter > second_class_limit)
        {
            bidsSorted.pop_front();
            number_of_bids--;
            bottleneck_drops++;
            continue;
        } ;
        if (filter_bottleneck && bottleneck(env, waiting_tasks[best_task_id].locations))
        {
            bidsSorted.pop_front();
            number_of_bids--;
            bottleneck_drops++;
            continue;
        } ;
        if (filter_timed_bottleneck)
        {
            std::vector<int> path = {};
            std::vector<int> stays = {};
            if (singleagent) {singleAgentAStar(env, env->curr_states[best_agent_id].location, waiting_tasks[best_task_id].task_front, path,stays);}
            else {reducedMapWaypointsAStar(env, env->curr_states[best_agent_id].location, waiting_tasks[best_task_id].task_front, path);}
            path.insert(path.end(), waiting_tasks[best_task_id].locations.begin(), waiting_tasks[best_task_id].locations.end());
            stays.insert(stays.end(), waiting_tasks[best_task_id].stays.begin(), waiting_tasks[best_task_id].stays.end());
            if (timedBottleneck(env, path, stays))
            {
                bidsSorted.pop_front();
                number_of_bids--;
                bottleneck_drops++;
                continue;
            }
        } ;
        proposed_schedule[best_agent_id] = best_task_id;
        free_agents.erase(best_agent_id);
        if (heatflow_is_used || heatflow_filter_is_used) addToHeatFlowMap(env, waiting_tasks[best_task_id].locations);
        if (heat_is_used || heat_time_is_used || filter_bottleneck || filter_timed_bottleneck || multi_criteria_is_used)
        {
            if (heat_is_used || filter_bottleneck) addToHeatMap(env, waiting_tasks[best_task_id].locations);
            if (filter_timed_bottleneck || heat_time_is_used)
            {
                std::vector<int> path = {};
                std::vector<int> stays = {};
                if (singleagent) {singleAgentAStar(env, env->curr_states[best_agent_id].location, waiting_tasks[best_task_id].task_front, path,stays);}
                else {reducedMapWaypointsAStar(env, env->curr_states[best_agent_id].location, waiting_tasks[best_task_id].task_front, path);}
                path.insert(path.end(), waiting_tasks[best_task_id].locations.begin(), waiting_tasks[best_task_id].locations.end());
                stays.insert(stays.end(), waiting_tasks[best_task_id].stays.begin(), waiting_tasks[best_task_id].stays.end());
                waiting_tasks[best_task_id].locations.clear();
                waiting_tasks[best_task_id].locations = path;
                addToHeatTimeMap(env, path,stays);
            }
        }
        if (heat_is_used || agent_heat_is_used || filter_bottleneck || filter_timed_bottleneck)
        {
            total_makespan = total_makespan + waiting_tasks[best_task_id].makespan;
            avg_makespan = total_makespan/(working_agents +1);
        }
        if (waiting_tasks[best_task_id].second_class)
        {
            second_class_counter++;
            second_class_cycle_counter++;
        }
        if (game_map && waiting_tasks[best_task_id].makespan > game_long_limit) game_long_count++;
        running_tasks[best_task_id] = waiting_tasks[best_task_id];
        waiting_tasks.erase(best_task_id);
        running_tasks[best_task_id].started = env->curr_timestep;
        running_tasks[best_task_id].temperature = getFromHeatMap(env, running_tasks[best_task_id].locations);
        taskMap[running_tasks[best_task_id].task_front].erase(
                find(taskMap[running_tasks[best_task_id].task_front].begin(),
                taskMap[running_tasks[best_task_id].task_front].end(),
                best_task_id));
        working_agents++;
        bidsToKeep.clear();
        bidsToKeep2.clear();
        for(int i_bid = 0; i_bid < bidsSorted.size(); i_bid++)
        {
            if (!(bidsSorted[i_bid].second.first == best_agent_id || bidsSorted[i_bid].second.second == best_task_id))
            {
                bidsToKeep.push_back(bidsSorted[i_bid]);
            }
        }
        bidsSorted.clear();
        bidsSorted = bidsToKeep;
        bidsSorted.insert(bidsSorted.end(), bidsToKeep2.begin(), bidsToKeep2.end());
        number_of_bids = bidsSorted.size();
    }
    prev_task_schedule = proposed_schedule;
}
void schedule_plan_from_t(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
}
void schedule_plan_from_twm(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    TimePoint process_endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-130);
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-30);
    proposed_schedule.resize(env->num_of_agents, -1);
    proposed_schedule = env->curr_task_schedule;
    clock_t start = clock();
    cout << "scheduling from tasks with map ... working agents = " << working_agents << " or " << env->num_of_agents - free_agents.size() << " ***   starter agents = " << starter_agents << "    starter tasks = " << starter_tasks << "  bottleneck drops = " << bottleneck_drops << " total heat = " << total_heat << " task_counter= " << task_counter << " task_second_counter= " << task_second_counter << " task_second_finished= " << task_second_finished << " second_class_counter= " << second_class_counter << endl;
    curr_update_timestep = env->curr_timestep;
    int new_tasks_size = env->new_tasks.size();
    for (int i_task=0 ; i_task < new_tasks_size;i_task++)
    {
        queued_tasks.push(env->task_pool[env->new_tasks[i_task]]);
    }
    int new_free_agents_size = env->new_freeagents.size();
    for (int i = 0; i < new_free_agents_size; i++)
    {
        free_agents.insert(env->new_freeagents[i]);
        if (env->curr_task_schedule[env->new_freeagents[i]] == -1 && prev_task_schedule[env->new_freeagents[i]] != -1) {
            task_finished(env, env->new_freeagents[i]);
            working_agents--;
        }
    }
    freeAgentMap.clear();
    freeAgentMap.resize(env->map.size(), -1);
    for (int i : free_agents)
    {
        freeAgentMap[env->curr_states[i].location] = i;
    }
    prev_update_timestep = curr_update_timestep;
    if (agent_heat_is_used || multi_criteria_is_used) {
        addAgentsToHeatMap(env, env->curr_states);
    }
    while (!queued_tasks.empty() && std::chrono::steady_clock::now() < process_endtime)
    {
        taskMap[queued_tasks.front().locations[0]].push_back(queued_tasks.front().task_id);
        process_new_task(env, queued_tasks.front());
        queued_tasks.pop();
        starter_tasks++;
    }
    waitingTasksSorted.clear();
    for(auto mapelement : waiting_tasks)
    {
        if (std::chrono::steady_clock::now() >= endtime) break;
        ProcessedTask pt = mapelement.second;
        if (heat_is_used || agent_heat_is_used || fullheat_is_used)
        {
            int push_makespan = pt.makespan;
            push_makespan = makespan_coeff * push_makespan + heat_coeff * getFromHeatMap(env, pt.locations);
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
        else
        if (heatflow_is_used || heatflow_filter_is_used)
        {
            int push_makespan = makespan_coeff * pt.makespan + heat_coeff * getFromHeatFlowMap(env, pt.locations);
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
        else
        {
            int push_makespan = pt.makespan;
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
    }
    std::sort(waitingTasksSorted.begin(),waitingTasksSorted.end());
    int min_agent_id, min_agent_makespan, dist, task_front, task_id, size;
    while (!waitingTasksSorted.empty() && std::chrono::steady_clock::now() < endtime)
    {
        if (free_agents.size() == 0) break;
        task_id = waitingTasksSorted[0].second;
        if (game_map && waiting_tasks[task_id].second_class && second_class_counter > second_class_limit)
        {
            waitingTasksSorted.pop_front();
            bottleneck_drops++;
            continue;
        } ;
        if ((filter_bottleneck && bottleneck(env, waiting_tasks[task_id].locations)) ||
            (filter_timed_bottleneck && timedBottleneck(env, waiting_tasks[task_id].locations, waiting_tasks[task_id].stays)))
        {
            waitingTasksSorted.pop_front();
            bottleneck_drops++;
            continue;
        } ;
        task_front = waiting_tasks.at(task_id).task_front;
        min_agent_id = BFStoA(env, endtime, task_front);
        if (min_agent_id != -1){
            proposed_schedule[min_agent_id] = task_id;
            freeAgentMap[env->curr_states[min_agent_id].location] = -1;
            free_agents.erase(min_agent_id);
            if (heatflow_is_used || heatflow_filter_is_used) addToHeatFlowMap(env, waiting_tasks[task_id].locations);
            if (heat_is_used || filter_bottleneck || filter_timed_bottleneck || multi_criteria_is_used)
            {
                if (heat_is_used || filter_bottleneck) addToHeatMap(env, waiting_tasks[task_id].locations);
                if (filter_timed_bottleneck)
                {
                    std::vector<int> path = {};
                    std::vector<int> stays = {};
                    if (singleagent) {singleAgentAStar(env, env->curr_states[min_agent_id].location, task_front, path,stays);}
                    else {reducedMapWaypointsAStar(env, env->curr_states[min_agent_id].location, task_front, path);}
                    path.insert(path.end(), waiting_tasks[task_id].locations.begin(), waiting_tasks[task_id].locations.end());
                    stays.insert(stays.end(), waiting_tasks[task_id].stays.begin(), waiting_tasks[task_id].stays.end());
                    waiting_tasks[task_id].locations.clear();
                    waiting_tasks[task_id].locations = path;
                    addToHeatTimeMap(env, path,stays);
                }
            }
            if (heat_is_used || agent_heat_is_used || filter_bottleneck || filter_timed_bottleneck)
            {
                total_makespan = total_makespan + waiting_tasks[task_id].makespan;
                avg_makespan = total_makespan/(working_agents +1);
            }
            if (waiting_tasks[task_id].second_class)
            {
                second_class_counter++;
                second_class_cycle_counter++;
            }
            running_tasks[task_id] = waiting_tasks[task_id];
            waiting_tasks.erase(task_id);
            waitingTasksSorted.pop_front();
            running_tasks[task_id].started = env->curr_timestep;
            running_tasks[task_id].started_makespan = running_tasks[task_id].makespan;
            running_tasks[task_id].temperature = getFromHeatMap(env, running_tasks[task_id].locations);
            auto it = find(taskMap[running_tasks[task_id].task_front].begin(), taskMap[running_tasks[task_id].task_front].end(), task_id);
            if (it != taskMap[running_tasks[task_id].task_front].end()) taskMap[running_tasks[task_id].task_front].erase(it);
            starter_agents++;
            working_agents++;
        }
    }
    prev_task_schedule = proposed_schedule;
}
void schedule_plan_pibt(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    TimePoint process_endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-130);
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-30);
    proposed_schedule.resize(env->num_of_agents, -1);
    proposed_schedule = env->curr_task_schedule;
    clock_t start = clock();
    curr_update_timestep = env->curr_timestep;
    int new_tasks_size = env->new_tasks.size();
    for (int i_task=0 ; i_task < new_tasks_size;i_task++)
    {
        queued_tasks.push(env->task_pool[env->new_tasks[i_task]]);
    }
    int new_free_agents_size = env->new_freeagents.size();
    for (int i = 0; i < new_free_agents_size; i++)
    {
        free_agents.insert(env->new_freeagents[i]);
        if (env->curr_task_schedule[env->new_freeagents[i]] == -1 && prev_task_schedule[env->new_freeagents[i]] != -1) {
            task_finished(env, env->new_freeagents[i]);
            working_agents--;
        }
    }
    freeAgentMap.clear();
    freeAgentMap.resize(env->map.size(), -1);
    for (int i : free_agents)
    {
        freeAgentMap[env->curr_states[i].location] = i;
    }
    workingAgentMap.clear();
    workingAgentMap.resize(env->map.size(), -1);
    for (int i=0; i<env->curr_states.size(); i++)
    {
        if (freeAgentMap[env->curr_states[i].location] < 0) workingAgentMap[env->curr_states[i].location] = i;
    }
    prev_update_timestep = curr_update_timestep;
    while (!queued_tasks.empty() && std::chrono::steady_clock::now() < process_endtime)
    {
        taskMap[queued_tasks.front().locations[0]].push_back(queued_tasks.front().task_id);
        process_new_task(env, queued_tasks.front());
        ProcessedTask pt = waiting_tasks.at(queued_tasks.front().task_id);
        if (heat_is_used || agent_heat_is_used || fullheat_is_used)
        {
            int push_makespan = makespan_coeff * pt.makespan + heat_coeff * getFromHeatMap(env, pt.locations);
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
        else
        if (heatflow_is_used)
        {
            int push_makespan = makespan_coeff * pt.makespan + heat_coeff * getFromHeatFlowMap(env, pt.locations);
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
        else
        {
            int push_makespan = pt.makespan;
            int push_task_id = pt.task_id;
            waitingTasksSorted.push_back(make_pair(push_makespan,push_task_id));
        }
        queued_tasks.pop();
        starter_tasks++;
    }
    if (agent_heat_is_used || multi_criteria_is_used) {
        addAgentsToHeatMap(env, env->curr_states);
    }
    updateAgentMap(env);
    int current_agent_id, best_task_id;
    while (!free_agents.empty() && std::chrono::steady_clock::now() < endtime)
    {
        current_agent_id = *free_agents.begin();
        int loc_start = env->curr_states[current_agent_id].location;
        int working_agent_id = BFStoWorkingA(env, endtime,loc_start);
        if (working_agent_id < 0) continue;
        int loc_end;
        if (env->curr_task_schedule[working_agent_id] < 0) loc_end = env->task_pool[proposed_schedule[working_agent_id]].locations.back();
        else loc_end = env->task_pool[env->curr_task_schedule[working_agent_id]].locations.back();
        best_task_id = -1;
        int match = -1;
        int next_match = -1;
        int match_makespan = -1;
        for (auto wt : waiting_tasks)
        {
            next_match = (loc_start - wt.second.task_front)*(loc_start - wt.second.task_front) + (loc_end - wt.second.locations.back())*(loc_end - wt.second.locations.back());
            if (best_task_id < 0)
            {
                best_task_id = wt.second.task_id;
                match = next_match;
                match_makespan = wt.second.makespan;
            }
            else if (next_match < match || (next_match == match && match_makespan > wt.second.makespan))
            {
                best_task_id = wt.second.task_id;
                match = next_match;
                match_makespan = wt.second.makespan;
            }
        }
        search_path.clear();
        proposed_schedule[current_agent_id] = best_task_id;
        freeAgentMap[env->curr_states[current_agent_id].location] = -1;
        free_agents.erase(current_agent_id);
        workingAgentMap[env->curr_states[current_agent_id].location] = current_agent_id;
        if (heatflow_is_used || heatflow_filter_is_used) addToHeatFlowMap(env, waiting_tasks[best_task_id].locations);
        if (heat_is_used || filter_bottleneck || filter_timed_bottleneck || multi_criteria_is_used)
        {
            if (heat_is_used || filter_bottleneck) addToHeatMap(env, waiting_tasks[best_task_id].locations);
            if (filter_timed_bottleneck)
            {
                std::vector<int> path = {};
                std::vector<int> stays = {};
                if (singleagent) {singleAgentAStar(env, env->curr_states[current_agent_id].location, waiting_tasks[best_task_id].task_front, path,stays);}
                else {reducedMapWaypointsAStar(env, env->curr_states[current_agent_id].location, waiting_tasks[best_task_id].task_front, path);}
                path.insert(path.end(), waiting_tasks[best_task_id].locations.begin(), waiting_tasks[best_task_id].locations.end());
                stays.insert(stays.end(), waiting_tasks[best_task_id].stays.begin(), waiting_tasks[best_task_id].stays.end());
                waiting_tasks[best_task_id].locations.clear();
                waiting_tasks[best_task_id].locations = path;
                addToHeatTimeMap(env, path,stays);
            }
        }
        search_path = waiting_tasks[best_task_id].locations;
        if (heat_is_used || agent_heat_is_used || filter_bottleneck || filter_timed_bottleneck)
        {
            total_makespan = total_makespan + waiting_tasks[best_task_id].makespan;
            avg_makespan = total_makespan/(working_agents +1);
        }
        running_tasks[best_task_id] = waiting_tasks[best_task_id];
        waiting_tasks.erase(best_task_id);
        running_tasks[best_task_id].started = env->curr_timestep;
        running_tasks[best_task_id].temperature = getFromHeatMap(env, running_tasks[best_task_id].locations);
        taskMap[running_tasks[best_task_id].task_front].erase(
                find(taskMap[running_tasks[best_task_id].task_front].begin(),
                taskMap[running_tasks[best_task_id].task_front].end(),
                best_task_id));
        starter_agents++;
        working_agents++;
    }
    prev_task_schedule = proposed_schedule;
}
void schedule_plan_pibt_starter(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit-30);
    proposed_schedule.resize(env->num_of_agents, -1);
    proposed_schedule = env->curr_task_schedule;
    clock_t start = clock();
    curr_update_timestep = env->curr_timestep;
    int new_tasks_size = env->new_tasks.size();
    for (int i_task=0 ; i_task < new_tasks_size;i_task++)
    {
        queued_tasks.push(env->task_pool[env->new_tasks[i_task]]);
    }
    int new_free_agents_size = env->new_freeagents.size();
    for (int i = 0; i < new_free_agents_size; i++)
    {
        free_agents.insert(env->new_freeagents[i]);
        if (env->curr_task_schedule[env->new_freeagents[i]] == -1 && prev_task_schedule[env->new_freeagents[i]] != -1) {
            task_finished(env, env->new_freeagents[i]);
            working_agents--;
        }
    }
    freeAgentMap.clear();
    freeAgentMap.resize(env->map.size(), -1);
    for (int i : free_agents)
    {
        freeAgentMap[env->curr_states[i].location] = i;
    }
    workingAgentMap.clear();
    workingAgentMap.resize(env->map.size(), -1);
    for (int i=0; i<env->curr_states.size(); i++)
    {
        if (freeAgentMap[env->curr_states[i].location] < 0) workingAgentMap[env->curr_states[i].location] = i;
    }
    prev_update_timestep = curr_update_timestep;
    if (agent_heat_is_used || multi_criteria_is_used)
    {
        addAgentsToHeatMap(env, env->curr_states);
    }
    while (!queued_tasks.empty() && std::chrono::steady_clock::now() < endtime)
    {
        taskMap[queued_tasks.front().locations[0]].push_back(queued_tasks.front().task_id);
        process_new_task(env, queued_tasks.front());
        queued_tasks.pop();
        starter_tasks++;
    }
    std::vector<std::pair<int,int>> tasksMakespanSorted;
    bool still_to_go = false;
    for(auto pt : waiting_tasks)
    {
        int push_heat;
        if (heat_is_used || fullheat_is_used)
            {
                push_heat = getFromHeatMap(env, pt.second.locations);
            }
        if (heatflow_is_used) push_heat = getFromHeatFlowMap(env, pt.second.locations);
        if (filter_timed_bottleneck && !timedBottleneck(env, pt.second.locations, pt.second.stays))
        {
            tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
            still_to_go = true;
        }
        else if (filter_bottleneck && !bottleneck(env, pt.second.locations))
        {
            tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
            still_to_go = true;
        }
        else if (push_heat <= saturation_limit)
        {
            tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
            still_to_go = true;
        }
    }
    std::sort(tasksMakespanSorted.begin(),tasksMakespanSorted.end());
    int current_agent_id, best_task_id;
    while (still_to_go && free_agents.size() > 0 && std::chrono::steady_clock::now() < endtime)
    {
        current_agent_id = -1;
        best_task_id = tasksMakespanSorted.at(0).second;
        current_agent_id = BFStoA(env, endtime,waiting_tasks[best_task_id].task_front);
        if (current_agent_id < 0) continue;
        proposed_schedule[current_agent_id] = best_task_id;
        freeAgentMap[env->curr_states[current_agent_id].location] = -1;
        free_agents.erase(current_agent_id);
        if (heatflow_is_used || heatflow_filter_is_used) addToHeatFlowMap(env, waiting_tasks[best_task_id].locations);
        if (heat_is_used || filter_bottleneck || filter_timed_bottleneck || multi_criteria_is_used)
        {
            if (pibt_is_used || heat_is_used || filter_bottleneck) addToHeatMap(env, waiting_tasks[best_task_id].locations);
            if (filter_timed_bottleneck)
            {
                std::vector<int> path = {};
                std::vector<int> stays = {};
                if (singleagent) {singleAgentAStar(env, env->curr_states[current_agent_id].location, waiting_tasks[best_task_id].task_front, path,stays);}
                else {reducedMapWaypointsAStar(env, env->curr_states[current_agent_id].location, waiting_tasks[best_task_id].task_front, path);}
                path.insert(path.end(), waiting_tasks[best_task_id].locations.begin(), waiting_tasks[best_task_id].locations.end());
                stays.insert(stays.end(), waiting_tasks[best_task_id].stays.begin(), waiting_tasks[best_task_id].stays.end());
                waiting_tasks[best_task_id].locations.clear();
                waiting_tasks[best_task_id].locations = path;
                addToHeatTimeMap(env, path,stays);
            }
        }
        if (heat_is_used || agent_heat_is_used || filter_bottleneck || filter_timed_bottleneck)
        {
            total_makespan = total_makespan + waiting_tasks[best_task_id].makespan;
            avg_makespan = total_makespan/(working_agents +1);
        }
        running_tasks[best_task_id] = waiting_tasks[best_task_id];
        waiting_tasks.erase(best_task_id);
        running_tasks[best_task_id].started = env->curr_timestep;
        running_tasks[best_task_id].temperature = getFromHeatMap(env, running_tasks[best_task_id].locations);
        taskMap[running_tasks[best_task_id].task_front].erase(
                find(taskMap[running_tasks[best_task_id].task_front].begin(),
                taskMap[running_tasks[best_task_id].task_front].end(),
                best_task_id));
        starter_agents++;
        working_agents++;
        tasksMakespanSorted.clear();
        still_to_go = false;
        for(auto pt : waiting_tasks)
        {
            int push_heat;
            if (heat_is_used || fullheat_is_used)
                {
                    push_heat = getFromHeatMap(env, pt.second.locations);
                }
            if (heatflow_is_used) push_heat = getFromHeatFlowMap(env, pt.second.locations);
            if (filter_timed_bottleneck && !timedBottleneck(env, pt.second.locations, pt.second.stays))
            {
                tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
                still_to_go = true;
            }
            else if (filter_bottleneck && !bottleneck(env, pt.second.locations))
            {
                tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
                still_to_go = true;
            }
            else if (push_heat <= saturation_limit)
            {
                tasksMakespanSorted.push_back(make_pair(pt.second.makespan,pt.second.task_id));
                still_to_go = true;
            }
        }
        std::sort(tasksMakespanSorted.begin(),tasksMakespanSorted.end());
    }
    prev_task_schedule = proposed_schedule;
}
void task_finished(SharedEnvironment* env, int agent_id)
{
    if (heat_is_used || filter_bottleneck || multi_criteria_is_used || fullheat_is_used)
    {
        removeFromHeatMap(env, running_tasks[prev_task_schedule[agent_id]].locations);
    }
    if (heatflow_is_used || heatflow_filter_is_used)
    {
        removeFromHeatFlowMap(env, running_tasks[prev_task_schedule[agent_id]].locations);
    }
    if (heat_is_used || agent_heat_is_used || filter_bottleneck || filter_timed_bottleneck)
    {
        total_makespan = total_makespan - running_tasks[prev_task_schedule[agent_id]].makespan;
        avg_makespan = total_makespan/(working_agents -1);
    }
    if (running_tasks[prev_task_schedule[agent_id]].second_class)
    {
        second_class_counter--;
        task_second_finished++;
        second_class_histogram[running_tasks[prev_task_schedule[agent_id]].makespan]--;
    }
    if (game_map && running_tasks[prev_task_schedule[agent_id]].makespan > game_long_limit) game_long_count--;
    int actual_makespan = env->curr_timestep - running_tasks[prev_task_schedule[agent_id]].started;
    double current_coeff;
    if (running_tasks[prev_task_schedule[agent_id]].temperature == 0) {
        current_coeff = 0;
        }
    else {
        current_coeff = (env->curr_timestep - running_tasks[prev_task_schedule[agent_id]].started - running_tasks[prev_task_schedule[agent_id]].started_makespan) / running_tasks[prev_task_schedule[agent_id]].temperature;
    };
    measured_heat_coeff = (measured_heat_coeff * env->num_of_agents/20 + current_coeff) / (env->num_of_agents/20 + 1);
    heat_coeff = measured_heat_coeff;
    running_tasks.erase(prev_task_schedule[agent_id]);
    prev_task_schedule[agent_id] = -1;
}
void process_new_task(SharedEnvironment* env, Task task)
{
    ProcessedTask processed_task;
    processed_task.task_id = task.task_id;
    processed_task.task_front = task.locations[0];
    int makespan = 0;
    int size = task.locations.size();
    std::vector<int> waypoints = {};
    std::vector<int> stays = {};
    if (manhattan)
    {
        for (int i = 1; i < size; i++)
        {
            makespan = makespan + getManhattanDistance(env, task.locations[i-1], task.locations[i]);
        }
        processed_task.locations = task.locations;
    }
    else if (random_map)
    {
        for (int i = 1; i < size; i++)
        {
            if (singleagent)
            {
                makespan = makespan + singleAgentAStar(env, task.locations[i-1], task.locations[i], waypoints,stays);
            }
            else
            {
                makespan = makespan + reducedMapWaypointsAStar(env, task.locations[i-1], task.locations[i], waypoints);
            }
        }
        processed_task.locations = waypoints;
        processed_task.stays = stays;
    }
    else
    {
        for (int i = 1; i < size; i++)
        {
            if (singleagent)
            {
                makespan = makespan + singleAgentAStar(env, task.locations[i-1], task.locations[i], waypoints,stays);
            }
            else
            {
                std::pair<int,int> waypointStart = BFStoReducedMap(env, task.locations[i-1]);
                std::pair<int,int> waypointEnd = BFStoReducedMap(env, task.locations[i]);
                makespan = makespan + reducedMapWaypointsAStar(env, waypointStart.second, waypointEnd.second, waypoints) + waypointStart.first + waypointEnd.first;
            }
        }
        processed_task.locations = waypoints;
        processed_task.stays = stays;
    }
    if (warehouse_vertical)
    {
        int vertical = 0;
        for (int i = 0; i < size; i++)
        {
            vertical = vertical + task.locations[i]/env->cols;
        }
        vertical = vertical/size;
        if (vertical < 71) makespan = makespan + vertical;
        if (vertical > 70) makespan = makespan + 140 -vertical;
    }
    processed_task.makespan = makespan;
    if (offset_is_used)
    {
        int x_sum=0;
        int origo_x = origo/env->cols;
        for (int i = 0; i < size; i++)
        {
            x_sum = x_sum + task.locations[i]/env->cols - origo_x;
        }
        processed_task.makespan = makespan + std::abs(x_sum/size - origo_x);
    }
    if (game_map)
    {
        int loc_x, loc_y;
        int gate_count = 0;
        for (int i=0; i<processed_task.locations.size(); i++)
        {
            if (processed_task.second_class) break;
            loc_x = processed_task.locations[i]/env->cols;
            loc_y = processed_task.locations[i]%env->cols;
            if ( (loc_x > 80 && loc_y > 290 && loc_x < 98 && loc_y < 325) ||
                 (loc_x > 80 && loc_y > 395 && loc_x < 98 && loc_y < 420) )
                {
                    processed_task.second_class = true;
                    task_second_counter++;
                    second_class_histogram[processed_task.makespan]++;
                }
        }
    }
    if (city_map)
    {
        int loc_x, loc_y;
        for (int i=0; i<processed_task.locations.size(); i++)
        {
            if (processed_task.second_class) break;
            loc_x = processed_task.locations[i]/env->cols;
            loc_y = processed_task.locations[i]%env->cols;
            if ( (loc_x > 135 && loc_y > 153 && loc_x < 149 && loc_y < 166) ||
                 (loc_x > 91 && loc_y > 92 && loc_x < 96 && loc_y < 101) ||
                 (loc_x > 195 && loc_y > 248 && loc_x < 205 && loc_y < 256)
                )
                processed_task.second_class = true;
        }
    }
    if (fullheat_is_used) addToHeatMap(env,processed_task.locations);
    waiting_tasks[task.task_id] = processed_task;
    task_counter++;
}
void reduceMapStart(SharedEnvironment* env)
{
    reducedMap.clear();
    int distance = 0;
    for(int x = 0; x < env->rows;x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols;y++)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : distance + 1;
            if (distance > waypoint_step && x%waypoint_step == 0 && y%waypoint_step == 0) distance = 0;
            reducedMap.push_back(distance);
        }
    }
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > waypoint_step && x%waypoint_step == 0 && y%waypoint_step == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > waypoint_step && x%waypoint_step == 0 && y%waypoint_step == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > waypoint_step && x%waypoint_step == 0 && y%waypoint_step == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }
    reduceMapUpdate(env);
}
bool reduceMap(SharedEnvironment* env, bool keepSalient)
{
    bool changed = false;
    if(keepSalient)
    {
        for(int i = 0; i < reducedMap.size(); i++)
        {
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && areNeighboursTraversable(env, i) && ((countReducedVerticalNeighbours(env, i) + countReducedHorizontalNeighbours(env, i)) > 1))
            {
                changed = true;
                reducedMap[i] = 0;
            }
        }
    }
    else
    {
        std::vector<int> tmpReducedMap = reducedMap;
        for(int i = 0; i < reducedMap.size(); i++)
        {
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && (countReducedVerticalNeighbours(env, i) + countReducedHorizontalNeighbours(env, i)) == 1)
            {
                changed = true;
                tmpReducedMap[i] = 0;
            }
        }
        reducedMap = tmpReducedMap;
    }
    reduceMapUpdate(env);
    return changed;
}
bool areNeighboursTraversable(SharedEnvironment* env, int location)
{
    int x = location/env->cols;
    int y = location%env->cols;
    int horizontalNeighbourCount = countReducedHorizontalNeighbours(env, location);
    int verticalNeighbourCount = countReducedVerticalNeighbours(env, location);
    if((verticalNeighbourCount == 0 && horizontalNeighbourCount == 2) ||
       (verticalNeighbourCount == 2 && horizontalNeighbourCount == 0))
    {
        return false;
    }
    if((location >= env->cols && reducedMap[location-env->cols] > 0) &&
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isNorthEastEmpty(env, location, x, y))
    {
        return false;
    }
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) &&
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isSouthEastEmpty(env, location, x, y))
    {
        return false;
    }
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) &&
       (y > 0 && reducedMap[location-1] > 0) &&
       isSouthWestEmpty(env, location, x, y))
    {
        return false;
    }
    if((location >= env->cols && reducedMap[location-env->cols] > 0) &&
       (y > 0 && reducedMap[location-1] > 0) &&
       isNorthWestEmpty(env, location, x, y))
    {
        return false;
    }
    return true;
}
bool isNorthEastEmpty(SharedEnvironment* env, int location, int x, int y)
{
    if(x > 0 && y + 1 < env->cols && reducedMap[location-env->cols+1] > 0){return false;}
    return true;
}
bool isSouthEastEmpty(SharedEnvironment* env, int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 < env->cols && reducedMap[location+env->cols+1] > 0){return false;}
    return true;
}
bool isNorthWestEmpty(SharedEnvironment* env, int location, int x, int y)
{
    if(x > 0 && y > 0 && reducedMap[location-env->cols-1] > 0){return false;}
    return true;
}
bool isSouthWestEmpty(SharedEnvironment* env, int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 > 0 && reducedMap[location+env->cols-1] > 0){return false;}
    return true;
}
int countReducedVerticalNeighbours(SharedEnvironment* env, int location)
{
    int count = 0;
    if(location >= env->cols && reducedMap[location-env->cols] > 0) {count++;}
    if(location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) {count++;}
    return count;
}
int countReducedHorizontalNeighbours(SharedEnvironment* env, int location)
{
    int count = 0;
    int y = location % env->cols;
    if(y > 0 && reducedMap[location-1] > 0) {count++;}
    if(y+1 < env->cols && reducedMap[location+1] > 0) {count++;}
    return count;
}
void reduceMapUpdate(SharedEnvironment* env)
{
    int distance = 0;
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}
void reduceMapWaypointsStart(SharedEnvironment* env)
{
    reducedMapWaypoints.clear();
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
    }
    for(int i = 0; i < env->map.size(); i++)
    {
        if(reducedMap[i] > 0)
        {
            int locY = i % env->cols;
            if(i >= env->cols && reducedMap[i-env->cols] > 0)
            {
                int n = i-env->cols;
                reducedMapWaypoints[i].insert(make_pair(n,1));
            }
            if(locY + 1 < env->cols && reducedMap[i+1] > 0)
            {
                int e = i+1;
                reducedMapWaypoints[i].insert(make_pair(e,1));
            }
            if(i+env->cols < env->map.size() && reducedMap[i+env->cols] > 0)
            {
                int s = i+env->cols;
                reducedMapWaypoints[i].insert(make_pair(s,1));
            }
            if(locY > 0 && reducedMap[i-1] > 0)
            {
                int w = i-1;
                reducedMapWaypoints[i].insert(make_pair(w,1));
            }
        }
    }
}
bool reduceReduceMapWaypoints(SharedEnvironment* env, int distance)
{
    bool changed = false;
    for(int i = 0; i < env->map.size();i++)
    {
        std::unordered_map<int,int> neighbours = reducedMapWaypoints[i];
        if(reducedMap[i] > 0 && neighbours.size() == 2)
        {
            std::vector<int> keys;
            for(auto kv : neighbours)
            {
                keys.push_back(kv.first);
            }
            int n0 = keys[0];
            int n1 = keys[1];
            int newCost = 999999;
            if (reducedMapWaypoints[n0].find(n1) == reducedMapWaypoints[n0].end())
            {
                int locToN1Cost = reducedMapWaypoints[i].at(n1);
                int n0ToLocCost = reducedMapWaypoints[n0].at(i);
                newCost = n0ToLocCost + locToN1Cost;
                if(newCost == 2 && (countReducedVerticalNeighbours(env, i) != 2) && (countReducedHorizontalNeighbours(env, i) != 2))
                {
                    newCost++;
                }
                if(newCost < distance)
                {
                    reducedMap[i] = 0;
                    for (auto it = reducedMapWaypoints[n0].begin(); it != reducedMapWaypoints[n0].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n0].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n0].insert(make_pair(n1,newCost));
                    for (auto it = reducedMapWaypoints[n1].begin(); it != reducedMapWaypoints[n1].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n1].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n1].insert(make_pair(n0,newCost));
                    changed = true;
                }
            }
        }
    }
    return changed;
}
void hierarchyMapStart(SharedEnvironment* env, int grid)
{
    reducedMap.clear();
    for(int x = 0; x < env->rows;x++)
    {
        for(int y = 0; y < env->cols;y++)
        {
            int loc = x * env->cols + y;
            reducedMap.push_back(0);
        }
    }
    int gap_start = -1;
    for(int x = 0; x < env->rows;x++)
    {
        if (x%grid == 0)
        {
            gap_start = -1;
            for(int y = 0; y < env->cols;y++)
            {
                int loc = x * env->cols + y;
                if (gap_start >=0 && y == env->cols-1)
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                if (gap_start >=0 && y%grid == 0)
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                if (gap_start >= 0 && env->map[loc] == 1)
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                if (gap_start < 0 && env->map[loc] == 0)
                {
                    gap_start = loc;
                }
            }
        }
    }
    for(int y = 0; y < env->cols;y++)
    {
        if (y%grid == 0)
        {
            gap_start = -1;
            for(int x = 0; x < env->rows;x++)
            {
                int loc = x * env->cols + y;
                if (gap_start >=0 && x == env->rows-1)
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                if (gap_start >=0 && x%grid == 0)
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                if (gap_start >= 0 && env->map[loc] == 1)
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                if (gap_start < 0 && env->map[loc] == 0)
                {
                    gap_start = x;
                }
            }
        }
    }
}
void hierarchyMapWaypointsStart(SharedEnvironment* env, int grid)
{
    reducedMapWaypoints.clear();
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
    }
    for(int top_left_x = 0; top_left_x < env->rows; top_left_x = top_left_x + grid)
    {
        for(int top_left_y = 0; top_left_y < env->cols; top_left_y = top_left_y + grid)
        {
            for(int x = min(top_left_x + grid,env->rows-1); x > top_left_x; x--)
            {
                int loc_from = x * env->cols + top_left_y;
                if (reducedMap[loc_from] == 1)
                {
                    for(int y_top = top_left_y; y_top < min(top_left_y + grid,env->cols) ; y_top++)
                    {
                        int loc_to = top_left_x * env->cols + y_top;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                    for(int x_right = top_left_x; x_right < min(top_left_x + grid,env->rows); x_right++)
                    {
                        int loc_to = x_right * env->cols + top_left_y + grid;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to = (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                }
            }
            for(int y = top_left_y; y < min(top_left_y + grid,env->cols) ; y++)
            {
                int loc_from = top_left_x * env->cols + y;
                if (reducedMap[loc_from] == 1)
                {
                    for(int x_right = top_left_x; x_right < min(top_left_x + grid,env->rows); x_right++)
                    {
                        int loc_to = x_right * env->cols + top_left_y + grid;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,SOUTH,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to = (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,SOUTH,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                }
            }
            for(int x = top_left_x; x < min(top_left_x + grid,env->rows); x++)
            {
                int loc_from = x * env->cols + top_left_y + grid;
                if (reducedMap[loc_from] == 1)
                {
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to = (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(env, loc_from,WEST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;
                            }
                        }
                    }
                }
            }
        }
    }
}
int hierarchy_single_agent_plan(SharedEnvironment* env, int start,int start_direct,int end, int top_left_x, int top_left_y, int grid)
{
    int distance = -1;
    int bottom_right_x = min(top_left_x + grid,env->rows-1);
    int bottom_right_y = min(top_left_y + grid,env->cols-1);
    list<std::pair<int,int>> path;
    std::priority_queue<AstarNode*,std::vector<AstarNode*>,cmp> open_list;
    std::unordered_map<int,AstarNode*> all_nodes;
    std::unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(env, start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            distance = curr->g;
            break;
        }
        list<std::pair<int,int>> neighbors = getNeighbors(env, curr->location, curr->direction);
        for (const std::pair<int,int>& neighbor: neighbors)
        {
            int neighbor_x = neighbor.first/env->cols;
            int neighbor_y = neighbor.first%env->cols;
            if (neighbor_x < top_left_x || neighbor_y < top_left_y || neighbor_x > bottom_right_x || neighbor_y > bottom_right_y)
                continue;
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(env, neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return distance;
}
list<std::pair<int,int>> getNeighbors(SharedEnvironment* env, int location,int direction)
{
    list<std::pair<int,int>> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(env,forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction));
    return neighbors;
}
list<std::pair<int,int>> getNeighborsWithTurn(SharedEnvironment* env, int location,int direction)
{
    list<std::pair<int,int>> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int new_direction = direction;
    int forward = candidates[new_direction];
    if (forward>=0 && forward < env->map.size() && validateMove(env,forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    forward = candidates[new_direction];
    if (forward>=0 && forward < env->map.size() && validateMove(env,forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    forward = candidates[new_direction];
    if (forward>=0 && forward < env->map.size() && validateMove(env,forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    return neighbors;
}
void printReducedMap(SharedEnvironment* env)
{
}
int getManhattanDistance(SharedEnvironment* env, int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}
int single_agent_plan(SharedEnvironment* env,int start,int end, std::vector<int> & waypoints)
{
    int length = 0;
    int start_direct;
    if (start/env->cols < end/env->cols) start_direct = SOUTH; else start_direct = NORTH;
    std::priority_queue<AstarNode*,std::vector<AstarNode*>,cmp> open_list;
    std::unordered_map<int,AstarNode*> all_nodes;
    std::unordered_set<int> close_list;
    AstarNode* s1 = new AstarNode(start, SOUTH, 0, getManhattanDistance(env,start,end), nullptr);
    open_list.push(s1);
    all_nodes[start*4 + SOUTH] = s1;
    AstarNode* s2 = new AstarNode(start, NORTH, 0, getManhattanDistance(env,start,end), nullptr);
    open_list.push(s2);
    all_nodes[start*4 + NORTH] = s2;
    AstarNode* s3 = new AstarNode(start, EAST, 0, getManhattanDistance(env,start,end), nullptr);
    open_list.push(s3);
    all_nodes[start*4 + EAST] = s3;
    AstarNode* s4 = new AstarNode(start, WEST, 0, getManhattanDistance(env,start,end), nullptr);
    open_list.push(s4);
    all_nodes[start*4 + WEST] = s4;
    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            length = curr->g;
            while(curr->parent!=NULL)
            {
                waypoints.push_back(curr->location);
                curr = curr->parent;
            }
            break;
        }
        list<std::pair<int,int>> neighbors = getNeighbors(env, curr->location, curr->direction);
        for (const std::pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(env,neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return length;
}
int reducedMapWaypointsAStar(SharedEnvironment* env, int start, int end, std::vector<int> & waypoints)
{
    int length = 0;
    int newG;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::unordered_map<int,int> parents;
    std::pair<int,int> s = make_pair(getManhattanDistance(env,start,end), start);
    open_list.push(s);
    all_nodes[start] = 0;
    parents[start] = -1;
    while (!open_list.empty())
    {
        std::pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            length = all_nodes[currLocation];
            while(currLocation!=-1)
            {
                waypoints.push_back(currLocation);
                currLocation = parents[currLocation];
            }
            std::reverse( waypoints.begin(), waypoints.end() );
            break;
        }
        std::unordered_map<int,int> tmpMap = reducedMapWaypoints[currLocation];
        list<int> neighbors;
        for(auto kv : tmpMap)
        {
            neighbors.emplace_back(kv.first);
        }
        int currG = all_nodes[currLocation];
        for (const int& neighbor: neighbors)
        {
            if (parents[curr.second] >= 0)
            {
                if (neighbor == parents[currLocation]) { newG = currG + 2 + reducedMapWaypoints[currLocation].at(neighbor); }
                else if (neighbor/env->cols == parents[currLocation]/env->cols || neighbor%env->cols == parents[currLocation]%env->cols)
                {newG = currG + reducedMapWaypoints[currLocation].at(neighbor);}
                else {newG = currG + 1 + reducedMapWaypoints[currLocation].at(neighbor);};
            }
            else
            {
                newG = currG + reducedMapWaypoints[currLocation].at(neighbor);
            }
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(env,neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    parents.clear();
    return length;
}
int singleAgentAStar(SharedEnvironment* env, int start, int end, std::vector<int> & waypoints, std::vector<int> & stays)
{
    int length = 0;
    int newG, prevG, currG;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::unordered_map<int,int> parents;
    std::pair<int,int> s = make_pair(getManhattanDistance(env,start,end), start);
    open_list.push(s);
    all_nodes[start] = 0;
    parents[start] = -1;
    while (!open_list.empty())
    {
        std::pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            length = all_nodes[currLocation];
            prevG = length+1;
            while(currLocation!=-1)
            {
                waypoints.push_back(currLocation);
                currG = all_nodes[currLocation];
                stays.push_back(prevG - currG);
                prevG = currG;
                currLocation = parents[currLocation];
            }
            std::reverse( waypoints.begin(), waypoints.end() );
            std::reverse( stays.begin(), stays.end() );
            break;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        for (const int& neighbor: neighbors)
        {
            if (parents[curr.second] >= 0)
            {
                if (neighbor == parents[curr.second]) { newG = all_nodes[curr.second] + 3; }
                else if (neighbor/env->cols == parents[curr.second]/env->cols || neighbor%env->cols == parents[curr.second]%env->cols)
                {newG = all_nodes[curr.second] + 1;}
                else {newG = all_nodes[curr.second] + 2;};
            }
            else
            {
                newG = all_nodes[curr.second] + 1;
            };
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(env,neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    parents.clear();
    return length;
}
int BFStoA(SharedEnvironment* env, TimePoint endtime, int start)
{
    int agent = -1;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;
    int newG;
    while (!open_list.empty())
    {
        std::pair<int,int> curr = open_list.top();
        open_list.pop();
        if(freeAgentMap[curr.second] >= 0)
        {
            agent = freeAgentMap[curr.second];
            break;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
    return agent;
}
int BFStoWorkingA(SharedEnvironment* env, TimePoint endtime, int start)
{
    int agent = -1;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;
    while (!open_list.empty())
    {
        std::pair<int,int> curr = open_list.top();
        open_list.pop();
        if(workingAgentMap[curr.second] >= 0)
        {
            agent = workingAgentMap[curr.second];
            break;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        int newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
    return agent;
}
void BFStoTWithTurn(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<pair<int,int>> & tasks)
{
    int newG;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::unordered_map<int,int> parents;
    std::pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;
    int distance_limit = -1;
    parents[start] = -1;
    while (!open_list.empty() && std::chrono::steady_clock::now() < endtime)
    {
        std::pair<int,int> curr = open_list.top();
        open_list.pop();
        if( !taskMap[curr.second].empty())
        {
            for (int i=0; i < taskMap[curr.second].size(); i++)
                tasks.push_back(make_pair(curr.first,taskMap[curr.second].at(i)));
            if (distance_limit < 0) distance_limit = curr.first + bfstotask_extra;
            continue;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        if (distance_limit >=0 && all_nodes[curr.second] >= distance_limit) continue;
        for (const int& neighbor: neighbors)
        {
            if (parents[curr.second] >= 0)
            {
                if (neighbor == parents[curr.second]) { newG = all_nodes[curr.second] + 3; }
                else if (neighbor/env->cols == parents[curr.second]/env->cols || neighbor%env->cols == parents[curr.second]%env->cols)
                {newG = all_nodes[curr.second] + 1;}
                else {newG = all_nodes[curr.second] + 2;};
            }
            else
            {
                int candidates[4] = { start + 1, start + env->cols, start - 1, start - env->cols};
                if (neighbor == candidates[start_direct]) {newG = 1;}
                else {newG = 2;};
            };
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
                parents[neighbor] = curr.second;
            }
        }
    }
    all_nodes.clear();
    parents.clear();
}
void BFStoT(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<pair<int,int>> & tasks)
{
    int newG;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::unordered_map<int,int> parents;
    std::pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;
    int distance_limit = -1;
    while (!open_list.empty() && std::chrono::steady_clock::now() < endtime)
    {
        std::pair<int,int> curr = open_list.top();
        open_list.pop();
        if( !taskMap[curr.second].empty())
        {
            for (int i=0; i < taskMap[curr.second].size(); i++)
                tasks.push_back(make_pair(curr.first,taskMap[curr.second].at(i)));
            if (distance_limit < 0) distance_limit = curr.first + bfstotask_extra;
            continue;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        if (distance_limit >=0 && all_nodes[curr.second] >= distance_limit) continue;
        newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
}
void single_agent_BFStoT(SharedEnvironment* env, TimePoint endtime, int start, int start_direct, std::vector<std::pair<int,int>> & tasks)
{
    int length = 0;
    int gIncrement;
    std::priority_queue<AstarNode*,std::vector<AstarNode*>,cmp> open_list;
    std::unordered_map<int,AstarNode*> all_nodes;
    std::unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, 0, nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    int distance_limit = -1;
    while (!open_list.empty() && std::chrono::steady_clock::now() < endtime)
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if( !taskMap[curr->location].empty())
        {
            for (int i=0; i < taskMap[curr->location].size(); i++)
                tasks.push_back(make_pair(curr->g,taskMap[curr->location].at(i)));
            if (distance_limit < 0) distance_limit = curr->g + bfstotask_extra;
            continue;
        }
        list<std::pair<int,int>> neighbors = getNeighborsWithTurn(env, curr->location, curr->direction);
        for (const std::pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (curr->parent!=NULL)
            {
                if (neighbor.second == (curr->parent)->location) { gIncrement = 3; }
                else if (neighbor.second/env->cols == (curr->parent)->location/env->cols || neighbor.second%env->cols == (curr->parent)->location%env->cols)
                {gIncrement = 1;}
                else {gIncrement = 2;};
            }
            else
            {
                int candidates[4] = { start + 1, start + env->cols, start - 1, start - env->cols};
                if (neighbor.second == candidates[start_direct]) {gIncrement = 1;}
                else {gIncrement = 2;};
            };
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + gIncrement < old->g)
                {
                    old->g = curr->g+gIncrement;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+gIncrement,0, curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
}
std::pair<int,int> BFStoReducedMap(SharedEnvironment* env, int start)
{
    std::pair<int,int> waypoint;
    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>> > open_list;
    std::unordered_map<int,int> all_nodes;
    std::pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;
    while (!open_list.empty())
    {
        std::pair<int,int> curr = open_list.top();
        open_list.pop();
        if(reducedMap[curr.second] > 0)
        {
            waypoint = curr;
            break;
        }
        list<int> neighbors = getPlainNeighbors(env, curr.second);
        int newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
    return waypoint;
}
list<int> getPlainNeighbors(SharedEnvironment* env, int location)
{
    list<int> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    for(int i = 0; i < 4; i++)
    {
        int forward = candidates[i];
        if(forward >= 0 && forward < env->map.size() && validateMove(env, forward, location))
        {
            neighbors.emplace_back(forward);
        }
    }
    return neighbors;
}
bool validateMove(SharedEnvironment* env, int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;
    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;
}
void addAgentsToHeatMap(SharedEnvironment* env, std::vector<State> states)
{
    heatAgentMap.clear();
    heatAgentMap.resize(heat_map_size, 0);
    int size = states.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    for (int i=0; i<size;i++)
    {
        loc = states[i].location;
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
        heatAgentMap[heat_map_loc]++;
    }
}
void updateAgentMap(SharedEnvironment* env)
{
    agentMap.clear();
    agentMap.resize(env->map.size(),-1);
    for(int i = 0; i < env->num_of_agents; i++)
    {
        agentMap[env->curr_states[i].location] = i;
    }
}
void addToHeatMap(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc, prev_heat_map_loc;
    prev_heat_map_loc = -1;
    for (int i=0; i<size;i++)
    {
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
        if (heat_map_loc != prev_heat_map_loc)
        {
            total_heat++;
            heatMap[heat_map_loc]++;
        }
        prev_heat_map_loc = heat_map_loc;
    }
    avg_heat = total_heat/heat_map_size;
}
void removeFromHeatMap(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc, prev_heat_map_loc;
    prev_heat_map_loc = -1;
    for (int i=0; i<size;i++)
    {
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
        if (heat_map_loc != prev_heat_map_loc)
        {
            total_heat--;
            heatMap[heat_map_loc]--;
        }
        prev_heat_map_loc = heat_map_loc;
    }
    avg_heat = total_heat/heat_map_size;
}
double getFromHeatMap(SharedEnvironment* env, std::vector<int> locations)
{
    double sum = 0;
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    for (int i=0; i<size;i++)
    {
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
        sum = sum + heatMap_coeff*heatMap[heat_map_loc] + heatAgentMap_coeff*heatAgentMap[heat_map_loc];
    }
    return sum;
}
bool bottleneck(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    for (int i=0; i<size;i++)
    {
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
        if (heatMap_coeff*heatMap[heat_map_loc] + heatAgentMap_coeff*heatAgentMap[heat_map_loc] > heat_map_capacity_percentage * heatMapCapacity[heat_map_loc] / 100) return true;
    }
    return false;
}
void addToHeatTimeMap(SharedEnvironment* env, std::vector<int> locations, std::vector<int> stays)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc, prev_heat_map_loc;
    int start_step, time_step;
    int actual_window = std::min(size,timed_window);
    int step_window;
    prev_heat_map_loc = -1;
    time_step = env->curr_timestep + 1;
    for (int i=0; i<size;i++)
    {
        step_window = std::min(i/3,actual_window);
        step_window = step_window + stays[i];
        if (time_step >= MAX_STEPS - step_window) break;
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
            total_heat++;
            int k=0;
            for (int j=k; j<step_window; j++)
            {
                heatTimeMap[time_step+j][heat_map_loc]++;
            }
        prev_heat_map_loc = heat_map_loc;
        time_step = time_step + stays[i];
    }
}
void removeFromHeatTimeMap(SharedEnvironment* env, int added_time, std::vector<int> locations)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc, prev_heat_map_loc;
    int time_step;
    prev_heat_map_loc = -1;
    time_step = added_time + 1;
    for (int i=0; i < size; i++)
    {
        if (time_step >= MAX_STEPS-timed_window) break;
        loc = locations[i];
        loc_x = loc/env->cols;
        loc_y = loc%env->cols;
        heat_map_loc_x = loc_x/heatgridsize;
        heat_map_loc_y = loc_y/heatgridsize;
        heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
            total_heat--;
            int k;
            if (time_step < timed_window) {k = 0;} else {k = 0 - timed_window;};
            for (int j=k; j<timed_window; j++)
            {
                heatTimeMap[time_step][heat_map_loc]--;
            }
        prev_heat_map_loc = heat_map_loc;
        time_step++;
    }
}
bool timedBottleneck(SharedEnvironment* env, std::vector<int> locations, std::vector<int> stays)
{
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    int time_step;
    time_step = env->curr_timestep + 1;
    for (int i=0; i<size;i++)
    {
        for (int j=0; j<stays[i]; j++)
        {
            if (time_step >= MAX_STEPS) break;
            loc = locations[i];
            loc_x = loc/env->cols;
            loc_y = loc%env->cols;
            heat_map_loc_x = loc_x/heatgridsize;
            heat_map_loc_y = loc_y/heatgridsize;
            heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
            if (heatTimeMap[time_step][heat_map_loc] >= heat_map_capacity_percentage * heatMapCapacity[heat_map_loc] / 100) return true;
            time_step++;
        }
    }
    return false;
}
double getFromHeatTimeMap(SharedEnvironment* env, std::vector<int> locations, std::vector<int> stays)
{
    double sum = 0;
    int size = locations.size();
    int loc, loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    int time_step;
    time_step = env->curr_timestep + 1;
    for (int i=0; i<size;i++)
    {
        for (int j=0; j<stays[i]; j++)
        {
            if (time_step >= MAX_STEPS) break;
            loc = locations[i];
            loc_x = loc/env->cols;
            loc_y = loc%env->cols;
            heat_map_loc_x = loc_x/heatgridsize;
            heat_map_loc_y = loc_y/heatgridsize;
            heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
            sum = sum + heatTimeMap[time_step][heat_map_loc];
            time_step++;
        }
    }
    return sum;
}
void addToHeatMapWP(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    for (int i=0; i<size;i++)
    {
        heatMapWP[locations[i]] = heatMapWP[locations[i]] + 1/static_cast<double>(widthMap[locations[i]]);
    }
}
void removeFromHeatMapWP(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    for (int i=0; i<size;i++)
    {
        heatMapWP[locations[i]] = heatMapWP[locations[i]] - 1/static_cast<double>(widthMap[locations[i]]);
    }
}
double getFromHeatMapWP(SharedEnvironment* env, std::vector<int> locations)
{
    double sum = 0;
    int size = locations.size();
    for (int i=0; i<size;i++)
    {
        sum = sum + heatMapWP[locations[i]];
    }
    return sum/static_cast<double>(size);
}
void addToHeatFlowMap(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    int loc1, loc2;
    for (int i=1; i<size;i++)
    {
        loc1 = locations[i-1];
        loc2 = locations[i];
        if (heatFlowMap[loc2]->find(loc1) == heatFlowMap[loc2]->end()) heatFlowMap[loc2]->insert(make_pair(loc1,1));
        else heatFlowMap[loc2]->at(loc1)++;
    }
}
void removeFromHeatFlowMap(SharedEnvironment* env, std::vector<int> locations)
{
    int size = locations.size();
    int loc1, loc2;
    for (int i=1; i<size;i++)
    {
        loc1 = locations[i-1];
        loc2 = locations[i];
        heatFlowMap[loc2]->at(loc1)--;
    }
}
double getFromHeatFlowMap(SharedEnvironment* env, std::vector<int> locations)
{
    double sum = 0;
    int size = locations.size();
    for (int i=1; i<size;i++)
    {
        if (heatFlowMap[locations[i-1]]->find(locations[i]) == heatFlowMap[locations[i-1]]->end()) continue;
        sum = sum + heatFlowMap[locations[i-1]]->at(locations[i]);
    }
    return sum;
}
void createHeatMap(SharedEnvironment* env, int gridsize_here)
{
    int loc_x, loc_y, heat_map_loc_x, heat_map_loc_y, heat_map_loc;
    heat_map_cols = env->cols/gridsize_here + 1;
    heat_map_rows = env->rows/gridsize_here + 1;
    heat_map_size = heat_map_cols * heat_map_rows;
    if (heat_map_size > MAX_HEATMAPSIZE) {
        std::terminate();
    }
    heatMap.clear();
    if (random_map) {heatMap.resize(heat_map_size, 1);}
    else {heatMap.resize(heat_map_size, 1);}
    if (game_map)
    {
        for (int i=0; i < 1; i++)
        {
            vector<int> init = {46995, 123363};
            addToHeatMap(env, init);
            init = {46995};
            addToHeatMap(env, init);
        }
    }
    heatAgentMap.clear();
    heatAgentMap.resize(heat_map_size, 0);
    if (filter_timed_bottleneck || heat_time_is_used)
    {
        heatTimeMap.clear();
        for (int i=0; i <= MAX_STEPS; i++)
        {
            vector<int> v1;
            for (int j=0; j < heat_map_size; j++ )
            {
                v1.push_back(0);
            }
            heatTimeMap.push_back(v1);
        }
    }
    heatMapCapacity.clear();
    heatMapCapacity.resize(heat_map_size, 0);
    if (filter_bottleneck || filter_timed_bottleneck)
    {
        for (int loc=0; loc<env->map.size(); loc++)
        {
            loc_x = loc/env->cols;
            loc_y = loc%env->cols;
            heat_map_loc_x = loc_x/gridsize_here;
            heat_map_loc_y = loc_y/gridsize_here;
            heat_map_loc = heat_map_loc_x * heat_map_cols + heat_map_loc_y;
            if (env->map[loc] == 0) heatMapCapacity.at(heat_map_loc) = heatMapCapacity.at(heat_map_loc) + 1;
        }
    }
    if (heatflow_is_used || heatflow_filter_is_used)
    {
        heatFlowMap.clear();
        for(int i = 0; i < env->map.size(); i++)
        {
            std::unordered_map<int,int> *linkHeat = new std::unordered_map<int,int>;
            heatFlowMap.push_back(linkHeat);
        }
    }
}
}
