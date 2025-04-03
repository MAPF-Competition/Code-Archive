#include <cmath>
#include "PyCompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>
#include "global_greedy_matching.h"

using json = nlohmann::ordered_json;

// // This function might not work correctly with small map (w or h <=2)
// bool PyBaseSystem::valid_moves(vector<State>& prev, vector<Action>& action)
// {
//   return model->is_valid(prev, action);
// }

void PyBaseSystem::reset() {
    total_timetous = 0;
    // sync state
    sync_shared_env();
}

void PyBaseSystem::step() {
    
    vector<Action> all_wait_actions(num_of_agents, Action::NA);
    
    // schedule tasks and plan paths
    auto start = std::chrono::steady_clock::now();
    int timeout_timesteps = 0;
    plan(timeout_timesteps);
    auto end = std::chrono::steady_clock::now();
    auto diff = end-start;
    planner_times.push_back(std::chrono::duration<double>(diff).count());
    total_timetous += timeout_timesteps;

    for (int i = 0 ; i< timeout_timesteps; i ++){
        simulator->move(all_wait_actions);
    }

    // move drives
    vector<State> curr_states = simulator->move(proposed_actions);
    
    // update tasks
    task_manager->update_tasks(curr_states, proposed_schedule, simulator->get_curr_timestep());

    // report results
    int timestep = simulator->get_curr_timestep();
    logger->log_info("t: "+std::to_string(timestep)+", completed tasks: "+std::to_string(task_manager->num_of_task_finish)+
            ", throughput: "+std::to_string(float(task_manager->num_of_task_finish)/timestep), timestep);

    sync_shared_env();

}

void PyBaseSystem::step_init() {
    env->plan_start_time = std::chrono::steady_clock::now();
}

void PyBaseSystem::step_scheduler() {
    planner->scheduler->plan(plan_time_limit, proposed_schedule);
    planner->update_goal_locations(proposed_schedule);
}

// TODO: test
void PyBaseSystem::step_scheduler(const py::array_t<int> & _proposed_schedule) {
    proposed_schedule.clear();
    const int * proposed_schedule_ptr = _proposed_schedule.data();
    ssize_t size = _proposed_schedule.size();
    for (ssize_t i = 0; i < size; i++) {
        proposed_schedule.push_back(proposed_schedule_ptr[i]);
    }
    planner->update_goal_locations(proposed_schedule);
}

void PyBaseSystem::step_planner() {
    planner->planner->plan(plan_time_limit, proposed_actions);
    _update_plan();
}

void PyBaseSystem::step_planner(const py::array_t<int> & _proposed_actions) {
    proposed_actions.resize(num_of_agents, Action::NA);
    const int * proposed_actions_ptr = _proposed_actions.data();
    ssize_t size = _proposed_actions.size();
    for (ssize_t i = 0; i < size; i++) {
        proposed_actions[i] = static_cast<Action>(proposed_actions_ptr[i]);
    }
    _update_plan();
}

void PyBaseSystem::_update_plan() {
    // move drives
    vector<State> curr_states = simulator->move(proposed_actions);
    // update tasks
    task_manager->update_tasks(curr_states, proposed_schedule, simulator->get_curr_timestep());
    // report results
    int timestep = simulator->get_curr_timestep();
    logger->log_info("t: "+std::to_string(timestep)+", completed tasks: "+std::to_string(task_manager->num_of_task_finish)+
            ", throughput: "+std::to_string(float(task_manager->num_of_task_finish)/timestep), timestep);
    logger->flush();

    sync_shared_env();
}

py::array_t<int> PyBaseSystem::get_curr_positions() {
    py::array_t<int> positions(env->num_of_agents);
    
    int * data_ptr = positions.mutable_data();

    for (int i=0;i<env->num_of_agents;i++) {
        data_ptr[i] = env->curr_states[i].location;
    }

    return positions;
}

py::array_t<int> PyBaseSystem::get_curr_orientations() {
    py::array_t<int> orientations(env->num_of_agents);
    
    int * data_ptr = orientations.mutable_data();

    for (int i=0;i<env->num_of_agents;i++) {
        data_ptr[i] = env->curr_states[i].orientation;
    }

    return orientations;
}

py::array_t<int> PyBaseSystem::get_target_positions() {
    py::array_t<int> positions(env->num_of_agents);
    
    int * data_ptr = positions.mutable_data();

    for (int i=0;i<env->num_of_agents;i++) {
        data_ptr[i] = env->goal_locations[i].empty() ? (-1):(env->goal_locations[i].front().first);
    }

    return positions;
}

std::tuple<py::array_t<int>, py::array_t<int> > PyBaseSystem::get_assignable_agents() {
    auto assignable_agents_ids = new std::vector<int> ();
    auto assignable_agents_locs = new std::vector<int> ();

    for (int i_agent=0;i_agent<env->num_of_agents;i_agent++)
    {
        if (env->curr_task_schedule[i_agent] == -1) {
            assignable_agents_ids->push_back(i_agent);
            assignable_agents_locs->push_back(env->curr_states[i_agent].location);
        } else {
            auto task_id = env->curr_task_schedule[i_agent];
            auto & task = env->task_pool[task_id];
            if (task.idx_next_loc == 0) {
                assignable_agents_ids->push_back(i_agent);
                assignable_agents_locs->push_back(env->curr_states[i_agent].location);
            }
        }
    }

    // Wrap the vector in a NumPy array (without copying)
    auto capsule_ids = py::capsule(
        assignable_agents_ids, [](void *p) {
        delete reinterpret_cast<std::vector<int> *>(p); // Cleanup
    });

    auto capsule_locs = py::capsule(
        assignable_agents_locs, [](void *p) {
        delete reinterpret_cast<std::vector<int> *>(p); // Cleanup
    });

    return {
        py::array(
            assignable_agents_ids->size(), // shape
            assignable_agents_ids->data(), // the data pointer
            capsule_ids // numpy array references this parent
        ),
        py::array(
            assignable_agents_locs->size(), // shape
            assignable_agents_locs->data(), // the data pointer
            capsule_locs // numpy array references this parent
        )
    };
}

std::tuple<py::array_t<int>, py::array_t<int> > PyBaseSystem::get_assignable_tasks() {
    auto assignable_tasks_ids = new std::vector<int> ();
    auto assignable_tasks_locs = new std::vector<int> ();

    for (auto & pair : env->task_pool)
    {
        auto & task = pair.second;
        if (task.agent_assigned == -1 
            || task.idx_next_loc==0) {
            assignable_tasks_ids->push_back(task.task_id);

            // push back goal locations as well
            for (auto & loc : task.locations)
            {
                assignable_tasks_locs->push_back(loc);
            }
            for (size_t i = task.locations.size(); i < max_task_locs_num; i++)
            {
                assignable_tasks_locs->push_back(task.locations.back());
            }
        }
    }

    auto capsule_ids = py::capsule(
        assignable_tasks_ids, [](void *p) {
        delete reinterpret_cast<std::vector<int> *>(p); // Cleanup
    });

    auto capsule_locs = py::capsule(
        assignable_tasks_locs, [](void *p) {
        delete reinterpret_cast<std::vector<int> *>(p); // Cleanup
    });

    auto cols_locs = max_task_locs_num ;
    auto rows_locs = assignable_tasks_locs->size() / max_task_locs_num;

    return {
        py::array(
            assignable_tasks_ids->size(), // shape
            assignable_tasks_ids->data(), // the data pointer
            capsule_ids // numpy array references this parent
        ),
        py::array(
            {rows_locs, cols_locs}, // shape
            assignable_tasks_locs->data(), // the data pointer
            capsule_locs // numpy array references this parent
        )
    };
}

size_t PyBaseSystem::get_mask_task_loc_num() {
    if (env->map_name.substr(0,4)=="ware") {
        return 2;
    } else if (env->map_name.substr(0,4)=="sort") {
        return 2;
    } else if (env->map_name.substr(0,4)=="brc2") {
        return 3;
    } else {
        return 5;
    }
}

py::array_t<int> PyBaseSystem::global_greedy_matching(
    const py::array_t<int> & assignable_agents_indices,
    const py::array_t<int> & assignable_tasks_indices,
    const py::array_t<int> & sorted_tasks_costs,
    const py::array_t<int> & sorted_tasks_indices
) {
    GlobalGreedyMatching ggm(sorted_tasks_costs, sorted_tasks_indices);
    std::unordered_map<int,int> assignment=ggm.get_assignment();


    auto _assignable_agents_indices = assignable_agents_indices.data();
    auto _assignable_tasks_indices = assignable_tasks_indices.data();

    auto _proposed_schedule = new std::vector<int> (
        env->curr_task_schedule.begin(), 
        env->curr_task_schedule.end()
    );

    auto & proposed_schedule = *_proposed_schedule;
    
    // update the rescedule from the matching
    for (auto it: assignment)
    {
        if (it.first==-1 || it.second==-1) {
            std::cout<<"error: TODO 10314"<<std::endl;
        }
        int i_agent = _assignable_agents_indices[it.first];
        int i_task = _assignable_tasks_indices[it.second];
        proposed_schedule[i_agent] = i_task;
        // env->task_pool[i_task].agent_assigned = i_agent;
    }

    auto capsule = py::capsule(
        _proposed_schedule, [](void *p) {
        delete reinterpret_cast<std::vector<int> *>(p); // Cleanup
    });

    return py::array(
        _proposed_schedule->size(), // shape
        _proposed_schedule->data(), // the data pointer
        capsule // numpy array references this parent
    );

}



void PyBaseSystem::sync_shared_env() 
{
    if (!started)
    {
        env->goal_locations.resize(num_of_agents);
        task_manager->sync_shared_env(env);
        simulator->sync_shared_env(env);

        if (simulator->get_curr_timestep() == 0)
        {
            env->new_freeagents.reserve(num_of_agents); //new free agents are empty in task_manager on initialization, set it after task_manager sync
            for (int i = 0; i < num_of_agents; i++)
            {
                env->new_freeagents.push_back(i);
            }
        }
        //update proposed action to all wait
        proposed_actions.clear();
        proposed_actions.resize(num_of_agents, Action::W);
        //update proposed schedule to previous assignment
        proposed_schedule = env->curr_task_schedule;
        
    }
    else
    {
        env->curr_timestep = simulator->get_curr_timestep();
    }
}


bool PyBaseSystem::planner_wrapper()
{
    planner->compute(plan_time_limit, proposed_actions, proposed_schedule);
    return true;
}


void PyBaseSystem::plan(int & timeout_timesteps)
{

    using namespace std::placeholders;
    int timestep = simulator->get_curr_timestep();

    std::packaged_task<bool()> task(std::bind(&PyBaseSystem::planner_wrapper, this));


    future = task.get_future();
    if (task_td.joinable()){
        task_td.join();
    }
    env->plan_start_time = std::chrono::steady_clock::now();
    task_td = std::thread(std::move(task));

    started = true;

    while (timestep + timeout_timesteps < simulation_time){

        if (future.wait_for(std::chrono::milliseconds(plan_time_limit)) == std::future_status::ready)
            {
                task_td.join();
                started = false;
                auto res = future.get();

                logger->log_info("planner returns", timestep + timeout_timesteps);
                return;
            }
        logger->log_info("planner timeout", timestep + timeout_timesteps);
        timeout_timesteps += 1;
    }

    //
}


bool PyBaseSystem::planner_initialize()
{
    using namespace std::placeholders;
    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    env->plan_start_time = std::chrono::steady_clock::now();
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::milliseconds(preprocess_time_limit)) == std::future_status::ready)
    {
        init_td.join();
        return true;
    }

    init_td.detach();
    return false;
}


void PyBaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;
    if (succ)
    {
        logger->log_info("Preprocessing success", simulator->get_curr_timestep());
    } 
    else
    {
        logger->log_fatal("Preprocessing timeout", simulator->get_curr_timestep());
    }
    logger->flush();
}


void PyBaseSystem::simulate(int simulation_time)
{
    //init logger
    //Logger* log = new Logger();

    // NOTE: different from official codes, it is already initialized in the constructor
    // initialize();

    this->simulation_time = simulation_time;

    vector<Action> all_wait_actions(num_of_agents, Action::NA);

    for (; simulator->get_curr_timestep() < simulation_time; )
    {
        // find a plan
        sync_shared_env();

        auto start = std::chrono::steady_clock::now();

        int timeout_timesteps = 0;

        plan(timeout_timesteps);

        auto end = std::chrono::steady_clock::now();

        for (int i = 0 ; i< timeout_timesteps; i ++){
            simulator->move(all_wait_actions);
            for (int a = 0; a < num_of_agents; a++)
                {
                    if (!env->goal_locations[a].empty())
                        solution_costs[a]++;
                }
        }

        total_timetous+=timeout_timesteps;

        if (simulator->get_curr_timestep() >= simulation_time){

            auto diff = end-start;
            planner_times.push_back(std::chrono::duration<double>(diff).count());
            break;
        }

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        vector<State> curr_states = simulator->move(proposed_actions);
        int timestep = simulator->get_curr_timestep();
        // agents do not move


        auto diff = end-start;
        planner_times.push_back(std::chrono::duration<double>(diff).count());

        // update tasks
        task_manager->update_tasks(curr_states, proposed_schedule, simulator->get_curr_timestep());

        logger->log_info("t: "+std::to_string(timestep)+", completed tasks: "+std::to_string(task_manager->num_of_task_finish)+
            ", throughput: "+std::to_string(float(task_manager->num_of_task_finish)/timestep), timestep);
    }
}


void PyBaseSystem::initialize()
{
    env->num_of_agents = num_of_agents;
    env->rows = grid->rows;
    env->cols = grid->cols;
    env->map = grid->map;

    
    // // bool succ = load_records(); // continue simulating from the records
    // timestep = 0;
    // curr_states = starts;

    int timestep = simulator->get_curr_timestep();

    //planner initilise before knowing the first goals
    bool planner_initialize_success= planner_initialize();
    
    log_preprocessing(planner_initialize_success);
    if (!planner_initialize_success)
        _exit(124);

    // initialize_goal_locations();
    task_manager->reveal_tasks(timestep); //this also intialize env->new_tasks

    sync_shared_env();

    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }

    proposed_actions.resize(num_of_agents, Action::W);
    proposed_schedule.resize(num_of_agents, -1);
}


void PyBaseSystem::saveResults(const string &fileName, int screen) const
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF_T";
    js["version"] = "2024 LoRR";

    // std::string feasible = fast_mover_feasible ? "Yes" : "No";
    // js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    js["numTaskFinished"] = task_manager->num_of_task_finish;
    int makespan = 0;
    if (num_of_agents > 0)
    {
        makespan = solution_costs[0];
        for (int a = 1; a < num_of_agents; a++)
        {
            if (solution_costs[a] > makespan)
            {
                makespan = solution_costs[a];
            }
        }
    }
    js["makespan"] = makespan;

    js["numPlannerErrors"] = simulator->get_number_errors();
    js["numScheduleErrors"] = task_manager->get_number_errors();

    js["numEntryTimeouts"] = total_timetous;

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        js["start"] = simulator->starts_to_json();
    }
    
    if (screen <= 2)
    {
        js["actualPaths"] = simulator->actual_path_to_json();
    }

    if (screen <=1)
    {
        js["plannerPaths"] = simulator->planned_path_to_json();

        json planning_times = json::array();
        for (double time: planner_times)
            planning_times.push_back(time);
        js["plannerTimes"] = planning_times;

        // Save errors
        js["errors"] = simulator->action_errors_to_json();

        //actual schedules
        json aschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager->actual_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
            }  
            aschedules.push_back(schedules);
        }

        js["actualSchedule"] = aschedules;

        //planned schedules
        json pschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager->planner_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
                
            }  
            pschedules.push_back(schedules);
        }

        js["plannerSchedule"] = pschedules;

        // Save errors
        json schedule_errors = json::array();
        for (auto error: task_manager->schedule_errors)
        {
            std::string error_msg;
            int t_id;
            int agent1;
            int agent2;
            int timestep;
            std::tie(error_msg,t_id,agent1,agent2,timestep) = error;
            json e = json::array();
            e.push_back(t_id);
            e.push_back(agent1);
            e.push_back(agent2);
            e.push_back(timestep);
            e.push_back(error_msg);
            schedule_errors.push_back(e);
        }

        js["scheduleErrors"] = schedule_errors;

        // Save events
        json event = json::array();
        for(auto e: task_manager->events)
        {
            json ev = json::array();
            int timestep;
            int agent_id;
            int task_id;
            int seq_id;
            std::tie(timestep,agent_id,task_id,seq_id) = e;
            ev.push_back(timestep);
            ev.push_back(agent_id);
            ev.push_back(task_id);
            ev.push_back(seq_id);
            event.push_back(ev);
        }
        js["events"] = event;

        // Save all tasks
        json tasks = task_manager->to_json(grid->cols);
        js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;

}


