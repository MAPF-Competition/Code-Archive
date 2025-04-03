#pragma once

#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "Entry.h"
#include "Logger.h"
#include "TaskManager.h"
#include <pthread.h>
#include <future>
#include "Simulator.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

namespace py=pybind11;

class PyBaseSystem
{
public:
    Logger* logger = nullptr;

	PyBaseSystem(
        std::string & map_path, 
        std::string & agents_path, 
        std::string & tasks_path,
        int num_of_agents,
        int plan_time_limit,
        int preprocess_time_limit,
        float num_tasks_reveal,
        std::string & _log_level
    ): num_of_agents(num_of_agents) {
        int log_level;

        if (_log_level == "fatal")
        {
            log_level = 5;
        }
        else if (_log_level == "error")
        {
            log_level = 4;
        }
        else if (_log_level == "warning")
        {
            log_level = 3;
        }
        else if (_log_level == "info")
        {
            log_level = 2;
        }
        else if (_log_level == "debug")
        {
            log_level = 1;
        } else {
            std::cerr<<"unknown log level: "<<_log_level<<std::endl;
            exit(-1);
        }

        logger = new Logger("", log_level); // log_level = 5: fatal
    
        grid = std::make_shared<Grid>(map_path);
        planner = new Entry();
        planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);
        env = planner->env;

        model = new ActionModelWithRotate(*grid);
        model->set_logger(logger);

        start_locs = read_int_vec(agents_path, num_of_agents);
        tasks = read_int_vec(tasks_path);

        simulator = std::make_shared<Simulator>(*grid, start_locs, model);
        task_manager = std::make_shared<TaskManager>(tasks, num_of_agents);


        max_task_locs_num = get_mask_task_loc_num();

        this->set_logger(logger);
        this->set_plan_time_limit(plan_time_limit);
        this->set_preprocess_time_limit(preprocess_time_limit);

        this->set_num_tasks_reveal(num_tasks_reveal);

        this->initialize();
    };

	virtual ~PyBaseSystem()
    {
        //safely exit: wait for join the thread then delete planner and exit
        if (started)
        {
            task_td.join();
        }
        if (planner != nullptr)
        {
            delete planner;
        }
        if (logger != nullptr)
        {
            delete logger;
        }
        if (model != nullptr)
        {
            delete model;
        }
    };

    void set_num_tasks_reveal(float num){task_manager->set_num_tasks_reveal(num);};
    void set_plan_time_limit(int limit){plan_time_limit = limit;};
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};
    void set_log_level(int level){log_level = level;};
    void set_logger(Logger* logger){
        this->logger = logger;
        task_manager->set_logger(logger);
    }

    void simulate(int simulation_time);
    void plan(int & timeout_timesteps);
    bool planner_wrapper();

    //void saveSimulationIssues(const string &fileName) const;
    void saveResults(const string &fileName, int screen) const;

    std::shared_ptr<Grid> grid;

    int simulation_time;

    vector<Action> proposed_actions;
    vector<int> proposed_schedule;



    int total_timetous = 0;


    std::future<bool> future;
    std::thread task_td;
    bool started = false;

    vector<int> start_locs;
    vector<list<int>> tasks;
    Entry* planner;
    SharedEnvironment* env;
    ActionModelWithRotate* model;

    int preprocess_time_limit=10;

    int plan_time_limit = 3;

    int num_of_agents;

    int log_level = 1;

    // tasks that haven't been finished but have been revealed to agents;

    vector<list<std::tuple<int,int,std::string>>> events;

    //for evaluation
    vector<int> solution_costs;
    list<double> planner_times; 
    bool fast_mover_feasible = true;

    bool initialized = false;

    void initialize();
    bool planner_initialize();


    std::shared_ptr<TaskManager> task_manager;
    std::shared_ptr<Simulator> simulator;
    // deque<Task> task_queue;
    virtual void sync_shared_env();

    void move(vector<Action>& actions);
    bool valid_moves(vector<State>& prev, vector<Action>& next);

    void log_preprocessing(bool succ);
    // void log_event_assigned(int agent_id, int task_id, int timestep);
    // void log_event_finished(int agent_id, int task_id, int timestep);

    /**
     * RL 
     * first we need to call initialize()
     * Then for each simulation,
     * reset()
     * step() -> split to step_scheduler() and step_planner()
     * We will always directly access data
     **/

    void reset();
    void step();

    void step_init();
    void step_begin();
    void step_scheduler();
    void step_scheduler(const py::array_t<int> & _proposed_schedule);
    void step_planner();
    void step_planner(const py::array_t<int> & _proposed_actions);
    void _update_schedule();
    void _update_plan();
    void step_end();

    py::array_t<int> get_curr_positions();
    py::array_t<int> get_curr_orientations();
    py::array_t<int> get_target_positions();

    std::tuple<py::array_t<int>,py::array_t<int> > get_assignable_agents();
    std::tuple<py::array_t<int>,py::array_t<int> > get_assignable_tasks();
    size_t get_mask_task_loc_num();
    size_t max_task_locs_num;

    py::array_t<int> global_greedy_matching(
        const py::array_t<int> & assignable_agents_indices,
        const py::array_t<int> & assignable_tasks_indices,
        const py::array_t<int> & sorted_tasks_costs,
        const py::array_t<int> & sorted_tasks_indices
    );

    int get_num_task_finished() {
        return task_manager->num_of_task_finish;
    }
};