#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/chrono.h>
#include <pybind11/embed.h>

// #include "py_driver.h"
#include "PyCompetitionSystem.h"
#include "global_greedy_matching.h"

namespace py = pybind11;

PYBIND11_MODULE(py_driver, m) {
    py::class_<PyBaseSystem>(m, "PyBaseSystem")
        .def_readonly("env", &PyBaseSystem::env)
        .def_readonly("proposed_schedule", &PyBaseSystem::proposed_schedule)
        .def_readonly("proposed_actions", &PyBaseSystem::proposed_actions)
        .def(pybind11::init<std::string &, std::string &, std::string &, int, int, int, float, std::string &>())
        .def("simulate", &PyBaseSystem::simulate)
        .def("saveResults", &PyBaseSystem::saveResults)
        .def("reset", &PyBaseSystem::reset)
        .def("step", &PyBaseSystem::step)
        .def("step_init", &PyBaseSystem::step_init)
        .def("step_scheduler", py::overload_cast<>(&PyBaseSystem::step_scheduler))
        .def("step_scheduler", py::overload_cast<const py::array_t<int> &>(&PyBaseSystem::step_scheduler))
        .def("step_planner", py::overload_cast<>(&PyBaseSystem::step_planner))
        .def("step_planner", py::overload_cast<const py::array_t<int> &>(&PyBaseSystem::step_planner))
        .def("get_curr_positions", &PyBaseSystem::get_curr_positions)
        .def("get_curr_orientations", &PyBaseSystem::get_curr_orientations)
        .def("get_target_positions", &PyBaseSystem::get_target_positions)
        .def("get_assignable_agents", &PyBaseSystem::get_assignable_agents)
        .def("get_assignable_tasks", &PyBaseSystem::get_assignable_tasks)
        .def("global_greedy_matching", &PyBaseSystem::global_greedy_matching)
        .def("get_num_task_finished", &PyBaseSystem::get_num_task_finished)
        ;

    // m.def("build_simulator", &build_simulator); 

    // py::class_<GlobalGreedyMatching>(m, "GlobalGreedyMatching")
    //     .def(py::init<py::array_t<int> &, py::array_t<int> &>())
    //     .def("get_assignment", &GlobalGreedyMatching::get_assignment);

    // py::class_<PyDriver>(m, "PyDriver")
    //     .def(py::init<>())
    //     .def("init", &PyDriver::init)
    //     .def("simulate", &PyDriver::simulate, py::call_guard<py::gil_scoped_release>())
    //     .def("test", &PyDriver::test)
    //     .def("get_env", &PyDriver::get_env, py::return_value_policy::reference_internal);

    pybind11::class_<Grid>(m, "Grid")
        .def(pybind11::init<std::string>())
        .def_readonly("cols",&Grid::cols)
        .def_readonly("map",&Grid::map)
        .def_readonly("map_name",&Grid::map_name)
        .def_readonly("rows",&Grid::rows);


    pybind11::class_<State>(m, "State")
        .def(pybind11::init<>())
        .def_readonly("location",&State::location)
        .def_readonly("timestep",&State::timestep)
        .def_readonly("orientation",&State::orientation)
        .def(py::self==py::self)
        .def(py::self!=py::self)
        .def(pybind11::init<int,int,int>());


    pybind11::enum_<Action>(m,"Action")
        .value("FW",Action::FW)
        .value("CR",Action::CR)
        .value("CCR",Action::CCR)
        .value("W",Action::W);

    pybind11::class_<ActionModelWithRotate>(m,"ActionModelWithRotate")
        .def(pybind11::init<Grid &>())
        // .def("is_valid",&ActionModelWithRotate::is_valid)
        .def("set_logger",&ActionModelWithRotate::set_logger)
        .def("result_states",&ActionModelWithRotate::result_states);

    pybind11::class_<SharedEnvironment>(m,"SharedEnvironment")
        .def(pybind11::init<>())
        .def_readonly("rows",&SharedEnvironment::rows)
        .def_readonly("cols",&SharedEnvironment::cols)
        .def_readonly("num_of_agents",&SharedEnvironment::num_of_agents)
        .def_readonly("goal_locations",&SharedEnvironment::goal_locations)
        .def_readonly("curr_timestep",&SharedEnvironment::curr_timestep)
        .def_readonly("map",&SharedEnvironment::map)
        .def_readonly("map_name",&SharedEnvironment::map_name)
        .def_readonly("task_pool",&SharedEnvironment::task_pool)
        .def_readonly("new_tasks",&SharedEnvironment::new_tasks)
        .def_readonly("new_freeagents",&SharedEnvironment::new_freeagents)
        .def_readonly("curr_task_schedule",&SharedEnvironment::curr_task_schedule)
        .def_readonly("file_storage_path", &SharedEnvironment::file_storage_path)
        .def_readonly("curr_states",&SharedEnvironment::curr_states)
        .def_readonly("plan_start_time",&SharedEnvironment::plan_start_time)
        .def("plan_current_time",
            [](SharedEnvironment &env) {
                return std::chrono::steady_clock::now();
            }
        );

    
    pybind11::class_<Task>(m,"Task")
        .def(pybind11::init<int,list<int>,int>())
        .def("get_next_loc",&Task::get_next_loc)
        .def("is_finished",&Task::is_finished)
     
        .def_readonly("task_id",&Task::task_id)
        .def_readonly("locations",&Task::locations)
        .def_readonly("idx_next_loc",&Task::idx_next_loc)
        .def_readonly("t_revealed",&Task::t_revealed)
        .def_readonly("t_completed",&Task::t_completed)
        //agent assigned  must be readwrite so that python side can modify this attribute!
        .def_readwrite("agent_assigned",&Task::agent_assigned);

    pybind11::bind_vector<std::vector<int>>(m, "VectorInt");
    pybind11::bind_vector<std::vector<vector<std::pair<int,int>>>>(m, "VectorGoals");
    pybind11::bind_vector<std::vector<State>>(m, "VectorState");
    pybind11::bind_vector<std::vector<Task>>(m, "VectorTask");
    pybind11::bind_map<std::unordered_map<int, Task>>(m, "MapTask");

}