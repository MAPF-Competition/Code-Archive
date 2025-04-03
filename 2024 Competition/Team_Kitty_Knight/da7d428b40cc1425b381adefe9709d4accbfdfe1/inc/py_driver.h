#pragma once

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <signal.h>
#include <climits>
#include <memory>
#include <iostream>
#include <sstream>
#include <functional>

#include "nlohmann/json.hpp"

#include "Grid.h"
#include "SharedEnv.h"
#include "MAPFPlanner.h"
#include "States.h"
#include "CompetitionSystem.h"
#include "Evaluation.h"
#include "Tasks.h"

namespace po = boost::program_options;
using json = nlohmann::json;

/**
 * Note this is a interface for python to call C++, 
 * not for c++ to call python (see PythonWrapper.h for more details).
 */
class PyDriver {
public:

    po::variables_map vm;

    // TODO(rivers): the memory management is inconsistent...
    std::unique_ptr<BaseSystem> system_ptr;
    Logger * logger;
    Entry * planner;
    ActionModelWithRotate *model;
    std::unique_ptr<Grid> grid;
    std::vector<int> agents;
    std::vector<list<int>> tasks;

    PyDriver() {}

    ~PyDriver() {
        delete logger;
        // delete planner; planner will be handled by system_ptr automatically
        delete model;
    }

    void init(
        std::string options
    ) {
        po::options_description desc("Allowed options");
        desc.add_options()("help", "produce help message")
            ("inputFile,i", po::value<std::string>()->required(), "input file name")
            ("output,o", po::value<std::string>()->default_value("./output.json"), "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")
            ("outputScreen,c", po::value<int>()->default_value(1), "the level of details in the output file, 1--showing all the output, 2--ignore the events and tasks, 3--ignore the events, tasks, errors, planner times, starts and paths")
            ("evaluationMode,m", po::value<bool>()->default_value(false), "evaluate an existing output file")
            ("simulationTime,s", po::value<int>()->default_value(5000), "run simulation")
            ("fileStoragePath,f", po::value<std::string>()->default_value(""), "the large file storage path")
            ("planTimeLimit,t", po::value<int>()->default_value(1000), "the time limit for planner in milliseconds")
            ("preprocessTimeLimit,p", po::value<int>()->default_value(30000), "the time limit for preprocessing in milliseconds")
            ("logFile,l", po::value<std::string>()->default_value(""), "redirect stdout messages into the specified log file")
            ("logDetailLevel,d", po::value<int>()->default_value(1), "the minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only");
        
        po::store(po::command_line_parser(po::split_unix(options)).options(desc).run(), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            exit(1);
        }

        po::notify(vm);

        boost::filesystem::path p(vm["inputFile"].as<std::string>());
        boost::filesystem::path dir = p.parent_path();
        std::string base_folder = dir.string();
        if (base_folder.size() > 0 && base_folder.back() != '/')
        {
            base_folder += "/";
        }

        int log_level = vm["logDetailLevel"].as<int>();
        if (log_level <= 1)
            log_level = 2; //info
        else if (log_level == 2)
            log_level = 3; //warning
        else
            log_level = 5; //fatal

        logger = new Logger(vm["logFile"].as<std::string>(),log_level);

        boost::filesystem::path filepath(vm["output"].as<std::string>());
        if (filepath.parent_path().string().size() > 0 && !boost::filesystem::is_directory(filepath.parent_path()))
        {
            logger->log_fatal("output directory does not exist",0);
            _exit(1);
        }

        planner = new Entry();

        auto input_json_file = vm["inputFile"].as<std::string>();
        json data;
        std::ifstream f(input_json_file);
        try
        {
            data = json::parse(f);
        }
        catch (json::parse_error error)
        {
            std::cerr << "Failed to load " << input_json_file << std::endl;
            std::cerr << "Message: " << error.what() << std::endl;
            exit(1);
        }

        auto map_path = read_param_json<std::string>(data, "mapFile");
        grid=std::make_unique<Grid>(base_folder + map_path);

        planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);

        string file_storage_path = vm["fileStoragePath"].as<std::string>();
        if (file_storage_path==""){
            char const* tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
            if ( tmp != nullptr ) {
                file_storage_path = string(tmp);
            }
        }

        // check if the path exists;
        if (file_storage_path!="" &&!std::filesystem::exists(file_storage_path)){
        std::ostringstream stringStream;
        stringStream << "fileStoragePath (" << file_storage_path << ") is not valid";
        logger->log_warning(stringStream.str());
        }
        planner->env->file_storage_path = file_storage_path;

        model = new ActionModelWithRotate(*grid);
        model->set_logger(logger);

        int team_size = read_param_json<int>(data, "teamSize");

        agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
        tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
        if (agents.size() > tasks.size())
            logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

        system_ptr = std::make_unique<BaseSystem>(*grid, planner, agents, tasks, model);

        system_ptr->set_logger(logger);
        system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
        system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

        system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

        // TODO(rivers): setup signal handler?
    }

    void simulate() {
        system_ptr->simulate(vm["simulationTime"].as<int>());
        system_ptr->saveResults(vm["output"].as<std::string>(),vm["outputScreen"].as<int>());
    }

    void test() {
        std::cout<<"py_driver test"<<std::endl;
    }

    SharedEnvironment * get_env() {
        return planner->env;
    }

};