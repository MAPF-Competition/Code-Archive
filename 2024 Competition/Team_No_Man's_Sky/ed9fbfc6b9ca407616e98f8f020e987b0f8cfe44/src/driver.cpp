#include "CompetitionSystem.h"
#include "Evaluation.h"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <climits>
#include <memory>
#include <signal.h>

#include <Tools/tools.hpp>
#include <settings.hpp>


#ifdef PYTHON
#if PYTHON
#include "pyEntry.hpp"
#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"
#include <pybind11/embed.h>
#endif
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;

#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>


void sigint_handler(int a) {
    fprintf(stdout, "stop the simulation...\n");
    system_ptr->saveResults(vm["output"].as<std::string>(), vm["outputScreen"].as<int>());
    _exit(0);
}


int main(int argc, char **argv) {
#ifdef PYTHON
#if PYTHON
    pybind11::initialize_interpreter();
#endif
#endif
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help",
                       "produce help message")                                                                                                                                                                                        //
            ("unique_id,u", po::value<uint32_t>()->default_value(0),
             "my unique id for unique launch")                                                                                                                                        //
            ("inputFile,i", po::value<std::string>()->required(),
             "input file name")                                                                                                                                                          //
            ("output,o", po::value<std::string>()->default_value("./output.json"),
             "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")                                   //
            ("outputScreen,c", po::value<int>()->default_value(1),
             "the level of details in the output file, 1--showing all the output, 2--ignore the events and tasks, 3--ignore the events, tasks, errors, planner times, starts and paths")//
            ("evaluationMode,m", po::value<bool>()->default_value(false),
             "evaluate an existing output file")                                                                                                                                 //
            ("simulationTime,s", po::value<int>()->default_value(5000),
             "run simulation")                                                                                                                                                     //
            ("fileStoragePath,f", po::value<std::string>()->default_value(""),
             "the large file storage path")                                                                                                                                 //
            ("planTimeLimit,t", po::value<int>()->default_value(1000),
             "the time limit for planner in milliseconds")                                                                                                                          //
            ("preprocessTimeLimit,p", po::value<int>()->default_value(30000),
             "the time limit for preprocessing in milliseconds")                                                                                                             //
            ("logFile,l", po::value<std::string>()->default_value(""),
             "redirect stdout messages into the specified log file")                                                                                                                //
            ("logDetailLevel,d", po::value<int>()->default_value(1),
             "the minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only");
    clock_t start_time = clock();
    po::store(po::parse_command_line(argc, argv, desc), vm);

    get_unique_id() = vm["unique_id"].as<uint32_t>();
    PRINT(Printer() << "unique_id: " << get_unique_id() << '\n';);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    boost::filesystem::path p(vm["inputFile"].as<std::string>());
    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (base_folder.size() > 0 && base_folder.back() != '/') {
        base_folder += "/";
    }

    int log_level = vm["logDetailLevel"].as<int>();
    if (log_level <= 1)
        log_level = 2;//info
    else if (log_level == 2)
        log_level = 3;//warning
    else
        log_level = 5;//fatal

    Logger *logger = new Logger(vm["logFile"].as<std::string>(), log_level);

    std::filesystem::path filepath(vm["output"].as<std::string>());
    if (filepath.parent_path().string().size() > 0 && !std::filesystem::is_directory(filepath.parent_path())) {
        logger->log_fatal("output directory does not exist", 0);
        _exit(1);
    }


    Entry *planner = nullptr;

#ifdef PYTHON
#if PYTHON
    planner = new PyEntry();
#else
    planner = new Entry();
#endif
#endif

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try {
        data = json::parse(f);
    } catch (json::parse_error error) {
        std::cout << "Failed to load " << input_json_file << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }

    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);


    string file_storage_path = vm["fileStoragePath"].as<std::string>();
    if (file_storage_path == "") {
        char const *tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
        if (tmp != nullptr) {
            file_storage_path = string(tmp);
        }
    }

    // check if the path exists;
    if (file_storage_path != "" && !std::filesystem::exists(file_storage_path)) {
        std::ostringstream stringStream;
        stringStream << "fileStoragePath (" << file_storage_path << ") is not valid";
        logger->log_warning(stringStream.str());
    }
    planner->env->file_storage_path = file_storage_path;

    ActionModelWithRotate *model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    if (agents.size() > tasks.size())
        logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

    system_ptr = std::make_unique<BaseSystem>(grid, planner, agents, tasks, model);

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());


    system_ptr->saveResults(vm["output"].as<std::string>(), vm["outputScreen"].as<int>());

    delete model;
    delete logger;

#ifdef BUILD_META_INFO
#ifdef ENABLE_GG_SOLVER
    build_meta_info("Tmp/test" + std::to_string(get_unique_id()) + ".json",
                    "Tmp/meta" + std::to_string(get_unique_id()));
#else
    build_meta_info("test.json", "meta");
    //build_meta_info("Tmp/test" + std::to_string(get_unique_id()) + ".json",
    //                "Tmp/meta" + std::to_string(get_unique_id()));
#endif
#endif
    Printer().get().flush();

    _exit(0);

    {
        uint32_t finished_tasks = 0;
        {
            using json = nlohmann::basic_json<nlohmann::ordered_map>;
            json data;
            std::ifstream input(vm["output"].as<std::string>());
            try {
                data = json::parse(input);
            } catch (const json::parse_error &error) {
                FAILED_ASSERT(error.what());
            }
            finished_tasks = data["numTaskFinished"];
        }

        /*std::ifstream input("meta");
        // [pos][dir][action]
        Meta meta(get_gg().get_size());

        uint32_t rows, cols;
        input >> rows >> cols;

        ASSERT(get_gg().get_rows() == rows && get_gg().get_cols() == cols, "invalid rows/cols");
        for (uint32_t dir = 0; dir < 5; dir++) {
            for (uint32_t act = 0; act < 5; act++) {
                for (uint32_t pos = 0; pos < get_gg().get_size(); pos++) {
                    input >> meta[pos][dir][act];
                }
            }
        }

        auto calc = [&](uint32_t dir, uint32_t act) {
            uint64_t s = 0;
            for (uint32_t pos = 0; pos < get_gg().get_size(); pos++) {
                s += static_cast<uint64_t>(meta[pos][dir][act]) * meta[pos][dir][act];
            }
            return static_cast<double>(s) / (get_gg().get_size() - 1);
        };

        double score = 0;
        for (uint32_t dir = 0; dir < 4; dir++) {
            score -= calc(dir, 1);
            score -= calc(dir, 2);
            score -= calc(dir, 3) * 2;
        }
        score += finished_tasks * 5;*/

        Printer() << "tasks: " << finished_tasks << '\n';
        //Printer() << "score: " << score << '\n';
    }

    _exit(0);
}
