#include "TaskScheduler.h"

// #include "scheduler.h"
// #include "const.h"
#include "my_planner/scheduler.h"
#include "my_planner/const.h"

void TaskScheduler::load_configs()
{
    // load configs
    string config_path = "configs/" + env->map_name.substr(0, env->map_name.find_last_of(".")) + ".json";
    char *_config_path = getenv("CONFIG_PATH");
    if (_config_path != NULL)
    {
        config_path = std::string(_config_path);
        std::cout << "load config from " << config_path << std::endl;
    }
    std::ifstream f(config_path);
    try
    {
        config = nlohmann::json::parse(f);

        char *env_weight_path = getenv("MAP_WEIGHT_PATH");
        if (env_weight_path != NULL)
        {
            config["map_weights_path"] = env_weight_path;
            std::cout << "load weight from " << env_weight_path << std::endl;
        }

        std::cerr << config << std::endl;
        config["lifelong_solver_name"] = read_conditional_value(config, "lifelong_solver_name", env->num_of_agents);
        config["map_weights_path"] = read_conditional_value(config, "map_weights_path", env->num_of_agents);

        if (config.contains("max_task_completed"))
        {
            config["max_task_completed"] = read_conditional_value(config, "max_task_completed", env->num_of_agents);
        }

        if (config.contains("max_execution_steps"))
        {
            config["max_execution_steps"] = read_conditional_value(config, "max_execution_steps", env->num_of_agents);
        }

        config["LaCAM2"]["order_strategy"] = read_conditional_value(config["LaCAM2"], "order_strategy", env->num_of_agents);
        config["LNS"]["LaCAM2"]["order_strategy"] = read_conditional_value(config["LNS"]["LaCAM2"], "order_strategy", env->num_of_agents);
        config["disable_corner_target_agents"] = read_conditional_value(config, "disable_corner_target_agents", env->num_of_agents);
        config["max_agents_in_use"] = read_conditional_value(config, "max_agents_in_use", env->num_of_agents);

        config["LNS"]["fix_ng_bug"] = read_conditional_value(config["LNS"], "fix_ng_bug", env->num_of_agents);
        config["LNS"]["window_size_for_CT"] = read_conditional_value(config["LNS"], "window_size_for_CT", env->num_of_agents);
        config["LNS"]["window_size_for_CAT"] = read_conditional_value(config["LNS"], "window_size_for_CAT", env->num_of_agents);
        config["LNS"]["window_size_for_PATH"] = read_conditional_value(config["LNS"], "window_size_for_PATH", env->num_of_agents);
        config["LNS"]["LaCAM2"]["planning_window"] = read_conditional_value(config["LNS"]["LaCAM2"], "planning_window", env->num_of_agents);

        string s = config.dump();
        std::replace(s.begin(), s.end(), ',', '|');
        config["details"] = s;
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cout << "Failed to load " << config_path << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }
}

std::string TaskScheduler::load_map_weights(string weights_path)
{
    // : make weights float
    // we have at least 5 weights for a location: right,down,left,up,stay
    map_weights = std::make_shared<std::vector<float>>(env->rows * env->cols * 5, 1);
    std::string suffix = "all_one";

    if (weights_path != "")
    {
        std::ifstream f(weights_path);
        try
        {
            nlohmann::json _weights = nlohmann::json::parse(f);
            if (_weights.size() != map_weights->size())
            {
                std::cerr << "map weights size mismatch" << std::endl;
                exit(-1);
            }

            for (int i = 0; i < map_weights->size(); ++i)
            {
                (*map_weights)[i] = _weights[i].get<float>();
            }
        }
        catch (nlohmann::json::parse_error error)
        {
            std::cerr << "Failed to load " << weights_path << std::endl;
            std::cerr << "Message: " << error.what() << std::endl;
            exit(1);
        }

        boost::filesystem::path _weights_path(weights_path);
        suffix = _weights_path.stem().string();
    }
    return suffix;
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    // std::cout << "task scheduler initialization begins" << std::endl;
    load_configs();
    std::string weights_path = read_param_json<std::string>(config, "map_weights_path");
    std::string suffix = load_map_weights(weights_path);
    auto heuristics = std::make_shared<HeuristicTable>(env, map_weights, read_param_json<bool>(config["LaCAM2"], "use_orient_in_heuristic"));
    heuristics->preprocess(suffix);
    HT = heuristics;

    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    //  int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    //  DefaultPlanner::schedule_initialize(limit, env);
    int limit = preprocess_time_limit / 2 - MyPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    MyPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
    // give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    //  int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    //  DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    int limit = time_limit / 2 - MyPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    MyPlanner::schedule_plan(limit, proposed_schedule, env, HT);
}
