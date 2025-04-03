
#include "init.h"
#include <queue>
#include "util/HeuristicTable.h"

namespace MyPlanner{

Neighbors global_neighbors;
nlohmann::json configs;
int num_completed_task=0;

void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (global_neighbors.size()==0){
		// global_heuristictable.resize(env->map.size());
		init_neighbor(env);
	}

	// precompute the static_heuristic_table
    // TODO: use false or true?
    // It seems that false is better currently
    if (UTIL::static_heuristic_table.get() == nullptr) {
        // const auto map_weights = std::make_shared<std::vector<float> >(env->rows*env->cols*5, 1.0);
		// std::string weights_path = ""; //"data/map_weights/random-32-32-20_001.w";
		configs=load_configs(env);
        std::string weights_path=configs["map_weights_path"];
        auto [suffix, map_weights] = load_map_weights(env->rows, env->cols, weights_path);

        UTIL::static_heuristic_table = std::make_shared<UTIL::HeuristicTable>(env, map_weights, true);
		UTIL::static_heuristic_table->preprocess(suffix);
    	std::cout<<"static_heuristic_table preprocessed"<<std::endl;

        UTIL::static_heuristic_table_no_rot = std::make_shared<UTIL::HeuristicTable>(env, map_weights, false);
        UTIL::static_heuristic_table_no_rot->preprocess(suffix+"_no_rot");
        std::cout<<"static_heuristic_table_no_rot preprocessed"<<std::endl;

    }

}

std::pair<std::string, std::shared_ptr<std::vector<float> > > load_map_weights(int rows, int cols, string weights_path) {
    // TODO(rivers): make weights float
    // we have at least 5 weights for a location: right,down,left,up,stay
    auto map_weights=std::make_shared<std::vector<float> >(rows*cols*5,1);
    std::string suffix = "all_one";

    if (weights_path!=""){
        std::ifstream f(weights_path);
        try
        {
            nlohmann::json _weights = nlohmann::json::parse(f);
            if (_weights.size()!=map_weights->size()) {
                std::cerr<<"map weights size mismatch"<<std::endl;
                exit(-1);
            }

            for (int i=0;i<map_weights->size();++i){
                (*map_weights)[i]=_weights[i].get<float>();
            }
            
        }
        catch (nlohmann::json::parse_error error)
        {
            std::cerr << "Failed to load " << weights_path << std::endl;
            std::cerr << "Message: " << error.what() << std::endl;
            exit(1);
        }

        boost::filesystem::path _weights_path(weights_path);
        suffix=_weights_path.stem().string();
    }
    return {suffix, map_weights};
}

nlohmann::json load_configs(SharedEnvironment * env) {
    // load configs
	string configs_path="my_planner/configs/"+env->map_name.substr(0,env->map_name.find_last_of("."))+".json";
    char * _configs_path=getenv("CONFIGS_PATH");
    if (_configs_path!=NULL) {
        configs_path=std::string(_configs_path);
        std::cout<<"load config from "<<configs_path<<std::endl;
    }
    std::ifstream f(configs_path);
    nlohmann::json configs;
    try
    {
        configs = nlohmann::json::parse(f);
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cout << "Failed to load " << configs_path << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }

    configs["map_name"]=env->map_name.substr(0,env->map_name.find_last_of("."));

    return configs;
}


}
