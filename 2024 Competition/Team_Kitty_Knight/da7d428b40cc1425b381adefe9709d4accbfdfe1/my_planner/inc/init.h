
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.h"
#include "TrajLNS.h"

namespace MyPlanner{

void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

nlohmann::json load_configs(SharedEnvironment * env);

std::pair<std::string, std::shared_ptr<std::vector<float> > > load_map_weights(int rows, int cols, string weights_path);

extern Neighbors global_neighbors;

extern nlohmann::json configs;

extern int num_completed_task;

}
#endif