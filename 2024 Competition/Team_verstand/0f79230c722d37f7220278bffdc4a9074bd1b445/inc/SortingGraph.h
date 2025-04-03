//
// Created by take_ on 24-12-27.
//

#ifndef SORTINGGRAPH_H
#define SORTINGGRAPH_H

#include "BasicGraph.h"
#include "SharedEnv.h"

class SortingGrid :
        public BasicGraph
{
public:
    boost::unordered_map<std::string, int> inducts;
    boost::unordered_map<string, list<int> > ejects; // one eject station could have multiple eject fiducials

    bool load_map(string fname) override;
    bool load_LoRR_map(SharedEnvironment* env);
    void preprocessing(bool consider_rotation); // compute heuristics
    void preprocessing_LoRR(bool consider_rotation, SharedEnvironment* env); // compute heuristics
};

#endif // SORTINGGRAPH_H
