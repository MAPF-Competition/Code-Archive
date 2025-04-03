#pragma once

#include <Objects/Basic/time.hpp>

#include <ActionModel.h>
#include <SharedEnv.h>

class EPlanner {
    SharedEnvironment *env;
public:
    explicit EPlanner(SharedEnvironment *env);

    EPlanner();

    std::pair<std::vector<Action>, std::vector<uint32_t>> plan(TimePoint end_time);
};
