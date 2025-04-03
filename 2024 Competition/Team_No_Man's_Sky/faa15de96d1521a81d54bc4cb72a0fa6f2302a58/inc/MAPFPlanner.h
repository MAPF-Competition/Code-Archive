#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>
#include <ctime>

#include <Planner/eplanner.hpp>

#include <settings.hpp>

#ifdef ENABLE_SMART_PLANNER
#include "../Solution2/inc/MAPFPlanner.h"
#endif

class MAPFPlanner {
public:
    SharedEnvironment *env;

    EPlanner eplanner;

#ifdef ENABLE_SMART_PLANNER
    SmartMAPFPlanner smart_planner;
#endif

    explicit MAPFPlanner(SharedEnvironment *env) : env(env), eplanner(env)
#ifdef ENABLE_SMART_PLANNER
    , smart_planner(env)
#endif
    {
    }

    MAPFPlanner() {
        env = new SharedEnvironment();
    }

    virtual ~MAPFPlanner() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<Action> &plan);
};
