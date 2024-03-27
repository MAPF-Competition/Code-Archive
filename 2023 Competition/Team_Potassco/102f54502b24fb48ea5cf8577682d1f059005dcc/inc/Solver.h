#pragma once
#include "SharedEnv.h"
#include "Map.h"
#include "Solver.h"
#include <clingo.hh>

class Solver
{
    public:
        Solver(SharedEnvironment* env){}

    private:
        MAP map;
}