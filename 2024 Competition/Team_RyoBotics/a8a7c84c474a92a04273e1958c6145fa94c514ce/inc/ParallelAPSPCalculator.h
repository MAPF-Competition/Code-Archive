#pragma once
#include "APSPCalculator.h"
#include "APSPThreadPool.h"
#include <future>
#include "WeightTable.h"
#include "SharedEnv.h"
#include "CommonTypes.h"
namespace SchedulerUtils
{
    class ParallelAPSPCalculator : public APSPCalculator
    {
    public:
        static DirectionalPathCostMap calculateParallelAPSP(
            SharedEnvironment *env,
            int num_threads = std::thread::hardware_concurrency(), bool include_tabu_cost = false);

    private:
        // APSPCalculatorと同じプライベートメソッドを継承
        using APSPCalculator::bfs;
        using APSPCalculator::getNeighbors;
        using APSPCalculator::getPassableCells;
        using APSPCalculator::precomputeNeighbors;
        using APSPCalculator::validateMove;
    };
}