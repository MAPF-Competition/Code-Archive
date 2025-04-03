#pragma once
#include "APSPCalculatorGame.h"
#include "APSPThreadPoolGame.h"
#include <future>

namespace SchedulerUtilsGame
{
    class ParallelAPSPCalculatorGame : public APSPCalculator
    {
    public:
        static DirectionalPathCostMap calculateParallelAPSP(
            const std::vector<int> &map,
            int rows,
            int cols,
            int num_threads = std::thread::hardware_concurrency());

    private:
        // APSPCalculatorと同じプライベートメソッドを継承
        using APSPCalculator::bfs;
        using APSPCalculator::getNeighbors;
        using APSPCalculator::getPassableCells;
        using APSPCalculator::precomputeNeighbors;
        using APSPCalculator::validateMove;
    };
}