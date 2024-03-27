#pragma once

#include "SharedEnv.h"
#include "ash/preprocess.hpp"

#include <functional>

namespace ash
{

typedef std::function<int(const AgentPosition&,int)> Heuristic;

class WeakHeuristic
{
    public:
        WeakHeuristic(const SharedEnvironment& env);

        int operator()(const AgentPosition& src, int dst) const;

    private:
        const SharedEnvironment* env;
};


class StrongHeuristic
{
    public:
        StrongHeuristic(const Preprocessor& preprocessor);

        int operator()(const AgentPosition& src, int dst) const;

    private:
        const Preprocessor* preprocessor;
};

template<class S>
class HeuristicAdapter
{
    public:

        HeuristicAdapter() {}

        HeuristicAdapter(Heuristic heuristic) : heuristic(std::move(heuristic))
        {
        }

        int operator()(const S& src, int dst) const;

    private:

        Heuristic heuristic;
};

template<>
inline int HeuristicAdapter<int>::operator()(const int& src, int dst) const
{
    return std::min(heuristic({src, Orientation::EAST}, dst),
           std::min(heuristic({src, Orientation::SOUTH}, dst),
           std::min(heuristic({src, Orientation::WEST}, dst),
                    heuristic({src, Orientation::NORTH}, dst))));
}

template<>
inline int HeuristicAdapter<AgentPosition>::operator()(const AgentPosition& src,
                                                int dst) const
{
    return heuristic(src, dst);
}

template<>
inline int HeuristicAdapter<TimedAgentPosition>::operator()(
        const TimedAgentPosition& src, int dst) const
{
    return heuristic(src.position, dst);
}

} // namespace ash
