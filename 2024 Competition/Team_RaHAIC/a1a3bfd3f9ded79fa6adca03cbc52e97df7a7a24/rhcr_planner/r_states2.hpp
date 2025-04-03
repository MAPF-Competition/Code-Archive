#pragma once
#include "common.h"


struct State2
{
    // int location;
    State2(int timestep, std::pair<int,int> loc, int orientation) : timestep(timestep), loc(loc), orientation(orientation) {}
        std::pair<int,int> loc;
        int timestep;
        int orientation;  // 0:east, 1:south, 2:west, 3:north

        // struct Hasher
        // {
        //     size_t operator()(const State2& n) const
        //     {
        //         size_t loc_hash = std::hash<int>()(n.loc.first) ^ std::hash<int>()(n.loc.second);
        //         size_t time_hash = std::hash<int>()(n.timestep);
        //         size_t ori_hash = std::hash<int>()(n.orientation);
        //         return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        //     }
        // };

        void operator = (const State2& other)
        {
            timestep = other.timestep;
            loc = other.loc;
            orientation = other.orientation;
        }

        bool operator == (const State2& other) const
        {
            return timestep == other.timestep && loc == other.loc && orientation == other.orientation;
        }

        bool operator != (const State2& other) const
        {
            return timestep != other.timestep || loc != other.loc || orientation != other.orientation;
        }

        

        bool equalExceptTime(const State2& s) const { return loc.first == s.loc.first && loc.second == s.loc.second ; }

        State2(): loc({0,0}) , timestep(0), orientation(0) {}
        // State(int loc): loc(loc), timestep(0), orientation(0) {}
        // State(int loc, int timestep): loc(loc), timestep(timestep), orientation(0) {}
        State2(std::pair<int,int> loc, int timestep , int orientation):
            loc(loc), timestep(timestep), orientation(orientation) {}
        State2(const State2& other):
            loc(other.loc), timestep(other.timestep), orientation(other.orientation) {}
};

namespace std {
template <>
struct hash<State2> {
  size_t operator()(const State2& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.timestep);
    // boost::hash_combine(seed, s.loc.first);
    // boost::hash_combine(seed, s.loc.second);
    boost::hash_combine(seed, s.loc);
    boost::hash_combine(seed, s.orientation);
    return seed;
  }
};
}  // namespace std

std::ostream & operator << (std::ostream &out, const State2 &s);

typedef std::vector<State2> Path2;

std::ostream & operator << (std::ostream &out, const Path2 &path);
