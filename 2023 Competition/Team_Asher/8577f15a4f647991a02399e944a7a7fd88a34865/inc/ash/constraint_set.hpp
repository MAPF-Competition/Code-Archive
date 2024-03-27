#pragma once

#include "ash/utils.hpp"

#include <functional>
#include <unordered_map>


namespace ash
{

struct Constraint
{
    int u;
    int v;
    int t;

    bool operator==(const Constraint& other) const
    {
        return u==other.u && v==other.v && t==other.t;
    }

    bool is_vertex_constraint() const
    {
        return u == v;
    }

    bool is_edge_constraint() const
    {
        return u < v;
    }

    bool is_static() const
    {
        return t < 0;
    }
};

std::ostream& operator<<(std::ostream& out, const Constraint& constraint);

} // namespace ash


namespace std
{

template<>
struct hash<ash::Constraint>
{
    size_t operator()(const ash::Constraint& c) const noexcept
    {
        size_t seed = c.u;
        seed = seed*123'457 + c.v;
        seed = seed*123'457 + c.t;
        return seed;
    }
};

} // namespace std


namespace ash
{

class ConstraintSet
{
    public:

        typedef std::unordered_map<Constraint, int> ConstraintMap;
        typedef ConstraintMap::const_iterator const_iterator;

        ConstraintSet();

        template<class S>
        int get(const S& pos) const;

        template<class S>
        int get(const S& pos1, const S& pos2) const;

        template<class S>
        bool reserve(const S& pos, int weight);

        template<class S>
        bool reserve(const S& pos1, const S& pos2, int weight);

        template<class S>
        void reserve(const Plan<S>& plan, int start = 0, int weight = INF);

        template<class S>
        bool release(const S& pos);

        template<class S>
        bool release(const S& pos1, const S& pos2);

        template<class S>
        void release(const Plan<S>& plan, int start = 0);


        const_iterator begin() const
        {
            return constraints.begin();
        }

        const_iterator end() const
        {
            return constraints.end();
        }

        void clear()
        {
            constraints.clear();
        }

    private:

        int get(const Constraint& constraint) const;

        ConstraintMap constraints;
};

extern const ConstraintSet EMPTY_CONSTRAINT_SET;

std::ostream& operator<<(std::ostream& out, const ConstraintSet& cs);

} // namespace ash

