#include "ash/constraint_set.hpp"


namespace
{

ash::Constraint make_vertex_constraint(int location)
{
    return {location, location, -1};
}

ash::Constraint make_vertex_constraint(const ash::AgentPosition& agpos)
{
    return {agpos.location, agpos.location, -1};
}

ash::Constraint make_vertex_constraint(const ash::TimedAgentPosition& tpos)
{
    return {tpos.position.location, tpos.position.location, tpos.time};
}

ash::Constraint make_edge_constraint(int location1, int location2)
{
    if (location2 < location1)
    {
        std::swap(location1, location2);
    }
    return {location1, location2, -1};
}

ash::Constraint make_edge_constraint(const ash::AgentPosition& agpos1,
                                     const ash::AgentPosition& agpos2)
{
    return make_edge_constraint(agpos1.location, agpos2.location);
}

ash::Constraint make_edge_constraint(const ash::TimedAgentPosition& tpos1,
                                     const ash::TimedAgentPosition& tpos2)
{
    auto constraint = make_edge_constraint(tpos1.position, tpos2.position);
    constraint.t = tpos1.time;
    return constraint;
}

} // anonymous namespace

std::ostream& ash::operator<<(std::ostream& out, const Constraint& constraint)
{
    if (constraint.is_vertex_constraint())
    {
        return out << '(' << constraint.u << ", " << constraint.t << ')';
    }
    else
    {
        return out << '(' << constraint.u << ", " << constraint.v
                   << ", " << constraint.t << ')';
    }
}

ash::ConstraintSet::ConstraintSet()
{
}

template<class S>
int ash::ConstraintSet::get(const S& pos) const
{
    auto vc = make_vertex_constraint(pos);
    return get(vc);
}

template<class S>
int ash::ConstraintSet::get(const S& pos1, const S& pos2) const
{
    auto ec = make_edge_constraint(pos1, pos2);
    return get(ec);
}

template<class S>
bool ash::ConstraintSet::reserve(const S& pos, int weight)
{
    auto vc = make_vertex_constraint(pos);
    return constraints.emplace(vc, weight).second;
}

template<class S>
bool ash::ConstraintSet::reserve(const S& pos1, const S& pos2, int weight)
{
    auto ec = make_edge_constraint(pos1, pos2);
    if (!ec.is_edge_constraint())
    {
        return false;
    }
    return constraints.emplace(ec, weight).second;
}

template<class S>
void ash::ConstraintSet::reserve(const Plan<S>& plan, int start, int weight)
{
    reserve(plan[start].state, weight);
    for (size_t i = start+1; i < plan.size(); ++i)
    {
        reserve(plan[i-1].state, plan[i].state, weight);
        reserve(plan[i].state, weight);
    }
}

template<class S>
bool ash::ConstraintSet::release(const S& pos)
{
    auto vc = make_vertex_constraint(pos);
    return constraints.erase(vc);
}

template<class S>
bool ash::ConstraintSet::release(const S& pos1, const S& pos2)
{
    auto ec = make_edge_constraint(pos1, pos2);
    return constraints.erase(ec);
}

template<class S>
void ash::ConstraintSet::release(const Plan<S>& plan, int start)
{
    release(plan[start].state);
    for (size_t i = start+1; i < plan.size(); ++i)
    {
        release(plan[i-1].state, plan[i].state);
        release(plan[i].state);
    }
}

int ash::ConstraintSet::get(const Constraint& constraint) const
{
    auto it = constraints.find(constraint);
    //if (it==constraints.end() && !constraint.is_static())
    //{
        //auto static_constraint = constraint;
        //static_constraint.t = -1;
        //it = constraints.find(static_constraint);
    //}
    return it==constraints.end()? 0 : it->second;
}

template int ash::ConstraintSet::get(
        const int&) const;
template int ash::ConstraintSet::get(
        const AgentPosition&) const;
template int ash::ConstraintSet::get(
        const TimedAgentPosition&) const;

template int ash::ConstraintSet::get(
        const int&, const int&) const;
template int ash::ConstraintSet::get(
        const AgentPosition&, const AgentPosition&) const;
template int ash::ConstraintSet::get(
        const TimedAgentPosition&, const TimedAgentPosition&) const;

template bool ash::ConstraintSet::reserve(
        const int&, int);
template bool ash::ConstraintSet::reserve(
        const AgentPosition&, int);
template bool ash::ConstraintSet::reserve(
        const TimedAgentPosition&, int);

template bool ash::ConstraintSet::reserve(
        const int&, const int&, int);
template bool ash::ConstraintSet::reserve(
        const AgentPosition&, const AgentPosition&, int);
template bool ash::ConstraintSet::reserve(
        const TimedAgentPosition&, const TimedAgentPosition&, int);

template void ash::ConstraintSet::reserve(
        const Plan<int>& plan, int, int);
template void ash::ConstraintSet::reserve(
        const Plan<AgentPosition>& plan, int, int);
template void ash::ConstraintSet::reserve(
        const Plan<TimedAgentPosition>& plan, int, int);

template bool ash::ConstraintSet::release(
        const int&);
template bool ash::ConstraintSet::release(
        const AgentPosition&);
template bool ash::ConstraintSet::release(
        const TimedAgentPosition&);

template bool ash::ConstraintSet::release(
        const int&, const int&);
template bool ash::ConstraintSet::release(
        const AgentPosition&, const AgentPosition&);
template bool ash::ConstraintSet::release(
        const TimedAgentPosition&, const TimedAgentPosition&);

template void ash::ConstraintSet::release(
        const Plan<int>& plan, int);
template void ash::ConstraintSet::release(
        const Plan<AgentPosition>& plan, int);
template void ash::ConstraintSet::release(
        const Plan<TimedAgentPosition>& plan, int);

const ash::ConstraintSet ash::EMPTY_CONSTRAINT_SET;

std::ostream& ash::operator<<(std::ostream& out, const ConstraintSet& cs)
{
    out << "Constraints = {";
    bool first = true;
    for (const auto&[constraint, weight] : cs)
    {
        if (!first)
        {
            out << ", ";
        }
        out << '(' << constraint << ", weight=";
        if (weight < INF)
        {
            out << weight;
        }
        else
        {
            out << "inf";
        }
        out << ')';
        first = false;
    }
    return out << '}';
}

