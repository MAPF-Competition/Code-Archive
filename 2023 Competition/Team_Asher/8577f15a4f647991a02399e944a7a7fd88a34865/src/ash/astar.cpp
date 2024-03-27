#include "ash/astar.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

// Astar

template<class S>
ash::Astar<S>::Astar() :
    constraint_set(&EMPTY_CONSTRAINT_SET),
    timeout(Duration::max()),
    win_length(-1)
{
}

template<class S>
double ash::Astar<S>::get_effective_branching_factor() const
{
    constexpr int NR_ITERATIONS = 3;
    double M = number_of_generated_nodes;
    double d = get_plan().size() - 1;
    // calculate initial guess
    double x = std::pow(M, 1.0/d);
    // do a few rounds of Newton-Raphson to fine-tune the result
    for (int i = 0; i < NR_ITERATIONS; ++i)
    {
        double f = (std::pow(x, d+1.0) - 1) / (x-1) - (M+1);
        double df = ((d*x - d - 1)*std::pow(x, d) + 1.0) / ((x-1)*(x-1));
        x -= f/df;
    }
    return x;
}

template<class S>
bool ash::Astar<S>::search()
{
    auto start_time = Clock::now();
    setup();
	while (!open.empty() && !early_cutoff && !timed_out)
	{
		expand_windowless();
        timed_out = (Clock::now()-start_time) >= timeout;
	}
    elapsed = std::chrono::duration<double>(Clock::now()-start_time).count();
    return success();
}

template<>
bool ash::Astar<ash::TimedAgentPosition>::search()
{
    auto start_time = Clock::now();
    setup();
    if (win_length < 0)
    {
		while (!open.empty() && !early_cutoff && !timed_out)
		{
			expand_windowless();
            timed_out = (Clock::now()-start_time) >= timeout;
		}
	}
	else
	{
		while (!open.empty() && !early_cutoff && !timed_out)
		{
            expand_with_window();
            timed_out = (Clock::now()-start_time) >= timeout;
		}
	}
    elapsed = std::chrono::duration<double>(Clock::now()-start_time).count();
    return success();
}

template<class S>
ash::Plan<S> ash::Astar<S>::get_plan() const
{
    Plan<S> plan;
    auto current = plan_end;
    while (current != start)
    {
        const auto& node = visited.find(current)->second;
        plan.push_back({node.previous_action, current});
        current = node.previous_state;
    }
    plan.push_back({{}, start});
    std::reverse(plan.begin(), plan.end());
    return plan;
}

template<class S>
bool ash::Astar<S>::is_goal(const S& state) const
{
    if constexpr(std::is_same_v<S,int>)
    {
        return state == goal;
    }
    else if constexpr(std::is_same_v<S,AgentPosition>)
    {
        return state.location == goal;
    }
    else
    {
        return state.position.location == goal;
    }
}

template<class S>
int ash::Astar<S>::transition_cost(const S& state, const S& neighbor) const
{
	int base_cost;
	if constexpr(std::is_same_v<S,TimedAgentPosition>)
	{
		// let the base cost be 0 if we are waiting in the goal position
		base_cost = !is_goal(state) || state.position!=neighbor.position;
	}
	else
	{
		base_cost = 1;
	}
	
    int vertex_cost = constraint_set->get(neighbor);
    int edge_cost = constraint_set->get(state, neighbor);
    int cost = (vertex_cost==INF||edge_cost==INF)? INF :
	    (base_cost+vertex_cost+edge_cost);
    return cost;
}

template<class S>
int ash::Astar<S>::get_neighbors(Neighbors& neighbors, const S& state) const
{
    int number_of_neighbors = 0;
    for (auto action : ActionTypeTrait<S>::ALL_ACTIONS)
    {
        auto neighbor = get_neighbor(*env, state, action);
        if (!is_valid_state(neighbor))
        {
			continue;
		}
        int cost = transition_cost(state, neighbor);
        if (cost < INF)
        {
            neighbors[number_of_neighbors++] = {action, neighbor, cost};
        }
    }
    return number_of_neighbors;
}

template<class S>
void ash::Astar<S>::setup()
{
    visited.clear();
    visited[start] = {{}, {}, 0, false};

    open = {};
    int h_start = heur(start, goal);
    open.push(OpenNode{start, h_start, h_start});

    plan_cost = INF;
    early_cutoff = false;
    timed_out = false;

    elapsed = 0;
    number_of_expanded_nodes = 0;
    number_of_generated_nodes = 1;
}

template<class S>
void ash::Astar<S>::generate_neighbors(const S& state, int state_g)
{
	Neighbors neighbors;
    int num_of_neighbors = get_neighbors(neighbors, state);
    number_of_generated_nodes += num_of_neighbors;
    for (int i = 0; i < num_of_neighbors; ++i)
    {
        int tentative_g_neigh = state_g + neighbors[i].cost;
        auto& neigh_node = visited.insert({
                neighbors[i].state,
                {{}, {}, INF, false}
        }).first->second;
        if (tentative_g_neigh < neigh_node.g)
        {
            neigh_node.previous_state = state;
            neigh_node.previous_action = neighbors[i].action;
            neigh_node.g = tentative_g_neigh;
            int h_neigh = heur(neighbors[i].state, goal);
            int f_neigh = tentative_g_neigh + h_neigh;
            open.push(OpenNode{neighbors[i].state, f_neigh, h_neigh});
        }
    }
}

template<class S>
void ash::Astar<S>::expand_windowless()
{
    ++number_of_expanded_nodes;

    auto top = open.top();
    open.pop();

    auto& top_node = visited[top.state];
    if (top_node.closed)
    {
        return;
    }
    top_node.closed = true;

    if (is_goal(top.state))
    {
        plan_end = top.state;
        plan_cost = top_node.g;
        early_cutoff = true;
        return;
    }

    generate_neighbors(top.state, top_node.g);
}

template<class S>
template<class Ret>
ash::EnableIfTimed<S, Ret> ash::Astar<S>::expand_with_window()
{
    ++number_of_expanded_nodes;

    auto top = open.top();
    open.pop();
    
    if (top.f > plan_cost)
    {
        early_cutoff = true;
		return;
	}

    auto& top_node = visited[top.state];
    if (top_node.closed)
    {
        return;
    }
    top_node.closed = true;
    
    int depth = top.state.time - start.time;
    
    if (depth >= win_length)
    {
		if (top.f < plan_cost)
		{
			plan_cost = top.f;
			plan_end = top.state;
            early_cutoff = is_goal(top.state);
		}
        return;
	}


    generate_neighbors(top.state, top_node.g);
}

template class ash::Astar<int>;
template class ash::Astar<ash::AgentPosition>;
template class ash::Astar<ash::TimedAgentPosition>;

