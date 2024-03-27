#include "ash/push_search.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <unordered_map>
#include <queue>

namespace
{

struct VisitedNode
{
    ash::Orientation inbound_dir;
    int parent;
    int g;
    bool closed;
};

typedef ash::Vec2i OpenNode;

typedef std::priority_queue<OpenNode, std::vector<OpenNode>,
        std::greater<OpenNode>> OpenSet;
typedef std::unordered_map<int, VisitedNode> VisitedSet;

std::vector<ash::Orientation> reconstruct_path(const VisitedSet& visited,
        int push_loc, int last_loc)
{
    std::vector<ash::Orientation> path;
    if (last_loc == push_loc)
    {
        // loop! treat this as a special case
        const auto& visited_node = visited.find(last_loc)->second;
        path.push_back(visited_node.inbound_dir);
        last_loc = visited_node.parent;
    }
    while (last_loc != push_loc)
    {
        const auto& visited_node = visited.find(last_loc)->second;
        path.push_back(visited_node.inbound_dir);
        last_loc = visited_node.parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // anonymous namespace

ash::PushSearch::PushSearch()
{
}

void ash::PushSearch::init(const SharedEnvironment& env,
        Heuristic heuristic)
{
    this->env = &env;
    this->heuristic = std::move(heuristic);
}

void ash::PushSearch::sync()
{
    agent_table.assign(env->num_of_agents, {});
    cell_table.assign(env->map.size(), {-1, -1});
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        int agloc = env->curr_states[agid].location;
        int agor = env->curr_states[agid].orientation;
        agent_table[agid] = AgentInfo{
            AgentPosition{agloc, static_cast<Orientation>(agor)},
            {},
            0,
            get_goal(agid) == agloc
        };
        cell_table[agloc] = CellInfo{agid, -1};
    }
}

bool ash::PushSearch::is_valid_shift(int agid, const std::vector<Orientation>& path) const
{
    if (path.empty())
    {
        return false;
    }
    int curr_loc = agent_table[agid].current_state.location;
    for (auto dir : path)
    {
        if (!is_pushable(curr_loc))
        {
            return false;
        }
        curr_loc = get_neighbor(*env, curr_loc, dir); 
    }
    return is_valid_end_of_shift(agid, curr_loc);
}

int ash::PushSearch::shift_delay(int agid,
        const std::vector<Orientation>& path) const
{
    int delay = 1;
    int curr_loc = agent_table[agid].current_state.location;
    for (auto dir : path)
    {
        const auto& curr_cell = cell_table[curr_loc];
        const auto& curr_agent = agent_table[curr_cell.current_agent];
        int this_delay = move_cost(curr_agent.current_state.orientation, dir);
        delay = std::max(delay, this_delay);
        curr_loc = get_neighbor(*env, curr_loc, dir); 
    }
    delay = std::max(delay, availability(curr_loc));
    return delay;
}

int ash::PushSearch::shift_delay() const
{
    return shift_delay(pusher, opt_path);
}

std::vector<int> ash::PushSearch::get_affected_agents(
        int agid, const std::vector<Orientation>& path) const
{
    std::vector<int> affected_agents;
    int curr_loc = agent_table[agid].current_state.location;
    for (auto dir : path)
    {
        const auto& cell = cell_table[curr_loc];
        affected_agents.push_back(cell.current_agent);
        curr_loc = get_neighbor(*env, curr_loc, dir); 
    }
    return affected_agents;
}


std::vector<int> ash::PushSearch::get_affected_agents() const
{
    return get_affected_agents(pusher, opt_path);
}


int ash::PushSearch::commit_shift(int agid,
        const std::vector<Orientation>& path)
{
    int delay = shift_delay(agid, path);
    int curr_loc = agent_table[agid].current_state.location;
    for (auto dir : path)
    {
        int next_loc = get_neighbor(*env, curr_loc, dir);

        auto& curr_cell = cell_table[curr_loc];
        auto& next_cell = cell_table[next_loc];
        next_cell.reserved_to = curr_cell.current_agent;

        auto& curr_agent = agent_table[curr_cell.current_agent];
        curr_agent.target_state = {next_loc, dir};
        curr_agent.remaining_moves = delay;

        curr_loc = next_loc;
    }
    return delay;
}

int ash::PushSearch::commit_shift()
{
    return commit_shift(pusher, opt_path);
}

std::vector<Action> ash::PushSearch::step()
{
    auto action_vector = get_action_vector();
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        auto& agent = agent_table[agid];
        if (!agent.is_scheduled())
        {
            continue;
        }
        Action action = action_vector[agid];
        int prev_loc = agent.current_state.location;
        agent.current_state = get_neighbor(*env, agent.current_state, action);
        --agent.remaining_moves;
        if (agent.current_state == agent.target_state)
        {
            cell_table[agent.target_state.location] = {agid, -1};
            if (cell_table[prev_loc].current_agent == agid)
            {
                cell_table[prev_loc].current_agent = -1;
            }
        }
        if (agent.current_state.location == get_goal(agid))
        {
            agent.has_reached_goal = true;
        }
    }
    return action_vector;
}

bool ash::PushSearch::search(int agid)
{
    typedef std::chrono::high_resolution_clock Clock;
    auto start = Clock::now();
    this->pusher = agid;
    bool found = search_impl();
    elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    return found;
}

std::vector<Action> ash::PushSearch::get_action_vector() const
{
    std::vector<Action> action_vector(env->num_of_agents);
    for (int agid = 0; agid < env->num_of_agents; ++agid)
    {
        action_vector[agid] = get_next_action(agid);
    }
    return action_vector;
}

bool ash::PushSearch::is_pushable(int loc) const
{
    const auto& cell = cell_table[loc];
    if (!cell.is_occupied() || cell.is_reserved())
    {
        return false;
    }
    const auto& agent = agent_table[cell.current_agent];
    if (agent.is_scheduled())
    {
        return false;
    }
    return true;
}

bool ash::PushSearch::is_valid_end_of_shift(int pusher_id, int loc) const
{
    const auto& cell = cell_table[loc];
    if (cell.is_reserved())
    {
        return false;
    }
    if (!cell.is_occupied())
    {
        return true;
    }
    const auto& agent = agent_table[cell.current_agent];
    return cell.current_agent==pusher_id || agent.is_scheduled();
}

int ash::PushSearch::availability(int loc) const
{
    const auto& cell = cell_table[loc];
    if (cell.reserved_to>=0)
    {
        return -1;
    }
    if (cell.current_agent>=0)
    {
        return agent_table[cell.current_agent].remaining_moves;
    }
    return 0;
}

Action ash::PushSearch::get_next_action(int agid) const
{
    if (!agent_table[agid].is_scheduled())
    {
        return W;
    }
    auto current_dir = agent_table[agid].current_state.orientation;
    auto target_dir = agent_table[agid].target_state.orientation;
    int nturns = number_of_turns(current_dir, target_dir);
    if (nturns > 0)
    {
        return CR;
    }
    else if (nturns < 0)
    {
        return CCR;
    }
    assert(agent_table[agid].remaining_moves > std::abs(nturns));
    return agent_table[agid].remaining_moves==1? FW : W;
}

int ash::PushSearch::transition_cost(int u, int neigh, Orientation dir) const
{
    int agid = cell_table[u].current_agent;
    const auto& agent = agent_table[agid];

    int transition_cost = move_cost(agent.current_state.orientation, dir);
    int goal = get_goal(agid);
    if (goal != -1 && !agent.has_reached_goal)
    {
        int u_h = heuristic(agent.current_state, goal);
        int neigh_h = heuristic({neigh, dir}, goal);
        transition_cost += neigh_h - u_h;
    }
    assert(transition_cost >= 0);
    return transition_cost;
}

bool ash::PushSearch::search_impl()
{
    opt_cost = std::numeric_limits<int>::max();
    int goal = get_goal(pusher);
    if (goal == -1)
    {
        return false;
    }
    auto& agent = agent_table[pusher];
    int curr_h = heuristic(agent.current_state, goal);
    int push_loc = agent.current_state.location;
    for (const auto[dir,neighbor] : get_neighbors(*env, push_loc))
    {
        AgentPosition neigh_pos{neighbor, dir};
        int neigh_h = heuristic(neigh_pos, goal);
        if (neigh_h>curr_h || availability(neighbor)<0)
        {
            continue;
        }
        auto&&[path,cost] = search_impl(push_loc, dir);
        assert(cost==std::numeric_limits<int>::max() ||
               is_valid_shift(pusher, path));
        if (cost < opt_cost)
        {
            opt_cost = cost;
            opt_path = std::move(path);
        }
    }
    return opt_cost < std::numeric_limits<int>::max();
}

std::pair<std::vector<ash::Orientation>,int> ash::PushSearch::search_impl(
        int push_loc, Orientation push_dir)
{
    int start = get_neighbor(*env, push_loc, push_dir);
    int g_start = transition_cost(push_loc, start, push_dir);

    VisitedSet visited;
    OpenSet open;

    visited[start] = {push_dir, push_loc, g_start, false};
    open.emplace(g_start, start);

    while (!open.empty())
    {
        auto[g, u] = open.top();
        open.pop();
        auto& u_visited_node = visited.find(u)->second;
        if (u_visited_node.closed)
        {
            continue;
        }
        u_visited_node.closed = true;

        if (is_valid_end_of_shift(pusher, u))
        {
            // success!!
            return {reconstruct_path(visited, push_loc, u), g};
        }

        for (const auto[dir,neighbor] : get_neighbors(*env, u))
        {
            if ((u==start && neighbor==push_loc) || availability(neighbor)<0)
            {
                continue; // avoid cycles of size 2 and non-available cells
            }
            int tentative_g = g + transition_cost(u, neighbor, dir);
            auto& neigh_visited_node = visited.emplace(
                    neighbor,
                    VisitedNode{
                        {},
                        -1, 
                        std::numeric_limits<int>::max(),
                        false
                    }).first->second;
            if (tentative_g < neigh_visited_node.g)
            {
                neigh_visited_node.inbound_dir = dir;
                neigh_visited_node.parent = u;
                neigh_visited_node.g = tentative_g;
                open.emplace(tentative_g, neighbor);
            }
        }
    }
    return {{}, std::numeric_limits<int>::max()};
}

