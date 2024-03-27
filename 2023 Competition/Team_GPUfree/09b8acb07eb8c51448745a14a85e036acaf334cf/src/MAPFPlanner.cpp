#include <MAPFPlanner.h>

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    penalties = precompute_penalty_matrix();
    planners.resize(env->num_of_agents, planner(env, &penalties));
    generators = std::vector<std::mt19937>(env->num_of_agents, std::mt19937(env->num_of_agents));
    pool.reset();
}

double MAPFPlanner::get_avg_distance(int loc)
{
    std::queue<int> fringe;
    fringe.emplace(loc);
    auto result = std::vector<int>(env->map.size(),-1);
    result[loc] = 0;
    std::vector<int> moves = {1, -1, env->cols, -env->cols};
    double total_dist(0);
    int total_nodes(1);
    while(!fringe.empty()) {
        loc = fringe.front();
        int old_j = loc%env->cols;
        fringe.pop();
        for(const auto& move: moves) {
            int new_loc(loc + move);
            int new_j = new_loc%env->cols;
            if(new_loc < 0 || new_loc >= env->map.size() || abs(new_j - old_j) > 1)
                continue;
            if(env->map[new_loc] == 0 && result[new_loc] < 0) {
                result[new_loc] = result[loc] + 1;
                fringe.emplace(new_loc);
                total_nodes++;
                total_dist += result[new_loc];
            }
        }
    }
    return total_dist/total_nodes;
}

std::vector<int> MAPFPlanner::precompute_penalty_matrix()
{
    auto penalty_matrix = std::vector<int>(env->map.size(), 0);
    auto precise_matrix = std::vector<double>(env->map.size(), 0);
    double max_avg_dist(0);
    for(size_t i = 0; i < env->map.size(); i++)
        if(env->map[i] == 0) {
            precise_matrix[i] = get_avg_distance(i);
            if(precise_matrix[i] > max_avg_dist)
                max_avg_dist = precise_matrix[i];
        }
    for(size_t i = 0; i < env->map.size(); i++)
        if(env->map[i] == 0)
            penalty_matrix[i] = static_cast<int>(10 * max_avg_dist / precise_matrix[i]);
    return penalty_matrix;
}

void MAPFPlanner::revert_action(int agent_idx, int next_loc, std::unordered_map<int, std::set<int>>& used_cells, vector<Action>& actions)
{
    if(actions[agent_idx] == Action::FW)
        actions[agent_idx] = Action::W;
    used_cells[next_loc].erase(agent_idx);
    int loc = env->curr_states[agent_idx].location;
    if(used_cells.count(loc) > 0)
    {
        int other_agent = *used_cells[loc].begin();
        used_cells[loc].insert(agent_idx);
        revert_action(other_agent, loc, used_cells, actions);
    }
    else
        used_cells[loc].insert(agent_idx);
}

void MAPFPlanner::cooperate_actions(vector<Action> & actions)
{
    std::unordered_map<int, std::set<int>> used_cells;
    std::map<std::pair<int, int>, std::set<int>> used_edges;
    std::vector<int> next_locs(env->num_of_agents);

    for(int i = 0; i < env->num_of_agents; i++) {
        int loc = env->curr_states[i].location;
        int next_loc = planners[i].get_loc(env->curr_states[i].location, env->curr_states[i].orientation, actions[i]);
        if (next_loc < 0)
        {
            next_loc = loc;
            actions[i] = Action::W;
        }
        next_locs[i] = next_loc;
        used_cells[next_loc].insert(i);
        if(next_loc != loc) {
            used_edges[{next_loc, loc}].insert(i);
            used_edges[{loc, next_loc}].insert(i);
        }
    }
    for(int i = 0; i < env->num_of_agents; i++) {
        int loc = env->curr_states[i].location;
        int next_loc = next_locs[i];
        if(used_edges[{loc, next_loc}].size() > 1)
        {
            used_cells[next_loc].erase(i);
            used_cells[loc].insert(i);
            next_locs[i] = loc;
            actions[i] = Action::W;
        }
    }
    for(int i = env->num_of_agents - 1; i >= 0; i--) {
        int next_loc = next_locs[i];
        if(used_cells[next_loc].size() > 1)
            revert_action(i, next_loc, used_cells, actions);
    }
}

std::vector<float> MAPFPlanner::generate_input(size_t a_id, int radius, const std::list<int>& path)
{
    int obs_size = radius * 2 + 1;
    std::vector<float> input(obs_size*obs_size*3, 0);
    for(int i = -radius; i <= radius; i++)
        for(int j = -radius; j <= radius; j++) {
            int loc = env->curr_states[a_id].location + i*env->cols + j;
            if(loc < 0 || loc >= env->map.size())
                continue;
            if(env->map[loc])
                input[(i + radius) * obs_size + j + radius] = -1;
            switch(agents_pos[loc])
            {
                case 0: break;
                case 1: input[(i + radius) * obs_size + j + radius + obs_size * obs_size] = 1; break;
                case 4: input[(i + radius) * obs_size + j + radius + obs_size * obs_size] = -1; break;
                case 3: input[(i + radius) * obs_size + j + radius + obs_size * obs_size*2] = 1; break;
                case 2: input[(i + radius) * obs_size + j + radius + obs_size * obs_size*2] = -1; break;
            }
        }
    if(!path.empty()) {
        int cur_i = env->curr_states[a_id].location/env->cols;
        int cur_j = env->curr_states[a_id].location%env->cols;

        for (const auto &p: path) {
            int pi = (p % env->map.size()) / env->cols;
            int pj = (p % env->map.size()) % env->cols;
            if (std::abs(pi - cur_i) > radius || std::abs(pj - cur_j) > radius)
                break;
            input[obs_size * (radius + pi - cur_i) + radius + pj - cur_j] = 1;
        }
    }

    return input;
}

Action MAPFPlanner::get_action(int agent_idx)
{
    planners[agent_idx].update_path(env->curr_states[agent_idx].location, env->curr_states[agent_idx].orientation, env->goal_locations[agent_idx].front().first);
    auto path = planners[agent_idx].get_path();
    auto input = generate_input(agent_idx, actor.obs_radius, path);
    auto result = actor.get_output({input, {0}});
    std::vector<int> score;
    score.reserve(result.first.size());
    for(auto v: result.first)
        score.push_back(static_cast<int>(v * 1e6));
    std::discrete_distribution<int> distr(score.begin(), score.end());
    int action = distr(generators[agent_idx]);
    switch(action)
    {
        case 0: return Action::W;
        case 1: return Action::CR;
        case 2: return Action::CCR;
        case 3: return Action::FW;
        default: return Action::W;
    }
}

void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    if(!MAPFPlanner::env->file_storage_path.empty())
        actor = RL_module(MAPFPlanner::env->file_storage_path + "/model.onnx");
    else
        actor = RL_module("../model.onnx");
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    agents_pos = std::vector<int>(env->map.size(), 0);
    for (int i = 0; i < env->num_of_agents; i++)
        agents_pos[env->curr_states[i].location] = env->curr_states[i].orientation + 1;
    BS::multi_future<Action> future(env->num_of_agents);
    for(size_t agent_idx = 0; agent_idx < env->num_of_agents; agent_idx++) {
        if (env->goal_locations[agent_idx].empty())
            continue;
        future[agent_idx] = pool.submit(&MAPFPlanner::get_action, this, agent_idx);
    }
    actions = future.get();
    cooperate_actions(actions);
}
