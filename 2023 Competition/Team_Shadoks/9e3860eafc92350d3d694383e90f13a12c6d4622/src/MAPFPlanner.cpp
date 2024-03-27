#include <MAPFPlanner.h>
#include <random>

/**
 * Start implementation of the planner, which uses a A* search algorithm
 * This does not avoid colisions between nodes.
 */

// Node for the A* search algorithm
struct AstarNode
{
    int location;
    int direction;
    int f; // sum of g and h
    int g; // cost of the current path
    int h; // estimate of the cost of the remaining path
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};

// Comparison for the A* search algorithm
struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    m_paths.resize(env->num_of_agents);
    cout << "planner initialize done" << endl;
    cout << "AGL: Initialization happens here!" << endl;
}

void MAPFPlanner::reset_paths() {
    m_paths.clear();
    m_paths.resize(env->num_of_agents);
}


// plan using simple A* that ignores the time dimension
// We plan a path for each robot and take the first step for this timestemp
void MAPFPlanner::plan(int time_limit, vector<Action> & actions) 
{
    reset_paths();
    actions = std::vector<Action>(env->curr_states.size(), Action::W); // set the default action of waiting

    for (int i = 0; i < env->num_of_agents; i++) // for each agent (robot)
    {
        /* Make a path for each agent to its goal.
           If there is no goal, make just a path of one step (the same position it has).
           Otherwise, plan its path to the current (or latest) goal 
        */
        const Position cur_pos(env->curr_states[i].location, env->curr_states[i].orientation, 0);
        if (env->goal_locations.at(i).empty()) { // if no goal, do not move, that is, keep the same location and orientation
            cout << "ERROR : agent " << i << "has no task" << endl;
        } else {
            plan_path(i, cur_pos, env->goal_locations.at(i).front().first);
        }
        /* Now that the path is computed, we set the action of the agent:
           - if the first location of the path is not the current location, then it means that the agent walks forward
           - otherwise, the agent is rotating
        */
        actions.at(i) = m_paths.at(i).at(1).action(cur_pos); 
    }
  return;
}

// Get the shortest path for agent `agent` without collisions with the already planned agents
// agent:   index of the current agent
// start:   start position
// end:     end location
void MAPFPlanner::plan_path(int agent, const Position &start, int end) {
    cout << "Planning the path of agent " << agent 
         << " from " << start.location%env->cols << "," << start.location/env->cols 
         << " to " << end%env->cols << "," << end/env->cols 
         << endl;
    vector<Position> path;
    priority_queue<AstarNode*, vector<AstarNode*>, cmp> open_list; // search queue
    unordered_map<int, AstarNode*> all_nodes; // nodes indexed by location
    unordered_set<int> close_list; // set of done nodes
    AstarNode* s = new AstarNode(start.location, start.direction, 0, getManhattanDistance(start.location, end), nullptr);
    open_list.push(s);
    all_nodes[start.encoding()] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top(); open_list.pop(); // get the next node
        close_list.emplace(curr->location*4 + curr->direction); // mark it as done
        if (curr->location == end) // if we find the goal, trace back the path
        {
            while (curr->parent != NULL) 
            {
                const Position pos(curr->location, curr->direction, curr->g);
                path.push_back(pos);
                curr = curr->parent;
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            break;
        }
        // otherwise, try every next step possible and add the node to the open list
        list<Position> neighborhood = neighbors(agent, Position(curr->location, curr->direction, curr->g));
        for (const Position &neighbor : neighborhood)
        {
            if (close_list.find(neighbor.encoding()) != close_list.end()) // the node is already done
                continue;
            if (all_nodes.find(neighbor.encoding()) != all_nodes.end()) { // the node is already in the list: update its value
                AstarNode* old = all_nodes[neighbor.encoding()];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g + 1;
                    old->f = old->h + old->g;
                    old->parent = curr;
                }
            } else { // create the node
                AstarNode* next_node = new AstarNode(
                    neighbor.location, 
                    neighbor.direction,
                    curr->g + 1,
                    getManhattanDistance(neighbor.location, end), 
                    curr
                );
                open_list.push(next_node);
                all_nodes[neighbor.encoding()] = next_node;
            }
        }
    }
    for (auto n : all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    if (path.empty()) {
        cout << "ERROR: empty path" << endl;
        path.push_back(start);
        path.push_back(start);
    }
    m_paths.at(agent) = path;
    cout << "Path for agent " << agent << " computed." << endl;
    // print_path(agent);
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2) const {
    int loc1_y = loc1/env->cols;
    int loc1_x = loc1%env->cols;
    int loc2_y = loc2/env->cols;
    int loc2_x = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


// Compute all the possible steps after a given position
list<Position> MAPFPlanner::neighbors(int agent, const Position &position) const {
    list<Position> neighbors;

    // Wait
    Position pos_w(position.location, position.direction, position.time + 1);
    if (collision_free(agent, position, pos_w))
        neighbors.push_back(pos_w);

    // Forward
    Position pos_fw(position.location, position.direction, position.time + 1);
    const int col = position.location % env->cols;
    const int row = position.location / env->cols;
    if (position.direction == 0 && col+1 < env->cols) // move east
        pos_fw.location++;
    else if (position.direction == 1 && row+1 < env->rows)
        pos_fw.location += env->cols;
    else if (position.direction == 2 && col-1 >= 0)
        pos_fw.location--;
    else if (position.direction == 3 && row-1 >= 0)
        pos_fw.location -= env->cols;
    else
        pos_fw.location = -1;
    if (pos_fw.location >= 0 && collision_free(agent, position, pos_fw))
        neighbors.push_back(pos_fw);

    // Clock-wise
    Position pos_cw(position.location, (position.direction+1) % 4, position.time + 1);
    if (collision_free(agent, position, pos_cw))
        neighbors.push_back(pos_cw);

    // Counter-clock-wise
    Position pos_ccw(position.location, (position.direction+3) % 4, position.time + 1);
    if (collision_free(agent, position, pos_ccw))
        neighbors.push_back(pos_ccw);

    return neighbors;
}

bool MAPFPlanner::collision_free(int agent, const Position &cur_pos, const Position &next) const {
    // We hit an obstacle
    if (env->map.at(next.location) == 1)
        return false;
    // We hit another robot
    const int t = cur_pos.time;
    for (int i = 0; i < m_paths.size(); i++) {
        if (m_paths.at(i).size() > t+1 && i != agent) {
            if (m_paths.at(i).at(t+1).location == next.location) {
                return false;
            }
        }
    }
    // We cross another robot
    for (int i = 0; i < m_paths.size(); i++) {
        if (m_paths.at(i).size() > t+1 && i != agent) {
            if (m_paths.at(i).at(t).location == next.location && m_paths.at(i).at(t+1).location == cur_pos.location) {
                return false;
            }
        }
    }
    return true;
}

void MAPFPlanner::print_path(int i) const {
    cout << "Path for agent " << i << ":";
    for (const Position pos : m_paths.at(i)) {
        const int x = pos.location % env->cols;
        const int y = pos.location / env->cols;
        cout << " (x:" << x << ", y:" << y << ", dir:" << pos.direction << ", t:" << pos.time << ")";
    }
    cout << endl;
}