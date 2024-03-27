#include <MAPFPlanner.h>
#include <random>


struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


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
    eps_vec.resize(env->num_of_agents,0.0);
    path.resize(env->num_of_agents);
    on_path.resize(env->num_of_agents,false);
    //I try to estimate epsilon, the higher the number of obstacles, the higher the epsilon
    // int num_obstacles = 0;
    // for(int i = 0; i < env->map.size(); i++)
    //   if(env->map[i] == 1)
    //     num_obstacles++;
    // setEpsilon(sqrt(static_cast<double>(num_obstacles)/static_cast<double>(env->map.size())));
    setEpsilon(0.85);
    cout << "planner initialize done" << endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    int next_act;
    if(actions.size()!=env->num_of_agents)
      actions = std::vector<Action>(env->curr_states.size(), Action::W);
    //std::cerr << "epsilon is " << epsilon << endl;
    bool move_is_random = false;
    bool performed_path_step = false;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
      do
      {
        if(ElRandom::Uniform<double>(0,1) < eps_vec[i])
        {
          move_is_random = true;
          next_act = ElRandom::Uniform<int>(0,6);
          if(next_act<=3)
            actions[i] = Action::FW;
          else if(next_act==4)
            actions[i] = Action::CR;
          else if(next_act==5)
            actions[i] = Action::CCR;
          else
            actions[i] = Action::NA; 
          
          // switch (next_act)
          // {
          //   if
          //   case 0: actions[i] = Action::FW; break;
          //   case 1: actions[i] = Action::CR; break;
          //   case 2: actions[i] = Action::CCR; break;
          //   case 3: actions[i] = Action::W; break;
          //   default: actions[i] = Action::NA; break;
          // }
        }
        else
        {
          performed_path_step = false;
          move_is_random = false;
          if (env->goal_locations[i].empty() || env->curr_states[i].location == env->goal_locations[i].front().first) 
          {
              path[i] = list<pair<int,int>>(1,{env->curr_states[i].location, env->curr_states[i].orientation});
              on_path[i] = true;
          } 
          else if(!on_path[i])
          {
            //std::cerr << "I'm replanning for agent " << i 
            //     << ", env->curr_states[i].location = " << env->curr_states[i].location
            //     << ", env->goal_locations[i].front().first = " << env->goal_locations[i].front().first << endl;
            //I recompute the path
            path[i] = single_agent_plan(env->curr_states[i].location,
                                      env->curr_states[i].orientation,
                                      env->goal_locations[i].front().first);
            on_path[i] = true;
          }
          
          if(path[i].front().first == env->curr_states[i].location || path[i].front().second != env->curr_states[i].orientation)
          {
            //I have to rotate
            int incr = path[i].front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3 || incr == 2 || incr == -2)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            }  
          }
          else if (path[i].front().first != env->curr_states[i].location)
          {
            //I check that I am in the current orientation
            actions[i] = Action::FW; //forward action
            
            // std::cerr << "(" << i << ") " << "my current location is " << env->curr_states[i].location 
            // << ", path wants me in " << path[i].front().first;
            // int new_loc = env->curr_states[i].location;
            // if(env->curr_states[i].orientation == 0)
            //   new_loc++;
            // else if(env->curr_states[i].orientation == 1)
            //   new_loc+=env->cols;
            // else if(env->curr_states[i].orientation == 2)
            //   new_loc--;
            // else if(env->curr_states[i].orientation == 3)
            //   new_loc-=env->cols;
            // std::cerr << ", I will move to " << new_loc << endl;

          } 
          else
          {
            actions[i] = Action::W; //wait action
          }

          if(!feasibleAction(i, actions))
            eps_vec[i] = 1.0;
        }
      } while (!feasibleAction(i, actions));

      eps_vec[i]*=epsilon;
      if(move_is_random)
        on_path[i] = false;
      else 
      {
        on_path[i] = true;
        pair<int,int> new_pos = simulateAction(pair<int,int>({env->curr_states[i].location,env->curr_states[i].orientation}), actions[i]);
        if(new_pos == path[i].front())
            performed_path_step = true;        
        if(new_pos.first == env->goal_locations[i].front().first)
          on_path[i] = false; //I've hit my goal, I force it to recompute a new path in the next step
        //I remove the first element of the path
        if(performed_path_step)
          path[i].pop_front();
      }
    }
    
  printActions(actions);

  return;
}


list<pair<int,int>>  MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
{
    list<pair<int,int>>  this_path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                //Nota: stiamo costruendo il path dall'ultimo nodo al primo,
                //quindi l'ordine è invertito rispetto a quello che ci aspettiamo
                this_path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return this_path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}


void MAPFPlanner::printActions(const vector<Action>& actions) const
{
  for (int i = 0; i < env->num_of_agents; i++) 
  {
    if(actions[i] == Action::FW)
      cout << "FW  ";
    else if(actions[i] == Action::CR)
      cout << "CR  ";
    else if(actions[i] == Action::CCR)
      cout << "CCR ";
    else if(actions[i] == Action::W)
      cout << "W   ";
    else
      cout << "NA  ";
  }
  cout << std::endl;
}


bool MAPFPlanner::feasibleAction(int i, const vector<Action>& actions) const
{
  vector<vector<bool>> feasible_point(5,vector<bool>(5,false)); //matrix 5x5, where (2,2) is the agent itself
  pair <int,int> agent_coord = getCoords(env->curr_states[i].location,env->cols);
  pair <int,int> new_agent_coord = agent_coord;
  if(actions[i] == Action::FW)
  {
    if(env->curr_states[i].orientation == 0)
      new_agent_coord.second++;
    else if(env->curr_states[i].orientation == 1)
      new_agent_coord.first++;
    else if(env->curr_states[i].orientation == 2)
      new_agent_coord.second--;
    else if(env->curr_states[i].orientation == 3)
      new_agent_coord.first--;
  }

  for(int x = agent_coord.first-2; x <= agent_coord.first+2; x++)
    for(int y = agent_coord.second-2; y <= agent_coord.second+2; y++)
    {
      if(x >= 0 && y >= 0 && x < env->rows && y < env->cols)
        if(env->map[x*env->cols+y] == 0)
          feasible_point[2+x-agent_coord.first][2+y-agent_coord.second] = true;

      //if the cell is not feasible, and the agent is performing the action of going there, then the action is not feasible
      //I'm moving in a cell that is not feasible or I AM in a cell that is not feasible
      if( ((new_agent_coord.first == x && new_agent_coord.second == y) || (agent_coord.first == x && agent_coord.second == y))
          && !feasible_point[2+x-agent_coord.first][2+y-agent_coord.second])
      {
          return false;
      }

    }
  
  //I check if there is some agent in the new agent position, or if some agent is going to the agent position, or if some agent's move is in conflict 
  //Serve sicuramente una struttura dati ridondadnte che ricorda per ogni cella che agente c'è
  for(int j = 0; j < env->num_of_agents; j++)
  {
    if(i == j) continue;
    pair <int,int> other_agent_coord = getCoords(env->curr_states[j].location,env->cols);
    if(other_agent_coord.first >= agent_coord.first-2 && other_agent_coord.first <= agent_coord.first+2 &&
       other_agent_coord.second >= agent_coord.second-2 && other_agent_coord.second <= agent_coord.second+2)
    {
      pair <int,int> new_other_agent_coord = other_agent_coord;
      if(actions[j] == Action::FW)
      {
        if(env->curr_states[j].orientation == 0)
          new_other_agent_coord.second++;
        else if(env->curr_states[j].orientation == 1)
          new_other_agent_coord.first++;
        else if(env->curr_states[j].orientation == 2)
          new_other_agent_coord.second--;
        else if(env->curr_states[j].orientation == 3)
          new_other_agent_coord.first--;
      }
      //I have a conflict if the other agent is going to the same cell where I am going,
      //or if they are traversing the same edge
      //Cell conflict
      if(new_agent_coord.first == new_other_agent_coord.first && new_agent_coord.second == new_other_agent_coord.second)
        return false;
      //Edge conflict
      if(new_agent_coord.first == other_agent_coord.first && new_agent_coord.second == other_agent_coord.second &&
         agent_coord.first == new_other_agent_coord.first && agent_coord.second == new_other_agent_coord.second)
        return false;
    }
  }
  return true;
}
    



pair<int, int> MAPFPlanner::simulateAction(pair<int, int> loc_or, Action action)
{
  if(action == Action::FW)
  {
    if(loc_or.second == 0)
      loc_or.first++;
    else if(loc_or.second == 1)
      loc_or.first+=env->cols;
    else if(loc_or.second == 2)
      loc_or.first--;
    else if(loc_or.second == 3)
      loc_or.first-=env->cols;
  }else if (action == Action::CR)
  {
    loc_or.second--;
    if(loc_or.second == -1)
      loc_or.second = 3;
  }
  else if (action == Action::CCR)
  {
    loc_or.second++;
    if(loc_or.second == 4)
      loc_or.second = 0;
  }
  return loc_or;
}