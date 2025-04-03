#include "ActionFreedom.h"

void ActionFreedom::worker(int agent_id)
{
  State curr_state = env->curr_states[agent_id];
  occupy_list[agent_id] = new Occupy(curr_state.location, curr_state.orientation);
  get_child(occupy_list[agent_id]);
  if (occupy_list[agent_id]->children.size() < 4)
  {
    curr_cell_constraints[curr_state.location] = agent_id;
  }
  else if (occupy_list[agent_id]->children.size() == 4)
  {
    next_edge_constraints[*occupy_list[agent_id]->edge] = agent_id;
  }
  else
  {
    assert(false);
  }
  freedom[agent_id] = occupy_list[agent_id]->children.size();
  for (auto child : occupy_list[agent_id]->children)
  {
    get_child(child);
    freedom[agent_id] += child->children.size();
  }
}

void ActionFreedom::get_child(Occupy *root)
{
  int next_loc = root->loc + moves[root->orientation];
  if (next_loc >= 0 && next_loc < env->map.size() && env->map[next_loc] == 0)
  {
    Occupy *child = new Occupy(next_loc, root->orientation);
    root->edge = new GraphEdge(root->loc, next_loc);
    root->children.push_back(child);
  }
  Occupy *child2 = new Occupy(root->loc, (root->orientation + 1) % 4);
  root->children.push_back(child2);
  Occupy *child3 = new Occupy(root->loc, (root->orientation + 3) % 4);
  root->children.push_back(child3);
  Occupy *child4 = new Occupy(root->loc, root->orientation);
  root->children.push_back(child4);
}

void ActionFreedom::init()
{
  std::vector<std::thread> t;
  for (int i = 0; i < agent_num; ++i)
  {
    t.emplace_back(std::bind(&ActionFreedom::worker, this, i));
  }
  for (auto &th : t)
  {
    if (th.joinable())
    {
      th.join();
    }
  }
  return;
}