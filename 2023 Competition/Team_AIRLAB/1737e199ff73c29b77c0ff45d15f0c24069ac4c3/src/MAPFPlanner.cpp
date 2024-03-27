#include <MAPFPlanner.h>

#include <cmath>
#include <random>

struct AstarNode {
  int location;
  int direction;
  int f, g, h;
  AstarNode* parent;
  int t = 0;
  bool closed = false;
  AstarNode(int _location, int _direction, int _g, int _h, AstarNode* _parent)
      : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h), parent(_parent) {}
  AstarNode(int _location, int _direction, int _g, int _h, int _t, AstarNode* _parent)
      : location(_location),
        direction(_direction),
        f(_g + _h),
        g(_g),
        h(_h),
        t(_t),
        parent(_parent) {}
};

struct cmp {
  bool operator()(AstarNode* a, AstarNode* b) {
    if (a->f == b->f)
      return a->g <= b->g;
    else
      return a->f > b->f;
  }
};

void MAPFPlanner::initialize(int preprocess_time_limit) {
  // for (int i = 0; i < env->num_of_agents; i++) agents.emplace_back(instance, i, true);
  int puzzle_depth = 2;
  std::string fname_puzzle;
  fname_puzzle = env->file_storage_path + "/" + "puzzleGrid_depth_" + std::to_string(puzzle_depth) +
                 "_" + env->map_name + ".txt";

  puzzleGrid = new PuzzleGrid(env, fname_puzzle, env->rows, env->cols, puzzle_depth);
  if (!puzzleGrid->load_unweighted_map(env)) {
    cout << "Failed to load map file: " << env->map_name << endl;
    exit(1);
  }
  std::ifstream myfile(fname_puzzle.c_str());
  if (!myfile.is_open()) {
    cout << "Failed to open puzzle grid file: " << fname_puzzle << endl;
    puzzleGrid->generatePuzzleGrid(fname_puzzle);
  }
  puzzleGrid->puzzleGridRead(fname_puzzle.c_str());
  puzzleGrid->preprocessing(env, puzzleGrid);
}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
  getConst(env->map_name, env->num_of_agents);
  instance.initialize();
  if (time_step % time_interval == 0) {
    cout <<"========================================"<<endl;
    cout << "Map name: " << env->map_name << " | # of agents: " << env->num_of_agents
         << " | time step: " << time_step << " | time interval: " << time_interval << endl;
    double hi = std::clock();

    lns = new LNS(instance, INT_MAX, initAlgo, replanAlgo, destoryStrategy, neighborSize,
                  maxIterations, initLNS, initDestoryStrategy, sipp, screen);
    instance.start_states.clear();
    instance.start_states.resize(env->num_of_agents);
    instance.goal_locations.clear();
    instance.goal_locations.resize(env->num_of_agents);

    double t = std::clock();
    for (int id = 0; id < env->num_of_agents; id++) {
      instance.start_states[id] =
          State(env->curr_states[id].location, 0, env->curr_states[id].orientation);
      instance.goal_locations[id] = env->goal_locations[id].front().first;
    }
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    cout << "Load Puzzle Heurisitic time(frag: " << ratio_frag << " ): " << runtime << endl;
    if (ratio_frag<1)
      frag = puzzleGrid->getFrag(instance, env, ratio_frag);

    lns->initialize();

    time_t now = time(0);
    tm* ltm = localtime(&now);
    for (int id = 0; id < env->num_of_agents; id++) {
      puzzleGrid->heuristicsAgent(lns->agents[id].path_planner);
    }
    succ = lns->run(running_time);
    if (longest_time < running_time) longest_time = running_time;

    cout << "Init time for " << time_step << "s (현재시간: " << ltm->tm_min << "분 " << ltm->tm_sec
         << "초): " << (std::clock() - hi) / CLOCKS_PER_SEC << " | longest_time: " << longest_time
         << endl;
    // if (succ) {
    //   lns->validateSolution();
    //   lns->writePathsToFile(outputPaths);
    // }
    // lns->writeResultToFile(output);
  }

  actions = std::vector<Action>(env->curr_states.size(), Action::W);
  for (int i = 0; i < env->num_of_agents; i++) {
    list<pair<int, int>> path;
    // if (env->goal_locations[i].empty()) {
    if (lns->agents[i].path.size() <= 1 + time_step % time_interval) {
      path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
    } else {
      path.emplace_back(lns->agents[i].path[1 + time_step % time_interval].location,
                        lns->agents[i].path[1 + time_step % time_interval].orientation);
    }
    if (path.front().first != env->curr_states[i].location) {
      actions[i] = Action::FW;  // forward action
    } else if (path.front().second != env->curr_states[i].orientation) {
      int incr = path.front().second - env->curr_states[i].orientation;
      if (incr == 1 || incr == -3) {
        actions[i] = Action::CR;  // C--counter clockwise rotate
      } else if (incr == -1 || incr == 3) {
        actions[i] = Action::CCR;  // CCR--clockwise rotate
      }
    }
  }

  time_step++;
  if (time_step % time_interval == 0) delete lns;
  return;
}

void MAPFPlanner::getConst(string map_name, int num_of_agents) {
  auto get_frag = [this](float start_frag, float asymptote, int critical_time) {
    if (cnt_overtime == 0) ratio_frag = start_frag;
    if (running_time > critical_time) {
      ratio_frag = (start_frag - asymptote) * std::exp(-cnt_overtime) + asymptote ;
      cnt_overtime += 0.5 / instance.window;
    }
  };
  if (env->map_name == "random-32-32-20.map") {
    if (env->num_of_agents <= 20){
      time_interval = 5;
      instance = Instance(env, 15);
      neighborSize = 8;
    } else if (env->num_of_agents <= 200) {
      time_interval = 5;
      instance = Instance(env, 15);
      neighborSize = 24;
    } else if (env->num_of_agents <= 400) {
      time_interval = 15;
      instance = Instance(env, 30);
      neighborSize = 24;
      get_frag(1.0, 0.85, 30);
    } else {
      time_interval = 15;
      instance = Instance(env, 30);
      ratio_frag = 0.05;
    }
  }

  else if (env->map_name == "warehouse_large.map") {
    if (env->num_of_agents < 600) {
      time_interval = 10;
      instance = Instance(env, 15);
    } else if (env->num_of_agents <= 2000) {
      time_interval = 15;
      instance = Instance(env, 30);
      if (time_step > 1000) ratio_frag = 0.8;
    } else if (env->num_of_agents <= 3000) {
      time_interval = 15;
      instance = Instance(env, 30);
      neighborSize = 32;
      get_frag(0.8, 0.7, 50);
    } else if (env->num_of_agents <= 5000) {
      time_interval = 40;
      instance = Instance(env, 50);
      neighborSize = 32;
      ratio_frag = 0.4;
    } else {
      time_interval = 40;
      instance = Instance(env, 50);
      ratio_frag = 0.2;
    }
  } else if (env->map_name == "sortation_large.map") {
    if (env->num_of_agents < 600) {
      time_interval = 10;
      instance = Instance(env, 15);
    } else if (env->num_of_agents <= 2000) {
      time_interval = 15;
      instance = Instance(env, 30);
    } else if (env->num_of_agents <= 3000) {
      time_interval = 30;
      instance = Instance(env, 45);
      get_frag(0.8, 0.5, 50);
    } else if (env->num_of_agents <= 5000) {
      time_interval = 30;
      instance = Instance(env, 35);
      ratio_frag = 0.4;
    } else {
      time_interval = 40;
      instance = Instance(env, 50);
      ratio_frag = 0.2;
    }
  } else if (env->map_name == "Paris_1_256.map") {
    if (env->num_of_agents < 600) {
      time_interval = 10;
      instance = Instance(env, 15);
    } else if (env->num_of_agents <= 2000) {
      time_interval = 15;
      instance = Instance(env, 30);
    } else if (env->num_of_agents <= 3000) {
      time_interval = 30;
      instance = Instance(env, 45);
      get_frag(0.8, 0.5, 50);
    } else if (env->num_of_agents <= 5000) {
      time_interval = 30;
      instance = Instance(env, 45);
      get_frag(0.5, 0.3, 50);
    } else {
      time_interval = 30;
      instance = Instance(env, 50);
      neighborSize = 32;
      ratio_frag = 0.1;
    }
  } else {  // brc202d.map
    if (env->num_of_agents < 600) {
      time_interval = 10;
      instance = Instance(env, 15);
      neighborSize = 8;
    } else if (env->num_of_agents <= 2000) {
      time_interval = 15;
      instance = Instance(env, 30);
      neighborSize = 32;
    } else if (env->num_of_agents <= 3000) {
      time_interval = 15;
      instance = Instance(env, 30);
      get_frag(0.5, 0.3, 50);
      neighborSize = 32;
    } else if (env->num_of_agents <= 5000) {
      time_interval = 30;
      instance = Instance(env, 50);
      neighborSize = 32;
      get_frag(0.5, 0.3, 50);
    } else { 
      time_interval = 40;
      instance = Instance(env, 50);
      neighborSize = 48;
      ratio_frag = 0.2;
    }
  }
}

list<pair<int, int>> MAPFPlanner::single_agent_plan(int start, int start_direct, int end) {
  list<pair<int, int>> path;
  priority_queue<AstarNode*, vector<AstarNode*>, cmp> open_list;
  unordered_map<int, AstarNode*> all_nodes;
  unordered_set<int> close_list;
  AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start, end), nullptr);
  open_list.push(s);
  all_nodes[start * 4 + start_direct] = s;

  while (!open_list.empty()) {
    AstarNode* curr = open_list.top();
    open_list.pop();
    close_list.emplace(curr->location * 4 + curr->direction);
    if (curr->location == end) {
      while (curr->parent != NULL) {
        path.emplace_front(make_pair(curr->location, curr->direction));
        curr = curr->parent;
      }
      break;
    }
    list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction);
    for (const pair<int, int>& neighbor : neighbors) {
      if (close_list.find(neighbor.first * 4 + neighbor.second) != close_list.end()) continue;
      if (all_nodes.find(neighbor.first * 4 + neighbor.second) != all_nodes.end()) {
        AstarNode* old = all_nodes[neighbor.first * 4 + neighbor.second];
        if (curr->g + 1 < old->g) {
          old->g = curr->g + 1;
          old->f = old->h + old->g;
          old->parent = curr;
        }
      } else {
        AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second, curr->g + 1,
                                             getManhattanDistance(neighbor.first, end), curr);
        open_list.push(next_node);
        all_nodes[neighbor.first * 4 + neighbor.second] = next_node;
      }
    }
  }
  for (auto n : all_nodes) {
    delete n.second;
  }
  all_nodes.clear();
  return path;
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {
  int loc1_x = loc1 / env->cols;
  int loc1_y = loc1 % env->cols;
  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2) {
  int loc_x = loc / env->cols;
  int loc_y = loc % env->cols;

  if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1) return false;

  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1) return false;
  return true;
}

list<pair<int, int>> MAPFPlanner::getNeighbors(int location, int direction) {
  list<pair<int, int>> neighbors;
  // forward
  int candidates[4] = {location + 1, location + env->cols, location - 1, location - env->cols};
  int forward = candidates[direction];
  int new_direction = direction;
  if (forward >= 0 && forward < env->map.size() && validateMove(forward, location))
    neighbors.emplace_back(make_pair(forward, new_direction));
  // turn left
  new_direction = direction - 1;
  if (new_direction == -1) new_direction = 3;
  neighbors.emplace_back(make_pair(location, new_direction));
  // turn right
  new_direction = direction + 1;
  if (new_direction == 4) new_direction = 0;
  neighbors.emplace_back(make_pair(location, new_direction));
  neighbors.emplace_back(make_pair(location, direction));  // wait
  return neighbors;
}
