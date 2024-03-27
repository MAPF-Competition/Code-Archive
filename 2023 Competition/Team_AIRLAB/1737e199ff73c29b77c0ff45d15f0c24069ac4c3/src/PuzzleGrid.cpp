#include "PuzzleGrid.h"

void PuzzleGrid::puzzleGridRead(std::string fname) {
  std::ifstream file(fname);
  if (!file.is_open()) {
    std::cerr << "File is not existed." << std::endl;
  }
  file >> num_of_ids;
  num_of_ids++;

  std::vector<bool> isIdRead(num_of_ids, false);
  puzzle_grid.resize(N);
  center_locations.resize(num_of_ids);
  int id, location;
  for (int i = 0; i < N; i++) {
    if (!(file >> id >> location)) continue;
    puzzle_grid[i] = std::make_pair(id, location);
    if (id == -1) continue;
    if (!isIdRead[id]) {
      center_locations[id] = location;
      isIdRead[id] = true;
    }
  }
}

void PuzzleGrid::heuristicsAgent(SingleAgentSolver *path_planner) {
  int goal_location = path_planner->goal_location;
  int goalLocationId = get<0>(puzzle_grid[goal_location]);

  auto it = heuristics.find(goalLocationId);
  if (it == heuristics.end()) return;

  path_planner->my_heuristic.resize(N, MAX_TIMESTEP);
  for (int i = 0; i < N; i++) {
    int iId = get<0>(puzzle_grid[i]);
    if (iId != -1) {
      path_planner->my_heuristic[i] = it->second[iId];
    }
  }
}

void PuzzleGrid::puzzleGridPrint() {
  for (auto &h : heuristics) {
    cout << h.first << ": ";
    for (auto &v : h.second) {
      cout << v << " ";
    }
    cout << endl;
  }
}

void PuzzleGrid::generatePuzzleGrid(string fname) {
  puzzle_grid.resize(env->rows * env->cols, std::make_tuple(-1, -1));
  for (int location = 0; location < env->rows * env->cols; location++) {
    puzzle_grid[location] = std::make_tuple(-1, location);
  }
  int id = 0;
  for (int location = 0; location < env->rows * env->cols; location++) {
    if (env->map[location] == 1) continue;
    if (std::get<0>(puzzle_grid[location]) != -1) continue;
    //       generatePuzzleByCell(location, id++);
    generatePuzzleByDepth(location, id++);
  }
  //    postProcessing();
  std::ofstream file(fname);
  int maxId = std::get<0>(puzzle_grid[0]);
  for (const auto &[id, location] : puzzle_grid) {
    if (id > maxId) maxId = id;
  }
  file << maxId << std::endl;
  for (const auto &[id, location] : puzzle_grid) {
    file << id << " " << location << std::endl;
  }
  file.close();
};

void PuzzleGrid::generatePuzzleByCell(int startLocation, int id) {
  std::queue<int> locationsQueue;
  locationsQueue.push(startLocation);
  puzzle_grid[startLocation] = std::make_tuple(id, startLocation);

  int filledCells = 1;

  while (!locationsQueue.empty() && filledCells <= cells) {
    int currentLocation = locationsQueue.front();
    locationsQueue.pop();

    int currentRow = currentLocation / env->cols;

    for (int direction : directions) {
      int nextLocation = currentLocation + direction;

      if (nextLocation < 0 || nextLocation >= env->rows * env->cols) continue;
      if (direction == 1 && currentRow < nextLocation / env->cols) continue;
      if (direction == -1 && currentRow > nextLocation / env->cols) continue;
      if (std::get<0>(puzzle_grid[nextLocation]) != -1) continue;
      if (env->map[nextLocation] == 1) continue;

      locationsQueue.push(nextLocation);
      puzzle_grid[nextLocation] = std::make_tuple(id, nextLocation);
      filledCells++;

      if (filledCells > cells) {
        break;
      }
    }
  }
}

void PuzzleGrid::generatePuzzleByDepth(int startLocation, int id) {
  std::queue<int> locationsQueue;
  std::queue<int> depthQueue;

  locationsQueue.push(startLocation);
  depthQueue.push(0);
  puzzle_grid[startLocation] = std::make_tuple(id, startLocation);

  while (!locationsQueue.empty()) {
    int currentLocation = locationsQueue.front();
    int currentDepth = depthQueue.front();
    locationsQueue.pop();
    depthQueue.pop();

    if (currentDepth >= depth) break;

    int currentRow = currentLocation / env->cols;

    for (int direction : directions) {
      int nextLocation = currentLocation + direction;

      if (nextLocation < 0 || nextLocation >= env->rows * env->cols) continue;
      if (direction == 1 && currentRow < nextLocation / env->cols) continue;
      if (direction == -1 && currentRow > nextLocation / env->cols) continue;
      if (std::get<0>(puzzle_grid[nextLocation]) != -1) continue;
      if (env->map[nextLocation] == 1) continue;

      locationsQueue.push(nextLocation);
      depthQueue.push(currentDepth + 1);
      puzzle_grid[nextLocation] = std::make_tuple(id, nextLocation);
    }
  }
}

void PuzzleGrid::postProcessing() {
  int max_id = INT_MIN;  // Start with the smallest possible integer
  // Iterate over all pairs and find the maximum first element
  for (const auto &element : puzzle_grid) {
    if (std::get<0>(element) > max_id) max_id = std::get<0>(element);
  }

  num_of_cells_in_id.resize(max_id + 1, 0);
  for (int location = 0; location < env->rows * env->cols; location++) {
    int id = std::get<0>(puzzle_grid[location]);
    num_of_cells_in_id[id] += 1;
  }

  int num_of_one_ids = 0;
  for (int location = 0; location < env->rows * env->cols; location++) {
    int id = std::get<0>(puzzle_grid[location]);
    if (num_of_cells_in_id[id] == 1) {
      num_of_one_ids++;
      int numAdjacentCells;
      int minNumAdjacentCells = INT_MAX;
      int adjacentId;
      int currentRow = location / env->cols;
      for (int direction : directions) {
        int nextLocation = location + direction;
        if (nextLocation < 0 || nextLocation >= env->rows * env->cols) continue;
        if (direction == 1 && currentRow < nextLocation / env->cols) continue;
        if (direction == -1 && currentRow > nextLocation / env->cols) continue;
        if (env->map[nextLocation] == 1) continue;
        numAdjacentCells = num_of_cells_in_id[std::get<0>(puzzle_grid[nextLocation])];
        if (numAdjacentCells < minNumAdjacentCells) {
          minNumAdjacentCells = numAdjacentCells;
          adjacentId = std::get<0>(puzzle_grid[nextLocation]);
        }
      }
      puzzle_grid[location] = make_tuple(adjacentId, location);
      num_of_cells_in_id[adjacentId]++;
      num_of_cells_in_id[id]--;
    }
  }

  cout << num_of_one_ids << endl;
}

bool PuzzleGrid::load_unweighted_map(SharedEnvironment *env) {
  std::cout << "*** Loading map ***" << std::endl;
  clock_t t = std::clock();

  types.resize(rows * cols);
  weights.resize(rows * cols);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      int id = cols * i + j;
      weights[id].resize(5, WEIGHT_MAX);
      if (env->map[id] == 1)  // obstacle
      {
        types[id] = "Obstacle";
      } else {
        types[id] = "Travel";
        weights[id][4] = 1;
      }
    }
  }

  for (int i = 0; i < cols * rows; i++) {
    if (types[i] == "Obstacle") {
      continue;
    }
    for (int dir = 0; dir < 4; dir++) {
      if (0 <= i + directions[dir] && i + directions[dir] < cols * rows &&
          get_Manhattan_distance(i, i + directions[dir]) <= 1 &&
          types[i + directions[dir]] != "Obstacle")
        weights[i][dir] = 1;
      else
        weights[i][dir] = WEIGHT_MAX;
    }
  }
  double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
  std::cout << "Map size: " << rows << "x" << cols << " with ";
  std::cout << "Done! (" << runtime << " s)" << std::endl;
  return true;
}

void PuzzleGrid::preprocessing(SharedEnvironment *env, PuzzleGrid *puzzleGrid) {
  std::cout << "*** PreProcessing map ***" << std::endl;
  clock_t t = std::clock();
  heuristics.clear();

  std::string fname;
  fname = env->file_storage_path + "/" + env->map_name + "_depth_" +
          std::to_string(puzzleGrid->depth) + "_rotation_heuristics_table.txt";
  std::ifstream myfile(fname.c_str());
  bool succ = false;
  if (myfile.is_open()) {
    succ = load_heuristics_table(myfile);
    myfile.close();
  }

  if (!succ) {
    cout << "Heuristics table not found. Computing heuristics..." << endl;
    exit(-1);  //필요시 주석
    int divisor = 50;
    int chunkSize = puzzleGrid->num_of_ids / divisor;

    for (int start = puzzleGrid->num_of_ids; start > 0; start -= chunkSize) {
      int end = std::max(start - chunkSize, 0);  // Ensure that we don't go below 0.
      for (int id = end; id < start; id++) {
        heuristics[id] =
            compute_heuristics(puzzleGrid->center_locations[id], puzzleGrid->center_locations);
      }
      save_heuristics_table(fname, start, puzzleGrid->num_of_ids, puzzleGrid->center_locations);
      std::cout << "Saved heuristics table to " << fname << std::endl;
      std::cout << "Cleared heuristics table.  [ " << divisor - start / chunkSize << " / "
                << divisor << " ]" << std::endl;
      heuristics.clear();
    }
    cout << "Done!" << endl;
  }
  puzzleGrid->heuristics = heuristics;
  double runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
  std::cout << "Done! (" << runtime << " s)" << std::endl;
}

bool PuzzleGrid::load_heuristics_table(std::ifstream &myfile) {
  boost::char_separator<char> sep(",");
  boost::tokenizer<boost::char_separator<char>>::iterator beg;
  std::string line;

  getline(myfile, line);  // skip "table_size"
  getline(myfile, line);
  boost::tokenizer<boost::char_separator<char>> tok(line, sep);
  beg = tok.begin();
  int N = atoi((*beg).c_str());  // read number of cols
  beg++;
  int M = atoi((*beg).c_str());  // read number of rows
  // if (M != this->size()) return false;
  for (int i = 0; i < N; i++) {
    getline(myfile, line);
    int loc = atoi(line.c_str());
    getline(myfile, line);
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    beg = tok.begin();
    std::vector<double> h_table(M);
    for (int j = 0; j < M; j++) {
      h_table[j] = atof((*beg).c_str());
      if (h_table[j] >= INT_MAX && types[j] != "Obstacle") types[j] = "Obstacle";
      beg++;
    }
    heuristics[loc] = h_table;
  }
  return true;
}

vector<double> PuzzleGrid::compute_heuristics(int root_location,
                                              std::vector<int> &puzzleGridCenterPoint) {
  std::vector<double> res(N, INT_MAX);
  std::vector<double> resCenter(puzzleGridCenterPoint.size(), INT_MAX);
  fibonacci_heap<StateTimeAStarNode *, compare<StateTimeAStarNode::compare_node>> heap;
  unordered_set<StateTimeAStarNode *, StateTimeAStarNode::Hasher, StateTimeAStarNode::EqNode> nodes;

  State root_state(root_location);
  for (auto neighbor : get_reverse_neighbors(root_state)) {
    StateTimeAStarNode *root = new StateTimeAStarNode(
        State(root_location, -1, get_direction(neighbor.location, root_state.location)), 0, 0,
        nullptr, 0);
    root->open_handle = heap.push(root);  // add root to heap
    nodes.insert(root);                   // add root to hash_table (nodes)
  }

  while (!heap.empty()) {
    StateTimeAStarNode *curr = heap.top();
    heap.pop();
    for (auto next_state : get_reverse_neighbors(curr->state)) {
      double next_g_val = curr->g_val + get_weight(next_state.location, curr->state.location);
      StateTimeAStarNode *next = new StateTimeAStarNode(next_state, next_g_val, 0, nullptr, 0);
      auto it = nodes.find(next);
      if (it == nodes.end()) {  // add the newly generated node to heap and hash table
        next->open_handle = heap.push(next);
        nodes.insert(next);
      } else {          // update existing node's g_val if needed (only in the heap)
        delete (next);  // not needed anymore -- we already generated it before
        StateTimeAStarNode *existing_next = *it;
        if (existing_next->g_val > next_g_val) {
          existing_next->g_val = next_g_val;
          heap.increase(existing_next->open_handle);
        }
      }
    }
  }
  // iterate over all nodes and populate the distances
  for (auto it = nodes.begin(); it != nodes.end(); it++) {
    StateTimeAStarNode *s = *it;
    res[s->state.location] = std::min(s->g_val, res[s->state.location]);
    delete (s);
  }
  nodes.clear();
  heap.clear();
  for (int i = 0; i < puzzleGridCenterPoint.size(); i++) {
    int loc = puzzleGridCenterPoint[i];
    resCenter[i] = res[loc];
  }
  return resCenter;
}

list<State> PuzzleGrid::get_reverse_neighbors(const State &s) const {
  std::list<State> rneighbors;
  // no wait actions
  if (s.orientation >= 0) {
    if (s.location - directions[s.orientation] >= 0 && s.location - directions[s.orientation] < N &&
        weights[s.location - directions[s.orientation]][s.orientation] < WEIGHT_MAX - 1)
      rneighbors.emplace_back(s.location - directions[s.orientation], -1, s.orientation);  // move
    int next_orientation1 = s.orientation + 1;
    int next_orientation2 = s.orientation - 1;
    if (next_orientation2 < 0)
      next_orientation2 += 4;
    else if (next_orientation1 > 3)
      next_orientation1 -= 4;
    rneighbors.emplace_back(s.location, -1, next_orientation1);  // turn right
    rneighbors.emplace_back(s.location, -1, next_orientation2);  // turn left
  } else {
    for (int i = 0; i < 4; i++)  // move
      if (s.location - directions[i] >= 0 && s.location - directions[i] < N &&
          weights[s.location - directions[i]][i] < WEIGHT_MAX - 1)
        rneighbors.emplace_back(s.location - directions[i]);
  }
  return rneighbors;
}

double PuzzleGrid::get_weight(int from, int to) const {
  if (from == to)  // wait or rotate
    return weights[from][4];
  int dir = get_direction(from, to);
  if (dir >= 0)
    return weights[from][dir];
  else
    return WEIGHT_MAX;
}

int PuzzleGrid::get_direction(int from, int to) const {
  for (int i = 0; i < 4; i++) {
    if (directions[i] == to - from) return i;
  }
  if (from == to) return 4;
  return -1;
}

void PuzzleGrid::save_heuristics_table(std::string fname, int step, int first_step,
                                       std::vector<int> &puzzleGridCenterPoint) {
  std::ofstream myfile;
  myfile.open(fname, std::ios_base::app);
  if (step == first_step)
    myfile << "table_size" << std::endl
           << puzzleGridCenterPoint.size() << "," << puzzleGridCenterPoint.size() << std::endl;
  for (auto h_values : heuristics) {
    myfile << h_values.first << std::endl;
    for (double h : h_values.second) {
      myfile << h << ",";
    }
    myfile << std::endl;
  }
  myfile.close();
}

vector<int> PuzzleGrid::getFrag(Instance &instance, SharedEnvironment *env, float ratio_frag) {
  double hihi1 = std::clock();
  vector<int> map_id;
  map_id.resize(env->rows * env->cols, -1);
  for (int i = 0; i < env->num_of_agents; i++) {
    map_id[instance.start_states[i].location] = i;
  }

  std::queue<int> lotations_queue;
  vector<int> frag_ids;
  vector<int> selected;
  vector<int> visited;
  vector<int> result;
  selected.resize(env->rows * env->cols, 0);
  visited.resize(env->rows * env->cols, 0);
  result.resize(env->num_of_agents, 0);

  //int startLocation = instance.start_states[0].location;
  int startLocation = center_locations[0];
  if (env ->map_name == "warehouse_large.map"){
    startLocation = 10000;
  }
  //frag_ids.push_back(0);
  
  lotations_queue.push(startLocation);
  int cnt_queue = 0;
  // if (env->map_name == "brc202d.map"|| env->map_name == "warehouse_large.map") {
  for (int i = 0; i < env->num_of_agents* (1-ratio_frag); i++) {
    frag_ids.push_back(i);
  }
  // } 
  // else {
  //   while (!lotations_queue.empty()) {
  //     int current_location = lotations_queue.front();
  //     lotations_queue.pop();
  //     if (cnt_queue >= env->num_of_agents * (1-ratio_frag)) break;
  //     int currentRow = current_location / env->cols;

  //     for (int direction : directions) {
  //       int nextLocation = current_location + direction;
  //       if (nextLocation < 0 || nextLocation >= env->rows * env->cols) continue;
  //       // if (std::get<0>(puzzle_grid[nextLocation]) != -1) continue;
  //       if (visited[nextLocation] == 1) continue;
  //       if (env->map[nextLocation] == 1) continue;

  //       lotations_queue.push(nextLocation);
  //       visited[nextLocation] = 1;
  //       if (map_id[nextLocation] != -1 && selected[nextLocation] == 0) {
  //         frag_ids.push_back(map_id[nextLocation]);
  //         selected[nextLocation] = 1;
  //         cnt_queue++;
  //       }
  //     }
  //   }
  // }
  
  int cnt = 0;
  // for (int id = env->num_of_agents * ratio_frag; id < env->num_of_agents; id++) {
  for (int id : frag_ids) {
    bool flag = true;
    for (int i = 0; i < env->num_of_agents; i++) {
      if (instance.goal_locations[i] == env->curr_states[id].location) {
        flag = false;
        break;
      }
    }
    if (flag) {
      instance.goal_locations[id] = env->curr_states[id].location;
      result[id] = 1;
      cnt++;
    }
  }
  cout << "time for goal location(cnt: " << cnt << " ): " << (std::clock() - hihi1) / CLOCKS_PER_SEC
       << endl;
  return result;
}