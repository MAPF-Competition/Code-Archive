#include "LNS.h"

#include <queue>
#include <random>

LNS::LNS(const Instance& instance, double time_limit, const string& init_algo_name,
         const string& replan_algo_name, const string& destory_name, int neighbor_size,
         int num_of_iterations, bool use_init_lns, const string& init_destory_name, bool use_sipp,
         int screen)
    : BasicLNS(instance, time_limit, neighbor_size, screen),
      init_algo_name(init_algo_name),
      replan_algo_name(replan_algo_name),
      num_of_iterations(num_of_iterations),
      use_init_lns(use_init_lns),
      init_destory_name(init_destory_name),
      path_table(instance.window, instance.getDefaultNumberOfAgents(), instance.map_size) {
  start_time = Time::now();
  replan_time_limit = time_limit / 100;
  if (destory_name == "Adaptive") {
    ALNS = true;
    destroy_weights.assign(DESTORY_COUNT, 1);
    decay_factor = 0.1;
    reaction_factor = 0.1;
  } else if (destory_name == "RandomWalk")
    destroy_strategy = RANDOMWALK;
  else if (destory_name == "Intersection")
    destroy_strategy = INTERSECTION;
  else if (destory_name == "Random")
    destroy_strategy = RANDOMAGENTS;
  else {
    cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
    exit(-1);
  }

  preprocessing_time = ((fsec)(Time::now() - start_time)).count();
  if (screen >= 2) cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
}

void LNS::initialize() {
  if (agents.empty()) {
    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++) agents.emplace_back(instance, i, true);
  } else {
    for (auto& agent : agents) {
      agent.path.clear();
      agent.path_planner->start_state = instance.start_states[agent.id];
      agent.path_planner->goal_location = instance.goal_locations[agent.id];
      // if (agent.path_planner->goal_location == agent.path_planner->start_state.location) {
      //   for (int i = 0; i < 16; i++)
      //     agent.path.emplace_back(agent.path_planner->start_state);
      // }
    }
  }
}

bool LNS::run(double& running_time) {
  // only for statistic analysis, and thus is not included in runtime
  sum_of_distances = 0;
  for (const auto& agent : agents) {
    sum_of_distances += agent.path_planner->my_heuristic[agent.path_planner->start_state.location];
  }

  initial_solution_runtime = 0;
  start_time = Time::now();
  bool succ = getInitialSolution();
  initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
  if (!succ && initial_solution_runtime < time_limit) {
    if (use_init_lns) {
      init_lns = new InitLNS(instance, agents, time_limit - initial_solution_runtime,
                             replan_algo_name, init_destory_name, neighbor_size, screen);
      succ = init_lns->run();
      if (succ)  // accept new paths
      {
        path_table.reset();
        for (const auto& agent : agents) {
          path_table.insertPath(agent.id, agent.path);
        }
        init_lns->clear();
        initial_sum_of_costs = init_lns->sum_of_costs;
        sum_of_costs = initial_sum_of_costs;
      }
      initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
    } else  // use random restart
    {
      while (!succ && initial_solution_runtime < time_limit) {
        succ = getInitialSolution();
        initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
        restart_times++;
      }
    }
  }

  iteration_stats.emplace_back(neighbor.agents.size(), initial_sum_of_costs,
                               initial_solution_runtime, init_algo_name);
  runtime = initial_solution_runtime;
  if (succ) {
    if (screen >= 1)
      cout << "Initial solution cost = " << initial_sum_of_costs << ", "
           << "runtime = " << initial_solution_runtime << endl;
  } else {
    cout << "Failed to find an initial solution in " << runtime << " seconds after  "
         << restart_times << " restarts" << endl;
    return false;  // terminate because no initial solution is found
  }

  while (runtime < time_limit && iteration_stats.size() <= num_of_iterations) {
    runtime = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 1) validateSolution();
    if (ALNS) chooseDestroyHeuristicbyALNS();

    switch (destroy_strategy) {
      case RANDOMWALK:
        succ = generateNeighborByRandomWalk();
        break;
      case INTERSECTION:
        succ = generateNeighborByIntersection();
        break;
      case RANDOMAGENTS:
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
        if (neighbor.agents.size() > neighbor_size) {
          std::shuffle(neighbor.agents.begin(), neighbor.agents.end(),
                       std::mt19937(std::random_device()()));
          neighbor.agents.resize(neighbor_size);
        }
        succ = true;
        break;
      default:
        cerr << "Wrong neighbor generation strategy" << endl;
        exit(-1);
    }
    if (!succ) continue;

    // store the neighbor information
    neighbor.old_paths.resize(neighbor.agents.size());
    neighbor.old_sum_of_costs = 0;
    for (int i = 0; i < (int)neighbor.agents.size(); i++) {
      if (replan_algo_name == "PP") neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
      path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
      neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
    }

    if (replan_algo_name == "PP")
      succ = runPP();
    else {
      cerr << "Wrong replanning strategy" << endl;
      exit(-1);
    }

    if (ALNS)  // update destroy heuristics
    {
      if (neighbor.old_sum_of_costs > neighbor.sum_of_costs)
        destroy_weights[selected_neighbor] =
            reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) /
                neighbor.agents.size() +
            (1 - reaction_factor) * destroy_weights[selected_neighbor];
      else
        destroy_weights[selected_neighbor] =
            (1 - decay_factor) * destroy_weights[selected_neighbor];
    }
    runtime = ((fsec)(Time::now() - start_time)).count();
    sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
    if (screen >= 1)
      cout << "Iteration " << iteration_stats.size() << ", "
           << "group size = " << neighbor.agents.size() << ", "
           << "solution cost = " << sum_of_costs << ", "
           << "remaining time = " << time_limit - runtime << endl;
    iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name);
  }

  average_group_size = -iteration_stats.front().num_of_agents;
  for (const auto& data : iteration_stats) average_group_size += data.num_of_agents;
  if (average_group_size > 0) average_group_size /= (double)(iteration_stats.size() - 1);

  cout << getSolverName() << ": "
       << "runtime = " << runtime << ", "
       << "iterations = " << iteration_stats.size() << ", "
       << "solution cost = " << sum_of_costs << ", "
       << "initial solution cost = " << initial_sum_of_costs << ", "
       << "failed iterations = " << num_of_failures << endl;
  running_time = runtime;
  return true;
}

bool LNS::getInitialSolution() {
  neighbor.agents.resize(agents.size());
  for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
  neighbor.old_sum_of_costs = MAX_COST;
  neighbor.sum_of_costs = 0;
  bool succ = false;
  if (init_algo_name == "PP")
    succ = runPP();
  else {
    cerr << "Initial MAPF solver " << init_algo_name << " does not exist!" << endl;
    exit(-1);
  }
  if (succ) {
    initial_sum_of_costs = neighbor.sum_of_costs;
    sum_of_costs = neighbor.sum_of_costs;
    return true;
  } else {
    return false;
  }
}

bool LNS::runPP() {
  auto shuffled_agents = neighbor.agents;

  // for (int i = 0; i < shuffled_agents.size(); i++) {
  //   cout << shuffled_agents[i] << " " << priority.around_num[shuffled_agents[i]] << " // ";
  //   // cout << priority[i] << " "<< around_num[priority[i]] << " // ";
  // }
  int random = rand() % 3;
  std::shuffle(shuffled_agents.begin(), shuffled_agents.end(),
               std::mt19937(std::random_device()()));
  if (screen >= 2) {
    for (auto id : shuffled_agents)
      cout << id << "("
           << agents[id].path_planner->my_heuristic[agents[id].path_planner->start_state.location]
           << "->" << agents[id].path.size() - 1 << "), ";
    cout << endl;
  }
  int remaining_agents = (int)shuffled_agents.size();
  auto p = shuffled_agents.begin();
  neighbor.sum_of_costs = 0;
  runtime = ((fsec)(Time::now() - start_time)).count();
  double T = time_limit - runtime;  // time limit
  if (!iteration_stats.empty())     // replan
    T = min(T, replan_time_limit);
  auto time = Time::now();
  ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, &path_table);
  while (p != shuffled_agents.end() && ((fsec)(Time::now() - time)).count() < T) {
    int id = *p;
    if (screen >= 3)
      cout << "Remaining agents = " << remaining_agents
           << ", remaining time = " << T - ((fsec)(Time::now() - time)).count() << " seconds. "
           << endl
           << "Agent " << agents[id].id << endl;
    agents[id].path = agents[id].path_planner->findPath(constraint_table);
    if (agents[id].path.empty()) break;
    neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
    if (neighbor.sum_of_costs > neighbor.old_sum_of_costs) break;
    remaining_agents--;
    path_table.insertPath(agents[id].id, agents[id].path);
    ++p;
  }
  if (remaining_agents == 0 &&
      neighbor.sum_of_costs <= neighbor.old_sum_of_costs)  // accept new paths
  {
    return true;
  } else  // stick to old paths
  {
    if (p != shuffled_agents.end()) num_of_failures++;
    auto p2 = shuffled_agents.begin();
    while (p2 != p) {
      int a = *p2;
      path_table.deletePath(agents[a].id, agents[a].path);
      ++p2;
    }
    if (!neighbor.old_paths.empty()) {
      p2 = neighbor.agents.begin();
      for (int i = 0; i < (int)neighbor.agents.size(); i++) {
        int a = *p2;
        agents[a].path = neighbor.old_paths[i];
        path_table.insertPath(agents[a].id, agents[a].path);
        ++p2;
      }
      neighbor.sum_of_costs = neighbor.old_sum_of_costs;
    }
    return false;
  }
}

void LNS::chooseDestroyHeuristicbyALNS() {
  rouletteWheel();
  switch (selected_neighbor) {
    case 0:
      destroy_strategy = RANDOMWALK;
      break;
    case 1:
      destroy_strategy = INTERSECTION;
      break;
    case 2:
      destroy_strategy = RANDOMAGENTS;
      break;
    default:
      cerr << "ERROR" << endl;
      exit(-1);
  }
}

bool LNS::generateNeighborByIntersection() {
  if (intersections.empty()) {
    for (int i = 0; i < instance.map_size; i++) {
      if (!instance.isObstacle(i) && instance.getDegree(i) > 2) intersections.push_back(i);
    }
  }

  set<int> neighbors_set;
  auto pt = intersections.begin();
  std::advance(pt, rand() % intersections.size());
  int location = *pt;
  path_table.get_agents(neighbors_set, neighbor_size, location);
  if (neighbors_set.size() < neighbor_size) {
    set<int> closed;
    closed.insert(location);
    std::queue<int> open;
    open.push(location);
    while (!open.empty() && (int)neighbors_set.size() < neighbor_size) {
      int curr = open.front();
      open.pop();
      for (auto next : instance.getNeighbors(curr)) {
        if (closed.count(next) > 0) continue;
        open.push(next);
        closed.insert(next);
        if (instance.getDegree(next) >= 3) {
          path_table.get_agents(neighbors_set, neighbor_size, next);
          if ((int)neighbors_set.size() == neighbor_size) break;
        }
      }
    }
  }
  neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
  if (neighbor.agents.size() > neighbor_size) {
    std::shuffle(neighbor.agents.begin(), neighbor.agents.end(),
                 std::mt19937(std::random_device()()));
    neighbor.agents.resize(neighbor_size);
  }
  if (screen >= 2)
    cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location
         << endl;
  return true;
}

bool LNS::generateNeighborByRandomWalk() {
  if (neighbor_size >= (int)agents.size()) {
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
    return true;
  }

  int a = findMostDelayedAgent();
  if (a < 0) return false;

  set<int> neighbors_set;
  neighbors_set.insert(a);
  randomWalk(a, agents[a].path[0].location, 0, neighbors_set, neighbor_size,
             (int)agents[a].path.size() - 1);
  int count = 0;
  while (neighbors_set.size() < neighbor_size && count < 10) {
    int t = rand() % agents[a].path.size();
    randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size,
               (int)agents[a].path.size() - 1);
    count++;
    // select the next agent randomly
    int idx = rand() % neighbors_set.size();
    int i = 0;
    for (auto n : neighbors_set) {
      if (i == idx) {
        a = i;
        break;
      }
      i++;
    }
  }
  if (neighbors_set.size() < 2) return false;
  neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
  if (screen >= 2)
    cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
         << "("
         << agents[a].path_planner->my_heuristic[agents[a].path_planner->start_state.location]
         << "->" << agents[a].path.size() - 1 << ")" << endl;

  return true;
}

int LNS::findMostDelayedAgent() {
  int a = -1;
  int max_delays = -1;
  for (int i = 0; i < agents.size(); i++) {
    if (tabu_list.find(i) != tabu_list.end()) continue;
    int delays = agents[i].getNumOfDelays();
    if (max_delays < delays) {
      a = i;
      max_delays = delays;
    }
  }
  if (max_delays == 0) {
    tabu_list.clear();
    return -1;
  }
  tabu_list.insert(a);
  if (tabu_list.size() == agents.size()) tabu_list.clear();
  return a;
}

int LNS::findRandomAgent() const {
  int a = 0;
  int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
  int sum = 0;
  for (; a < (int)agents.size(); a++) {
    sum += agents[a].getNumOfDelays();
    if (sum >= pt) break;
  }
  assert(sum >= pt);
  return a;
}

// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size
// agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound) {
  int loc = start_location;
  for (int t = start_timestep; t < upperbound; t++) {
    auto next_locs = instance.getNeighbors(loc);
    next_locs.push_back(loc);
    while (!next_locs.empty()) {
      int step = rand() % next_locs.size();
      auto it = next_locs.begin();
      advance(it, step);
      int next_h_val = agents[agent_id].path_planner->my_heuristic[*it];
      if (t + 1 + next_h_val < upperbound)  // move to this location
      {
        path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
        loc = *it;
        break;
      }
      next_locs.erase(it);
    }
    if (next_locs.empty() || conflicting_agents.size() >= neighbor_size) break;
  }
}

void LNS::validateSolution() const {
  int sum = 0;
  for (const auto& a1_ : agents) {
    if (a1_.path.empty()) {
      cerr << "No solution for agent " << a1_.id << endl;
      exit(-1);
    } else if (a1_.path_planner->start_state.location != a1_.path.front().location) {
      cerr << "The path of agent " << a1_.id << " starts from location "
           << a1_.path.front().location << ", which is different from its start location "
           << a1_.path_planner->start_state.location << endl;
      exit(-1);
    } else if (a1_.path_planner->goal_location != a1_.path.back().location) {
      cerr << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
           << ", which is different from its goal location " << a1_.path_planner->goal_location
           << endl;
      exit(-1);
    }
    for (int t = 1; t < (int)a1_.path.size(); t++) {
      if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location)) {
        cerr << "The path of agent " << a1_.id << " jump from " << a1_.path[t - 1].location
             << " to " << a1_.path[t].location << " between timesteps " << t - 1 << " and " << t
             << endl;
        exit(-1);
      }
    }
    sum += (int)a1_.path.size() - 1;
    for (const auto& a2_ : agents) {
      if (a1_.id >= a2_.id || a2_.path.empty()) continue;
      const auto& a1 = a1_.path.size() <= a2_.path.size() ? a1_ : a2_;
      const auto& a2 = a1_.path.size() <= a2_.path.size() ? a2_ : a1_;
      int t = 1;
      for (; t < min((int)a1.path.size(), instance.window); t++) {
        if (a1.path[t].location == a2.path[t].location)  // vertex conflict
        {
          cerr << "Find a vertex conflict between agents " << a1.id << " and " << a2.id
               << " at location " << a1.path[t].location << " at timestep " << t << endl;
          exit(-1);
        } else if (a1.path[t].location == a2.path[t - 1].location &&
                   a1.path[t - 1].location == a2.path[t].location)  // edge conflict
        {
          cerr << "Find an edge conflict between agents " << a1.id << " and " << a2.id
               << " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location
               << ") at timestep " << t << endl;
          exit(-1);
        }
      }
      int target = a1.path.back().location;
      for (; t < min((int)a2.path.size(), instance.window); t++) {
        if (a2.path[t].location == target)  // target conflict
        {
          cerr << "Find a target conflict where agent " << a2.id << " (of length "
               << a2.path.size() - 1 << ") traverses agent " << a1.id << " (of length "
               << a1.path.size() - 1 << ")'s target location " << target << " at timestep " << t
               << endl;
          exit(-1);
        }
      }
    }
  }
  if (sum_of_costs != sum) {
    cerr << "The computed sum of costs " << sum_of_costs
         << " is different from the sum of the paths in the solution " << sum << endl;
    exit(-1);
  }
}

void LNS::writeIterStatsToFile(const string& file_name) const {
  if (init_lns != nullptr) {
    init_lns->writeIterStatsToFile(file_name + "-initLNS.csv");
  }
  if (iteration_stats.size() <= 1) return;
  string name = file_name;
  if (use_init_lns or num_of_iterations > 0)
    name += "-LNS.csv";
  else
    name += "-" + init_algo_name + ".csv";
  std::ofstream output;
  output.open(name);
  // header
  output << "num of agents,"
         << "sum of costs,"
         << "runtime,"
         << "cost lowerbound,"
         << "sum of distances,"
         << "MAPF algorithm" << endl;

  for (const auto& data : iteration_stats) {
    output << data.num_of_agents << "," << data.sum_of_costs << "," << data.runtime << ","
           << max(sum_of_costs_lowerbound, sum_of_distances) << "," << sum_of_distances << ","
           << data.algorithm << endl;
  }
  output.close();
}

void LNS::writeResultToFile(const string& file_name) const {
  if (init_lns != nullptr) {
    init_lns->writeResultToFile(file_name + "-initLNS.csv", sum_of_distances, preprocessing_time);
  }
  string name = file_name;
  if (use_init_lns or num_of_iterations > 0)
    name += "-LNS.csv";
  else
    name += "-" + init_algo_name + ".csv";
  std::ifstream infile(name);
  bool exist = infile.good();
  infile.close();
  if (!exist) {
    ofstream addHeads(name);
    addHeads << "runtime,solution cost,initial solution cost,lower bound,sum of distance,"
             << "iterations,"
             << "group size,"
             << "runtime of initial solution,restart times,area under curve,"
             << "LL expanded nodes,LL generated,LL reopened,LL runs,"
             << "preprocessing runtime,solver name,instance name" << endl;
    addHeads.close();
  }
  uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0, num_LL_runs = 0;
  for (auto& agent : agents) {
    agent.path_planner->reset();
    num_LL_expanded += agent.path_planner->accumulated_num_expanded;
    num_LL_generated += agent.path_planner->accumulated_num_generated;
    num_LL_reopened += agent.path_planner->accumulated_num_reopened;
    num_LL_runs += agent.path_planner->num_runs;
  }
  double auc = 0;
  if (!iteration_stats.empty()) {
    auto prev = iteration_stats.begin();
    auto curr = prev;
    ++curr;
    while (curr != iteration_stats.end() && curr->runtime < time_limit) {
      auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
      prev = curr;
      ++curr;
    }
    auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
  }
  ofstream stats(name, std::ios::app);
  stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << ","
        << max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << ","
        << iteration_stats.size() << "," << average_group_size << "," << initial_solution_runtime
        << "," << restart_times << "," << auc << "," << num_LL_expanded << "," << num_LL_generated
        << "," << num_LL_reopened << "," << num_LL_runs << "," << preprocessing_time << ","
        << getSolverName() << "," << instance.getInstanceName() << endl;
  stats.close();
}

void LNS::writePathsToFile(const string& file_name) const {
  std::ofstream output;
  output.open(file_name);
  // header
  // output << agents.size() << endl;

  for (const auto& agent : agents) {
    output << "Agent " << agent.id << ":";
    for (const auto& state : agent.path)
      output << "(" << instance.getRowCoordinate(state.location) << ","
             << instance.getColCoordinate(state.location) << "," << state.orientation << ")->";
    output << endl;
  }
  output.close();
}
