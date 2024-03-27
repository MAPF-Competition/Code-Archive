#include"ecbs.h"

namespace cbs
{
    
std::ostream& operator<<(std::ostream& os, const Conflict& con) {
  os << "(" << ConflictType2Str(con.type) << ", t1:" << con.time1 << ", t2:" << con.time2 << ", id"
     << con.id1 << ", id" << con.id2 << ", loc1:" << con.location1
     << ", loc2:" << con.location2 << ")";
  return os;
}

  std::unordered_map<int, std::vector<int>> ECBSSolver::paths2SoftObstacles(
      const std::vector<ECBSPath>& paths, int agent_id) {
    std::unordered_map<int, std::vector<int>> ret;
    for (const auto& p : paths) {
      if (p.id == agent_id) {
        continue;
      }
      const auto& vertices = p.vertices;
      for (int i = 0; i < vertices.size(); i++) {
        ret[vertices[i].location].emplace_back(i);
      }
    }
    for (auto& it : ret) {
      std::sort(it.second.begin(), it.second.end());
      it.second.erase(std::unique(it.second.begin(), it.second.end()),
                      it.second.end());
    }
    return ret;
  }

  std::unordered_map<int, std::vector<int>> ECBSSolver::paths2SoftObstacles(
      const std::vector<std::list<State>>& paths, int agent_id) {
    std::unordered_map<int, std::vector<int>> ret;
    for (int pi = 0;pi <paths.size();pi++) {
      if (pi  == agent_id) {
        continue;
      }
      const auto& vertices = std::vector<State>{paths[pi].begin(),paths[pi].end()};
      for (int i = 0; i < vertices.size(); i++) {
        ret[vertices[i].location].emplace_back(i);
      }
    }
    for (auto& it : ret) {
      std::sort(it.second.begin(), it.second.end());
      it.second.erase(std::unique(it.second.begin(), it.second.end()),
                      it.second.end());
    }
    return ret;
  }


    void ECBSSolver::updateFocal() {
    if (open.empty()) return;
    focal = ECBSNodePtrPQ();

    double min_f_val = open.begin()->get()->f;
    for (auto it = open.begin(); it != open.end(); it++) {
      // copy some nodes to focal
      auto n = *it;
      if (n->f >= min_f_val && n->f <= min_f_val * focal_w) {
        focal.push(n);
      }
    }
  }

  // get path from Dijkstra precomputed info
  Path ECBSSolver::getPathFromPredecessors(State start, State end) {
    std::list<State> path;
    State current = end;
    const auto& pred = predecessors[start.location];
    // std::cout<< "Start: "<<start<< " End: "<<end<< " getPathFromPredecessors:
    // "<<std::endl;

    // 添加终点
    path.push_front(current);

    while (current.location != start.location ||
           current.orientation != start.orientation) {  //
      int prev = pred[current.location];  // 获取前驱节点表格
      int next_orientation;
      if (current.location ==
          start.location) {  // seems we are at start, but orientation is wrong
        prev = current.location;
        int diff = (4 + current.orientation - start.orientation) %
                   4;                  // 加4确保差值为正数
        if (diff == 1 || diff == 3) {  // adjancent
          next_orientation = start.orientation;
        } else if (diff == 2) {  // opposite
          next_orientation = (start.orientation + 1) % 4;
        } else {
#ifdef DEBUG
          throw std::runtime_error("imposibble!");
#endif
        }
      } else {
        int dx = current.location % cols - prev % cols;
        int dy = current.location / cols - prev / cols;
        if (dx == 1) {
          next_orientation = 0;  // 向东
        } else if (dx == -1) {
          next_orientation = 2;  // 向西
        } else if (dy == 1) {
          next_orientation = 1;  // 向南
        } else if (dy == -1) {
          next_orientation = 3;  // 向北
        } else {
#ifdef DEBUG
          throw std::runtime_error("invalid move");
#endif
        }
      }

      if (current.orientation !=
          next_orientation) {  // 如果当前方向与需要面向的方向不同，添加一个新的状态来表示旋转
        int diff = (4 + next_orientation - current.orientation) %
                   4;                  // 加4确保差值为正数
        if (diff == 1 || diff == 3) {  // adjancent
          path.push_front(State{current.location, -1, next_orientation});
        } else if (diff == 2) {  // opposite
          path.push_front(
              State{current.location, -1, (current.orientation + 1) % 4});
          path.push_front(State{current.location, -1, next_orientation});
        } else {
#ifdef DEBUG
          throw std::runtime_error("imposibble!");
#endif
        }
      }

      // 将前驱节点设置为下一个当前节点
      current.location = prev;
      // assert(map[current.location] == 0);
      // 设置新的方向
      current.orientation = next_orientation;
      // 将当前节点加入路径
      path.push_front(current);
    }
    int t = 0;  // setup timestep
    for (auto& s : path) {
      s.timestep = t;
      t++;
    }
    // 转换为所需的路径格式并返回
    auto ret = Path(path.begin(), path.end());

#ifdef DEBUG
    for (int i = 0; i < ret.size() - 1; i++) {
      if (ret[i].location != ret[i + 1].location &&
          ret[i].orientation != ret[i + 1].orientation) {
        throw std::runtime_error(
            "getPathFromPredecessors: location and orientation change at the "
            "same time");
      }
    }
#endif
    return ret;
  }

  void ECBSSolver::constructRootNode(
      const std::vector<std::pair<State, State>>& tasks) {
    ECBSNodePtr root = std::make_shared<ECBSNode>();
    root->paths.resize(tasks.size());
    std::mutex mutex;

    std::set<int> start_locs;
    for(auto t:tasks){
        start_locs.insert(t.first.location);
    }
#ifndef DEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < tasks.size(); i++) {
      // using the low-level()
#if SIPP_START
      auto sipp_planner =
          std::make_shared<sipp::SIPP>(map, rows, cols, costmaps);
      sipp_planner->SetHorizon(horizon);
      auto p = sipp_planner->Search(
          {tasks[i].first.location, tasks[i].first.orientation},
          {tasks[i].second.location, tasks[i].second.orientation}, {}, {}, {},
          {});
#elif DIJKSTRA_START
      auto p = getPathFromPredecessors(
          {tasks[i].first.location, -1, tasks[i].first.orientation},
          {tasks[i].second.location, -1, tasks[i].second.orientation});
#endif
      std::lock_guard<std::mutex> lock(mutex);
      root->paths[i] = ECBSPath(i, p);

#ifdef DEBUG
      if (p.empty()) {
        std::cout << "  constructRootNode path " << i << " failed, start: ("
                  << tasks[i].first.location << ","
                  << tasks[i].first.orientation << ") end: : ("
                  << tasks[i].second.location << ")" << std::endl;
        throw std::runtime_error("constructRootNode path failed");
      }
#endif
    }
    root->UpdateCost();
    open.insert(root);
    focal.push(root);
  }

  // 返回的数组下标就是对应机器人的ID
  std::vector<std::list<State>> ECBSSolver::constructPaths(
      const std::vector<ECBSPath>& sol) {
    std::vector<std::list<State>> ret;
    ret.resize(sol.size());
    
    for (auto s : sol) {
      // ret[s.id] = {s.vertices.begin(),s.vertices.end()};
      for (auto v : s.vertices) {
        // assert(map.at(v.location) == 0);
        ret[s.id].push_back(v);
      }
    }
    return ret;
  }

  void ECBSSolver::getConflict(const Path& path1, const Path& path2, int id1, int id2, std::vector<Conflict>& con) {
    for (int i = 0; i < path1.size(); ++i) {
      if(i > horizon) break;
      int loc1 = path1[i].location;
      int timestep1 = path1[i].timestep;
      for (int j = i; j < path2.size(); ++j) {
        if(j > horizon) break;
        int loc2 = path2[j].location;
        int timestep2 = path2[j].timestep;

        // 检查点冲突
        if (loc1 == loc2 && timestep1 == timestep2) {
#ifdef DEBUG
          std::cout << " vertex" << std::endl;
          std::cout << "loc1: " << loc1 << ", loc2: " << loc2 << std::endl;
          std::cout << "timestep1: " << timestep1
                    << ", timestep2: " << timestep2 << std::endl;
#endif
          con.emplace_back(timestep1,timestep2, id1, id2, loc1, loc2, C_NODE);
          continue; // this j has node conflict, no need for edge check
        }

        // 检查交换冲突
        if (i < path1.size() - 1 && j < path2.size() - 1) {
          int loc1Next = path1[i + 1].location;
          int loc2Next = path2[j + 1].location;

          if (loc1 == loc2Next && loc1Next == loc2 &&
              (timestep1 == timestep2)) {
#ifdef DEBUG
            std::cout << "loc1: " << loc1 << ", loc2: " << loc2 << std::endl;
            std::cout << "timestep1: " << timestep1
                      << ", timestep2: " << timestep2 << std::endl;
            std::cout << "loc1Next: " << loc1Next << ", loc2Next: " << loc2Next
                      << std::endl;
            std::cout << " swap" << std::endl;
#endif
            con.emplace_back(timestep1, timestep2,id1, id2, loc1, loc1Next, C_SWAP);
          }
        }
      }
    }
  }

  std::vector<Conflict> ECBSSolver::getConflicts(
      const std::vector<State>& candidate,
      const std::vector<std::list<State>>& all_paths,int agent_id) {
#ifdef DEBUG
    std::cout << "getConflicts in" << std::endl;
    std::cout << " candidate: " << candidate << std::endl;
    for (int i = 0; i < all_paths.size(); i++) {
      std::cout << i << ": "
                << std::vector<State>{all_paths[i].begin(), all_paths[i].end()};
    }
#endif
    std::vector<Conflict> all_conflicts;
    auto& pathA = candidate;
    for (int ib = 0; ib < all_paths.size(); ib++) {
      auto pathB =
          std::vector<State>{all_paths[ib].begin(), all_paths[ib].end()};
          getConflict(pathA,pathB,agent_id,ib,all_conflicts);
    }
    if (all_conflicts.empty()) {
      return {};
    } else {
      std::sort(all_conflicts.begin(), all_conflicts.end(), compareConflicts);
      return {all_conflicts.front()};
    }
  }

  // return first confict, or empty
  std::vector<Conflict> ECBSSolver::getConflicts(const ECBSNodePtr p) {
    std::vector<Conflict> all_conflicts;
#ifdef DEBUG
    std::cout << "getConflicts in" << std::endl;
    for (int i = 0; i < p->paths.size(); i++) {
      std::cout << i << ": " << p->paths[i].vertices;
    }
#endif
    for (int ia = 0; ia < p->paths.size(); ia++) {
      auto& pathA = p->paths[ia];
      for (int ib = ia + 1; ib < p->paths.size(); ib++) {
        auto& pathB = p->paths[ib];
        getConflict(pathA.vertices,pathB.vertices,pathA.id,pathB.id,all_conflicts);
      }
    }
    p->conflict_num = all_conflicts.size();
    std::sort(all_conflicts.begin(), all_conflicts.end(), compareConflicts);
    return all_conflicts;
  }

// unstable
  std::list<State> ECBSSolver::SearchOnePath(const std::pair<State, State>& task,
                                 const std::vector<std::list<State>>& all_paths,
                                 const vector<State>& cur_states, int agent_id,
                                 const int time_limit) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::list<State>> all_paths_with_cur = all_paths;
    for (int i = 0; i < all_paths_with_cur.size(); i++) {
      if (i == agent_id) {
        all_paths_with_cur[i] = {};
        continue;
      }
      all_paths_with_cur[i].push_front(cur_states[i]);
    }
    auto sipp_planner = std::make_shared<sipp::SIPP>(map, rows, cols, costmaps);
    auto soft_obs = paths2SoftObstacles(all_paths_with_cur,agent_id);
    sipp_planner->SetHorizon(horizon);
    std::vector<sipp::NodeConstraint> ncons{};
    std::vector<sipp::EdgeConstraint> econs{};
    auto p = sipp_planner->Search(
        {task.first.location, task.first.orientation},
        {task.second.location, task.second.orientation}, ncons, econs, {},soft_obs);

    auto conflicts = getConflicts(p, all_paths_with_cur,agent_id);
    while (!conflicts.empty()) {
#ifdef DEBUG
      std::cout << "ncons.size(): " << ncons.size() << std::endl;
      std::cout << "econs.size(): " << econs.size() << std::endl;
#endif

      auto current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                          current_time - start_time)
                          .count();
      if (duration >= time_limit) {
        std::cout << "SearchOnePath time limit exceeded" << std::endl;
        return {};  // 超过时间限制，提前退出函数
      }

      auto C = conflicts.front();
#ifdef DEBUG
      std::cout << "C: " << C << std::endl;
#endif
      if (C.type == C_NODE) {
        ncons.push_back(sipp::NodeConstraint(C.time1, C.location1));
      } else {  // (C.type == C_SWAP)
        econs.push_back(sipp::EdgeConstraint(C.time1, C.location1, C.location2));
      }

      p = sipp_planner->Search({task.first.location, task.first.orientation},
                               {task.second.location, task.second.orientation},
                               ncons, econs, {}, {});
#ifdef DEBUG
      std::cout << "plan path: " << p << std::endl;
      std::cout << C.id2 << "'s path: "
                << Path{all_paths_with_cur[C.id2].begin(),
                        all_paths_with_cur[C.id2].end()}
                << std::endl;
#endif
        // all_paths_with_cur[agent_id] = 
      conflicts = getConflicts(p, all_paths_with_cur,agent_id);
    };

    if (!p.empty()) {
      return {p.begin() + 1, p.end()};  // first node is current, ignore
    } else {
      return {p.begin(), p.end()};
    }
  }

  std::vector<std::list<State>> ECBSSolver::Solve(
      const std::vector<std::pair<State, State>>& tasks, int time_limit,
      const std::vector<std::pair<State, State>>& next_tasks) {
    auto start_time = std::chrono::high_resolution_clock::now();
    open.clear();
    focal = ECBSNodePtrPQ();
    std::vector<std::list<State>> ret;
    ret.resize(tasks.size());
    constructRootNode(tasks);
    // exit(1);
    while (!open.empty()) {
      auto current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                          current_time - start_time)
                          .count();
      if (duration >= time_limit) {
        std::cout << " time limit exceeded" << std::endl;
        return ret;  // 超过时间限制，提前退出函数
      }
#ifdef DEBUG
      std::cout << "ecbs open.size(): " << open.size() << std::endl;
      std::cout << "ecbs focal.size(): " << focal.size() << std::endl;
#endif
      auto cur = focal.top();
      focal.pop();
      open.erase(cur);
      auto conflicts = getConflicts(cur);  //
#ifdef DEBUG
      std::cout << "ecbs cur->conflict_num: " << cur->conflict_num << std::endl;
#endif
      if (conflicts.empty()) {
        return constructPaths(cur->paths);
      }

      auto C = conflicts.front();
#ifdef DEBUG
#endif
      std::cout << "ECBS found conflict: " << C << std::endl;

      std::cout<<" path["<<C.id1 <<"]: "<<cur->paths.at(C.id1).vertices<<std::endl;
      std::cout<<" path["<<C.id2 <<"]: "<<cur->paths.at(C.id2).vertices<<std::endl;
      std::shared_ptr<sipp::SIPP> sipp_planner[2] = {nullptr, nullptr};
      sipp_planner[0] = std::make_shared<sipp::SIPP>(map, rows, cols, costmaps);
      sipp_planner[1] = std::make_shared<sipp::SIPP>(map, rows, cols, costmaps);
      ECBSNodePtr children[2] = {nullptr, nullptr};
      Path lowp[2] = {};
      std::unordered_map<int, std::vector<int>> soft_obs[2];
      std::map<int, std::vector<sipp::NodeConstraint>> tmp_ncons[2] = {};
      std::map<int, std::vector<sipp::EdgeConstraint>> tmp_econs[2] = {};
      const int agent_id[2] = {C.id1, C.id2};
      tmp_ncons[0] = cur->ncons; //left child
      tmp_ncons[1] = cur->ncons; //right child
      tmp_econs[0] = cur->econs;
      tmp_econs[1] = cur->econs;
      if (C.type == C_NODE) {
        tmp_ncons[0][agent_id[0]].push_back(sipp::NodeConstraint(C.time1, C.location1));
        tmp_ncons[1][agent_id[1]].push_back(sipp::NodeConstraint(C.time2, C.location1));
      } else {  // (C.type == C_SWAP)
        auto econ_to_add_0 =
            sipp::EdgeConstraint(C.time1, C.location1, C.location2);
        auto econ_to_add_1 =
            sipp::EdgeConstraint(C.time2, C.location2, C.location1);
        tmp_econs[0][agent_id[0]].push_back(econ_to_add_0);
        tmp_econs[1][agent_id[1]].push_back(econ_to_add_1);
      }
#ifndef DEBUG
#pragma omp parallel for
#endif
      for (int i = 0; i < 2; i++) {
        std::cout << agent_id[i] << " low-level plan start at "<< tasks[agent_id[i]].first<< std::endl;
        soft_obs[i] = paths2SoftObstacles(cur->paths, agent_id[i]);
        sipp_planner[i]->SetHorizon(horizon);
        lowp[i] = sipp_planner[i]->Search(
            {tasks[agent_id[i]].first.location,
             tasks[agent_id[i]].first.orientation},
            {tasks[agent_id[i]].second.location,
             tasks[agent_id[i]].second.orientation},
            tmp_ncons[i][agent_id[i]], tmp_econs[i][agent_id[i]], {}, soft_obs[i]);
        if (lowp[i].empty()) {
          std::cout << agent_id[i] << " low-level plan failed" << std::endl;
        } else {

          std::cout << agent_id[i] << " low-level plan success: "<< lowp[i]<< std::endl;
          children[i] = std::make_shared<ECBSNode>();
          children[i]->paths = cur->paths;
          children[i]->econs = tmp_econs[i];
          children[i]->ncons = tmp_ncons[i];
          children[i]->parent = cur;
          children[i]->conflict_num = cur->conflict_num;
          children[i]->f = cur->f;
          children[i]->g = cur->g;
          children[i]->h = cur->h;
          children[i]->cost = cur->cost;
        }
      }

      // std::vector<ECBSNodePtr> focal_to_push;
      if (children[0]) {
        children[0]->paths[agent_id[0]] = ECBSPath(agent_id[0], lowp[0]);
        children[0]->UpdateCost();
        // auto cs = getConflicts(children[0]);
        open.insert(children[0]);
        // focal_to_push.push_back(children[0]);
      }
      if (children[1]) {
        children[1]->paths[agent_id[1]] = ECBSPath(agent_id[1], lowp[1]);
        children[1]->UpdateCost();
        // auto cs = getConflicts(children[1]);
        open.insert(children[1]);
        // focal_to_push.push_back(children[1]);
      }

      // for (auto it : focal_to_push) {
      //   if (it->conflict_num <= open.begin()->get()->f * focal_w) {
      //     focal.push(it);
      //   }
      // }
      updateFocal();
      if (focal.empty()) {
        sipp_planner[0]->dump_debug();
        sipp_planner[1]->dump_debug();
      }
    }
    std::cout << " ecbs plan failed" << std::endl;

#ifdef DEBUG
    throw std::runtime_error("ecbs plan failed");
#endif
    return ret;
  }

} // namespace cbs
