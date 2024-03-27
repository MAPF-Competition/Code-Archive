#include "../inc/planner.hpp"
#include "graph.hpp"

#include <algorithm>
#include <iostream>
#include <ostream>
#include <vector>
// #define DEBUG
// #define DEADLINE

LConstraint::LConstraint()
    : who(std::vector<int>()), where(Vertices()), depth(0) {}

LConstraint::LConstraint(LConstraint *parent, int i, Vertex *v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1) {
  who.push_back(i);
  where.push_back(v);
}

LConstraint::~LConstraint(){};

Node::Node(Config _C, DistTable &D, Node *_parent)
    : C(_C), parent(_parent), pathes(C.size(), std::vector<int>()),
      conflicts(C.size(), 0), priorities(C.size(), 0), order(C.size(), 0),
      search_tree(std::queue<LConstraint *>()) {
  search_tree.push(new LConstraint());
  const auto N = C.size();

  // for (size_t i = 0; i < N; ++i) {
  //   auto path = D.get_path(i, C[i]);
  //   pathes[i] = path;
  //   // if (pathes[i].size() == 0) {
  //   //   std::cout << "1 path size 0" << i << std::endl;
  //   // }
  // }

  // calc_temporal_conflict(pathes);
  // conflicts = calc_temporal_conflict(pathes);

  // set priorities
  // for (size_t i = 0; i < N; ++i) {
  //   priorities[i] = conflicts[i] + (float)D.get(i, C[i]) / N;
  // }

  // // set priorities
  if (parent == nullptr) {
    depth = 0;
    // initialize
    for (size_t i = 0; i < N; ++i)
      priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    depth = parent->depth + 1;
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int i, int j) { return priorities[i] > priorities[j]; });
  // std::sort(order.begin(), order.end(),
  //           [&](int i, int j) { return conflicts[i] < conflicts[j]; });
}

Node::~Node() {
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance *_ins, std::mt19937 *_MT, int _verbose)
    : ins(_ins), MT(_MT), verbose(_verbose), N(ins->N), V_size(ins->G->size()),
      D(DistTable(ins)), C_next(Candidates(N, std::array<Vertex *, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)), A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr)) {}

Solution Planner::solve(int deadline, vector<int> skip_ids) {
  // info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");
  auto t_s = Time::now();
#ifdef DEADLINE
  cout << "start search in " << deadline << endl;
#endif

  // setup agents
  for (auto i = 0; i < N; ++i)
    A[i] = new Agent(i);

  // setup search queues
  std::stack<Node *> OPEN;
  std::unordered_map<Config, Node *, ConfigHasher> CLOSED;
  std::vector<LConstraint *> GC; // garbage collection of constraints

  // insert initial node
  auto S = new Node(ins->starts, D);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // depth first search
  // int loop_cnt = 0;
  std::deque<Config> solution;

  bool printed = false;
  // while (!OPEN.empty() && !is_expired(deadline)) {
  while (!OPEN.empty()) {
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() - t_s)
            .count();
    if (duration > deadline && !printed) {
      cout << "Time limit " << deadline << " reached" << endl;
      printed = true;
#ifdef DEADLINE
      // print starts and ends
      cout << "agents:" << endl;
      cout << ins->N << endl;
      for (auto i = 0; i < ins->starts.size(); ++i) {
        cout << ins->starts[i]->index << endl;
      }
      cout << endl;
      cout << "tasks:" << endl;
      cout << ins->N << endl;
      for (auto i = 0; i < ins->starts.size(); ++i) {
        cout << ins->goals[i]->index << endl;
      }
      cout << endl;

      // print starts and ends in row and col
      cout << "agents in position:" << endl;
      cout << ins->N << endl;
      for (auto i = 0; i < ins->starts.size(); ++i) {
        cout << ins->starts[i]->index % ins->G->width << ","
             << ins->starts[i]->index / ins->G->width << endl;
      }
      cout << endl;
      cout << "tasks in position:" << endl;
      cout << ins->N << endl;
      for (auto i = 0; i < ins->starts.size(); ++i) {
        cout << ins->goals[i]->index % ins->G->width << ","
             << ins->goals[i]->index / ins->G->width << endl;
      }
      cout << endl;

      return solution;
#endif
    }
    // loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();
#ifdef DEBUG
    std::cout << "OPEN " << OPEN.size() << " CLOSED " << CLOSED.size()
              << std::endl;
    // Output debug info S->C
    std::cout << "TOP S->C: ";
    for (auto i = 0; i < S->C.size(); ++i) {
      std::cout << S->C[i]->index % ins->G.width << ","
                << S->C[i]->index / ins->G.width << " ";
    }
    std::cout << std::endl;
#endif

    // check goal condition
    if (is_same_config(S->C, ins->goals) or
        (any_goal_same_config(S->C, ins->goals, skip_ids) and
         S->depth > ins->time_window)) {
      // backtrack
      while (S != nullptr) {
        solution.push_front(S->C);
        S = S->parent;
      }
      // std::reverse(solution.begin(), solution.end());
      break;
    }

    // low-level search end
    if (S->search_tree.empty()) {
      OPEN.pop();
#ifdef DEBUG
      std::cout << "TOP POP" << std::endl;
#endif
      continue;
    }

    // create successors at the low-level search
    // auto M = S->search_tree.front();
    // GC.push_back(M);
    // S->search_tree.pop();
    // if (M->depth < N) {
    //   auto i = S->order[M->depth];
    //   auto C = S->C[i]->neighbor;
    //   C.push_back(S->C[i]);
    //   // if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  //
    //   randomize for (auto u : C) S->search_tree.push(new LConstraint(M, i,
    //   u));
    // }

    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (M->depth < N) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;
      std::sort(C.begin(), C.end(), [&](Vertex *const v, Vertex *const u) {
        return D.get(i, v) < D.get(i, u);
      });
      C.insert(C.begin() + 1, S->C[i]);
      // if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
      for (auto u : C)
        S->search_tree.push(new LConstraint(M, i, u));
    }

    //     LConstraint* M;
    //     int conflict = 0;
    //     // while (conflict < 2) {
    //     // while (true) {
    //       M = S->search_tree.front();
    //       GC.push_back(M);
    //       S->search_tree.pop();
    //       if (M->depth < N) {
    //         auto i = S->order[M->depth];
    //         conflict = S->conflicts[i];
    //         auto path = S->pathes[i];
    //         std::vector<Vertex*> C;
    //         if (conflict > 0) {
    //           if (path.size() > 1) {
    //             // find next vertex of path in neighbor
    //             for (auto u : S->C[i]->neighbor) {
    //               if (u->id == path[1]) continue;
    //               C.push_back(u);
    //             }
    //             C.push_back(S->C[i]);
    //             C.push_back(ins->G.V[path[1]]);
    //           } else {
    //             C = S->C[i]->neighbor;
    //             C.push_back(S->C[i]);
    //           }
    //         } else {
    //           if (path.size() > 1) {
    //             C.push_back(ins->G.V[path[1]]);
    //           } else {
    //             C.push_back(ins->G.V[path[0]]);
    //           }
    //         }
    //         // reverse C
    //         std::reverse(C.begin(), C.end());
    // #ifdef DEBUG
    //         std::cout << "C size " << C.size() << ", i: " << i
    //                   << ", cf: " << conflict << ", C: ";
    //         for (auto i = 0; i < C.size(); ++i) {
    //           std::cout << C[i]->index % ins->G.width << ","
    //                     << C[i]->index / ins->G.width << " ";
    //         }
    //         std::cout << std::endl;
    // #endif
    //         // if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  //
    //         // randomize
    //         for (auto u : C) S->search_tree.push(new LConstraint(M, i, u));
    //         // continue;
    //       }
    //       // break;
    //     // }

#ifdef DEBUG
    std::cout << "M->who: ";
    for (auto i = 0; i < M->who.size(); ++i) {
      std::cout << M->who[i] << " ";
    }
    std::cout << " ";
    std::cout << "M->where: ";
    for (auto i = 0; i < M->where.size(); ++i) {
      std::cout << M->where[i]->index % ins->G.width << ","
                << M->where[i]->index / ins->G.width << " ";
    }
    std::cout << std::endl;
#endif

    // create successors at the high-level search
    if (!get_new_config(S, M))
      continue;

    // create new configuration
    auto C = Config(N, nullptr);
    for (auto a : A)
      C[a->id] = a->v_next;

    // check explored list
    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, D, S);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;

#ifdef DEBUG
    // Output debug info S->C
    std::cout << "New S->C: ";
    for (auto i = 0; i < S_new->C.size(); ++i) {
      std::cout << S_new->C[i]->index % ins->G.width << ","
                << S_new->C[i]->index / ins->G.width << " ";
    }
    std::cout << std::endl;
#endif
  }

  // info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
  //      solution.empty() ? (OPEN.empty() ? "no solution" : "failed")
  //                       : "solution found",
  //      "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());
  // memory management
  for (auto a : A)
    delete a;
  for (auto M : GC)
    delete M;
  for (auto p : CLOSED)
    delete p.second;

  return solution;
}

bool Planner::get_new_config(Node *S, LConstraint *M) {
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];       // agent
    const auto l = M->where[k]->id; // loc

    // check vertex collision
    if (occupied_next[l] != nullptr)
      return false;
    // check swap collision
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a))
      return false; // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent *ai) {
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT); // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex *const v, Vertex *const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr)
      continue;

    auto &ak = occupied_now[u->id];

    // avoid swap conflicts with constraints
    if (ak != nullptr && ak->v_next == ai->v_now)
      continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now)
      return true;

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT(ak))
      continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Solution solve(const Instance &ins, const int verbose, std::mt19937 *MT,
               int deadline, vector<int> skip_ids) {
  auto planner = Planner(&ins, MT, verbose);
  return planner.solve(deadline, skip_ids);
}
