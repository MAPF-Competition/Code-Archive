//
// Created by joonyeol on 23. 11. 13.
//

#ifndef LIFELONG_CUSTOMCOMMON_H
#define LIFELONG_CUSTOMCOMMON_H

#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::fixed
#include <list>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "States.h"

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::pairing_heap;
using std::cerr;
using std::clock;
using std::cout;
using std::endl;
using std::get;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::ofstream;
using std::pair;
using std::set;
using std::shared_ptr;
using std::string;
using std::tie;
using std::tuple;
using std::vector;
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

// std::ostream& operator<<(std::ostream&os, const Path&path);
//
// bool isSamePath(const Path&p1, const Path&p2);

struct IterationStats {
  int sum_of_costs;
  double runtime;
  int num_of_agents;
  string algorithm;
  int sum_of_costs_lowerbound;
  int num_of_colliding_pairs;

  IterationStats(int num_of_agents, int sum_of_costs, double runtime, const string& algorithm,
                 int sum_of_costs_lowerbound = 0, int num_of_colliding_pairs = 0)
      : num_of_agents(num_of_agents),
        sum_of_costs(sum_of_costs),
        runtime(runtime),
        sum_of_costs_lowerbound(sum_of_costs_lowerbound),
        algorithm(algorithm),
        num_of_colliding_pairs(num_of_colliding_pairs) {}
};

#endif  // LIFELONG_CUSTOMCOMMON_H
