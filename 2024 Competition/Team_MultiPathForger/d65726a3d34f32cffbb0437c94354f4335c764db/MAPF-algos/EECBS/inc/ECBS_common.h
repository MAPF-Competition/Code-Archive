#pragma once
#include <tuple>
#include <map>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::map;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;

// #define NDEBUG 

#define EECBS_MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
	int location = -1;
	// bool single = false;
  // int mdd_width;

  // bool is_single() const {
  //  return mdd_width == 1;
  //}
	PathEntry(int loc = -1) { location = loc; }
};

typedef vector<PathEntry> EECBS_PREFIX_Path;
std::ostream& operator<<(std::ostream& os, const EECBS_PREFIX_Path& path);

bool isSamePath(const EECBS_PREFIX_Path& p1, const EECBS_PREFIX_Path& p2);


