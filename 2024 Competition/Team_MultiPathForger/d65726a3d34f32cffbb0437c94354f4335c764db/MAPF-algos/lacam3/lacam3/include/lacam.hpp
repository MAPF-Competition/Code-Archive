#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"

LACAM_PREFIX_Solution solve(const LACAM_PREFIX_Instance &ins, const int verbose = 0,
               const Deadline *deadline = nullptr, int seed = 0);
