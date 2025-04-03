/*
 * Implementation of refiners
 *
 * references:
 * Iterative Refinement for Real-Time Multi-Robot LACAM_PREFIX_Path Planning.
 * Keisuke Okumura, Yasumasa Tamura, and Xavier Défago.
 * In Proceedings of IEEE/RSJ International Conference on Intelligent Robots and
 * Systems (IROS). 2021.
 *
 * Anytime multi-agent path finding via large neighborhood search.
 * Jiaoyang Li, Zhe Chen, Daniel Harabor, P Stuckey, and Sven Koenig.
 * In Proceedings of International Joint Conference on Artificial Intelligence
 * (IJCAI). 2021.
 */

#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "metrics.hpp"
#include "sipp.hpp"
#include "translator.hpp"
#include "utils.hpp"

LACAM_PREFIX_Solution refine(const LACAM_PREFIX_Instance *ins, const Deadline *deadline,
                const LACAM_PREFIX_Solution &solution, LACAM_PREFIX_DistTable *D, const int seed = 0,
                const int verbose = 0);
