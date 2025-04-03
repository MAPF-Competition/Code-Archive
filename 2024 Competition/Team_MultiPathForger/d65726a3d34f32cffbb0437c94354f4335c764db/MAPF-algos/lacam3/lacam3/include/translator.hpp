/*
 * translate between representations by paths and configurations
 */

#pragma once
#include "graph.hpp"
#include "metrics.hpp"
#include "utils.hpp"

std::vector<LACAM_PREFIX_Path> translateConfigsToPaths(const std::vector<LACAM_PREFIX_Config> &configs);
std::vector<LACAM_PREFIX_Config> translatePathsToConfigs(const std::vector<LACAM_PREFIX_Path> &paths);
