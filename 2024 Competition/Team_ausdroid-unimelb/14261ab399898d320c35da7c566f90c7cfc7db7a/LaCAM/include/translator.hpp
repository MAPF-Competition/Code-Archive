/*
 * translate between representations by paths and configurations
 */

#pragma once
#include "graph.hpp"
#include "metrics.hpp"
#include "utils.hpp"

std::vector<_Path> translateConfigsToPaths(const std::vector<Config> &configs);
std::vector<Config> translatePathsToConfigs(const std::vector<_Path> &paths);
