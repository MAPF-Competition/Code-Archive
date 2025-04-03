/*
 * solution evaluation metrics
 */

#pragma once

#include "dist_table.hpp"
#include "instance.hpp"
#include "utils.hpp"

int get_makespan(const LACAM_PREFIX_Solution &solution);
int get_makespan_paths(const std::vector<LACAM_PREFIX_Path> &solution);

int get_path_cost(const LACAM_PREFIX_Solution &solution, int i);  // single-agent path cost
int get_path_cost(const LACAM_PREFIX_Path &path);
int get_sum_of_costs(const LACAM_PREFIX_Solution &solution);
int get_sum_of_costs_paths(const std::vector<LACAM_PREFIX_Path> &solution);

int get_path_loss(const LACAM_PREFIX_Path &path);
int get_sum_of_loss(const LACAM_PREFIX_Solution &solution);
int get_sum_of_loss(const LACAM_PREFIX_Solution &solution, std::vector<int> &agents_subset);
int get_sum_of_loss_paths(const std::vector<LACAM_PREFIX_Path> &solution);

int get_makespan_lower_bound(const LACAM_PREFIX_Instance &ins, LACAM_PREFIX_DistTable &D);
int get_sum_of_costs_lower_bound(const LACAM_PREFIX_Instance &ins, LACAM_PREFIX_DistTable &D);
