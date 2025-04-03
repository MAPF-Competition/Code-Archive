
#pragma once
#include <vector>

namespace MyPlanner {

/**
 * \brief Solve an NxN assignment problem using the Jonker–Volgenant (JV) algorithm.
 *
 * \param cost_matrix  An NxN matrix of integer costs.
 * \return A vector of length N where element i is the assigned column for row i.
 *
 * \throws std::runtime_error if cost_matrix is not square.
 */
    std::vector<int> lap(const std::vector<std::vector<int>>& cost_matrix);
}