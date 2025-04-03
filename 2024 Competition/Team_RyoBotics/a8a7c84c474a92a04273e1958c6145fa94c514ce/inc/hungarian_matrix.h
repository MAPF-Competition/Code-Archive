#ifndef HUNGARIAN_MATRIX_H
#define HUNGARIAN_MATRIX_H

#include "hungarian.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

namespace Hungarian
{
    using Assignment = std::unordered_map<int, int>;

    class Matrix
    {
    public:
        Matrix(const std::vector<std::vector<int>> &costs, bool agent_surplus,
               const std::unordered_set<int> &high_cost_agents);

        void extendMatrix();
        Assignment solve();

    private:
        std::vector<std::vector<int>> cost_matrix;
        bool is_agent_surplus;
        std::unordered_set<int> high_cost_agent_indices;
        int original_size;
        int extended_size;
        int _size;

        std::vector<int> row_min;
        std::vector<int> col_min;
        std::vector<int> row_mate;
        std::vector<int> col_mate;
        std::vector<bool> row_covered;
        std::vector<bool> col_covered;
        std::vector<std::vector<bool>> star_matrix;
        std::vector<std::vector<bool>> prime_matrix;
    };

} // namespace Hungarian

#endif // HUNGARIAN_MATRIX_H