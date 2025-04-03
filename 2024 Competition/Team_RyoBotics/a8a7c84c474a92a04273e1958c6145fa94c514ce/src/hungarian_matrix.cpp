#include "hungarian_matrix.h"
#include <iostream>

namespace Hungarian
{
    Matrix::Matrix(const std::vector<std::vector<int>> &costs, bool agent_surplus,
                   const std::unordered_set<int> &high_cost_agents)
        : cost_matrix(costs), is_agent_surplus(agent_surplus),
          high_cost_agent_indices(high_cost_agents)
    {
        // 元の行数と列数を取得
        int num_rows = costs.size();
        int num_cols = costs[0].size();
        original_size = std::min(num_rows, num_cols);
        extended_size = std::max(num_rows, num_cols);
        _size = extended_size;

        // メンバー変数のサイズを調整
        row_min.resize(_size);
        col_min.resize(_size);
        row_mate.resize(_size, -1);
        col_mate.resize(_size, -1);
        row_covered.resize(_size, false);
        col_covered.resize(_size, false);
        star_matrix.resize(_size, std::vector<bool>(_size, false));
        prime_matrix.resize(_size, std::vector<bool>(_size, false));

        // コスト行列を拡張
        extendMatrix();
    }

    Assignment Matrix::solve()
    {
        try
        {
            // コスト行列から WeightedBipartiteEdge のリストを作成
            std::vector<WeightedBipartiteEdge> edges;
            for (int i = 0; i < _size; ++i)
            {
                for (int j = 0; j < _size; ++j)
                {
                    edges.push_back(WeightedBipartiteEdge(i, j, cost_matrix[i][j]));
                }
            }

            // 既存のハンガリアン法を呼び出し
            auto result = hungarianMinimumWeightPerfectMatching(_size, edges);

            // 結果が空の場合は、マッチングが見つからなかったことを示す
            if (result.empty())
            {
                return Assignment();
            }

            // 結果を Assignment 形式に変換（ダミーを除外）
            Assignment assignment;
            int original_size_row = is_agent_surplus ? extended_size : original_size;
            int original_size_col = is_agent_surplus ? original_size : extended_size;

            for (int i = 0; i < original_size_row; ++i)
            {
                if (result[i] != -1 && result[i] < original_size_col)
                {
                    assignment[i] = result[i];
                }
            }

            return assignment;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception in Matrix::solve: " << e.what() << std::endl;
            return Assignment();
        }
    }

    void Matrix::extendMatrix()
    {
        const int DUMMY_COST_AGENT = 50000; // 通常のダミーコスト
        const int DUMMY_COST_TASK = 50000;  // タスク用ダミーコスト

        // 行列を正方形に拡張
        // まず行を拡張
        cost_matrix.resize(extended_size);

        // 各行のサイズを拡張
        for (auto &row : cost_matrix)
        {
            row.resize(extended_size, 0);
        }

        if (is_agent_surplus)
        {
            // エージェント数 > タスク数 の場合
            for (int i = 0; i < extended_size; ++i)
            {
                for (int j = original_size; j < extended_size; ++j)
                {
                    if (high_cost_agent_indices.find(i) != high_cost_agent_indices.end())
                    {
                        cost_matrix[i][j] = DUMMY_COST_AGENT; // 高コストエージェント用のダミーコスト
                    }
                    else
                    {
                        cost_matrix[i][j] = DUMMY_COST_AGENT;
                    }
                }
            }
        }
        else
        {
            // エージェント数 < タスク数 の場合
            for (int i = original_size; i < extended_size; ++i)
            {
                for (int j = 0; j < extended_size; ++j)
                {
                    cost_matrix[i][j] = DUMMY_COST_TASK;
                }
            }
        }
    }

} // namespace Hungarian