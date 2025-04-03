#include "APSPCalculatorGame.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <limits>

namespace SchedulerUtilsGame
{

    std::vector<std::vector<State>> precomputedNeighbors;
    std::vector<int> cellToIndex;
    std::vector<int> passableCells;
    DirectionalPathCostMap APSPCalculator::calculateAPSP(
        const std::vector<int> &map,
        int rows,
        int cols)
    {
        // 通行可能なセルのマッピングを作成
        cellToIndex.resize(map.size(), -1);
        passableCells = getPassableCells(map, cellToIndex);

        const int passableCount = passableCells.size();
        const int total_states = passableCount * 4; // 通行可能なセルのみ * 4方向

        std::cout << "Passable cells: " << passableCount << std::endl;
        std::cout << "Total states: " << total_states << std::endl;

        // 隣接状態を事前計算
        precomputedNeighbors = precomputeNeighbors(map, rows, cols, cellToIndex);

        // distancesの初期化を修正
        std::vector<std::vector<unsigned short>> distances;
        distances.resize(total_states, std::vector<unsigned short>(total_states, std::numeric_limits<unsigned short>::max()));

        // 状態から位置への最短距離を格納する配列を追加
        std::vector<std::vector<unsigned short>> locationDistances;
        locationDistances.resize(total_states,
                                 std::vector<unsigned short>(passableCount, std::numeric_limits<unsigned short>::max()));

        int processed_states = 0;

        // 通行可能なセルのみに対してBFSを実行
        for (int i = 0; i < passableCount; ++i)
        {
            int loc = passableCells[i];
            for (int dir = 0; dir < 4; ++dir)
            {
                State start(loc, dir);
                int start_idx = i * 4 + dir; // 新しいインデックス計算

                std::vector<unsigned short> distances_from_start = bfs(start, map, rows, cols, cellToIndex, precomputedNeighbors);

                processed_states++;
                float progress = (processed_states * 100.0f) / total_states;
                std::cout << "\r計算進捗: " << std::fixed << std::setprecision(1)
                          << progress << "% (" << processed_states << "/"
                          << total_states << ")" << std::flush;

                // 各状態から各位置への最短距離を計算
                for (int end_loc = 0; end_loc < passableCount; ++end_loc)
                {
                    unsigned short min_cost = std::numeric_limits<unsigned short>::max();
                    for (int end_dir = 0; end_dir < 4; ++end_dir)
                    {
                        int end_idx = end_loc * 4 + end_dir;
                        min_cost = std::min(min_cost, distances_from_start[end_idx]);
                    }
                    locationDistances[start_idx][end_loc] = min_cost;
                }
            }
        }
        std::cout << std::endl;

        std::cout << "distances: " << distances.size() << std::endl;

        return DirectionalPathCostMap(
            std::move(distances),
            std::move(locationDistances), // 追加
            std::move(cellToIndex),
            std::move(passableCells),
            std::move(precomputedNeighbors));
    }

    std::vector<int> APSPCalculator::getPassableCells(
        const std::vector<int> &map,
        std::vector<int> &cellToIndex)
    {
        std::vector<int> passableCells;
        for (int i = 0; i < map.size(); i++)
        {
            if (map[i] == 0) // 通行可能なセル
            {
                cellToIndex[i] = passableCells.size();
                passableCells.push_back(i);
            }
        }
        return passableCells;
    }

    std::vector<unsigned short> APSPCalculator::bfs(
        const State &start,
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex,
        const std::vector<std::vector<State>> &precomputedNeighbors)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(), [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        std::vector<unsigned short> distances(total_states, std::numeric_limits<unsigned short>::max());
        std::queue<State> queue;

        if (cellToIndex[start.location] == -1)
            return distances;

        int start_idx = cellToIndex[start.location] * 4 + start.direction;
        queue.push(start);
        distances[start_idx] = 0;

        while (!queue.empty())
        {
            State current = queue.front();
            queue.pop();
            int current_idx = cellToIndex[current.location] * 4 + current.direction;
            int current_dist = distances[current_idx];

            // 事前計算された隣接状態を使用
            for (const State &next : precomputedNeighbors[current_idx])
            {
                int next_idx = cellToIndex[next.location] * 4 + next.direction;
                if (distances[next_idx] == std::numeric_limits<unsigned short>::max())
                {
                    distances[next_idx] = current_dist + 1;
                    queue.push(next);
                }
            }
        }

        return distances;
    }

    std::vector<State> APSPCalculator::getNeighbors(
        const State &current,
        const std::vector<int> &map,
        int rows,
        int cols)
    {
        std::vector<State> neighbors;

        // 前進
        // 方向は 0: 右(E), 1: 下(S), 2: 左(W), 3: 上(N)
        int dx[] = {0, 1, 0, -1}; // 行の変化（上下）
        int dy[] = {1, 0, -1, 0}; // 列の変化（左右）

        // locationから行列の位置を計算
        int x = current.location / cols; // 行（変更）
        int y = current.location % cols; // 列（変更）

        // 新しい位置を計算
        int new_x = x + dx[current.direction];
        int new_y = y + dy[current.direction];

        // 新しい位置が有効かチェック
        if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols)
        {
            int new_location = new_x * cols + new_y; // 変更
            if (map[new_location] != 1)
            {
                neighbors.emplace_back(new_location, current.direction);
            }
        }

        // 左回転
        int new_direction = (current.direction + 3) % 4;
        neighbors.emplace_back(current.location, new_direction);

        // 右回転
        new_direction = (current.direction + 1) % 4;
        neighbors.emplace_back(current.location, new_direction);

        return neighbors;
    }

    bool APSPCalculator::validateMove(
        int from,
        int to,
        const std::vector<int> &map,
        int rows,
        int cols)
    {
        if (to < 0 || to >= map.size() || map[to] == 1)
            return false;

        int from_x = from % cols;
        int from_y = from / cols;
        int to_x = to % cols;
        int to_y = to / cols;

        return std::abs(from_x - to_x) + std::abs(from_y - to_y) == 1;
    }

    std::vector<std::vector<State>> APSPCalculator::precomputeNeighbors(
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(),
                                                [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        std::vector<std::vector<State>> neighbors(total_states);

        // 各状態の隣接状態を計算
        for (int i = 0; i < map.size(); ++i)
        {
            if (cellToIndex[i] == -1)
                continue; // 通行不可能なセルはスキップ

            for (int dir = 0; dir < 4; ++dir)
            {
                State current(i, dir);
                int current_idx = cellToIndex[current.location] * 4 + current.direction;

                // 前進の計算
                int x = i / cols;
                int y = i % cols;
                int dx[] = {0, 1, 0, -1};
                int dy[] = {1, 0, -1, 0};

                int new_x = x + dx[dir];
                int new_y = y + dy[dir];

                if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols)
                {
                    int new_location = new_x * cols + new_y;
                    if (map[new_location] != 1 && cellToIndex[new_location] != -1)
                    {
                        neighbors[current_idx].emplace_back(new_location, dir);
                    }
                }

                // 回転の追加
                neighbors[current_idx].emplace_back(i, (dir + 1) % 4); // 右回転
                neighbors[current_idx].emplace_back(i, (dir + 3) % 4); // 左回転
            }
        }

        return neighbors;
    }

} // namespace DefaultPlanner