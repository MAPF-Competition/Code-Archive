#include "APSPCalculator.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <limits>
#include <unordered_map>
#include <utility>
#include "SchedulerUtils.h"
#include <boost/heap/d_ary_heap.hpp>

namespace SchedulerUtils
{

    std::vector<FixedNeighborList> precomputedNeighbors;
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
        std::vector<std::vector<CostType>> distances;
        distances.resize(total_states, std::vector<CostType>(total_states, std::numeric_limits<CostType>::max()));

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

                std::vector<CostType> distances_from_start = bfs(start, map, rows, cols, cellToIndex, precomputedNeighbors);

                processed_states++;
                float progress = (processed_states * 100.0f) / total_states;
                std::cout << "\r計算進捗: " << std::fixed << std::setprecision(1)
                          << progress << "% (" << processed_states << "/"
                          << total_states << ")" << std::flush;

                // 各状態から各位置への最短距離を計算
                for (int end_loc = 0; end_loc < passableCount; ++end_loc)
                {
                    CostType min_cost = std::numeric_limits<CostType>::max();
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
        std::vector<std::vector<unsigned char>> visited_tabu_states;

        // return DirectionalPathCostMap(
        //     std::move(distances),
        //     std::move(locationDistances), // 追加
        //     std::move(cellToIndex),
        //     std::move(passableCells),
        //     std::move(precomputedNeighbors),
        //     std::move(visited_tabu_states),
        //     total_states);
        return DirectionalPathCostMap();
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

    std::pair<std::vector<CostType>, std::vector<WeightedCostType>> APSPCalculator::bfsWithWeight(
        const State &start,
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex,
        const std::vector<FixedNeighborList> &precomputedNeighbors,
        const WeightTable &weightTable)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(), [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        // std::cout << "total_states at bfsWithTabu: " << total_states << std::endl;
        std::vector<CostType> distances(total_states, std::numeric_limits<CostType>::max());
        std::queue<State> queue;
        // 最短経路のweightを追跡する。startから各状態への最短経路のweightの合計値
        std::vector<WeightedCostType> path_weights;

        if (cellToIndex[start.location] == -1)
            return std::pair<std::vector<CostType>, std::vector<WeightedCostType>>(distances, path_weights);

        path_weights.resize(total_states, 0);

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
            for (const State &next : precomputedNeighbors[current_idx].neighbors)
            {
                int next_idx = cellToIndex[next.location] * 4 + next.direction;
                // unsigned char next_tabu_visits = visited_tabu_states[current_idx] + base_visited_tabu_states[next_idx];
                // std::cout << "next.location: " << next.location << std::endl;
                WeightedCostType next_weight = weightTable.getWeight(next.location, next.from_direction_to_current);
                // if (tabu_locs.find(next.location) != tabu_locs.end())
                // {
                //     ++next_tabu_visits;
                // }

                if (distances[next_idx] == std::numeric_limits<unsigned short>::max())
                {
                    path_weights[next_idx] = path_weights[current_idx] + next_weight;

                    distances[next_idx] = current_dist + 1;
                    queue.push(next);
                }
                // else if (distances[next_idx] == current_dist + 1 && path_weights[next_idx] < path_weights[current_idx] + next_weight)
                // {
                //     path_weights[next_idx] = path_weights[current_idx] + next_weight;
                //     queue.push(next);
                // }
            }
        }

        return std::pair<std::vector<CostType>, std::vector<WeightedCostType>>(distances, path_weights);
    }

    std::vector<WeightedCostType> APSPCalculator::dijkstraWithWeight(
        const State &start,
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex,
        const std::vector<FixedNeighborList> &precomputedNeighbors,
        const WeightTable &weightTable,
        const size_t passableCount)
    {
        const size_t total_states = passableCount * 4;

        // 距離(distances)を無限大で初期化
        std::vector<WeightedCostType> distances(total_states, std::numeric_limits<WeightedCostType>::max());

        // start.location が通行不可(-1)なら何もできない
        if (cellToIndex[start.location] == -1)
        {
            return distances;
        }

        // 開始状態インデックス
        size_t start_idx = cellToIndex[start.location] * 4 + start.direction;
        distances[start_idx] = 0;

        // d-ary heap を利用した優先度付きヒープに変更
        // PQItem は (コスト, 状態インデックス) のペアです
        using PQItem = std::pair<WeightedCostType, size_t>;
        using DaryHeap = boost::heap::d_ary_heap<PQItem, boost::heap::arity<4>, boost::heap::compare<std::greater<PQItem>>>;
        DaryHeap heap;
        heap.push(std::make_pair(0, start_idx));

        while (!heap.empty())
        {
            PQItem top = heap.top();
            heap.pop();
            WeightedCostType current_cost = top.first;
            size_t current_idx = top.second;

            // すでに他の経路でより良いコストが発見されていた場合はスキップ
            if (current_cost > distances[current_idx])
            {
                continue;
            }

            // current_idx から隣接する状態を走査
            for (const State &next : precomputedNeighbors[current_idx].neighbors)
            {
                size_t next_idx = cellToIndex[next.location] * 4 + next.direction;

                // next へ進むためのコスト
                WeightedCostType edge_cost = weightTable.getWeight(next.from_location, next.from_direction_to_current);

                // 新しい距離
                WeightedCostType new_cost = current_cost + edge_cost;

                // 距離更新
                if (new_cost < distances[next_idx])
                {
                    distances[next_idx] = new_cost;
                    heap.push(std::make_pair(new_cost, next_idx));
                }
            }
        }

        // デバッグ出力部分は維持
        int row1 = 12;
        int col1 = 29;
        int tmp_loc1 = row1 * cols + col1;
        int tmp_dir = 1;
        int row2 = 22;
        int col2 = 25;
        int tmp_loc2 = row2 * cols + col2;
        if (start.location == tmp_loc1 && start.direction == tmp_dir)
        {
            int tmp_idx = cellToIndex[tmp_loc2] * 4 + tmp_dir;
            std::cout << "start.location: " << start.location << std::endl;
            std::cout << "start_idx: " << start_idx << std::endl;
            std::cout << "tmp_idx: " << tmp_idx << std::endl;
            std::cout << "distances[tmp_idx]: " << distances[tmp_idx] << std::endl;
        }

        // 最終的に、distances[i] に start から状態 i までの最短コストが格納される
        return distances;
    }
    std::pair<std::vector<CostType>, std::vector<unsigned char>> APSPCalculator::bfsWithTabu(
        const State &start,
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex,
        const std::vector<FixedNeighborList> &precomputedNeighbors)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(), [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        // std::cout << "total_states at bfsWithTabu: " << total_states << std::endl;
        std::vector<CostType> distances(total_states, std::numeric_limits<CostType>::max());
        std::queue<State> queue;
        // tabu_visitsが0でない状態を追跡するマップ
        std::vector<unsigned char> visited_tabu_states_;
        std::vector<unsigned short> visited_tabu_states;

        if (cellToIndex[start.location] == -1)
            return std::pair<std::vector<CostType>, std::vector<unsigned char>>(distances, visited_tabu_states_);

        visited_tabu_states.resize(total_states, 0);
        visited_tabu_states_.resize(total_states, 0);
        std::vector<unsigned short> base_visited_tabu_states;
        // std::vector<unsigned char> base_visited_tabu_states;
        base_visited_tabu_states.resize(total_states, 0);

        int start_idx = cellToIndex[start.location] * 4 + start.direction;
        for (int tabu_loc : tabu_locs)
        {
            for (int dir = 0; dir < 4; ++dir)
            {
                int tabu_idx = cellToIndex[tabu_loc] * 4 + dir;
                base_visited_tabu_states[tabu_idx] = 1;
            }
        }
        visited_tabu_states[start_idx] = base_visited_tabu_states[start_idx];
        queue.push(start);
        distances[start_idx] = 0;

        while (!queue.empty())
        {
            State current = queue.front();
            queue.pop();
            int current_idx = cellToIndex[current.location] * 4 + current.direction;
            int current_dist = distances[current_idx];

            unsigned short current_tabu_visits = visited_tabu_states[current_idx];

            // 事前計算された隣接状態を使用
            for (const State &next : precomputedNeighbors[current_idx].neighbors)
            {
                int next_idx = cellToIndex[next.location] * 4 + next.direction;
                // unsigned char next_tabu_visits = visited_tabu_states[current_idx] + base_visited_tabu_states[next_idx];
                unsigned short next_tabu_visits = visited_tabu_states[current_idx] + base_visited_tabu_states[next_idx];
                // if (tabu_locs.find(next.location) != tabu_locs.end())
                // {
                //     ++next_tabu_visits;
                // }

                if (distances[next_idx] == std::numeric_limits<unsigned short>::max())
                {
                    visited_tabu_states[next_idx] = next_tabu_visits;

                    distances[next_idx] = current_dist + 1;
                    queue.push(next);
                }
                else if (distances[next_idx] == current_dist + 1 && visited_tabu_states[next_idx] < next_tabu_visits)
                {
                    visited_tabu_states[next_idx] = next_tabu_visits;
                    queue.push(next);
                }
            }
        }

        // visited_tabu_statesをvisited_tabu_states_にコピー（std::transformを利用）
        std::transform(visited_tabu_states.begin(), visited_tabu_states.end(), visited_tabu_states_.begin(),
                       [](unsigned short value) -> unsigned char
                       {
                           return static_cast<unsigned char>(std::min(value, static_cast<unsigned short>(std::numeric_limits<unsigned char>::max())));
                       });

        // デバッグ出力部分は維持
        int row1 = 70;
        int col1 = 97;
        int tmp_loc1 = row1 * cols + col1;
        int tmp_dir = 1;
        int row2 = 120;
        int tmp_loc2 = row2 * cols + col1;

        if (start.location == tmp_loc1 && start.direction == tmp_dir)
        {
            int tmp_idx = cellToIndex[tmp_loc2] * 4 + tmp_dir;
            std::cout << "start.location: " << start.location << std::endl;
            std::cout << "start_idx: " << start_idx << std::endl;
            std::cout << "tmp_idx: " << tmp_idx << std::endl;
            std::cout << "visited_tabu_states[tmp_idx]: " << static_cast<int>(visited_tabu_states[tmp_idx]) << std::endl;
        }

        return std::pair<std::vector<CostType>, std::vector<unsigned char>>(distances, visited_tabu_states_);
    }
    std::vector<CostType> APSPCalculator::bfs(
        const State &start,
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex,
        const std::vector<FixedNeighborList> &precomputedNeighbors)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(), [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        std::vector<CostType> distances(total_states, std::numeric_limits<CostType>::max());
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
            for (const State &next : precomputedNeighbors[current_idx].neighbors)
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

    std::vector<FixedNeighborList> APSPCalculator::precomputeNeighbors(
        const std::vector<int> &map,
        int rows,
        int cols,
        const std::vector<int> &cellToIndex)
    {
        const int passableCount = std::count_if(cellToIndex.begin(), cellToIndex.end(),
                                                [](int index)
                                                { return index != -1; });
        const int total_states = passableCount * 4;
        std::vector<FixedNeighborList> neighbors(total_states);
        const int dx[] = {0, 1, 0, -1};
        const int dy[] = {1, 0, -1, 0};
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

                int new_x = x + dx[dir];
                int new_y = y + dy[dir];

                if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols)
                {
                    int new_location = new_x * cols + new_y;
                    if (map[new_location] != 1 && cellToIndex[new_location] != -1)
                    {
                        neighbors[current_idx].neighbors[neighbors[current_idx].count++] = State(new_location, dir, i);
                    }
                }

                // 回転の追加
                neighbors[current_idx].neighbors[neighbors[current_idx].count++] = State(i, (dir + 1) % 4, i); // 右回転
                neighbors[current_idx].neighbors[neighbors[current_idx].count++] = State(i, (dir + 3) % 4, i); // 左回転
            }
        }

        return neighbors;
    }

} // namespace DefaultPlanner