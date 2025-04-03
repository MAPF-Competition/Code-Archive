#include "ParallelAPSPCalculatorGame.h"
#include <iostream>
#include <iomanip>
#include <mutex>

namespace SchedulerUtilsGame
{
    DirectionalPathCostMap ParallelAPSPCalculatorGame::calculateParallelAPSP(
        const std::vector<int> &map,
        int rows,
        int cols,
        int num_threads)
    {
        // 通行可能なセルのマッピングを作成
        std::vector<int> cellToIndex(map.size(), -1);
        std::vector<int> passableCells = getPassableCells(map, cellToIndex);

        const int passableCount = passableCells.size();
        const int total_states = passableCount * 4;

        std::cout << "Passable cells: " << passableCount << std::endl;
        std::cout << "Total states: " << total_states << std::endl;
        std::cout << "Using " << num_threads << " threads" << std::endl;

        // 隣接状態を事前計算（共有リソース）
        auto precomputedNeighbors = precomputeNeighbors(map, rows, cols, cellToIndex);

        // distancesの初期化（三角行列のみ）
        std::vector<std::vector<unsigned short>> distances;
        distances.resize(total_states);
        for (int i = 0; i < total_states; ++i)
        {
            distances[i].resize(total_states - i, std::numeric_limits<unsigned short>::max());
        }

        // 状態から位置への最短距離を格納する配列を追加
        std::vector<std::vector<unsigned short>> locationDistances;
        locationDistances.resize(total_states,
                                 std::vector<unsigned short>(passableCount, std::numeric_limits<unsigned short>::max()));

        // スレッドプールを作成
        APSPThreadPool pool(num_threads);
        std::vector<std::future<std::pair<std::vector<unsigned short>, std::vector<unsigned short>>>> futures;

        // 進捗表示用の変数
        std::atomic<int> processed_states{0};
        std::mutex cout_mutex;

        // 各状態からのBFSをスレッドプールに投入
        for (int i = 0; i < passableCount; ++i)
        {
            int loc = passableCells[i];
            for (int dir = 0; dir < 4; ++dir)
            {
                State start(loc, dir);
                int start_idx = i * 4 + dir;

                futures.push_back(
                    pool.enqueue([&, start, start_idx]()
                                 {
                        auto distances_from_start = bfs(start, map, rows, cols, cellToIndex, precomputedNeighbors);
                        
                        // 各状態から各位置への最短距離を計算（スレッド内で実行）
                        std::vector<unsigned short> location_distances(passableCount, std::numeric_limits<unsigned short>::max());
                        for (int end_loc = 0; end_loc < passableCount; ++end_loc)
                        {
                            unsigned short min_cost = std::numeric_limits<unsigned short>::max();
                            for (int end_dir = 0; end_dir < 4; ++end_dir)
                            {
                                int end_idx = end_loc * 4 + end_dir;
                                min_cost = std::min(min_cost, distances_from_start[end_idx]);
                            }
                            location_distances[end_loc] = min_cost;
                        }
                        
                        // 進捗表示
                        int current_processed = ++processed_states;
                        {
                            std::lock_guard<std::mutex> lock(cout_mutex);
                            float progress = (current_processed * 100.0f) / total_states;
                            std::cout << "\r計算進捗: " << std::fixed << std::setprecision(1)
                                     << progress << "% (" << current_processed << "/"
                                     << total_states << ")" << std::flush;
                        }

                        return std::make_pair(distances_from_start, location_distances); }));
            }
        }

        // 結果の収集（三角行列の形式で保存）
        for (int start_idx = 0; start_idx < total_states; ++start_idx)
        {
            auto [distances_from_start, location_distances] = futures[start_idx].get();
            for (int end_idx = start_idx; end_idx < total_states; ++end_idx)
            {
                distances[start_idx][end_idx - start_idx] = distances_from_start[end_idx];
            }
            // 位置への最短距離を保存
            for (int end_loc = 0; end_loc < passableCount; ++end_loc)
            {
                locationDistances[start_idx][end_loc] = location_distances[end_loc];
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
}