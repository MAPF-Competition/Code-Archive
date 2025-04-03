#include "ParallelAPSPCalculator.h"
#include <iostream>
#include <iomanip>
#include <mutex>

namespace SchedulerUtils
{

    DirectionalPathCostMap ParallelAPSPCalculator::calculateParallelAPSP(
        SharedEnvironment *env,
        int num_threads, bool include_tabu_cost)
    {
        const std::vector<int> &map = env->map;
        int rows = env->rows;
        int cols = env->cols;

        std::string file_storage_path = env->file_storage_path;
        // weights_pathをenv->map_nameから対応するものに設定
        std::string weights_path = "";
        if (env->map_name == "brc202d.map")
        {
            weights_path = file_storage_path + "/brc202d_weight_002.w";
        }
        else if (env->map_name == "Paris_1_256.map")
        {
            weights_path = file_storage_path + "/paris_weight_014.w";
        }
        else if (env->map_name == "warehouse_large.map")
        {
            weights_path = file_storage_path + "/warehouse_large_weight_008.w";
        }
        else if (env->map_name == "sortation_large.map")
        {
            weights_path = file_storage_path + "/sortation_large_weight_008.w";
        }
        else if (env->map_name == "random-32-32-20.map")
        {
            if (env->num_of_agents == 100)
            {
                weights_path = "";
            }
            else if (env->num_of_agents == 200)
            {
                weights_path = file_storage_path + "/random_weight_101.w";
            }
            else if (env->num_of_agents == 300)
            {
                weights_path = file_storage_path + "/random_weight_40.w";
            }
            else if (env->num_of_agents == 600)
            {
                weights_path = file_storage_path + "/random_weight_50.w";
            }
            else if (env->num_of_agents == 800)
            {
                weights_path = file_storage_path + "/random_800_uncompressed_weights_with_wait_5000_steps.w";
            }
        }
        WeightTable weightTable(rows, cols);
        if (!weights_path.empty())
        {
            weights_path = weightTable.loadWeights(weights_path);
            std::cout << "weights_path: " << weights_path << std::endl;
        }
        // weightTable.loadWeights(weights_path);
        // 通行可能なセルのマッピングを作成
        std::vector<int> cellToIndex(map.size(), -1);
        std::vector<int> passableCells = getPassableCells(map, cellToIndex);

        const size_t passableCount = passableCells.size();
        const size_t total_states = passableCount * 4;
        const size_t totalElementsLocToLoc = passableCount * passableCount;

        std::cout << "Passable cells: " << passableCount << std::endl;
        std::cout << "Total states: " << total_states << std::endl;
        std::cout << "Total elements: " << totalElementsLocToLoc << std::endl;
        std::cout << "Using " << num_threads << " threads" << std::endl;

        // 隣接状態を事前計算（共有リソース）
        auto precomputedNeighbors = precomputeNeighbors(map, rows, cols, cellToIndex);

        // distancesの初期化（三角行列のみ）
        // std::vector<WeightedCostType> distances(totalElements, std::numeric_limits<WeightedCostType>::max());
        // 状態から位置への最短距離を格納する配列を追加
        std::vector<WeightedCostType> stateToLocationDist(total_states * passableCount, std::numeric_limits<WeightedCostType>::max());
        std::vector<WeightedCostType> locationToLocationDist(totalElementsLocToLoc,
                                                             std::numeric_limits<WeightedCostType>::max());

        // スレッドプールを作成
        APSPThreadPool pool(num_threads);
        std::vector<std::future<std::vector<WeightedCostType>>> futures;

        // 進捗表示用の変数
        std::atomic<int> processed_states{0};
        std::mutex cout_mutex;
        std::cout << "start dijkstra" << std::endl;
        // 各状態からのdijkstraをスレッドプールに投入
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
                                    std::vector<WeightedCostType> distances_from_start;
                                    // std::tie(distances_from_start, visited_tabu_states_from_start) = bfsWithTabu(start, map, rows, cols, cellToIndex, precomputedNeighbors);
                                    distances_from_start = dijkstraWithWeight(start, map, rows, cols, cellToIndex, precomputedNeighbors, weightTable, passableCount);
                        
                        // 各状態から各位置への最短距離を計算（スレッド内で実行）
                        std::vector<WeightedCostType> location_weighted_distances(passableCount, std::numeric_limits<WeightedCostType>::max());
                        for (int end_loc = 0; end_loc < passableCount; ++end_loc)
                        {
                            const int base = end_loc * 4;
                            const WeightedCostType d0 = distances_from_start[base];
                            const WeightedCostType d1 = distances_from_start[base + 1];
                            const WeightedCostType d2 = distances_from_start[base + 2];
                            const WeightedCostType d3 = distances_from_start[base + 3];
                            WeightedCostType min_cost = d0;
                            if(d1 < min_cost) min_cost = d1;
                            if(d2 < min_cost) min_cost = d2;
                            if(d3 < min_cost) min_cost = d3;
                            location_weighted_distances[end_loc] = min_cost;
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

                        return  location_weighted_distances; }));
            }
        }

        std::cout << "start collect" << std::endl;

        // 結果の収集
        for (size_t start_idx = 0; start_idx < total_states; ++start_idx)
        {
            auto location_weighted_distances = futures[start_idx].get();
            // dijkstraタスクから取得した distances_from_start の部分をコピー
            // size_t offset = triangularIndex(start_idx, start_idx, total_states);
            std::copy(location_weighted_distances.begin(), location_weighted_distances.end(),
                      stateToLocationDist.begin() + start_idx * passableCount);
        }

        // 各状態からのdijkstraタスクの収集が終わった後の処理
        // stateToLocationDist は各状態から各位置への最小距離が格納されている

        // locationToLocationDist はすでに std::numeric_limits<WeightedCostType>::max() で初期化されている

        // ※ 以下では、全行列 (i, j) に対して処理を行います。
        size_t num_rows = passableCount;
        // 各スレッドに担当する行数で分割
        size_t rows_per_thread = (num_rows + num_threads - 1) / num_threads;
        std::vector<std::future<void>> reductionFutures;
        for (int t = 0; t < num_threads; ++t)
        {
            size_t start_i = t * rows_per_thread;
            size_t end_i = std::min(num_rows, start_i + rows_per_thread);
            reductionFutures.push_back(
                pool.enqueue([&, start_i, end_i]()
                             {
                    for (size_t i = start_i; i < end_i; ++i)
                    {
                        for (size_t j = 0; j < passableCount; ++j)
                        {
                            // 一次元インデックスに変換: idx = i * passableCount + j
                            size_t idx = i * passableCount + j;
                            // 各方向について最小値を算出
                            for (int dir = 0; dir < 4; ++dir)
                            {
                                size_t state_idx = i * 4 + dir;
                                auto cost_from_startdir = stateToLocationDist[state_idx * passableCount + j];
                                locationToLocationDist[idx] = std::min(locationToLocationDist[idx], cost_from_startdir);
                            }
                        }
                    } }));
        }

        // 並列リダクションタスクの完了を待機
        for (auto &f : reductionFutures)
        {
            f.get();
        }

        return DirectionalPathCostMap(
            std::move(locationToLocationDist),
            std::move(stateToLocationDist),
            std::move(cellToIndex),
            std::move(passableCells),
            std::move(precomputedNeighbors),
            total_states,
            totalElementsLocToLoc);
    }

}