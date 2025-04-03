#include "distance_table.h"
#include <limits>
#include <iostream>
#include "heuristics.h"
#include <iomanip> // std::setprecision用
#include <queue>

namespace DefaultPlanner
{

    void DistanceTable::computeAndSave(SharedEnvironment *env, const std::string &filepath)
    {
        auto fast_dist = computeAllPairsShortestPaths(env);

        std::ofstream file(filepath, std::ios::binary);
        if (!file)
        {
            std::cerr << "Failed to open file for writing: " << filepath << std::endl;
            return;
        }

        fast_dist.save(file);
        file.close();
    }

    FastDistanceTable DistanceTable::load(const std::string &filepath)
    {
        std::cout << "距離テーブルファイルを読み込み中: " << filepath << std::endl;

        std::ifstream file(filepath, std::ios::binary);
        if (!file)
        {
            std::cerr << "ファイルを開けませんでした: " << filepath << std::endl;
            throw std::runtime_error("ファイルオープンエラー");
        }

        // ファイルサイズを確認
        file.seekg(0, std::ios::end);
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::cout << "ファイルサイズ: " << size << " bytes" << std::endl;

        try
        {
            return FastDistanceTable::load(file);
        }
        catch (const std::exception &e)
        {
            std::cerr << "距離テーブルの読み込みに失敗: " << e.what() << std::endl;
            throw;
        }
    }

    FastDistanceTable DistanceTable::computeAllPairsShortestPaths(SharedEnvironment *env)
    {
        init_neighbor(env);
        int V = env->map.size();
        std::cout << "V: " << V << std::endl;
        std::cout << "env->map.size(): " << env->map.size() << std::endl;

        // 通行可能なセルのインデックスをマッピング
        std::vector<int> passableCells;
        std::vector<int> cellToIndex(V, -1);
        for (int i = 0; i < V; i++)
        {
            if (env->map[i] == 0)
            {
                cellToIndex[i] = passableCells.size();
                passableCells.push_back(i);
            }
        }

        int passableCount = passableCells.size();
        std::vector<std::vector<int>> dist(passableCount, std::vector<int>(passableCount, std::numeric_limits<int>::max()));

        // BFSで各セルからの最短距離を計算
        std::cout << "BFSで最短距離を計算中..." << std::endl;
        for (int i = 0; i < passableCount; i++)
        {
            float progress = (float)i / passableCount * 100.0f;
            std::cout << "\r進捗: " << std::fixed << std::setprecision(1) << progress << "% "
                      << "(" << i << "/" << passableCount << ")" << std::flush;

            // BFSで1つの始点からの最短距離を計算
            std::queue<int> q;
            std::vector<bool> visited(passableCount, false);

            int start = passableCells[i];
            q.push(start);
            dist[i][i] = 0;
            visited[i] = true;

            while (!q.empty())
            {
                int current = q.front();
                q.pop();
                int currentIndex = cellToIndex[current];

                std::vector<int> neighbors;
                getNeighborLocs(&global_neighbors, neighbors, current);

                for (int next : neighbors)
                {
                    if (env->map[next] == 0) // 通行可能なセルの場合
                    {
                        int nextIndex = cellToIndex[next];
                        if (!visited[nextIndex])
                        {
                            visited[nextIndex] = true;
                            dist[i][nextIndex] = dist[i][currentIndex] + 1;
                            q.push(next);
                        }
                    }
                }
            }
        }
        std::cout << "\r進捗: 100% (" << passableCount << "/" << passableCount << ")" << std::endl;
        std::cout << "計算完了" << std::endl;

        int i = 298, j = 818;
        std::cout << "dist[" << i << "][" << j << "]: " << dist[i][j] << std::endl;

        // FastDistanceTable オブジェクトを作成して返す
        return FastDistanceTable(dist, cellToIndex, passableCells, V, passableCount);
    }

    void DistanceTable::floydWarshall(std::vector<std::vector<int>> &dist, int V)
    {
        std::cout << "Floyd-Warshall アルゴリズムを実行中..." << std::endl;
        for (int k = 0; k < V; k++)
        {
            // プログレス表示（パーセンテージ）
            float progress = (float)k / V * 100.0f;
            std::cout << "\r進捗: " << std::fixed << std::setprecision(1) << progress << "% "
                      << "(" << k << "/" << V << ")" << std::flush;

            for (int i = 0; i < V; i++)
            {
                for (int j = 0; j < V; j++)
                {
                    if (dist[i][k] != std::numeric_limits<int>::max() &&
                        dist[k][j] != std::numeric_limits<int>::max() &&
                        dist[i][k] + dist[k][j] < dist[i][j])
                    {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
        std::cout << "\r進捗: 100% (" << V << "/" << V << ")" << std::endl;
        std::cout << "計算完了" << std::endl;
    }

}