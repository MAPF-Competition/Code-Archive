#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace DefaultPlanner
{
    class PathFinder
    {
    public:
        // 二点間の最短経路を探索する
        // start: 開始地点のlocation
        // goal: 目標地点のlocation
        // map: 障害物情報を含むグリッドマップ（0:通行可能、1:障害物）
        // rows, cols: マップの行数と列数
        // 戻り値: locationのベクトル。経路が見つからない場合は空のベクトルを返す
        static std::vector<int> findPath(
            int start,
            int goal,
            const std::vector<int> &map,
            int rows,
            int cols);

    private:
        static std::vector<int> reconstructPath(int start, int goal, const std::unordered_map<int, int> &parent);

        // マンハッタン距離を計算
        static int calculateHeuristic(int current, int goal, int cols);

        // 指定されたlocationの隣接セルを取得
        static std::vector<int> getNeighbors(int location, const std::vector<int> &map, int rows, int cols);
    };
}