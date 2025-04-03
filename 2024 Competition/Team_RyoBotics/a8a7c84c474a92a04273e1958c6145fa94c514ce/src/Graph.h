#pragma once

#include <vector>
#include "SchedulerUtils.h"
class Graph
{
public:
    // コンストラクタをコンペ仕様に合わせる
    Graph(const std::vector<int> &map, int rows, int cols);

    // グラフ操作の基本メソッド
    std::vector<int> getNeighbors(int location) const;
    int getCost(int from, int to, int goal) const;
    int getEstimatedDistance(int from, int to) const;

    // ユーティリティメソッド
    bool isValidLocation(int location) const;
    std::pair<int, int> locationToXY(int location) const;
    int xyToLocation(int x, int y) const;
    bool isObstacle(int location) const;

    // グリッドの取得
    const std::vector<int> &getMap() const { return map; }
    int getRows() const { return rows; }
    int getCols() const { return cols; }

private:
    std::vector<int> map; // 0: 通行可能, 1: 障害物
    int rows;
    int cols;

    // 隣接ノードの方向（上下左右）
    const int dx[4] = {0, 1, 0, -1}; // 行の変化
    const int dy[4] = {1, 0, -1, 0}; // 列の変化

    // マンハッタン距離の計算
    int calculateManhattanDistance(int x1, int y1, int x2, int y2) const;
};