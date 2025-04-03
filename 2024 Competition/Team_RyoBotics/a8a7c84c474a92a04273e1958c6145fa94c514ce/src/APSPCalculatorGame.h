#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <iostream>
namespace SchedulerUtilsGame
{
    extern std::vector<int> cellToIndex;
    extern std::vector<int> passableCells;
    // 前方宣言を追加
    class DirectionalPathCostMap;

    struct State
    {
        int location;  // グリッド上の位置
        int direction; // 0: 右, 1: 下, 2: 左, 3: 上
        // 対向する方向を返す
        int getOppositeDirection() const
        {
            return (direction + 2) % 4;
        }

        State(int loc = 0, int dir = 0) : location(loc), direction(dir) {}

        bool operator==(const State &other) const
        {
            return location == other.location && direction == other.direction;
        }
    };
    extern std::vector<std::vector<State>> precomputedNeighbors;

    // Stateをハッシュマップのキーとして使用するためのハッシュ関数
    struct StateHash
    {
        std::size_t operator()(const State &state) const
        {
            return std::hash<int>()(state.location) ^ (std::hash<int>()(state.direction) << 1);
        }
    };

    class DirectionalPathCostMap
    {
    public:
        DirectionalPathCostMap() = default;

        DirectionalPathCostMap(
            std::vector<std::vector<unsigned short>> distances,
            std::vector<std::vector<unsigned short>> locationDistances,
            std::vector<int> cellToIndex,
            std::vector<int> passableCells,
            std::vector<std::vector<State>> precomputedNeighbors)
            : distances_(std::move(distances)),
              locationDistances_(std::move(locationDistances)),
              cellToIndex_(std::move(cellToIndex)),
              passableCells_(std::move(passableCells)),
              precomputedNeighbors_(std::move(precomputedNeighbors)) {}

        // 変換前の位置と方向からコストを取得
        int getCost(int fromLoc, int fromDir, int toLoc, int toDir) const
        {
            if (distances_.empty() || cellToIndex_.empty())
            {
                return std::numeric_limits<unsigned short>::max();
            }

            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return std::numeric_limits<unsigned short>::max();
            }

            int startStateIdx = fromIdx * 4 + fromDir;
            int endStateIdx = toIdx * 4 + toDir;

            // 小さい方のインデックスを行、大きい方を列として使用
            if (startStateIdx <= endStateIdx)
            {
                return static_cast<unsigned short>(distances_[startStateIdx][endStateIdx - startStateIdx]);
            }
            else
            {
                return static_cast<unsigned short>(distances_[endStateIdx][startStateIdx - endStateIdx]);
            }
        }

        // 新しいメソッド：特定の方向を持つ状態から目的地までの最短コストを返す
        int getCostToLocation(int fromLoc, int fromDir, int toLoc) const
        {
            if (locationDistances_.empty() || cellToIndex_.empty())
            {
                return std::numeric_limits<unsigned short>::max();
            }

            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return std::numeric_limits<unsigned short>::max();
            }

            int stateIdx = fromIdx * 4 + fromDir;
            return locationDistances_[stateIdx][toIdx];
        }
        int getCellToIndex(int location) const
        {
            return cellToIndex_[location];
        }
        std::vector<State> getNeighbors(int location, int direction) const
        {
            int stateIdx = cellToIndex_[location] * 4 + direction;
            return precomputedNeighbors_[stateIdx];
        }

    private:
        std::vector<std::vector<unsigned short>> distances_;
        std::vector<std::vector<unsigned short>> locationDistances_;
        std::vector<int> cellToIndex_;
        std::vector<int> passableCells_;
        std::vector<std::vector<State>> precomputedNeighbors_;
    };

    class APSPCalculator
    {
    public:
        // グリッドマップと行数、列数を受け取ってAPSPを計算
        static DirectionalPathCostMap calculateAPSP(
            const std::vector<int> &map,
            int rows,
            int cols);
        // 通行可能なセルのインデックスマッピング用の補助関数を追加
        static std::vector<int> getPassableCells(
            const std::vector<int> &map,
            std::vector<int> &cellToIndex);

        // 新しいメソッドを追加
        static std::vector<std::vector<State>> precomputeNeighbors(
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex);

    protected:
        // 単一始点からのBFS
        static std::vector<unsigned short> bfs(
            const State &start,
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex,
            const std::vector<std::vector<State>> &precomputedNeighbors);

        // 隣接状態の取得
        static std::vector<State> getNeighbors(
            const State &current,
            const std::vector<int> &map,
            int rows,
            int cols);

        // 移動の妥当性チェック
        static bool validateMove(
            int from,
            int to,
            const std::vector<int> &map,
            int rows,
            int cols);
    };

} // namespace DefaultPlanner