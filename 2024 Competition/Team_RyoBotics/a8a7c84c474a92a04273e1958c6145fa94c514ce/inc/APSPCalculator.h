#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <iostream>
#include "WeightTable.h"
#include "CommonTypes.h"
namespace SchedulerUtils
{
    // 距離の型を管理する型エイリアス
    extern std::vector<int> cellToIndex;
    extern std::vector<int> passableCells;
    // 前方宣言を追加
    class DirectionalPathCostMap;
    struct State;

    struct State
    {
        int location;                  // グリッド上の位置
        unsigned short direction;      // 0: 右, 1: 下, 2: 左, 3: 上
        int from_location;             // 前の位置
        unsigned short from_direction; // 前の方向
        // 前の状態からどの方向に移動したか
        int from_direction_to_current; // 0: 右, 1: 下, 2: 左, 3: 上, 4: 回転orWait
        // 対向する方向を返す
        int getOppositeDirection() const
        {
            return (direction + 2) % 4;
        }

        State(int loc = 0, int dir = 0, int from_loc = 0) : location(loc), direction(dir), from_location(from_loc)
        {
            // loc == from_locなら4
            from_direction_to_current = loc == from_loc ? 4 : dir;
        }

        bool operator==(const State &other) const
        {
            return location == other.location && direction == other.direction;
        }
    };
    struct StateWithVisits : public State
    {
        unsigned short tabu_visits; // 訪問回数を追跡

        StateWithVisits(int loc = 0, int dir = 0, int visits = 0)
            : State(loc, dir), tabu_visits(visits) {}

        bool operator==(const StateWithVisits &other) const
        {
            return location == other.location &&
                   direction == other.direction &&
                   tabu_visits == other.tabu_visits;
        }
    };

    constexpr int MAX_NEIGHBORS = 3;
    struct FixedNeighborList
    {
        int count;                      // 実際に登録された隣接状態の数
        State neighbors[MAX_NEIGHBORS]; // 固定サイズ配列
    };
    extern std::vector<FixedNeighborList> precomputedNeighbors;

    // Stateをハッシュマップのキーとして使用するためのハッシュ関数
    struct StateHash
    {
        std::size_t operator()(const State &state) const
        {
            return std::hash<int>()(state.location) ^ (std::hash<int>()(state.direction) << 1);
        }
    };
    inline size_t triangularIndex(size_t i, size_t j, size_t total_states)
    {
        // i <= j であることを前提とする
        return i * total_states - (i * (i - 1)) / 2 + (j - i);
    }
    class DirectionalPathCostMap
    {
    public:
        DirectionalPathCostMap() = default;

        DirectionalPathCostMap(
            std::vector<WeightedCostType> locationToLocationDist,
            std::vector<WeightedCostType> stateToLocationDist,
            std::vector<int> cellToIndex,
            std::vector<int> passableCells,
            std::vector<FixedNeighborList> precomputedNeighbors,
            size_t total_states,
            size_t total_elements_loc_to_loc)
            : locationToLocationDist_(std::move(locationToLocationDist)),
              stateToLocationDist_(std::move(stateToLocationDist)),
              cellToIndex_(std::move(cellToIndex)),
              passableCells_(std::move(passableCells)),
              precomputedNeighbors_(std::move(precomputedNeighbors)),
              total_states_(total_states),
              total_elements_loc_to_loc_(total_elements_loc_to_loc) {}

        // 変換前の位置と方向からコストを取得
        CostType getCost(int fromLoc, int fromDir, int toLoc, int toDir) const
        {
            if (locationToLocationDist_.empty() || cellToIndex_.empty())
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
                size_t idx = triangularIndex(startStateIdx, endStateIdx, total_states_);
                return locationToLocationDist_[idx];
            }
            else
            {
                size_t idx = triangularIndex(endStateIdx, startStateIdx, total_states_);
                return locationToLocationDist_[idx];
            }
        }
        WeightedCostType getWeightedCost(int fromLoc, int fromDir, int toLoc, int toDir) const
        {
            if (locationToLocationDist_.empty() || cellToIndex_.empty())
            {
                return 0;
            }
            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return 0;
            }

            int startStateIdx = fromIdx * 4 + fromDir;
            int endStateIdx = toIdx * 4 + toDir;

            // 小さい方のインデックスを行、大きい方を列として使用
            if (startStateIdx <= endStateIdx)
            {
                size_t idx = triangularIndex(startStateIdx, endStateIdx, total_states_);
                return locationToLocationDist_[idx];
                // return static_cast<int>(visitedTabuStates_[startStateIdx][endStateIdx - startStateIdx]);
            }
            else
            {
                // return static_cast<int>(visitedTabuStates_[endStateIdx][startStateIdx - endStateIdx]);
                size_t idx = triangularIndex(endStateIdx, startStateIdx, total_states_);
                return locationToLocationDist_[idx];
            }
        }

        // 新しいメソッド：特定の方向を持つ状態から目的地までの最短コストを返す
        WeightedCostType getWeightedCostFromStateToLocation(int fromLoc, int fromDir, int toLoc) const
        {
            if (stateToLocationDist_.empty() || cellToIndex_.empty())
            {
                return std::numeric_limits<WeightedCostType>::max();
            }

            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return std::numeric_limits<WeightedCostType>::max();
            }

            int stateIdx = fromIdx * 4 + fromDir;
            size_t idx = stateIdx * passableCells_.size() + toIdx;
            return stateToLocationDist_[idx];
        }
        // 特定の位置から特定の位置までの最短コストを返す
        WeightedCostType getWeightedCostFromLocationToLocation(int fromLoc, int toLoc) const
        {
            if (locationToLocationDist_.empty() || cellToIndex_.empty())
            {
                return std::numeric_limits<WeightedCostType>::max();
            }
            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return std::numeric_limits<WeightedCostType>::max();
            }
            size_t idx = fromIdx * passableCells_.size() + toIdx;
            return locationToLocationDist_[idx];
        }

        CostType getCostToLocation(int fromLoc, int fromDir, int toLoc) const
        {
            if (locationToLocationDist_.empty() || cellToIndex_.empty())
            {
                return std::numeric_limits<CostType>::max();
            }

            int fromIdx = cellToIndex_[fromLoc];
            int toIdx = cellToIndex_[toLoc];

            if (fromIdx == -1 || toIdx == -1)
            {
                return std::numeric_limits<CostType>::max();
            }

            int stateIdx = fromIdx * 4 + fromDir;
            return locationToLocationDist_[stateIdx];
        }
        int getCellToIndex(int location) const
        {
            return cellToIndex_[location];
        }
        FixedNeighborList getNeighbors(int location, int direction) const
        {
            int stateIdx = cellToIndex_[location] * 4 + direction;
            return precomputedNeighbors_[stateIdx];
        }

    private:
        std::vector<WeightedCostType> locationToLocationDist_;
        std::vector<WeightedCostType> stateToLocationDist_;
        std::vector<int> cellToIndex_;
        std::vector<int> passableCells_;
        std::vector<FixedNeighborList> precomputedNeighbors_;
        size_t total_states_;
        size_t total_elements_loc_to_loc_;
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
        static std::vector<FixedNeighborList> precomputeNeighbors(
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex);

    protected:
        // 単一始点からのBFS
        static std::vector<CostType> bfs(
            const State &start,
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex,
            const std::vector<FixedNeighborList> &precomputedNeighbors);
        static std::pair<std::vector<CostType>, std::vector<unsigned char>> bfsWithTabu(
            const State &start,
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex,
            const std::vector<FixedNeighborList> &precomputedNeighbors);
        static std::pair<std::vector<CostType>, std::vector<WeightedCostType>> bfsWithWeight(
            const State &start,
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex,
            const std::vector<FixedNeighborList> &precomputedNeighbors,
            const WeightTable &weightTable);

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
        static std::vector<WeightedCostType> dijkstraWithWeight(
            const State &start,
            const std::vector<int> &map,
            int rows,
            int cols,
            const std::vector<int> &cellToIndex,
            const std::vector<FixedNeighborList> &precomputedNeighbors,
            const WeightTable &weightTable,
            const size_t passableCount);
    };

} // namespace DefaultPlanner