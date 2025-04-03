#include "Graph.h"
#include <stdexcept>
#include <limits>

Graph::Graph(const std::vector<int> &input_map, int r, int c)
    : map(input_map), rows(r), cols(c)
{
    if (r <= 0 || c <= 0)
    {
        throw std::invalid_argument("Rows and cols must be positive");
    }
    if (static_cast<int>(input_map.size()) != r * c)
    {
        throw std::invalid_argument("Map size does not match rows * cols");
    }
}

std::vector<int> Graph::getNeighbors(int location) const
{
    std::vector<int> neighbors;
    auto [x, y] = locationToXY(location);

    // 上下左右の移動
    for (int i = 0; i < 4; ++i)
    {
        int new_x = x + dx[i];
        int new_y = y + dy[i];

        // 新しい位置が有効かチェック
        if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols)
        {
            int new_location = xyToLocation(new_x, new_y);
            if (map[new_location] == 0) // 通行可能なセルのみ
            {
                neighbors.push_back(new_location);
            }
        }
    }
    // waitの場合
    neighbors.push_back(location);

    return neighbors;
}

std::pair<int, int> Graph::locationToXY(int location) const
{
    if (!isValidLocation(location))
    {
        throw std::out_of_range("Invalid location");
    }
    return {location / cols, location % cols}; // {row, col}
}

int Graph::xyToLocation(int x, int y) const
{
    if (x < 0 || x >= rows || y < 0 || y >= cols)
    {
        throw std::out_of_range("Invalid coordinates");
    }
    return x * cols + y; // row * cols + col
}

bool Graph::isValidLocation(int location) const
{
    return location >= 0 && location < rows * cols;
}

bool Graph::isObstacle(int location) const
{
    return map[location] != 0; // 0以外は通行不可
}

int Graph::getCost(int from, int to, int goal) const
{
    if (from == to && from == goal)
    {
        return 0;
    }
    if (from == to)
    {
        return 1;
    }
    auto [x1, y1] = locationToXY(from);
    auto [x2, y2] = locationToXY(to);

    // 隣接していない場合は大きな値を返す
    if (std::abs(x1 - x2) + std::abs(y1 - y2) != 1)
    {
        std::cerr << "エラー発生位置: from=" << from << ", to=" << to << std::endl;
        return std::numeric_limits<int>::max();
    }

    return 1;
}

int Graph::getEstimatedDistance(int from, int to) const
{
    // auto [x1, y1] = locationToXY(from);
    // auto [x2, y2] = locationToXY(to);
    // return calculateManhattanDistance(x1, y1, x2, y2);
    return SchedulerUtils::getDistance(from, to);
}

int Graph::calculateManhattanDistance(int x1, int y1, int x2, int y2) const
{
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}