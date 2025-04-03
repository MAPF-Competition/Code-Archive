#include "PathFinder.h"
#include <algorithm>
#include <set>
#include <unordered_set>
#include <queue>
#include <limits>
namespace DefaultPlanner
{
    std::vector<int> PathFinder::findPath(
        int start,
        int goal,
        const std::vector<int> &map,
        int rows,
        int cols)
    {
        // ゴールのバリデーション
        if (goal < 0 || goal >= map.size() || map[goal] == 1)
        {
            return std::vector<int>();
        }

        // gScoreを大きな値で初期化
        std::unordered_map<int, int> gScore;
        for (int i = 0; i < map.size(); i++)
        {
            gScore[i] = std::numeric_limits<int>::max();
        }
        gScore[start] = 0;

        // openSetをpriority_queueで管理
        using Node = std::pair<int, int>; // <f_score, location>
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
        std::unordered_set<int> closedSet;
        std::unordered_map<int, int> parent;

        openSet.push({calculateHeuristic(start, goal, cols), start});

        while (!openSet.empty())
        {
            int current = openSet.top().second;
            openSet.pop();

            if (current == goal)
            {
                return reconstructPath(start, goal, parent);
            }

            if (closedSet.find(current) != closedSet.end())
            {
                continue;
            }
            closedSet.insert(current);

            for (int next : getNeighbors(current, map, rows, cols))
            {
                if (closedSet.find(next) != closedSet.end())
                {
                    continue;
                }

                int tentative_gScore = gScore[current] + 1;

                if (tentative_gScore < gScore[next])
                {
                    parent[next] = current;
                    gScore[next] = tentative_gScore;
                    int fScore = tentative_gScore + calculateHeuristic(next, goal, cols);
                    openSet.push({fScore, next});
                }
            }
        }

        return std::vector<int>();
    }

    int PathFinder::calculateHeuristic(int current, int goal, int cols)
    {
        int current_x = current % cols;
        int current_y = current / cols;
        int goal_x = goal % cols;
        int goal_y = goal / cols;

        return std::abs(current_x - goal_x) + std::abs(current_y - goal_y);
    }

    std::vector<int> PathFinder::getNeighbors(int location, const std::vector<int> &map, int rows, int cols)
    {
        std::vector<int> neighbors;
        int x = location % cols;
        int y = location / cols;

        // 4方向の移動を確認（東西南北）
        const int dx[] = {1, -1, 0, 0};
        const int dy[] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i)
        {
            int new_x = x + dx[i];
            int new_y = y + dy[i];

            // マップの範囲内かつ障害物でない場合のみ追加
            if (new_x >= 0 && new_x < cols && new_y >= 0 && new_y < rows)
            {
                int new_location = new_y * cols + new_x;
                if (map[new_location] != 1)
                {
                    neighbors.push_back(new_location);
                }
            }
        }

        return neighbors;
    }

    std::vector<int> PathFinder::reconstructPath(int start, int goal, const std::unordered_map<int, int> &parent)
    {
        std::vector<int> path;
        int current = goal;

        while (current != start)
        {
            path.push_back(current);
            auto it = parent.find(current);
            if (it == parent.end())
            {
                return std::vector<int>(); // パスが見つからない場合
            }
            current = it->second;
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }
}