#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

using namespace std;

struct Node {
    int x, y;
    double g, h; // g: 从起点到当前点的代价，h: 启发式估计从当前点到目标点的代价
    Node* parent;

    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const { return g + h; } // f = g + h
};

bool operator>(const Node& a, const Node& b) {
    return a.f() > b.f();
}

vector<Node> AStar(int startX, int startY, int endX, int endY, const vector<vector<int>>& grid) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // 方向：上下左右
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    auto heuristic = [endX, endY](int x, int y) {
        return abs(endX - x) + abs(endY - y); // 曼哈顿距离
    };

    priority_queue<Node, vector<Node>, greater<Node>> openList;
    unordered_map<int, unordered_map<int, bool>> closedList; // 记录已访问的节点

    Node* start = new Node(startX, startY, 0.0, heuristic(startX, startY));
    openList.push(*start);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // 到达目标
        if (current.x == endX && current.y == endY) {
            vector<Node> path;
            Node* pathNode = &current;
            while (pathNode) {
                path.push_back(*pathNode);
                pathNode = pathNode->parent;
            }
            reverse(path.begin(), path.end()); // 反转路径
            return path;
        }

        closedList[current.x][current.y] = true;

        // 检查邻居
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;
            
            if (newX >= 0 && newY >= 0 && newX < rows && newY < cols && grid[newX][newY] != 1 && !closedList[newX][newY]) {
                double newG = current.g + 1.0; // 假设每个格子代价为1
                double newH = heuristic(newX, newY);
                Node* neighbor = new Node(newX, newY, newG, newH, new Node(current.x, current.y, current.g, current.h));

                openList.push(*neighbor);
            }
        }
    }

    return {}; // 如果没有路径
}

int main() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };

    vector<Node> path = AStar(0, 0, 4, 4, grid);

    for (const Node& node : path) {
        cout << "(" << node.x << ", " << node.y << ")\n";
    }

    return 0;
}
