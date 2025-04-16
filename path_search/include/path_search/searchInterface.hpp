#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <memory>

// 基础类型定义
struct Node {
    int x, y;       // 通用坐标
    float theta;     // Hybrid A*专用方向角
    float g, h;      // 代价值
    bool operator==(const Node& o) const { 
        return x == o.x && y == o.y && theta == o.theta; 
    }
};

namespace std {
template<> struct hash<Node> {
    size_t operator()(const Node& n) const {
        return hash<int>()(n.x) ^ hash<int>()(n.y) ^ hash<float>()(n.theta);
    }
};
}

// 地图接口抽象类
class MapInterface {
public:
    virtual ~MapInterface() = default;
    virtual bool isTraversable(int x, int y) const = 0;
    virtual float getHeuristic(int x1, int y1, int x2, int y2) const = 0;
};

// 搜索算法基类
class SearchAlgorithm {
protected:
    struct NodeCompare {
        bool operator()(const Node& a, const Node& b) const { 
            return (a.g + a.h) > (b.g + b.h); 
        }
    };

    std::priority_queue<Node, std::vector<Node>, NodeCompare> open_set;
    std::unordered_map<Node, Node> came_from;
    std::unordered_map<Node, float> cost_so_far;
    const MapInterface* map;
    Node goal;

public:
    virtual ~SearchAlgorithm() = default;
    
    void initialize(const MapInterface* map_ptr, Node start, Node goal_node) {
        map = map_ptr;
        goal = goal_node;
        open_set = {};
        came_from.clear();
        cost_so_far.clear();
        start.g = 0;
        start.h = heuristic(start);
        open_set.push(start);
        cost_so_far[start] = 0;
    }

    virtual std::vector<Node> search() {
        std::vector<Node> path;
        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();

            if (isGoal(current)) {
                path = reconstructPath(current);
                break;
            }

            for (const auto& next : getNeighbors(current)) {
                float new_cost = cost_so_far[current] + movementCost(current, next);
                if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    Node next_node = next;
                    next_node.g = new_cost;
                    next_node.h = heuristic(next);
                    open_set.push(next_node);
                    came_from[next] = current;
                }
            }
        }
        return path;
    }

protected:
    virtual float heuristic(const Node& n) const {
        return map->getHeuristic(n.x, n.y, goal.x, goal.y);
    }

    virtual bool isGoal(const Node& n) const {
        return n.x == goal.x && n.y == goal.y;
    }

    virtual std::vector<Node> getNeighbors(const Node& n) const = 0;
    
    virtual float movementCost(const Node& a, const Node& b) const {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    std::vector<Node> reconstructPath(Node current) const {
        std::vector<Node> path;
        while (came_from.find(current) != came_from.end()) {
            path.push_back(current);
            current = came_from.at(current);
        }
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// A* 算法实现
class AStar : public SearchAlgorithm {
protected:
    std::vector<Node> getNeighbors(const Node& n) const override {
        std::vector<Node> neighbors;
        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

        for (int i = 0; i < 8; ++i) {
            Node neighbor{n.x + dx[i], n.y + dy[i]};
            if (map->isTraversable(neighbor.x, neighbor.y)) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }
};

// JPS 算法实现
class JPS : public AStar {
protected:
    std::vector<Node> getNeighbors(const Node& n) const override {
        // 实现跳点搜索逻辑（此处需要补充完整）
        // 需要实现方向剪枝和跳点检测
        std::vector<Node> neighbors;
        // ... JPS specific logic ...
        return neighbors;
    }

    float heuristic(const Node& n) const override {
        // 使用对角线距离
        int dx = abs(n.x - goal.x);
        int dy = abs(n.y - goal.y);
        return (dx + dy) + (sqrt(2) - 2) * std::min(dx, dy);
    }
};

// Hybrid A* 算法实现
class HybridAStar : public SearchAlgorithm {
private:
    const float steering_res = M_PI / 4; // 转向角分辨率
    const float step_length = 1.0f;      // 步长

protected:
    std::vector<Node> getNeighbors(const Node& n) const override {
        std::vector<Node> neighbors;
        // 生成可能的运动方向
        for (float steer : {-steering_res, 0.0f, steering_res}) {
            Node next;
            next.theta = n.theta + steer;
            next.x = n.x + step_length * cos(next.theta);
            next.y = n.y + step_length * sin(next.theta);
            
            if (map->isTraversable(static_cast<int>(next.x), static_cast<int>(next.y))) {
                neighbors.push_back(next);
            }
        }
        return neighbors;
    }

    float movementCost(const Node& a, const Node& b) const override {
        // 考虑转向惩罚
        float theta_diff = fabs(a.theta - b.theta);
        return step_length + theta_diff * 0.1f;
    }

    bool isGoal(const Node& n) const override {
        return std::hypot(n.x - goal.x, n.y - goal.y) < 1.0f &&
               fabs(n.theta - goal.theta) < 0.1f;
    }

    float heuristic(const Node& n) const override {
        // 结合传统启发式和动态规划启发式
        float traditional = map->getHeuristic(n.x, n.y, goal.x, goal.y);
        float obstacle_aware = 0; // 可添加ESDF相关启发式
        return std::max(traditional, obstacle_aware);
    }
};

// 示例地图实现
class ExampleMap : public MapInterface {
public:
    bool isTraversable(int x, int y) const override {
        // 实现具体的障碍物检测逻辑
        return x >= 0 && x < 100 && y >= 0 && y < 100; // 简单示例
    }

    float getHeuristic(int x1, int y1, int x2, int y2) const override {
        return std::hypot(x1 - x2, y1 - y2); // 欧式距离
    }
};

int main() {
    ExampleMap map;
    Node start{5, 5}, goal{95, 95};

    // 使用A*算法
    AStar astar;
    astar.initialize(&map, start, goal);
    auto path_a = astar.search();

    // 使用Hybrid A*
    HybridAStar hybrid;
    start.theta = 0; goal.theta = 0;
    hybrid.initialize(&map, start, goal);
    auto path_hybrid = hybrid.search();

    return 0;
}