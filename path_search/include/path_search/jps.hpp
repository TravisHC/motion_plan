#pragma once
#include "base_astar.hpp"
#include <unordered_map>

namespace astar_core {

class JPS : public BaseAStar<GridNode> {
public:
    struct GridNode {
        Eigen::Vector2i index;
        double cost;
        GridNode* parent;
        
        bool operator==(const GridNode& other) const {
            return index == other.index;
        }
        
        struct Hash {
            size_t operator()(const GridNode& node) const {
                return std::hash<int>()(node.index.x()) ^ 
                      (std::hash<int>()(node.index.y()) << 1);
            }
        };
    };

    JPS(std::shared_ptr<map_manager::BaseMap> map) : BaseAStar(map) {}

    void initialize(const Eigen::VectorXd& start, 
                   const Eigen::VectorXd& goal) override {
        start_ = start.head<2>().cast<int>();
        goal_ = goal.head<2>().cast<int>();
        
        open_list_.clear();
        closed_list_.clear();
        
        GridNode start_node{start_, 0, nullptr};
        open_list_.insert(start_node);
    }

    NodePtr getNextNode() override {
        return *open_list_.begin(); // 使用优先队列更高效
    }

    void expandNeighbors(NodePtr current) override {
        auto directions = getPrunedDirections(current);
        for (const auto& dir : directions) {
            if (auto jump_point = jump(current->index, dir)) {
                processJumpPoint(current, *jump_point);
            }
        }
    }

protected:
    std::vector<Eigen::Vector2i> getPrunedDirections(NodePtr node) {
        // 实现JPS的方向剪枝规则
        if (!node->parent) {
            return {Eigen::Vector2i(1,0), Eigen::Vector2i(0,1), /*...*/};
        }
        // 根据父节点计算强制邻居等...
    }

    std::optional<Eigen::Vector2i> jump(Eigen::Vector2i pos, Eigen::Vector2i dir) {
        // 实现跳跃点搜索逻辑
        while (true) {
            pos += dir;
            if (!map_->isTraversable(pos.cast<double>())) return std::nullopt;
            if (isGoalPos(pos)) return pos;
            if (hasForcedNeighbor(pos, dir)) return pos;
        }
    }

private:
    std::unordered_set<GridNode, GridNode::Hash> open_list_;
    std::unordered_set<GridNode, GridNode::Hash> closed_list_;
    Eigen::Vector2i start_, goal_;
};
} // namespace astar_core