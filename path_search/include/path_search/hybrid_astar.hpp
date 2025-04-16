#pragma once
#include "base_astar.hpp"
#include <queue>

namespace astar_core {

class HybridAStar : public BaseAStar<VehicleNode> {
public:
    struct VehicleNode {
        Eigen::Vector3d state; // x,y,theta
        double cost_g;
        double cost_h;
        VehicleNode* parent;
        std::vector<Eigen::Vector3d> path_segment;
        
        bool operator<(const VehicleNode& other) const {
            return (cost_g + cost_h) > (other.cost_g + other.cost_h); // 小顶堆
        }
    };

    HybridAStar(std::shared_ptr<map_manager::BaseMap> map, 
               double min_turn_radius = 1.0)
        : BaseAStar(map), min_radius_(min_turn_radius) {}

    void initialize(const Eigen::VectorXd& start,
                   const Eigen::VectorXd& goal) override {
        start_ = start;
        goal_ = goal;
        
        while (!open_list_.empty()) open_list_.pop();
        closed_list_.clear();
        
        VehicleNode start_node{start, 0, heuristic(start), nullptr, {}};
        open_list_.push(start_node);
    }

    void expandNeighbors(NodePtr current) override {
        constexpr int num_steering_angles = 5;
        for (int i = 0; i < num_steering_angles; ++i) {
            double steering = -max_steering_ + i * (2*max_steering_)/(num_steering_angles-1);
            auto path = generateMotionPrimitive(current->state, steering);
            
            if (!isPathValid(path)) continue;
            
            VehicleNode new_node = createNewNode(current, path);
            open_list_.push(new_node);
        }
    }

private:
    std::vector<Eigen::Vector3d> generateMotionPrimitive(
        const Eigen::Vector3d& start, double steering) {
        // 车辆运动学模型生成路径段
        std::vector<Eigen::Vector3d> path;
        // 实现Reeds-Shepp曲线或Dubins路径
        return path;
    }

    double heuristic(const Eigen::Vector3d& state) {
        // 结合欧式距离和方向偏差的启发式
        double dx = goal_[0] - state[0];
        double dy = goal_[1] - state[1];
        double dtheta = angles::shortest_angular_distance(state[2], goal_[2]);
        return std::hypot(dx, dy) + 0.1 * std::abs(dtheta);
    }

    std::priority_queue<VehicleNode> open_list_;
    std::unordered_map<Eigen::Vector3d, double, Vector3dHash> closed_list_;
    double min_radius_;
    double max_steering_ = M_PI / 4;
    Eigen::Vector3d start_, goal_;
};
} // namespace astar_core