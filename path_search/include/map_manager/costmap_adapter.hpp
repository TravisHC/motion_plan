#pragma once
#include "base_map.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace map_manager {

class CostmapAdapter : public BaseMap {
public:
    explicit CostmapAdapter(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
        : costmap_(costmap_ros->getCostmap()) {}
    
    double getCost(const Eigen::VectorXd& position) override {
        unsigned int mx, my;
        if (!worldToMap(position, mx, my)) return nav2_costmap_2d::LETHAL_OBSTACLE;
        return costmap_->getCost(mx, my);
    }
    
    bool isOccupied(const Eigen::VectorXd& position) override {
        return getCost(position) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    
    Eigen::VectorXd getGradient(const Eigen::VectorXd& position) override {
        // 实现基于Costmap的梯度计算（需要近似）
        Eigen::Vector2d gradient;
        double step = costmap_->getResolution();
        gradient.x() = (getCost(position + Eigen::Vector2d(step, 0)) - 
                       getCost(position - Eigen::Vector2d(step, 0))) / (2*step);
        gradient.y() = (getCost(position + Eigen::Vector2d(0, step)) - 
                      getCost(position - Eigen::Vector2d(0, step))) / (2*step);
        return gradient.normalized();
    }

private:
    bool worldToMap(const Eigen::VectorXd& position, unsigned int& mx, unsigned int& my) {
        double wx = position[0], wy = position[1];
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    nav2_costmap_2d::Costmap2D* costmap_;
};
} // namespace map_manager