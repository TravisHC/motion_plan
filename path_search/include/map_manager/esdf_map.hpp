#pragma once
#include "base_map.hpp"
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace map_manager {

class ESDFMap : public BaseMap {
public:
    explicit ESDFMap(std::shared_ptr<grid_map::GridMap> grid_map) {
        sdf_.calculateSignedDistanceField(*grid_map, "elevation");
    }
    
    double getCost(const Eigen::VectorXd& position) override {
        float distance = sdf_.getDistanceAt(position.head<2>());
        return distanceToCost(distance);
    }
    
    bool isOccupied(const Eigen::VectorXd& position) override {
        return sdf_.getDistanceAt(position.head<2>()) <= 0;
    }
    
    Eigen::VectorXd getGradient(const Eigen::VectorXd& position) override {
        Eigen::Vector2f grad;
        sdf_.getGradientAt(position.head<2>(), grad);
        return grad.cast<double>();
    }

private:
    double distanceToCost(float distance) {
        // 实现距离到代价的映射
        return distance < 0 ? 100.0 : 
               (distance < 1.0 ? 50.0 * (1.0 - distance) : 0.0);
    }

    grid_map::SignedDistanceField sdf_;
};
} // namespace map_manager