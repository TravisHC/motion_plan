#pragma once
#include <vector>
#include <memory>
#include "eigen3/Eigen/Core"

namespace astar_core {

template<typename NodeT>
class BaseAStar {
public:
    using NodePtr = std::shared_ptr<NodeT>;
    
    BaseAStar(std::shared_ptr<map_manager::BaseMap> map) : map_(map) {}
    
    virtual std::vector<Eigen::VectorXd> plan(
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& goal) 
    {
        // 通用A*流程
        initialize(start, goal);
        while (!terminateCondition()) {
            NodePtr current = getNextNode();
            if (isGoal(current)) {
                return getPath(current);
            }
            expandNeighbors(current);
        }
        return {}; // 失败返回空
    }

protected:
    // 子类必须实现的差异化方法
    virtual void initialize(const Eigen::VectorXd& start,
                           const Eigen::VectorXd& goal) = 0;
    virtual NodePtr getNextNode() = 0;
    virtual void expandNeighbors(NodePtr current) = 0;
    virtual bool isGoal(NodePtr node) = 0;
    virtual bool terminateCondition() = 0;
    virtual std::vector<Eigen::VectorXd> getPath(NodePtr node) = 0;

    std::shared_ptr<map_manager::BaseMap> map_;
    // 其他通用成员...
};
} // namespace astar_core