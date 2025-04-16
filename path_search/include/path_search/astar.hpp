class AStar : public BaseAStar<GridNode> {
    public:
        struct GridNode {
            Eigen::Vector2i index;
            double cost;
            // 节点比较运算符重载...
        };
    
        void initialize(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal) override 
        {
            // 初始化开放列表、起始节点...
        }
    
        void expandNeighbors(NodePtr current) override {
            for (const auto& direction : directions_) {
                Eigen::Vector2i neighbor_idx = current->index + direction;
                if (map_->isOccupied(neighbor_idx)) continue;
                // 处理新节点...
            }
        }
        // 其他方法实现...
    };