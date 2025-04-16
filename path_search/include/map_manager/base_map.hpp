namespace map_manager {
    class BaseMap {
    public:
        virtual double getCost(const Eigen::VectorXd& position) = 0;
        virtual bool isOccupied(const Eigen::VectorXd& position) = 0;
        virtual bool isTraversable(const Eigen::VectorXd& position) = 0;
        virtual Eigen::VectorXd getGradient(const Eigen::VectorXd& position) = 0;
        virtual ~BaseMap() = default;
    };
    } // namespace map_manager