#include "trajectory.hpp"
#include "visualizer.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

double timeTrapzVel(const double dist, const double vel, const double acc) {
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d) {
        return 2.0 * sqrt(dist / acc);
    } else {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

// clang-format off
void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum, const Eigen::Vector3d &initialPos, const Eigen::Vector3d &initialVel, const Eigen::Vector3d &initialAcc, const Eigen::Vector3d &terminalPos, const Eigen::Vector3d &terminalVel, const Eigen::Vector3d &terminalAcc, const Eigen::Matrix3Xd &intermediatePositions, const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix) {
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 * pieceNum, 6 * pieceNum);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6 * pieceNum, 3);
    Eigen::MatrixXd F0(3, 6);
    F0 << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0;
    M.block(0, 0, 3, 6) = F0;
    b.block(0, 0, 3, 3) << initialPos(0), initialPos(1), initialPos(2), initialVel(0), initialVel(1), initialVel(2), initialAcc(0), initialAcc(1), initialAcc(2);
    double tm(timeAllocationVector(pieceNum - 1));
    Eigen::MatrixXd EM(3, 6);
    EM << 1, tm, pow(tm, 2), pow(tm, 3), pow(tm, 4), pow(tm, 5), 0, 1, 2 * tm, 3 * pow(tm, 2), 4 * pow(tm, 3), 5 * pow(tm, 4), 0, 0, 2, 6 * tm, 12 * pow(tm, 2), 20 * pow(tm, 3);
    M.block(6 * pieceNum - 3, 6 * pieceNum - 6, 3, 6) = EM;
    b.block(6 * pieceNum - 3, 0, 3, 3) << terminalPos(0), terminalPos(1), terminalPos(2), terminalVel(0), terminalVel(1), terminalVel(2), terminalAcc(0), terminalAcc(1), terminalAcc(2);
    for (int i = 1; i < pieceNum; i++) {
        double t(timeAllocationVector(i - 1));
        Eigen::MatrixXd Fi(6, 6), Ei(6, 6);
        Eigen::Vector3d Di(intermediatePositions.transpose().row(i - 1));
        Ei << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
                1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
                0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
                0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3),
                0, 0, 0, 6, 24 * t, 60 * pow(t, 2),
                0, 0, 0, 0, 24, 120 * t;
        Fi << 0, 0, 0, 0, 0, 0,
                -1, 0, 0, 0, 0, 0,
                0, -1, 0, 0, 0, 0,
                0, 0, -2, 0, 0, 0,
                0, 0, 0, -6, 0, 0,
                0, 0, 0, 0, -24, 0;
        M.block(6 * i - 3, 6 * i - 6, 6, 6) = Ei;
        M.block(6 * i - 3, 6 * i, 6, 6)     = Fi;
        b.block(6 * i - 3, 0, 6, 3) << Di(0), Di(1), Di(2),
                                        0, 0, 0,
                                        0, 0, 0,
                                        0, 0, 0,
                                        0, 0, 0,
                                        0, 0, 0;
    }

    coefficientMatrix = M.inverse() * b;
    std::cout << "Coeff row: " << coefficientMatrix.rows() << "  col: " << coefficientMatrix.cols() << std::endl;
    // ------------------------ Put your solution above ------------------------
}
// clang-format on

class ClickGen : public rclcpp::Node {
private:
    // 直接声明参数
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
    std::shared_ptr<Visualizer> visualizer;
    Eigen::Matrix3Xd positions;
    Eigen::VectorXd times;
    int positionNum;
    Trajectory<5> traj;

public:
    ClickGen() 
      : Node("click_gen_node"),
        targetTopic("/goal_pose"),
        clickHeight(1.5),
        initialVel({0.0, 0.0, 0.0}),
        initialAcc({0.0, 0.0, 0.0}),
        terminalVel({0.0, 0.0, 0.0}),
        terminalAcc({0.0, 0.0, 0.0}),
        allocationSpeed(1.0),
        allocationAcc(1.0),
        maxPieceNum(5),
        positions(3, maxPieceNum + 1),
        times(maxPieceNum),
        positionNum(0)
    {
        // 声明参数
        this->declare_parameter<std::string>("TargetTopic", targetTopic);
        this->declare_parameter<double>("ClickHeight", clickHeight);
        this->declare_parameter<std::vector<double>>("InitialVel", initialVel);
        this->declare_parameter<std::vector<double>>("InitialAcc", initialAcc);
        this->declare_parameter<std::vector<double>>("TerminalVel", terminalVel);
        this->declare_parameter<std::vector<double>>("TerminalAcc", terminalAcc);
        this->declare_parameter<double>("AllocationSpeed", allocationSpeed);
        this->declare_parameter<double>("AllocationAcc", allocationAcc);
        this->declare_parameter<int>("MaxPieceNum", maxPieceNum);

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            targetTopic, 10, std::bind(&ClickGen::targetCallBack, this, std::placeholders::_1));

        visualizer = std::make_shared<Visualizer>();
    }

    void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {        
        if (positionNum > maxPieceNum) {
            positionNum = 0;
            traj.clear();
        }
        positions(0, positionNum) = msg->pose.position.x;
        positions(1, positionNum) = msg->pose.position.y;
        positions(2, positionNum) = std::fabs(msg->pose.orientation.z) * clickHeight;

        if (positionNum > 0) {
            const double dist = (positions.col(positionNum) - positions.col(positionNum - 1)).norm();
            times(positionNum - 1) = timeTrapzVel(dist, allocationSpeed, allocationAcc);
        }

        ++positionNum;
        if (positionNum > 1) {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(this->initialVel[0], this->initialVel[1], this->initialVel[2]);
            const Eigen::Vector3d initialAcc(this->initialAcc[0], this->initialAcc[1], this->initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(this->terminalVel[0], this->terminalVel[1], this->terminalVel[2]);
            const Eigen::Vector3d terminalAcc(this->terminalAcc[0], this->terminalAcc[1], this->terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);


            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);

            for (int i = 0; i < pieceNum; i++) {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }
        visualizer->visualize(traj, positions.leftCols(positionNum));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto clickGen = std::make_shared<ClickGen>();
    auto visualizer = std::make_shared<Visualizer>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(clickGen);
    executor.add_node(visualizer); 
    executor.spin();
    rclcpp::shutdown();
    return 0;
}