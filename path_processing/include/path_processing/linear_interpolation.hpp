#ifndef PATH_PROCESSING__LINEAR_INTERPOLATION_HPP_
#define PATH_PROCESSING__LINEAR_INTERPOLATION_HPP_

#include "processing_module.hpp"

class LinearInterpolation : public ProcessingModule
{
private:
    double max_distance_{0.05}; // 5cm
public:
    nav_msgs::msg::Path doAlgorithm(const nav_msgs::msg::Path &path) override
    {
        nav_msgs::msg::Path processed_path;
        processed_path.header = path.header;

        // 线性插值
        for (size_t i = 0; i < path.poses.size() - 1; i++)
        {
            auto start_pose = path.poses[i];
            auto end_pose = path.poses[i + 1];

            // 计算两点之间的距离
            double dx = end_pose.pose.position.x - start_pose.pose.position.x;
            double dy = end_pose.pose.position.y - start_pose.pose.position.y;
            double dz = end_pose.pose.position.z - start_pose.pose.position.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 计算两点之间的点数
            int num_points = static_cast<int>(dist / max_distance_);

            // 线性插值
            for (int j = 0; j < num_points; j++)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = start_pose.pose.position.x + dx / dist * j * max_distance_;
                pose.pose.position.y = start_pose.pose.position.y + dy / dist * j * max_distance_;
                pose.pose.position.z = start_pose.pose.position.z + dz / dist * j * max_distance_;
                processed_path.poses.push_back(pose);
            }
        }

        // 添加最后一个点
        processed_path.poses.push_back(path.poses.back());

        return processed_path;
    }

    void setMaxDistance(double max_distance)
    {
        max_distance_ = max_distance;
    }
};

#endif // PATH_PROCESSING__LINEAR_INTERPOLATION_HPP_