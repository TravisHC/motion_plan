#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("path_interpolation");
    auto publisher = node->create_publisher<nav_msgs::msg::Path>("/path", 10);
    auto msg = std::make_shared<nav_msgs::msg::Path>();
    msg->header.frame_id = "odom";
    msg->header.stamp = node->now();
    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        msg->poses.clear();
        for (int i = 0; i < 10; i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = i;
            pose.pose.position.y = i;
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            msg->poses.push_back(pose);
        }
        publisher->publish(*msg);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}