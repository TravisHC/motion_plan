#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_processing/processing_module.hpp"
#include "path_processing/linear_interpolation.hpp"


class PathProcessingNode : public rclcpp::Node
{
public:
    PathProcessingNode()
        : Node("path_processing_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "input_path");
        this->declare_parameter("output_topic", "processed_path");
        this->declare_parameter("intput_vis_topic", "input_vis");
        this->declare_parameter("output_vis_topic", "output_vis");
        this->declare_parameter("method", "linear");
        this->declare_parameter("max_points", 100);
        this->declare_parameter("line_width", 0.1);
        this->declare_parameter("original_color", std::vector<double>{0.0, 1.0, 0.0});
        this->declare_parameter("processed_color", std::vector<double>{1.0, 0.0, 0.0});

        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string input_vis_topic = this->get_parameter("intput_vis_topic").as_string();
        std::string output_vis_topic = this->get_parameter("output_vis_topic").as_string();
        std::string method = this->get_parameter("method").as_string();
        int max_points = this->get_parameter("max_points").as_int();
        double line_width = this->get_parameter("line_width").as_double();
        original_color_ = this->get_parameter("original_color").as_double_array();
        processed_color_ = this->get_parameter("processed_color").as_double_array();

        // 订阅原始路径话题
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            input_topic, 10, std::bind(&PathProcessingNode::pathCallback, this, std::placeholders::_1));

        // 发布处理后的路径
        processed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_topic, 10);

        // 发布可视化标记
        input_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(input_vis_topic, 10);
        output_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(output_vis_topic, 10);

        // 初始化处理模块
        initProcessingModule(method);
    }

private:
    // using pm_ptr = std::unique_ptr<ProcessingModule>;
    void initProcessingModule(const std::string &method)
    {
        if (method == "linear")
        {
            RCLCPP_INFO(this->get_logger(), "Using linear interpolation method");
            processing_module_ = std::make_shared<LinearInterpolation>();
        }
        else if (method == "spline")
        {
            // processing_module_ = pm_ptr(new SplineInterpolation());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown method: %s", method.c_str());
            assert(false);
        }
    }
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // 调用处理模块
        auto processed_path = processing_module_->doAlgorithm(*msg);

        // 发布处理后的路径
        processed_path_pub_->publish(processed_path);

        // 可视化原始路径和处理后的路径
        visualizePaths(*msg, processed_path);
    }

    void visualizePaths(const nav_msgs::msg::Path &original_path, const nav_msgs::msg::Path &processed_path)
    {
        // visualization_msgs::msg::MarkerArray marker_array;

        // 可视化原始路径
        auto original_marker = createWayPointsMarker(original_path, "original_path", original_color_[0], original_color_[1], original_color_[2]);

        // 可视化处理后的路径
        auto processed_marker = createPathMarker(processed_path, "processed_path", processed_color_[0], processed_color_[1], processed_color_[2]);

        input_vis_pub_->publish(original_marker);
        output_vis_pub_->publish(processed_marker);

    }

    visualization_msgs::msg::Marker createPathMarker(const nav_msgs::msg::Path &path, const std::string &ns, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.pose.orientation.w = 1.00;
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = line_width_;
        marker.scale.y = line_width_;
        marker.scale.z = line_width_;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
    
        if (!path.poses.empty()) {
            for (const auto &pose : path.poses) {
                marker.points.push_back(pose.pose.position);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Path is empty, cannot create marker!");
        }
    
        return marker;
    }

    visualization_msgs::msg::Marker createWayPointsMarker(const nav_msgs::msg::Path &path, const std::string &ns, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.pose.orientation.w = 1.00;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = ns;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

    
        if (!path.poses.empty()) {
            for (const auto &pose : path.poses) {
                // marker.points.push_back(pose.pose.position);
                geometry_msgs::msg::Point point;
                point.x = pose.pose.position.x;
                point.y = pose.pose.position.y;
                point.z = pose.pose.position.z;
                marker.points.push_back(point);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Path is empty, cannot create marker!");
        }
    
        return marker;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr input_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr output_vis_pub_;
    std::shared_ptr<ProcessingModule> processing_module_;
    double line_width_;
    std::vector<double> original_color_;
    std::vector<double> processed_color_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}