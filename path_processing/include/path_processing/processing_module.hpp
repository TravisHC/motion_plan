#ifndef PATH_PROCESSING__PROCESSING_MODULE_HPP_
#define PATH_PROCESSING__PROCESSING_MODULE_HPP_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>
#include <cmath>


class ProcessingModule
{
public:
    virtual nav_msgs::msg::Path doAlgorithm(const nav_msgs::msg::Path &path) = 0;
};


#endif // PATH_PROCESSING__PROCESSING_MODULE_HPP_