#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace iotb
{
struct Context
{
    std::shared_ptr<rclcpp::Node> node;
};
}  // namespace iotb