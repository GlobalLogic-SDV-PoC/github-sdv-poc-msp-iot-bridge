#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace iotb
{
struct Context
{
    using SubscriptionPtr = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
    
    std::shared_ptr<rclcpp::Node> node;
    SubscriptionPtr iot;
};
}