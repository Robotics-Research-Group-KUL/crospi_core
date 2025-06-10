#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace etasl {

class Robot : public rclcpp::Node {
public:
    typedef std::shared_ptr<Robot> SharedPtr;

    Robot(const std::string& name, const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options)
    {
    }
    virtual ~Robot() { }
};

} // namespace etasl