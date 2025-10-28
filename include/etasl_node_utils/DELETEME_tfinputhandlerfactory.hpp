#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace etasl {


void registerTFInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node);


} // namespace