#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace etasl {


void registerTopicInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node);


} // namespace