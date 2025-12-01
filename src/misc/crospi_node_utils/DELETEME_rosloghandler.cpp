#include "crospi_node_utils/rosloghandler.hpp"
#include "rclcpp/rclcpp.hpp"


namespace etasl {

RosLogHandler::RosLogHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr _node) : node(_node) {
}

LogHandler::SharedPtr RosLogHandler::create(rclcpp_lifecycle::LifecycleNode::SharedPtr _node) {
    return std::make_shared<RosLogHandler>(_node);
}

void RosLogHandler::log(Level L, const std::string& s ) {
    switch(L) {
        case DEBUG:
            RCLCPP_DEBUG(node->get_logger(),s.c_str());
            break;
        case INFO:
            RCLCPP_INFO(node->get_logger(),s.c_str());
            break;
        case WARN:
            RCLCPP_WARN(node->get_logger(),s.c_str());
            break;
        case ERROR:
            RCLCPP_ERROR(node->get_logger(),s.c_str());
            break;
        case FATAL:
            RCLCPP_FATAL(node->get_logger(),s.c_str());
            break;
    }
}


}; // namespace etasl