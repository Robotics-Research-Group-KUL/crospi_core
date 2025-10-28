#ifndef ROSLOGHANDLER_HPP
#define ROSLOGHANDLER_HPP


#include <memory>
#include "etasl_task_utils/loghandler.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace etasl {

    class RosLogHandler : public LogHandler {
        public:
            rclcpp_lifecycle::LifecycleNode::SharedPtr node;

            RosLogHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr _node);

            static LogHandler::SharedPtr create( rclcpp_lifecycle::LifecycleNode::SharedPtr node);
            /**
             * log the given string
             */
            virtual void log(Level L, const std::string& s ) override;

            virtual ~RosLogHandler() {};

    };



} // namespace etasl


#endif