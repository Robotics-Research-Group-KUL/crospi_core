#ifndef ETASL_ROS2_SIMPLE_NODE_HPP
#define ETASL_ROS2_SIMPLE_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp" // Include Time message header


class etaslNode : public rclcpp::Node
{

    public:
        etaslNode();
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        size_t count_;
        sensor_msgs::msg::JointState joint_state_msg;
        double angle;


        void publishJointState();


};




#endif