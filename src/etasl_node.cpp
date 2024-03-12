#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp" // Include Time message header



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class etaslNode : public rclcpp::Node
{
  public:
    etaslNode()
    : Node("etasl_node"), count_(0)
    {
      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // timer_ = this->create_wall_timer(500ms, std::bind(&etaslNode::timer_callback, this));

      publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&etaslNode::publishJointState, this));
    }

  private:

    void publishJointState() {
        // auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

        
        // Populate the joint state message according to your robot configuration
        // For example:
        joint_state_msg.header.stamp = this->now(); // Set the timestamp to the current time
        joint_state_msg.name = {"joint1"};
        joint_state_msg.position = {angle};
        joint_state_msg.velocity = {0.0};
        joint_state_msg.effort = {0.0};
        angle = angle + 0.01;
        // joint_state_msg.name = {"joint1", "joint2", "joint3"};
        // joint_state_msg.position = {1.0, 2.0, 3.0};
        // joint_state_msg.velocity = {0.1, 0.2, 0.3};
        // joint_state_msg.effort = {10.0, 20.0, 30.0};

        publisher_->publish(joint_state_msg);
    }

    // void timer_callback()
    // {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    // }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    size_t count_;
    sensor_msgs::msg::JointState joint_state_msg = sensor_msgs::msg::JointState();
    double angle = 0.0;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<etaslNode>());
  rclcpp::shutdown();
  return 0;
}