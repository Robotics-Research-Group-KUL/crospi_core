#pragma once
#include "etasl_task_utils/robot.hpp"
#include "jsoncpp/json/json.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fmt/format.h>

namespace etasl {

class RobotSimulator : public Robot {
    using InputMessage = sensor_msgs::msg::JointState;
    using OutputMessage = sensor_msgs::msg::JointState;
    bool ignore_non_existing;
    std::string name;
    double time;
    double sample_time;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<InputMessage>::SharedPtr sub;
    rclcpp::Publisher<OutputMessage>::SharedPtr pub;
    OutputMessage outmsg;
    std::unordered_map<std::string, int> buffer;
    int cnt;

public:
    typedef std::shared_ptr<RobotSimulator> SharedPtr;
    rclcpp::CallbackGroup::SharedPtr cbg;

    RobotSimulator(
        const std::string& name,
        const rclcpp::NodeOptions& options,
        double _sample_time,
        const std::string& input_topicname,
        const std::string& output_topicname,
        bool _ignore_non_existing,
        const std::unordered_map<std::string, double> initial_values)
        : Robot(name, options)
        , sample_time(_sample_time)
        , ignore_non_existing(_ignore_non_existing)
        , outmsg(OutputMessage())
    {
        time = 0.0;
        cnt++;
        int i = 0;
        for (auto& [k, v] : initial_values) {
            buffer[k] = i;
            i++;
            outmsg.name.push_back(k);
            outmsg.position.push_back(v);
            outmsg.velocity.push_back(0.0);
        }
        RCLCPP_INFO(get_logger(), "Sample time : %f", sample_time);
        timer = this->create_wall_timer(
            std::chrono::duration<double>(sample_time),
            std::bind(&RobotSimulator::onTimer, this));

        sub = this->create_subscription<InputMessage>(
            input_topicname,
            rclcpp::SensorDataQoS(),
            std::bind(
                &RobotSimulator::onControlInput,
                this,
                std::placeholders::_1));
        pub = this->create_publisher<OutputMessage>(output_topicname, rclcpp::SensorDataQoS());
        //pub = this->create_publisher<OutputMessage>(output_topicname, rclcpp::QoS(10));
    }

    void onControlInput(const InputMessage::ConstSharedPtr msg)
    {
        if (msg->name.size() != msg->velocity.size()) {
            throw std::logic_error("Robotsimulator received wrongly formatted JointState message: size of names is not equal to size of velocities");
        }
        for (auto i = 0; i < msg->name.size(); ++i) {
            auto p = buffer.find(msg->name[i]);
            if ((p == buffer.end()) && (!ignore_non_existing)) {
                throw std::logic_error(fmt::format("RobotSimulator received values for a joint with an unknown name"));
            } else {
                outmsg.velocity[p->second] = msg->velocity[i];
            }
        }
    }

    void onTimer()
    {
        for (auto i = 0; i < outmsg.name.size(); ++i) {
            outmsg.position[i] += outmsg.velocity[i] * sample_time;
        }
        outmsg.header.stamp = get_clock()->now();
        pub->publish(outmsg);
        cnt++;
        // if (cnt % 200 == 0) {
        //RCLCPP_INFO(get_logger(), "onTimer()");
        //}
    }
};

} // namespace etasl
