#include "etasl_node_utils/jointstateoutputhandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fmt/format.h>

namespace etasl {

JointStateOutputHandler::JointStateOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::string& _topicname)
    : node(_node)
    , topicname(_topicname)
{
    name = fmt::format("JointStateOutputHandler({})", topicname);
}

const std::string& JointStateOutputHandler::getName() const
{
    return name;
}

void JointStateOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    for (auto n : jnames) {
        msg.name.push_back(cut_global(n));
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
    }
    // pub = node->create_publisher<MsgType>(topicname, rclcpp::SensorDataQoS());
    pub = node->create_publisher<MsgType>(topicname, rclcpp::QoS(10));
}

void JointStateOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    for (size_t i = 0; i < jnames.size(); ++i) {
        msg.position[i] = jpos[i];
        msg.velocity[i] = jvel[i];
    }
    // msg.header.set__stamp()
    // RCLCPP_INFO(node->get_logger(), "JointStateOutputHandler message: ");
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(jnames.begin(), jnames.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jpos.begin(), jpos.end(), ", ") ).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jvel.begin(), jvel.end(), ", ") ).c_str());
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);
}

void JointStateOutputHandler::finalize()
{
    for (size_t i = 0; i < msg.velocity.size(); ++i) {
        msg.velocity[i] = 0.0;
    }
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);
}

void JointStateOutputHandler::on_activate(Context::Ptr ctx) {
    pub->on_activate();
}

void JointStateOutputHandler::on_deactivate(Context::Ptr ctx) {
    for (size_t i = 0; i < msg.velocity.size(); ++i) {
        msg.velocity[i] = 0.0;
    }
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);

    pub->on_deactivate();
}

} // namespace etasl
