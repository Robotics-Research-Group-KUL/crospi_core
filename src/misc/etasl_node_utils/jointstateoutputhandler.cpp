#include "etasl_node_utils/jointstateoutputhandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fmt/format.h>

namespace etasl {

JointStateOutputHandler::JointStateOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::string& _topicname)
    : node(_node)
    , topicname(_topicname)
    , initialized(false)
    , activated(false)
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
    // std::cout << "entering initialize JointStateOutputHandler =======================" << std::endl;

    if(!initialized){
        // pub = node->create_publisher<MsgType>(topicname, rclcpp::SensorDataQoS());
        pub = node->create_publisher<MsgType>(topicname, rclcpp::QoS(10));
        pub->on_deactivate();
        initialized = true;
        std::cout << "initialized JointStateOutputHandler=======================" << std::endl;

    }
    else{
        RCLCPP_WARN(node->get_logger(), "Ignoring request: the jointstateoutputhandler was already initialized, so it cannot be initialized again.");
    }
}

void JointStateOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
        // std::cout << "entering on update =======================" << std::endl;

    if(!activated){
        // RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be updated since it has not been initialized yet");
        return;
    }
    assert(msg.position.size() == jnames.size() /* size of msg.position and jnames vector is not the same */); 
    assert(msg.velocity.size() == jnames.size() /* size of msg.velocity and jnames vector is not the same */); 


    for (size_t i = 0; i < jnames.size(); ++i) {
        msg.position[i] = jpos[i];
        msg.velocity[i] = jvel[i];
    }
    // msg.header.set__stamp()
    // RCLCPP_INFO(node->get_logger(), "JointStateOutputHandler message: ");
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(jnames.begin(), jnames.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jpos.begin(), jpos.end(), ", ") ).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jvel.begin(), jvel.end(), ", ") ).c_str());
    // msg.header.stamp = node->get_clock()->now();
    msg.header.stamp = node->now();
    pub->publish(msg);
}


void JointStateOutputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }
    for (auto n : jnames) {
        msg.name.push_back(cut_global(n));
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
    }
    assert(msg.position.size() == jnames.size() /* size of msg.position and jnames vector is not the same */); 
    assert(msg.velocity.size() == jnames.size() /* size of msg.velocity and jnames vector is not the same */); 

    pub->on_activate();
    activated = true;
}

void JointStateOutputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;
   if(initialized){
        for (size_t i = 0; i < msg.velocity.size(); ++i) {
            msg.velocity[i] = 0.0;
        }
        msg.header.stamp = node->get_clock()->now();
        pub->publish(msg);

        pub->on_deactivate();
    }
    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    activated=false;
   
}

void JointStateOutputHandler::finalize()

{
        // std::cout << "entering on finalize =======================" << std::endl;
   if(initialized){
        for (size_t i = 0; i < msg.velocity.size(); ++i) {
            msg.velocity[i] = 0.0;
        }
        msg.header.stamp = node->get_clock()->now();
        pub->publish(msg);

        pub->on_deactivate();
    }
    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    activated=false;
}

} // namespace etasl
