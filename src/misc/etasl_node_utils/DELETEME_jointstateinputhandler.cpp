#include "etasl_node_utils/jointstateinputhandler.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

namespace etasl {

JointStateInputHandler::JointStateInputHandler()
{

}

bool JointStateInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);
    depth = jsonchecker->asInt(parameters, "depth");
    nroftries = jsonchecker->asInt(parameters, "number_of_tries");
}

bool JointStateInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    auto cb = std::bind(&JointStateInputHandler::on_new_message, this, _1);
    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    sub = node->create_subscription<MsgType>(topicname, qos, cb);
    // cbg = node->create_callback_group()
    // sub = node->create_subscription<MsgType>(_topic_name, rclcpp::QoS(10), cb);
    it = 0;


    assert(jnames.size() == jpos.size());
    if (it == 0) {
        buffer.clear();
        RCLCPP_INFO(node->get_logger(), "initialize first time");
        // for (auto& [k, v] : buffer) {
        //     v.adapted = false;
        // }
    } else {
        RCLCPP_INFO(node->get_logger(), "initialize %d time", it);
    }
    it++;
    if (it > nroftries) {
        throw std::logic_error(
            fmt::format("JointStateInputHandler({}) :could not receive messages specifying all inputs after trying {} times",
                name, nroftries));
    }
    bool success = true;
    for (auto i = 0; i < jnames.size(); ++i) {
        auto p = buffer.find(jnames[i]);
        if (p == buffer.end()) {
            RCLCPP_INFO(node->get_logger(), "initialize: could not find '%s'", jnames[i].c_str());
            // return false, and expect to be called again later.+-+
            success = false;
        }
    }
    if (!success) {
        return false;
    }
    it = 0;
    // buffer.clear();
    // for (auto i = 0; i < jnames.size(); ++i) {
    //     buffer[jnames[i]].adapted = false;
    //     buffer[jnames[i]].most_recent_value = 0.0;
    // }
    // RCLCPP_INFO(node->get_logger(), fmt::format("{}", fmt::join(jnames, ", ")).c_str());
    return true;
}

void JointStateInputHandler::on_new_message(const JointStateInputHandler::MsgType& msg)
{
    if (msg.name.size() != msg.velocity.size()) {
        throw std::logic_error("wrongly formatted JointState input message: size of name array and velocity array should be the same");
    }
    for (auto i = 0; i < msg.name.size(); ++i) {
        buffer[msg.name[i]].adapted = true;
        buffer[msg.name[i]].most_recent_value = msg.position[i];
        // auto p = buffer.find(msg.name[i]);
        // if (p != buffer.end()) {
        //     RCLCPP_INFO(node->get_logger(), "new message '%s' updated", p->first.c_str());
        //     p->second.adapted = true;
        //     p->second.most_recent_value = msg.position[i];
        // }
        // auto el = &buffer[msg.name[i]];
        // el->most_recent_value = msg.position[i];
        // el->adapted = true;
    }
    // if (it > 0) {
    //     for (auto i = 0; i < msg.name.size(); ++i) {
    //         buffer[msg.name[i]].adapted = true;
    //         buffer[msg.name[i]].most_recent_value = msg.position[i];
    //     }
    //     RCLCPP_INFO(node->get_logger(), "JointStateInputHandler message (velocities will be ignored afterward): ");
    //     RCLCPP_INFO(node->get_logger(), fmt::format("{}", fmt::join(msg.name.begin(), msg.name.end(), ", ")).c_str());
    //     RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(msg.position.begin(), msg.position.end(), ", ")).c_str());
    //     RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(msg.velocity.begin(), msg.velocity.end(), ", ")).c_str());
    // }
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(msg.name.begin(), msg.name.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(msg.position.begin(), msg.position.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(msg.velocity.begin(), msg.velocity.end(), ", ")).c_str());
}

void JointStateInputHandler::update(
    double time,
    const std::vector<std::string>& jnames,
    Eigen::VectorXd& jpos,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& fpos)
{
    assert(jnames.size() == buffer.size());
    assert(jnames.size() == jpos.size());
    assert(fnames.size() == fpos.size());
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
    // std::vector<bool> upd;
    // upd.resize(jnames.size());
    for (auto i = 0; i < jnames.size(); ++i) {
        // upd[i] = false;
        auto p = buffer.find(jnames[i]);
        if (p != buffer.end()) {
            if (p->second.adapted) {
                jpos[i] = p->second.most_recent_value;
                p->second.adapted = false;
                // upd[i] = true;
            }
        }
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHandler update: ");
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(jnames.begin(), jnames.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(upd.begin(), upd.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jpos.begin(), jpos.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jpos.begin(), jpos.end(), ", ")).c_str());
}

const std::string& JointStateInputHandler::getName() const
{
    return name;
}

JointStateInputHandler::~JointStateInputHandler() {

};

} // namespace etasl
