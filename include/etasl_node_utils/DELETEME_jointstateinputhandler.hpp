#pragma once

#include "sensor_msgs/msg/joint_state.hpp"
#include "etasl_task_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>
#include <mutex>

namespace etasl {
using namespace KDL;

class JointStateInputHandler : public InputHandler {
public:
    typedef sensor_msgs::msg::JointState MsgType;
    typedef std::shared_ptr<JointStateInputHandler> SharedPtr;

private:
    struct BufferElement {
        double most_recent_value; // most recent value obtained during the current sample period
        bool adapted; // the inputChannel is only updated when a value was received.
    };
    std::unordered_map<std::string, BufferElement> buffer;

    std::string topicname;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<MsgType>::SharedPtr sub;
    MsgType msg;
    std::string name;
    int nroftries;
    int it;
    int depth;
    rclcpp::CallbackGroup::SharedPtr cbg;

public:
    JointStateInputHandler();

    virtual bool construct(
        std::string name,    
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& parameters,
        boost::shared_ptr<etasl::JsonChecker> jsonchecker) override;

    /**
     * will only return true if it has received values for all the joints named in jnames.
    */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos) override;

    virtual void on_new_message(const MsgType& msg);

    virtual void update(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos) override;

    virtual const std::string& getName() const override;

    virtual ~JointStateInputHandler();
};

} // namespace etasl
