#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "etasl_task_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>
#include <mutex>

namespace etasl {
using namespace KDL;

class TwistInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Twist MsgType;
    typedef std::shared_ptr<TwistInputHandler> SharedPtr;

    enum FlowStatus 
    {   NoData = 0, 
        OldData = 1, 
        NewData = 2
    };

private:
    // struct BufferElement {
    //     double most_recent_value; // most recent value obtained during the current sample period
    //     bool adapted; // the inputChannel is only updated when a value was received.
    // };
    // std::unordered_map<std::string, BufferElement> buffer;


    struct InputData {
        MsgType data;
        FlowStatus fs;
    } input_msg;

    std::string topicname;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<MsgType>::SharedPtr sub;
    MsgType msg;
    MsgType default_msg;
    KDL::Twist twist;
    std::string when_unpublished;
    bool initialized;
    bool activated;
    std::string name;
    int nroftries;
    int counter;
    int depth;
    std::string varname;
    etasl::VariableType<KDL::Twist>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;

public:
    TwistInputHandler(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const std::string& _topic_name,
        int nroftries,
        int depth,
        MsgType default_msg,
        const std::string& _when_unpublished,
        std::string varname);

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

    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames) override;


    virtual void on_deactivate(Context::Ptr ctx) override;

    virtual const std::string& getName() const override;

    void consume_data(const bool& make_old_data);

    virtual ~TwistInputHandler();
};

} // namespace etasl
