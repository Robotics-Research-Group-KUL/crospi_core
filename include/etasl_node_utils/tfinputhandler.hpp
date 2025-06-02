#pragma once

#include "etasl_task_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

#include "etasl_task_utils/flowstatus.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

// #include <mutex>

namespace etasl {
using namespace KDL;

class TFInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Transform MsgType;
    typedef std::shared_ptr<TFInputHandler> SharedPtr;

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

    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    int nroftries;
    MsgType default_msg;
    tf2::Duration cache_time;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    std::string target_frame;
    std::string source_frame;
    std::string varname;
    std::string when_unpublished;
    KDL::Frame frame;
    int counter;
    bool initialized;
    bool activated;
    MsgType msg;
    etasl::VariableType<KDL::Frame>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;
    geometry_msgs::msg::TransformStamped transform_stamped;


public:
    TFInputHandler(
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        int _nroftries,
        MsgType _default_msg,
        double cache_time_,
        const std::string& _when_unpublished,
        const std::string& _target_frame,
        const std::string& _source_frame,
        const std::string& _varname);

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

    virtual ~TFInputHandler();
};

} // namespace etasl
