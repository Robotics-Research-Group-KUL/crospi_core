#pragma once

#include "etasl_task_utils/inputhandler.hpp"
#include "etasl_task_utils/outputhandler.hpp"

#include <expressiongraph/context.hpp>
// #include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

// #include "etasl_task_utils/flowstatus.hpp"

// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/msg/transform.hpp>
// #include "geometry_msgs/msg/twist.hpp"

#include <jsoncpp/json/json.h>
#include "etasl_task_utils/json_checker.hpp"
#include <pluginlib/class_loader.hpp> //For plugins such as robot drivers


// TODO: Remove these:
// #include "etasl_task_utils/registry.hpp"
// #include "etasl_task_utils/outputhandler.hpp"
// #include "etasl_task_utils/outputhandlerfactory.hpp"
// #include "etasl_node_utils/jointstateoutputhandlerfactory.hpp"
// #include "etasl_node_utils/topicoutputhandlerfactory.hpp"
// #include "etasl_task_utils/fileoutputhandlerfactory.hpp"
// #include "etasl_node_utils/tfoutputhandlerfactory.hpp"

namespace etasl {
// using namespace KDL;

class IOHandlerManager {
public:
    typedef std::shared_ptr<IOHandlerManager> SharedPtr;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    const Json::Value parameters;
    std::shared_ptr<etasl::JsonChecker> jsonchecker;

    std::vector<etasl::OutputHandler::SharedPtr> outputhandlers;
    std::vector<etasl::InputHandler::SharedPtr> inputhandlers;
    std::shared_ptr<pluginlib::ClassLoader<etasl::InputHandler>>  inputhandler_loader;
    std::shared_ptr<pluginlib::ClassLoader<etasl::OutputHandler>>  outputhandler_loader;

    std::vector<bool> input_h_initialized;
    std::vector<bool> otput_h_initialized;


public:
    IOHandlerManager(
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node, 
        const Json::Value _param,
        std::shared_ptr<etasl::JsonChecker> _jsonchecker);

    void construct_input_handlers();
    void construct_output_handlers();

    void initialize_input_handlers(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos);

    void initialize_output_handlers(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames);

    void configure_input_handlers(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos);

    void configure_output_handlers(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos);

    void update_input_handlers(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos);

    void update_output_handlers(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos);

    void activate_input_handlers(
        Context::Ptr ctx,    
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames);

    void activate_output_handlers(
        Context::Ptr ctx,    
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames);


    void deactivate_input_handlers(Context::Ptr ctx);
    void deactivate_output_handlers(Context::Ptr ctx);

    void cleanup_input_handlers(Context::Ptr ctx);
    void cleanup_output_handlers(Context::Ptr ctx);

    void finalize_input_handlers();
    void finalize_output_handlers();


    // ~IOHandlerManager();
};

} // namespace etasl
