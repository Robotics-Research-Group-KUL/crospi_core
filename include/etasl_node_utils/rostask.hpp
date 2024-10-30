#ifndef ROSTASK_HPP_234DEA1
#define ROSTASK_HPP_234DEA1

#include <cstdio>

#include "etasl_task_utils/task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace etasl {

using namespace KDL;

/**
 * @brief RosTask
 * 
 * ROS-Specific part of an eTaSL Task
 */
class RosTask : public etasl::Task {
protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<RosTask> SharedPtr;
    /**
     * @brief constructs a new RosTask
     * @param node Nodes in which to run this eTaSL task.
     */
    explicit RosTask(rclcpp_lifecycle::LifecycleNode::SharedPtr node, Json::Value& param);
    //void setNode(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    //virtual void load(const std::string& filename) override;
    //virtual void initialize() override;
    //virtual void startLoop() override;
    //virtual bool onTimer() override;
};

}; // namespace etasl
#endif
