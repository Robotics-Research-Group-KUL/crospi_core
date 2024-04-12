#include "etasl_node_utils/topicoutputhandler.hpp"

namespace etasl {

TopicOutputHandler::TopicOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::string& _topicname,
    const std::vector<std::string>& _varnames)
    : node(_node)
    , topicname(_topicname)
    , varnames(_varnames.size())
{
    name = fmt::format("TopicOutputHandler(){}", topicname);
}

TopicOutputHandler::TopicOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::string& _topicname)
    : node(_node)
    , topicname(_topicname)
{
    name = fmt::format("TopicOutputHandler(){}", topicname);
}

const std::string& TopicOutputHandler::getName() const {
    return name;
}

void TopicOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    if (varnames.size() == 0) {
        for (auto ov : ctx->output_vars) {
            auto ptr = ctx->getOutputExpression<double>(ov.first);
            if (ptr != 0) {
                msg.names.push_back(cut_global(ov.first));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
            }
        }
        pub = node->create_publisher<etasl_interfaces::msg::Output>(topicname, 10);
    } else {
        msg.names = varnames;
        msg.data.resize(varnames.size());
        for (auto i = 0; i < varnames.size(); ++i) {
            msg.names[i] = varnames[i];
            outp[i] = ctx->getOutputExpression<double>(varnames[i]);
            if (!outp[i]) {
                not_found.push_back(varnames[i]);
                outp[i] = Constant(0.0);
            }
        }
        pub = node->create_publisher<etasl_interfaces::msg::Output>(topicname, 10);
    }
}

void TopicOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    unsigned int L = outp.size();
    for (unsigned int i = 0; i < L; ++i) {
        msg.data[i] = outp[i]->value();
    }
    pub->publish(msg);
}

} // namespace etasl