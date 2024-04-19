#include "etasl_node_utils/topicoutputhandler.hpp"

namespace etasl {

TopicOutputHandler::TopicOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::string& _topicname,
    const std::vector<std::string>& _varnames)
    : node(_node)
    , topicname(_topicname)
    , varnames(_varnames)
    , initialized(false)
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
    pub = node->create_publisher<etasl_interfaces::msg::Output>(topicname, 10);
    initialized = true;
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
    if (initialized){
        pub->publish(msg);
    }
}

void TopicOutputHandler::on_activate(Context::Ptr ctx) {
    std::cout << "entering on activate =======================" << std::endl;
    // if (outp.size()>0){
    //     throw std::runtime_error("Topicoutputhandler: Before calling on_activate again, on_deactivate must be called ");
    //     return;
    // }

    if (varnames.size() == 0) {
        std::cout << "topic output hello1............................" << std::endl;
        for (auto ov : ctx->output_vars) {
            auto ptr = ctx->getOutputExpression<double>(ov.first);
            std::cout << "topic output hello4444............................" << ptr->value() << std::endl;
            if (ptr != 0) {
                msg.names.push_back(cut_global(ov.first));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
        }
    } else {
        // msg.names = varnames;
        // msg.data.resize(varnames.size());
        for (auto i = 0; i < varnames.size(); ++i) {
            auto ptr = ctx->getOutputExpression<double>(varnames[i]);
            if (ptr != 0) {
            std::cout << "topic output hello1222............................" << varnames[i] << std::endl;
                std::cout << varnames[i] << "value:" << ptr->value() << std::endl;

                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
            else{
                std::cout << "topic output hello3333............................" << varnames[i] << std::endl;
                not_found.push_back(varnames[i]);
                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(Constant(0.0));
                msg.data.push_back(0.0);
                msg.is_declared.push_back(false);
            }
        }
    }
    if (initialized){
        pub->on_activate();
    }
}

void TopicOutputHandler::on_deactivate(Context::Ptr ctx) {
    outp.clear();
    msg.data.clear();
    msg.names.clear();
    msg.is_declared.clear();
    if(initialized){
        pub->on_deactivate();
    }
}

// void TopicOutputHandler::finalize()
// {
//     pub.reset();
// }

} // namespace etasl