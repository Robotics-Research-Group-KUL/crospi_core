#include "etasl_task_utils/robotfactory.hpp"
#include "etasl_node_utils/robotsimulator.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

class RobotSimulatorFactory : public RobotFactory {
    const rclcpp::NodeOptions& options;

public:
    typedef std::shared_ptr<RobotSimulatorFactory> SharedPtr;

    RobotSimulatorFactory(const rclcpp::NodeOptions& _options)
        : options(_options)
    {
    }

    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    virtual const char* getName()
    {
        return "robotsimulator";
    }

    virtual Robot::SharedPtr create(const Json::Value& parameters) override
    {
        double sample_time = parameters["sample_time"].asDouble();
        std::string name = parameters["name"].asString();
        std::string input_topicname = parameters["input_topicname"].asString();
        std::string output_topicname = parameters["output_topicname"].asString();
        bool ignore_non_existing = parameters["ignore_non_existing"].asBool();
        std::unordered_map<std::string, double> iv;
        for (auto& k : parameters["initial_values"].getMemberNames()) {
            iv[k] = parameters["initial_values"][k].asDouble();
        }

        return std::make_shared<RobotSimulator>(name, options, sample_time, input_topicname, output_topicname, ignore_non_existing,iv);
    }

    virtual ~RobotSimulatorFactory() { }
};

void registerRobotSimulatorFactory(const rclcpp::NodeOptions& options)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotFactory>::registerFactory(std::make_shared<RobotSimulatorFactory>(options));
}

} // namespace