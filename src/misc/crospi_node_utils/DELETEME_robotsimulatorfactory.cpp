#include "crospi_utils/robotfactory.hpp"
#include "crospi_node_utils/robotsimulator.hpp"
#include "crospi_utils/etasl_error.hpp"
#include "crospi_utils/registry.hpp"
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

    virtual Robot::SharedPtr create(const Json::Value& parameters, std::shared_ptr<etasl::JsonChecker> jsonchecker) override
    {
        double sample_time = jsonchecker->asDouble(parameters, "sample_time");
        std::string name = jsonchecker->asString(parameters, "name");
        std::string input_topicname = jsonchecker->asString(parameters, "input_topicname");
        std::string output_topicname = jsonchecker->asString(parameters, "output_topicname");
        bool ignore_non_existing = jsonchecker->asBool(parameters, "ignore_non_existing");
        std::unordered_map<std::string, double> iv;
        for (auto& k : parameters["initial_values"].getMemberNames()) {
            iv[k] = jsonchecker->asDouble(parameters, "initial_values/k");
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