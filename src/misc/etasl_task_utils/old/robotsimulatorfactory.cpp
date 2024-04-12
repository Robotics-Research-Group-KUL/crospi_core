#include "etasl_task_utils/robotfactory.hpp"
#include "etasl_task_utils/robotsimulator.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

class RobotSimulatorFactory : public RobotFactory {

public:
    typedef std::shared_ptr<RobotSimulatorFactory> SharedPtr;

    RobotSimulatorFactory()
    {
    }

    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$schema": "http://json-schema.org/draft-04/schema",
                        "$id": "robotsimulator.json",
                        "title": "Robot Simulator",
                        "description": "Simulator class for a robot",
                        "type": "object",
                        "properties": {
                            "$schema" : {
                                "description" : "Allows us to indicate the schema in a json data file. Allows editors to support the schema",
                                "type":"string"
                            },
                            "is-robotsimulator": {
                                "description": "A key to indicate that this describes a Robot Simulator",
                                "type": "boolean"
                            },
                            "ignore-non-existing-variables": {
                                "description":"If true, non-existing variables will be ignored. If false, an exception will be thrown",
                                "type":"boolean",
                                "default":false
                            },
                            "initial_values": {
                                "title": "Initial values",
                                "description": "Defines both the names of the joint variables and their initial value",
                                "type": "object",
                                "additionalProperties": {
                                    "type": "number"
                                }
                            }
                        },
                        "additionalProperties": false,
                        "required":["is-robotsimulator","initial_values","ignore-non-existing-variables"]
                    }
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
        return std::make_shared<RobotSimulator>(parameters);
    }

    virtual ~RobotSimulatorFactory() { }
};

void registerRobotSimulatorFactory()
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotFactory>::registerFactory(std::make_shared<RobotSimulatorFactory>());
}

} // namespace