
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/jointstateinputhandler.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create JointStateOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class JointStateInputHandlerFactory : public InputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;

    JointStateInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
        : node(_node)
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$schema": "http://json-schema.org/draft-04/schema",
                        "$id":"jointstateinputhandler",
                        "type":"object",
                        "properties":{
                            "is-jointstateinputhandler" : {
                                "description":"To indicate that this describes a jointstate inputhandler",
                                "type":"boolean",
                                "default":true
                            },
                            "topicname":{
                                "description":"name of the topic to subscribe to",
                                "type":"string"
                            },
                            "number_of_tries" : {
                                "description":"the number of times unsuccesfully initialize() can be called before throwing an error",
                                "type":"number"
                            },
                            "depth" : {
                                "description":"Depth of the topic queue.  In case of multiple sources for the joint values, it can be useful to set a depth larger than 1",
                                "type":"number",
                                "default": 1
                            }
                        }
                    }
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "jointstateinputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual InputHandler::SharedPtr create(const Json::Value& parameters)
    {
        std::string topic_name = parameters["topic-name"].asString();
        int depth = parameters["depth"].asInt();
        int nroftries = parameters["number_of_tries"].asInt();
        return std::make_shared<JointStateInputHandler>(
            node,
            topic_name,
            nroftries,
            depth);
    }

    virtual ~JointStateInputHandlerFactory() { }
};

void registerJointStateInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<InputHandlerFactory>::registerFactory(std::make_shared<JointStateInputHandlerFactory>(_node));
}

} // namespace