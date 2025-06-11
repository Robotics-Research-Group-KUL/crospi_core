
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/wrenchinputhandler.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create WrenchInputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class WrenchInputHandlerFactory : public InputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;

    WrenchInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
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
                        "$id":"wrenchinputhandler",
                        "type":"object",
                            "properties":{
                                "is-wrenchinputhandler" : {
                                    "description":"To indicate that this describes a wrench inputhandler which reads from a geometry_msgs/Wrench Message topic and converts it into a wrench etasl expression",
                                    "type":"boolean",
                                    "default":true
                                },
                                "topic-name" : {
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
                                },
                                "default_wrench" : { 
                                    "description": "Default wrench used in case that the topic defined in topic-name is not yet publishing",    
                                    "$ref":"wrench.json"
                                },
                                "when_unpublished" : {
                                    "description": "Defines the behavior of the node when nothing has been published to the topic defined in topic-name during activation time of the node",
                                    "enum": ["use_default", "throw_error"],
                                    "oneOf": [
                                        {
                                            "type": "string", 
                                            "const": "use_default",
                                            "title": "Use default value",
                                            "description": "Uses the defined default value in the default_wrench field while nothing has been published to the subscribed topic defined in topic-name. An INFO message will be logged."
                                        },
                                        {
                                            "type": "string", 
                                            "const": "throw_error",
                                            "title": "Throw error",
                                            "description": "Throws an error if nothing has been published to the subscribed topic defined in topic-name at the time of activation of the node, preventing its activation. "
                                        }
                                    ] 
                                },
                                "varname" : {            
                                    "description":"Name of the expression variable that was created in the task specification with createInputChannelWrench. The data coming from the topic will be available in the task specification through this variable",
                                    "type":"string"
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
        return "wrenchinputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual InputHandler::SharedPtr create(const Json::Value& parameters, std::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
         std::string topic_name = jsonchecker->asString(parameters, "topic-name");
        
        int depth = jsonchecker->asInt(parameters, "depth");
        int nroftries = jsonchecker->asInt(parameters, "number_of_tries");

        std::string when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

        std::string varname = jsonchecker->asString(parameters, "varname");


        geometry_msgs::msg::Wrench def_msg;
        def_msg.force.x = jsonchecker->asDouble(parameters, "default_wrench/force/x");
        def_msg.force.y = jsonchecker->asDouble(parameters, "default_wrench/force/y");
        def_msg.force.z = jsonchecker->asDouble(parameters, "default_wrench/force/z");
        def_msg.torque.x = jsonchecker->asDouble(parameters, "default_wrench/torque/x");
        def_msg.torque.y = jsonchecker->asDouble(parameters, "default_wrench/torque/y");
        def_msg.torque.z = jsonchecker->asDouble(parameters, "default_wrench/torque/z");

        return std::make_shared<WrenchInputHandler>(
            node,
            topic_name,
            nroftries,
            depth,
            def_msg,
            when_unpublished,
            varname);
    }

    virtual ~WrenchInputHandlerFactory() { }
};

void registerWrenchInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<InputHandlerFactory>::registerFactory(std::make_shared<WrenchInputHandlerFactory>(_node));
}

} // namespace