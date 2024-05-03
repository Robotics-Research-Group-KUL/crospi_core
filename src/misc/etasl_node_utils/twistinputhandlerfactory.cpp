
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/twistinputhandler.hpp"
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
class TwistInputHandlerFactory : public InputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;

    TwistInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
        : node(_node)
    {
    }

    // TODO: Change the SCHEMA (I kept the JointState schema provisionally)
    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$schema": "http://json-schema.org/draft-04/schema",
                        "$id":"twistinputhandler",
                        "type":"object",
                        "properties":{
                            "is-twistinputhandler" : {
                                "description":"To indicate that this describes a twist inputhandler",
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
        return "twistinputhandler";
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

        std::string when_unpublished = parameters["when_unpublished"].asString();

        std::string varname = parameters["varname"].asString();


        geometry_msgs::msg::Twist def_msg;
        def_msg.linear.x = parameters["default_twist"]["linear"]["x"].asDouble();
        def_msg.linear.y = parameters["default_twist"]["linear"]["y"].asDouble();
        def_msg.linear.z = parameters["default_twist"]["linear"]["z"].asDouble();
        def_msg.angular.x = parameters["default_twist"]["angular"]["x"].asDouble();
        def_msg.angular.y = parameters["default_twist"]["angular"]["y"].asDouble();
        def_msg.angular.z = parameters["default_twist"]["angular"]["z"].asDouble();

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        return std::make_shared<TwistInputHandler>(
            node,
            topic_name,
            nroftries,
            depth,
            def_msg,
            when_unpublished,
            varname);
    }

    virtual ~TwistInputHandlerFactory() { }
};

void registerTwistInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<InputHandlerFactory>::registerFactory(std::make_shared<TwistInputHandlerFactory>(_node));
}

} // namespace