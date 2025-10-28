#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/topicinputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {


/**
 * This is a factory that can create TopicOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
*/
class TopicInputHandlerFactory : public InputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;

    // TopicInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
    //     : node(_node)
    // {
    // }

    TopicInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
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
                        "$id": "topicinputhandler.json",
                        "type" : "object",
                        "title":"Parameters for TopicInputHandler",
                        "description" : "",
                        "properties" : {
                            "is-topicinputhandler" : {
                                "type":"boolean"
                            },                            
                            "topic-name" : {
                                "type":"string"
                            }
                        },
                        "required": ["is-topicinputhandler","topic-name"],
                        "additionalProperties" : false
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
        return "topicinputhandler";
    }

    /**
     * @brief create the topic with the given parameters
     *
     */
    virtual InputHandler::SharedPtr create(const Json::Value& parameters, std::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string topic_name = jsonchecker->asString(parameters, "topic-name");

        return std::make_shared<TopicInputHandler>(
            node,
            topic_name);
    }

    virtual ~TopicInputHandlerFactory() { }
};

// void registerTopicInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
// {
//     // be sure to use the BASE CLASS as template parameter for the Registry!
//     Registry<InputHandlerFactory>::registerFactory(std::make_shared<TopicInputHandlerFactory>(_node));
// }

void registerTopicInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<InputHandlerFactory>::registerFactory(std::make_shared<TopicInputHandlerFactory>(_node));
}

} // namespace