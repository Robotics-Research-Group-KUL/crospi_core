
#include "etasl_task_utils/etasl_error.hpp"
#include "robot_interfacing_utils/robotdriverfactory.hpp"
#include "etasl_task_utils/registry.hpp"
#include "robot_interfacing_utils/feedback_struct.hpp"
#include <jsoncpp/json/json.h>

#include "kuka_iiwa_driver/kukaiiwa_robotdriver.hpp"

#include <iostream>

namespace etasl {

/**
 * This is a factory that can create a KukaIiwaRobotDriver
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class KukaIiwaRobotDriverFactory : public RobotDriverFactory {

    FeedbackMsg* feedback_ptr;
    SetpointMsg* setpoint_ptr; 

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;

    KukaIiwaRobotDriverFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
    :feedback_ptr(_feedback_ptr)
    ,setpoint_ptr(_setpoint_ptr)
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
                        "$id":"kukaiiwarobotdriver.json",
                        "type":"object",
                        "properties":{
                            "is-kukaiiwarobotdriver" : {
                                "description":"To indicate that the task will be executed using a kuka iiwa robot.",
                                "type":"boolean",
                                "default":true
                            },
                            "ip_address" : {
                                "description":"IP address of the robot using Ipv4 format.",
                                "type":"string",
                                "default": "xxx.xxx.xx.xx"
                            },
                            "fri_port" : {
                                "description":"FRI port number (related to communication).",
                                "type":"integer",
                                "default": 30200
                            }
                        },
                        "required":["is-kukaiiwarobotdriver","ip_address", "fri_port"],
                        "additionalProperties": false
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
        return "kukaiiwarobotdriver";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters)
    {
        std::string p_ip_address = parameters["ip_address"].asString();
        
        // get fri_port from parameters
        unsigned int p_fri_port = parameters["fri_port"].asUInt();

        // print fri_port
        std::cout << "------IP: " << p_ip_address << "  ,   fri_port: " << p_fri_port << " ,   type:" << parameters["fri_port"].isNull() << std::endl;



        std::vector<double> init_joints;
        // init_joints.resize(parameters["initial_joints"].size(), 0.0);
        for (auto n : parameters["initial_joints"]) {
            init_joints.push_back(n.asDouble());
        }
        std::string name = getName();

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        // KukaIiwaRobotDriver::KukaIiwaRobotDriver(
        //     std::string robot_name,
        //     FeedbackMsg* fb,
        //     SetpointMsg* sp,
        //     double periodicity_val,
        //     std::vector<double> init_joints)



        return std::make_shared<KukaIiwaRobotDriver>(
        name, 
        feedback_ptr, 
        setpoint_ptr, 
        p_ip_address, 
        p_fri_port);
    }

    virtual ~KukaIiwaRobotDriverFactory() { }
};

void registerKukaIiwaRobotDriverFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotDriverFactory>::registerFactory(std::make_shared<KukaIiwaRobotDriverFactory>(_feedback_ptr, _setpoint_ptr));
}

} // namespace