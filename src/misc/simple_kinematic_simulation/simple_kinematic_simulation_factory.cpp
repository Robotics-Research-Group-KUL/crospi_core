
#include "etasl_task_utils/etasl_error.hpp"
#include "robot_interfacing_utils/robotdriverfactory.hpp"
#include "simple_kinematic_simulation/simple_kinematic_simulation.hpp"
#include "etasl_task_utils/registry.hpp"
#include "robot_interfacing_utils/feedback_struct.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create a simple_kinematic_simulation
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class simple_kinematic_simulation_factory : public RobotDriverFactory {

    FeedbackMsg* feedback_ptr;
    SetpointMsg* setpoint_ptr; 

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;

    simple_kinematic_simulation_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
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
                        "$schema": "http://json-schema.org/draft-06/schema",
                        "$id":"simple_kinematic_simulation",
                        "type":"object",
                        "properties":{
                            "is-simple_kinematic_simulation" : {
                                "description":"To indicate that the task will be executed in simulation, with a virtual robot that integrates joint velocities and gives back joint positions to eTaSL.",
                                "type":"boolean",
                                "default":true
                            },
                            "initial_joints" : {
                                "description":"Initial joint values in [rad] for rotational joints or [m] for prismatic joints. The size must coincide with the number of robot variables in eTaSL.",
                                "type":"array",
                                "default": [0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0]
                            },
                            "periodicity" : {
                                "description":"Periodicity in which the integration will be performed in seconds. 1/periodicity is the update frequency in Hz.",
                                "type":"number",
                                "default": 0.01
                            }
                        },
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
        return "simple_kinematic_simulation";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        double periodicity = jsonchecker->asDouble(parameters, "periodicity");

        std::vector<double> init_joints;
        // init_joints.resize(parameters["initial_joints"].size(), 0.0);
        for (auto n : jsonchecker->asArray(parameters, "initial_joints")) {
            init_joints.push_back(jsonchecker->asDouble(n, ""));
        }
 
        std::string name = getName();

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        // simple_kinematic_simulation::simple_kinematic_simulation(
        //     std::string robot_name,
        //     FeedbackMsg* fb,
        //     SetpointMsg* sp,
        //     double periodicity_val,
        //     std::vector<double> init_joints)

       auto shared_robot_driv =  std::make_shared<simple_kinematic_simulation>();

        shared_robot_driv->construct(name, feedback_ptr, setpoint_ptr, parameters, jsonchecker);

        return shared_robot_driv;
    }

    virtual ~simple_kinematic_simulation_factory() { }
};

void register_simple_kinematic_simulation_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotDriverFactory>::registerFactory(std::make_shared<simple_kinematic_simulation_factory>(_feedback_ptr, _setpoint_ptr));
}

} // namespace