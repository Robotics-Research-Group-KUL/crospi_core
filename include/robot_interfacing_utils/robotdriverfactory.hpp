#pragma once

#include "jsoncpp/json/json.h"
#include "robot_interfacing_utils/robotdriver.hpp"
#include <memory>
#include <unordered_map>
#include "etasl_task_utils/json_checker.hpp"


namespace etasl {



/**
 * This (abstract) class describes a Factory class for RobotDriver
 * 
 * 
*/
class RobotDriverFactory {

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;
    typedef RobotDriver::SharedPtr ProductSharedPtr;
    // typedef OutputHandler::SharedPtr SharedProductPtr; // Base-class of the Product class

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema() = 0;

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName() = 0;

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<JsonChecker> jsonchecker) = 0;

    virtual ~RobotDriverFactory() { }
};

} // etasl