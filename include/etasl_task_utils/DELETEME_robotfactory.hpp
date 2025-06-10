#pragma once

#include "jsoncpp/json/json.h"
#include "etasl_task_utils/robot.hpp"
#include <memory>
#include <unordered_map>

#include <boost/shared_ptr.hpp>
#include "etasl_task_utils/json_checker.hpp"


namespace etasl {


class RobotFactory {

public:
    typedef std::shared_ptr<RobotFactory> SharedPtr;
    typedef Robot::SharedPtr ProductSharedPtr;


    virtual Json::Value getSchema() = 0;

    virtual const char* getName() = 0;

    virtual Robot::SharedPtr create(const Json::Value& parameters,  boost::shared_ptr<etasl::JsonChecker> jsonchecker) = 0;

    virtual ~RobotFactory() { }
};

} // etasl