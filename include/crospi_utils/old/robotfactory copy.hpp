#pragma once

#include "jsoncpp/json/json.h"
#include "crospi_utils/robot.hpp"
#include <memory>
#include <unordered_map>

namespace etasl {


class RobotFactory {

public:
    typedef std::shared_ptr<RobotFactory> SharedPtr;
    typedef Robot::SharedPtr ProductSharedPtr;


    virtual Json::Value getSchema() = 0;

    virtual const char* getName() = 0;

    virtual Robot::SharedPtr create(const Json::Value& parameters) = 0;

    virtual ~RobotFactory() { }
};

} // etasl