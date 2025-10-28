#pragma once

// #include "robot_interfacing_utils/robotdriver.hpp"
#include "robotdriver2.hpp"

namespace etasl {

class RobotSimulator : public RobotDriver {
public:
    virtual ~RobotSimulator() = default;
    // No new methods — inherits everything from RobotDriver
};

} // namespace etasl
