#pragma once

#include "robot_interfacing_utils/robotdriver.hpp"

namespace etasl {

class RobotSimulator : public RobotDriver {
public:
    virtual ~RobotSimulator() = default;
    // No new methods — inherits everything from RobotDriver
};

} // namespace etasl
