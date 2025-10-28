#pragma once

// #include "robot_interfacing_utils/feedback_struct.hpp"
#include "robot_interfacing_utils/robot_data_structures.hpp"

namespace etasl {

void registerKukaIiwaRobotDriverFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr);


} // namespace
