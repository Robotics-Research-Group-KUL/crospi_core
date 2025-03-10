#pragma once

#include "robot_interfacing_utils/feedback_struct.hpp"

namespace etasl {

void register_simple_kinematic_simulation_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr);


} // namespace
