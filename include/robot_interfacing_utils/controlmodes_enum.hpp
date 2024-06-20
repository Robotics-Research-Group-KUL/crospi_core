#pragma once


namespace ControlMode {

    enum ControlMode
    {
        IDLE                = 0, //Robot stays still while reporting feedback
        JOINT_POSITION		= 1, 
        JOINT_VELOCITY		= 2,
        JOINT_IMPEDANCE		= 3,
        JOINT_TORQUE		= 4,
        CARTESIAN_IMPEDANCE	= 5,
        CARTESIAN_WRENCH	= 6,
        CARTESIAN_POSE		= 7
    };

}  // namespace ControlMode