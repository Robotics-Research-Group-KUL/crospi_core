#pragma once

#include <string>


#include "robotsimulator2.hpp"
// #include "robot_interfacing_utils/robotsimulator.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"


namespace robotdrivers {
    
typedef boost::lockfree::spsc_value< robotdrivers::JointData<7> > triple_buffer_joint;

class simple_kinematic_simulation : public RobotSimulator {
    public:
        typedef std::shared_ptr<simple_kinematic_simulation> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // std::string name;; Defined in super class RobotSimulator at header file robotsimulator.hpp

        float periodicity;
        ControlMode::ControlMode control_mode;

        std::array<float,7> initial_joints;
        std::array<float,7> joint_pos;
        std::array<float,7> joint_vel_setpoints;
        // std::vector<double> initial_joints;
        // std::vector<double> joint_pos;


        triple_buffer_joint triple_buffer_joint_velocity_setpoint;
        triple_buffer_joint triple_buffer_joint_position_feedback;
        

    public:
        simple_kinematic_simulation();

        virtual void construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        virtual void* getSetpointJointVelocityBufferPtr() override;
        virtual void* getJointPositionBufferPtr() override;

        // virtual const std::string& getName() const override;

        virtual ~simple_kinematic_simulation();
};

} // namespace robotdrivers
