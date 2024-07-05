#pragma once

#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"
#include <string>

// KUKA FRI SDK
#include "kukaiiwa_client.hpp"
#include <friUdpConnection.h>
#include <friClientApplication.h>


namespace etasl {

class KukaIiwaRobotDriver : public RobotDriver {
    public:
        typedef std::shared_ptr<KukaIiwaRobotDriver> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        double periodicity;
        ControlMode::ControlMode control_mode;

        std::vector<double> joint_pos;
        std::string ip_address;
        unsigned int fri_port;

        ///FRI-related
        iiwaClient         client;
        UdpConnection     connection;
        ClientApplication app;
        bool iiwa_connected;

    public:
        KukaIiwaRobotDriver(
            std::string robot_name,
            FeedbackMsg* fb,
            SetpointMsg* sp,
            std::string p_ip_address,
            unsigned int p_fri_port);

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

        // virtual const std::string& getName() const override;

        virtual ~KukaIiwaRobotDriver();
};

} // namespace etasl
