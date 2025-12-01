#pragma once

#include <memory>
#include <atomic>
// #include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

// #include "robot_interfacing_utils/feedback_struct.hpp"
#include "robot_interfacing_utils/robot_data_structures.hpp"
// #include "robot_data_structures.hpp"
#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"

#include <pluginlib/class_loader.hpp> //For plugins such as robot drivers


// For logging in ros2
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>

#include "robot_interfacing_utils/thread_manager.hpp"
#include "robot_interfacing_utils/robotdriver.hpp"


// using namespace KDL;

namespace etasl {
    using namespace KDL;
    
    class RobotDriverManager {

        private: 

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        bool simulation_;

        etasl::RobotDriver::SharedPtr               robotdriver_;
        std::shared_ptr<pluginlib::ClassLoader<etasl::RobotDriver>>  driver_loader_;

        const Json::Value parameters_;
        std::shared_ptr<etasl::JsonChecker> jsonchecker_;
        std::atomic<bool>* stopFlagPtr_;

        std::shared_ptr<robotdrivers::FeedbackMsg> feedback_shared_ptr;
        std::shared_ptr<robotdrivers::SetpointMsg> setpoint_shared_ptr;

        std::map<std::string, bool> feedback_report;

        struct InputChannelsFeedback {
            std::vector<etasl::VariableType<double>::Ptr> joint_vel;
            std::vector<etasl::VariableType<double>::Ptr> joint_torque;
            std::vector<etasl::VariableType<double>::Ptr> joint_current;
            etasl::VariableType<KDL::Vector>::Ptr cartesian_pos;
            etasl::VariableType<KDL::Rotation>::Ptr cartesian_quat;
            etasl::VariableType<KDL::Twist>::Ptr cartesian_twist;
            etasl::VariableType<KDL::Wrench>::Ptr cartesian_wrench;
            etasl::VariableType<KDL::Vector>::Ptr base_pos;
            etasl::VariableType<KDL::Rotation>::Ptr base_quat;
            etasl::VariableType<KDL::Twist>::Ptr base_twist;
        } input_channels_feedback;


        KDL::Vector vector_inp_;
        KDL::Twist twist_inp_;
        KDL::Wrench wrench_inp_;

        std::shared_ptr<t_manager::thread_t> thread_str_driver;

        // protected:
        // /**
        //  * @brief FeedbackMsg structure that stores feedback message
        //  * 
        //  */
        // FeedbackMsg feedback_struct_;

        // /**
        //  * @brief SetpointMsg structure that stores feedback message
        //  * 
        //  */
        // SetpointMsg setpoint_struct_;



        // boost::lockfree::spsc_value< std::vector<double> > inputport_joint_vel_setpoints;


        public:
            typedef std::shared_ptr<RobotDriverManager> SharedPtr;

            /**
             * @brief Construct the driver manager.
             * @param config additional configuration parameters coming from the JSON configuration
             * @param jsonchecker a pointer to the jsonchecker that can be used to check the validity of the configuration
             */
            RobotDriverManager(rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value config,
                std::shared_ptr<etasl::JsonChecker> jsonchecker,
                const bool& simulation,
                std::atomic<bool>* stopFlagPtr_p);


            void construct_driver(int num_joints);

            /**
             * @brief Initialize the communication with the Robot
             * @return true if initialized, false if not initialized, e.g. because some data is not yet available. 
             * @details can be used to initialize the driver, or can be used to only change jpos/fpos
             *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
             *           feature variables)
             */
            [[nodiscard]] bool initialize(std::shared_ptr<robotdrivers::FeedbackMsg> feedback_copy_ptr, Context::Ptr ctx);

            

            void update( std::shared_ptr<robotdrivers::FeedbackMsg> feedback_copy_ptr, const Eigen::VectorXd& jvel_etasl);


            /**
             * @brief brief doc 
             * 
             * @return * void 
             */
            void on_configure(Context::Ptr ctx);

            std::vector<double> get_position_feedback();

            /**
             * @brief on_activate
             * Handles the deactivation of the robotDriverManager
             */
            void on_activate();

            /**
             * @brief on_activate
             * Handles the deactivation of the robotDriverManager
             */
            void on_deactivate();

            /**
             * @brief on_cleanup
             * Handles the cleanup of the robot driver
             */
            void on_cleanup();

            /**
             * @brief safely finalizes the communication with the robot.
             * 
             */
            void finalize();

            std::shared_ptr<t_manager::thread_t> create_thread_str(std::atomic<bool> & stopFlag);



            /**
             * @brief Destroy the Robot Driver object
             * 
             */
            ~RobotDriverManager();

    }; // RobotDriverManager

} // namespace etasl
