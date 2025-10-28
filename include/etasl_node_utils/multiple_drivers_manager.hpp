#pragma once

#include <memory>
#include <atomic>
// #include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

// #include "robot_interfacing_utils/feedback_struct.hpp"
#include "robot_interfacing_utils/robot_data_structures.hpp"
// #include "robot_data_structures.hpp"
#include <jsoncpp/json/json.h>
#include "etasl_task_utils/json_checker.hpp"

#include <pluginlib/class_loader.hpp> //For plugins such as robot drivers


// For logging in ros2
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>

#include "robot_interfacing_utils/thread_manager.hpp"
// #include "robot_interfacing_utils/robotdriver.hpp"

#include "etasl_node_utils/robot_driver_manager_lockfree.hpp"



// using namespace KDL;

namespace etasl {
    using namespace KDL;
    
    class MultipleDriversManager {
        
        private: 
        
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        
        bool simulation_;
        
        // etasl::RobotDriver::SharedPtr               robotdriver_;
        std::vector<etasl::RobotDriverManagerLockFree::SharedPtr>   robotdriver_manager_vector_;
        std::vector<std::shared_ptr<t_manager::thread_t>> thread_str_drivers_vector_;
        std::shared_ptr<pluginlib::ClassLoader<etasl::RobotDriver>>  driver_loader_;
        // std::vector<std::shared_ptr<robotdrivers::FeedbackMsg>> feedback_copies_vec;
        std::vector<robotdrivers::DynamicJointDataField> joint_positions_drivers_;

        std::vector<Eigen::VectorXd> joint_velocities_setpoints_drivers_;

        const Json::Value parameters_;
        Json::Value driver_params_;
        std::shared_ptr<etasl::JsonChecker> jsonchecker_;
        std::atomic<bool>* stopFlagPtr_;
        Json::Value param_robotdrivers_;
        // std::map<std::string,int> name_ndx_; //Map of indexes of all joints in the eTaSL expression (indexes of etasl variables)
        // std::vector<std::vector<unsigned int>> separated_jindices_all_;
        std::vector<std::vector<std::string>> robot_joints_drivers_separated;
        std::vector<std::string> robot_joints;
        // std::vector<std::vector<std::string>> separated_jnames_in_expr_;
        std::vector<std::string> jnames_in_expr_;
        // std::unordered_map<std::string,int>  robot_joint_names_ndx_map;

        // std::vector<robotdrivers::DynamicJointDataField> feedback_separate_joint_pos_;

        unsigned int num_joints_in_all_drivers_;
        // robotdrivers::DynamicJointDataField& combined_joint_pos_;


        robotdrivers::DynamicJointDataField jvel_etasl_copy;

        std::map<std::string, bool> feedback_report;

        struct InputChannelsFeedback {
            std::vector<etasl::VariableType<float>::Ptr> joint_vel;
            std::vector<etasl::VariableType<float>::Ptr> joint_torque;
            std::vector<etasl::VariableType<float>::Ptr> joint_current;
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

        // std::shared_ptr<t_manager::thread_t> thread_str_driver;


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

        void demultiplexer(const Eigen::VectorXd& joint_vel_etasl, std::vector<Eigen::VectorXd>& jvel_etasl_separated_vector );

        // bool multiplexer(const std::vector<std::shared_ptr<robotdrivers::FeedbackMsg>>& separate_feedback_copies_vec, std::shared_ptr<robotdrivers::FeedbackMsg>& combined_feedback_copy_ptr);
        bool multiplexer(const std::vector<robotdrivers::DynamicJointDataField>& separate_joint_pos, std::vector<float>& combined_joint_pos);

        void update_joint_indices(const std::map<std::string,int>& name_ndx);


        public:
            typedef std::shared_ptr<MultipleDriversManager> SharedPtr;

            /**
             * @brief Construct the driver manager.
             * @param config additional configuration parameters coming from the JSON configuration
             * @param jsonchecker a pointer to the jsonchecker that can be used to check the validity of the configuration
             */
            MultipleDriversManager(rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value config,
                std::shared_ptr<etasl::JsonChecker> jsonchecker,
                const bool& simulation,
                std::atomic<bool>* stopFlagPtr_p);


            void construct_drivers(std::vector<float>& joint_positions_feedback);

            /**
             * @brief Initialize the communication with the Robot
             * @return true if initialized, false if not initialized, e.g. because some data is not yet available. 
             * @details can be used to initialize the driver, or can be used to only change jpos/fpos
             *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
             *           feature variables)
             */
            [[nodiscard]] bool initialize(Context::Ptr ctx);

            
            void update( std::vector<float>& joint_positions, const Eigen::VectorXd& jvel_etasl);


            /**
             * @brief brief doc 
             * 
             * @return * void 
             */
            void on_configure(Context::Ptr ctx);

            std::vector<float> get_position_feedback();

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

            std::vector<std::shared_ptr<t_manager::thread_t>> create_driver_threads_structures(std::atomic<bool> & stopFlag);

            std::vector<std::string> get_robot_joints();





            /**
             * @brief Destroy the Robot Driver object
             * 
             */
            ~MultipleDriversManager();

    }; // MultipleDriversManager

} // namespace etasl
