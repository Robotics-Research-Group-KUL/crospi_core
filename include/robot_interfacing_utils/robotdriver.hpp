#pragma once

#include <memory>
#include <atomic>
// #include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

#include "feedback_struct.hpp"
#include <jsoncpp/json/json.h>

// #include <expressiongraph/context.hpp>


namespace etasl {
    
    class RobotDriver {
        protected:
        /**
         * @brief Pointer to FeedbackMsg structure that stores feedback message
         * 
         */
        FeedbackMsg* feedback_ptr;

        /**
         * @brief Pointer to SetpointMsg structure that stores feedback message
         * 
         */
        SetpointMsg* setpoint_ptr;

        /**
         * @brief name of the robot
         * 
         */
        std::string name;

        

        public:
            typedef std::shared_ptr<RobotDriver> SharedPtr;

            /**
             * @brief Construct the plugin. Since the constructor of a ROS2 plugin cannot have arguments, the following method is used to pass those arguments.
             * This member function should be called right after the constructor of the super class is called.
             * @param robot_name the name of the robot whose driver interfaces with 
             * @param fb pointer to the feedback message structure where shared-memory communication occurs  
             * @param sp pointer to the setpoingt message structure where shared-memory communication occurs  
             * @param config additional configuration parameters coming from the JSON configuration
             * @return void
             */
            virtual void construct(std::string robot_name, 
                                    FeedbackMsg* fb, 
                                    SetpointMsg* sp,
                                    const Json::Value& config) = 0;

            /**
             * @brief Initialize the communication with the Robot
             * @return true if initialized, false if not initialized, e.g. because some data is not yet available. 
             * @details can be used to initialize the driver, or can be used to only change jpos/fpos
             *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
             *           feature variables)
             */
            [[nodiscard]] virtual bool initialize() = 0;
            
            /**
             * @brief update hook used to update one time-step of the control of the robot (i.e. report feedback and execute action specified through the setpoint)
             *
             * @param stopFlag atomic flag to know when to stop the real-time control loop that executes this update function 
             */
            virtual void update(volatile std::atomic<bool>& stopFlag) = 0;


            /**
             * @brief brief doc 
             * 
             * @return * void 
             */
            virtual void on_configure() {};

            /**
             * @brief on_activate
             * Handles the deactivation of the robotDriver
             */
            virtual void on_activate() {};

            /**
             * @brief on_activate
             * Handles the deactivation of the robotDriver
             */
            virtual void on_deactivate() {};

            /**
             * @brief on_cleanup
             * Handles the cleanup of the robot driver
             */
            virtual void on_cleanup() {};

            /**
             * @brief safely finalizes the communication with the robot.
             * 
             */
            virtual void finalize() {};

            /**
             * @brief Get the name of the robot
             * 
             * @return const std::string 
             */
            virtual const std::string& getName() const
            {
                return name;
            }
            // virtual const std::string& getName() const = 0;

            /**
             * @brief Destroy the Robot Driver object
             * 
             */
            virtual ~RobotDriver() {};

    }; // RobotDriver

} // namespace etasl
