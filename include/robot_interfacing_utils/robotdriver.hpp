//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#pragma once

#include <memory>
#include <atomic>
#include <vector>
#include <array>
#include <chrono>
// #include <boost/chrono.hpp>
// #include <boost/shared_ptr.hpp>

// #include "feedback_struct.hpp"
// #include "robot_data_structures.hpp"
#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"

#include <boost/lockfree/spsc_value.hpp>
#include "robot_interfacing_utils/robot_data_structures.hpp"




// #include <expressiongraph/context.hpp>


namespace etasl {
    
    
    /**
     * @brief Abstract base class for robot drivers in the Crospi framework.
     *
     * RobotDriver defines the interface that all robot-specific driver plugins must implement.
     * It provides triple-buffered lock-free communication ports for both setpoints (commands sent
     * to the robot) and feedback (sensor data received from the robot). Derived classes must
     * implement the pure virtual methods: construct(), initialize(), and update().
     *
     * The lifecycle of a RobotDriver follows these states:
     * - construct(): Parse configuration and allocate resources. (executed in the main crospi thread)
     * - initialize(): Establish communication with the robot hardware. (executed in the main crospi thread)
     * - on_configure() / on_activate(): Can be used as callback functions at Lifecycle state transitions of the Crospi node. Normally, this is left empty. (executed in the main crospi thread)
     * - update(): Real-time control loop callback. This function executes in a separate thread (different from all other functions)
     * - on_deactivate() / on_cleanup(): Can be used as callback functions at Lifecycle state transitions of the Crospi node. Normally, this is left empty. (executed in the main crospi thread)
     * - finalize(): Safely shut down communication with robot. (executed in the main crospi thread)
     */
    class RobotDriver {

        protected:

        /**
         * @brief Name of the robot, set by the derived class during construct().
         */
        std::string name;

        /**
         * @brief Lock-free triple buffer type for dynamic joint data fields.
         * @details Uses Boost.Lockfree SPSC (Single Producer, Single Consumer) queue based on tripple state buffer
         *          to provide wait-free reads and writes between the control thread
         *          and the driver thread.
         */
        typedef boost::lockfree::spsc_value< robotdrivers::DynamicJointDataField> triple_buffer_joint_type;
        /**
         * @brief Lock-free triple buffer type for 3D vector fields (e.g., Cartesian position).
         */
        typedef boost::lockfree::spsc_value< robotdrivers::Vector3Field > triple_buffer_vector3_type;
        /**
         * @brief Lock-free triple buffer type for quaternion fields (e.g., orientation).
         */
        typedef boost::lockfree::spsc_value< robotdrivers::QuaternionField > triple_buffer_quaternion_type;
        /**
         * @brief Lock-free triple buffer type for screw fields (e.g., wrench, twist).
         */
        typedef boost::lockfree::spsc_value< robotdrivers::ScrewField > triple_buffer_screw_type;


        /**
         * @brief Collection of triple-buffered feedback ports.
         *
         * Each port is a lock-free SPSC buffer that carries sensor feedback from the robot
         * hardware to the Crospi control thread. Ports are only allocated if the corresponding
         * feedback type is flagged as available in AvailableFeedback.
         */
        struct FeedbackTripleBuffers {
            std::shared_ptr<triple_buffer_joint_type> joint_pos;
            std::shared_ptr<triple_buffer_joint_type> joint_vel;
            std::shared_ptr<triple_buffer_joint_type> joint_torque;
            std::shared_ptr<triple_buffer_joint_type> joint_current;
            std::shared_ptr<triple_buffer_vector3_type> cartesian_pos;
            std::shared_ptr<triple_buffer_quaternion_type> cartesian_quat;
            std::shared_ptr<triple_buffer_screw_type> cartesian_twist;
            std::shared_ptr<triple_buffer_screw_type> cartesian_wrench;
            std::shared_ptr<triple_buffer_vector3_type> base_pos;
            std::shared_ptr<triple_buffer_quaternion_type> base_quat;
            std::shared_ptr<triple_buffer_screw_type> base_twist;
        } feedback_ports;


        /**
         * @brief Flags indicating which feedback types are supported by the robot hardware.
         *
         * Derived classes should set the relevant booleans to true in their construct()
         * implementation before calling constructPorts(), so that only the needed buffers
         * are allocated.
         */
        struct AvailableFeedback {
            bool joint_pos = false;
            bool joint_vel= false;
            bool joint_torque = false;
            bool joint_current = false;
            bool cartesian_pos = false;
            bool cartesian_quat = false;
            bool cartesian_twist = false;
            bool cartesian_wrench = false;
            bool base_pos = false;
            bool base_quat = false;
            bool base_twist = false;
        };

        /**
         * @brief Instance of AvailableFeedback set by constructPorts().
         */
        AvailableFeedback available_feedback;

        /**
         * @brief Collection of triple-buffered setpoint ports.
         *
         * Setpoint ports carry commands from the Crospi control thread to the robot driver.
         * Currently only joint velocity setpoints (computed by Crospi) are supported.
         */
        struct SetpointTripleBuffers {
            std::shared_ptr<triple_buffer_joint_type> joint_vel;
        } setpoint_ports;
        
        /**
         * @brief Allocate and initialize the triple-buffered communication ports.
         *
         * Must be called by the derived class during construct() after setting the
         * AvailableFeedback flags. Only ports whose corresponding feedback flag is
         * true will be allocated.
         *
         * @param num_joints Number of joints of the robot (degrees of freedom).
         * @param available_fb Struct indicating which feedback channels are available.
         */
        virtual void constructPorts(const std::size_t& num_joints, AvailableFeedback available_fb) {

            available_feedback = available_fb;

            //Setpoint port
            setpoint_ports.joint_vel = std::make_shared<triple_buffer_joint_type>(robotdrivers::DynamicJointDataField(num_joints));

            //Construct join-related Feedback ports
            if (available_fb.joint_pos) {
                feedback_ports.joint_pos = std::make_shared<triple_buffer_joint_type>(robotdrivers::DynamicJointDataField(num_joints));
            }
            if( available_fb.joint_vel ) {
                feedback_ports.joint_vel = std::make_shared<triple_buffer_joint_type>(robotdrivers::DynamicJointDataField(num_joints));
            }
            if( available_fb.joint_torque ) {
                feedback_ports.joint_torque = std::make_shared<triple_buffer_joint_type>(robotdrivers::DynamicJointDataField(num_joints));
            }
            if( available_fb.joint_current ) {
                feedback_ports.joint_current = std::make_shared<triple_buffer_joint_type>(robotdrivers::DynamicJointDataField(num_joints));
            }

            //Construct Cartesian-related Feedback ports
            if( available_fb.cartesian_pos ) {
                feedback_ports.cartesian_pos = std::make_shared<triple_buffer_vector3_type>(robotdrivers::Vector3Field());
            }
            if( available_fb.cartesian_quat ) {
                feedback_ports.cartesian_quat = std::make_shared<triple_buffer_quaternion_type>(robotdrivers::QuaternionField());
            }
            if( available_fb.cartesian_twist ) {
                feedback_ports.cartesian_twist = std::make_shared<triple_buffer_screw_type>(robotdrivers::ScrewField());
            }
            if( available_fb.cartesian_wrench ) {
                feedback_ports.cartesian_wrench = std::make_shared<triple_buffer_screw_type>(robotdrivers::ScrewField());
            }

            //Construct Base-related Feedback ports

            if( available_fb.base_pos ) {
                feedback_ports.base_pos = std::make_shared<triple_buffer_vector3_type>(robotdrivers::Vector3Field());
            }
            if( available_fb.base_quat ) {
                feedback_ports.base_quat = std::make_shared<triple_buffer_quaternion_type>(robotdrivers::QuaternionField());
            }
            if( available_fb.base_twist ) {
                feedback_ports.base_twist = std::make_shared<triple_buffer_screw_type>(robotdrivers::ScrewField());
            }


        };

        // -----------Functions for this driver -------------- 

        // bool readSetpointJointVelocity(robotdrivers::FixedJointDataField<NUM_JOINTS>& sp ) noexcept{

        //     // std::cout << "3 im reading joint velocity setpoints" << std::endl;
        //     return setpoint_ports.joint_vel->read(sp);

        //     // #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        //     // buff.timestamp = std::chrono::steady_clock::now();
        //     //TODO: benchmark
        //     // #endif
        // } 

        // void writeFeedbackJointPosition(const robotdrivers::FixedJointDataField<NUM_JOINTS>& fb ) noexcept{

        //     // std::cout << "4 im writing joint position feedback" << std::endl;
        //     feedback_ports.joint_pos->write(fb);
        // }

        //Dynamic (std::vector) implementation

        /**
         * @brief Read the current joint velocity setpoint from the lock-free triple buffer.
         *
         * Called by the derived class in its update() loop to retrieve the latest
         * velocity command computed by the Crospi controller.
         *
         * @param[out] sp Reference to a DynamicJointDataField to be filled with the setpoint.
         * @return true if a new value was read, false otherwise.
         */
        virtual bool readSetpointJointVelocity(robotdrivers::DynamicJointDataField& sp ) noexcept{

            return setpoint_ports.joint_vel->read(sp);

        } 

        /**
         * @brief Write joint position feedback into the lock-free triple buffer.
         * @param fb Joint position feedback data to publish.
         */
        virtual void writeFeedbackJointPosition(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_pos->write(fb);}
        /**
         * @brief Write joint velocity feedback into the lock-free triple buffer.
         * @param fb Joint velocity feedback data to publish.
         */
        virtual void writeFeedbackJointVelocity(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_vel->write(fb);}
        /**
         * @brief Write joint torque feedback into the lock-free triple buffer.
         * @param fb Joint torque feedback data to publish.
         */
        virtual void writeFeedbackJointTorque(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_torque->write(fb);}
        /**
         * @brief Write joint current feedback into the lock-free triple buffer.
         * @param fb Joint current feedback data to publish.
         */
        virtual void writeFeedbackJointCurrent(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_current->write(fb);}
        /**
         * @brief Write Cartesian position feedback into the lock-free triple buffer.
         * @param fb Cartesian position (x, y, z) to publish.
         */
        virtual void writeFeedbackCartesianPosition(const robotdrivers::Vector3Field& fb ) noexcept{feedback_ports.cartesian_pos->write(fb);}
        /**
         * @brief Write Cartesian orientation (quaternion) feedback into the lock-free triple buffer.
         * @param fb Quaternion (w, x, y, z) to publish.
         */
        virtual void writeFeedbackCartesianQuaternion(const robotdrivers::QuaternionField& fb ) noexcept{feedback_ports.cartesian_quat->write(fb);}
        /**
         * @brief Write Cartesian twist (linear and angular velocity) feedback into the lock-free triple buffer.
         * @param fb Screw field (linear and angular components) to publish.
         */
        virtual void writeFeedbackCartesianTwist(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.cartesian_twist->write(fb);}
        /**
         * @brief Write Cartesian wrench (force and torque) feedback into the lock-free triple buffer.
         * @param fb Screw field (force and torque components) to publish.
         */
        virtual void writeFeedbackCartesianWrench(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.cartesian_wrench->write(fb);}
        /**
         * @brief Write base position feedback into the lock-free triple buffer.
         * @param fb Base position (x, y, z) to publish.
         */
        virtual void writeFeedbackBasePosition(const robotdrivers::Vector3Field& fb ) noexcept{feedback_ports.base_pos->write(fb);}
        /**
         * @brief Write base orientation (quaternion) feedback into the lock-free triple buffer.
         * @param fb Quaternion (w, x, y, z) to publish.
         */
        virtual void writeFeedbackBaseQuaternion(const robotdrivers::QuaternionField& fb ) noexcept{feedback_ports.base_quat->write(fb);}
        /**
         * @brief Write base twist (linear and angular velocity) feedback into the lock-free triple buffer.
         * @param fb Screw field (linear and angular components) to publish.
         */
        virtual void writeFeedbackBaseTwist(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.base_twist->write(fb);}

        

        public:
            /**
             * @brief Shared pointer alias for RobotDriver.
             */
            typedef std::shared_ptr<RobotDriver> SharedPtr;

            /**
             * @brief Construct the plugin. Since the constructor of a ROS2 plugin cannot have arguments, the following method is used to pass those arguments.
             * This member function should be called right after the constructor of the super class is called.
             * @param robot_name the name of the robot whose driver interfaces with   
             * @param config additional configuration parameters coming from the JSON configuration
             * @param jsonchecker a pointer to the jsonchecker that can be used to check the validity of the configuration
             * @return void
             */
            virtual void construct(std::string robot_name, 
                                    const Json::Value& config,
                                    std::shared_ptr<etasl::JsonChecker> jsonchecker) = 0;

            /**
             * @brief Initialize the communication with the Robot
             * @return true if initialized, false if not initialized, e.g. because communication with robot could not be established.
             * @details Each robot manufacturer provides its own communication protocol, and the implementation of this function is specific to each robot driver plugin. The function should establish the connection with the robot hardware and prepare it for receiving commands and sending feedback.
             *          This function is called in the main crospi thread, not in the real-time control thread. Therefore, it is safe to perform blocking operations here, such as waiting for a response from the robot or initializing communication protocols that may take some time.
             *          This function should be called after construct() and before update(). It is expected that the derived class will implement this.
             */
            [[nodiscard]] virtual bool initialize() = 0;
            
            /**
             * @brief Real-time control loop callback executed periodically by the Crospi framework.
             *
             * This function is called at the periodicity specified in the robot driver configuration
             * (e.g., every 10 ms for a 100 Hz control loop). It runs in a dedicated real-time thread
             * separate from the main Crospi thread and all other lifecycle methods.
             *
             * Each call to update() should perform one time-step of the control cycle:
             * -# Read the latest joint velocity setpoint from the triple buffer using
             *    readSetpointJointVelocity().
             * -# Send the setpoint command to the robot hardware (e.g., via RTDE, FRI, or other manufacturer-specific protocol).
             * -# Read position sensor feedback from the robot hardware and write it in the corresponding triple buffer with writeFeedbackJointPosition(...).
             * -# (optionally, if available) Read other sensor feedback from the robot hardware (velocities, wrench, etc.) and write the data into the corresponding triple buffers using the
             *    writeFeedback*() methods so the Crospi controller can consume them.
             *
             * @warning This function must NOT perform any blocking operations (e.g., network I/O
             *          that waits indefinitely, sleeps, etc). Blocking
             *          calls will disrupt the real-time control loop and cause timing jitter or missed
             *          control cycles. All communication with the robot hardware should be
             *          non-blocking.
             *
             * @param stopFlag Atomic flag monitored by the framework. When set to true, the
             *                 real-time control loop should exit cleanly at the next opportunity.
             *                 The derived implementation should not check this flag, but rather change it's value if there is an error with the robot communication, so that Crospi can handle the error and stop the control loop.
             */
            virtual void update(volatile std::atomic<bool>& stopFlag) = 0;


            /**
             * @brief Callback invoked when the Crospi node enters the configured state.
             *
             * This method is normally left empty by derived classes. It exists as a hook
             * in case custom logic needs to be executed during the Crospi node's lifecycle
             * transition to the configured state.
             *
             * @warning This method executes in the main Crospi thread, NOT in the
             *          real-time control thread. It is NOT thread-safe with respect to
             *          update(). Do NOT read or modify any variable that may be
             *          concurrently accessed by update().
             */
            virtual void on_configure() {};

            /**
             * @brief Callback invoked when the Crospi node enters the active state.
             *
             * This method is normally left empty by derived classes. It exists as a hook
             * in case custom logic needs to be executed when the Crospi node transitions
             * to the active state.
             *
             * @warning This method executes in the main Crospi thread, NOT in the
             *          real-time control thread. It is NOT thread-safe with respect to
             *          update(). Do NOT read or modify any variable that may be
             *          concurrently accessed by update().
             */
            virtual void on_activate() {};

            /**
             * @brief Callback invoked when the Crospi node exits the active state.
             *
             * This method is normally left empty by derived classes. It exists as a hook
             * in case custom logic needs to be executed when the Crospi node transitions
             * out of the active state.
             *
             * @warning This method executes in the main Crospi thread, NOT in the
             *          real-time control thread. It is NOT thread-safe with respect to
             *          update(). Do NOT read or modify any variable that may be
             *          concurrently accessed by update().
             */
            virtual void on_deactivate() {};

            /**
             * @brief Callback invoked when the Crospi node enters the cleanup state.
             *
             * This method is normally left empty by derived classes. It exists as a hook
             * in case custom logic needs to be executed during the Crospi node's cleanup
             * phase.
             *
             * @warning This method executes in the main Crospi thread, NOT in the
             *          real-time control thread. It is NOT thread-safe with respect to
             *          update(). Do NOT read or modify any variable that may be
             *          concurrently accessed by update().
             */
            virtual void on_cleanup() {};

            /**
             * @brief Safely finalize communication with the robot.
             *
             * Called when the robot driver is being destroyed, after the control loop has stopped. Override to make the robot change to e.g. an idle state, disconnect
             * from the hardware, and release all resources.
             */
            virtual void finalize() {};

            /**
             * @brief Check whether joint position feedback is available.
             * @return true if the joint position port was allocated during constructPorts().
             */
            bool isJointPosAvailable() {return (feedback_ports.joint_pos != nullptr);}
            /**
             * @brief Check whether joint velocity feedback is available.
             * @return true if the joint velocity port was allocated during constructPorts().
             */
            bool isJointVelAvailable() {return (feedback_ports.joint_vel != nullptr);}
            /**
             * @brief Check whether joint torque feedback is available.
             * @return true if the joint torque port was allocated during constructPorts().
             */
            bool isJointTorqueAvailable() {return (feedback_ports.joint_torque != nullptr);}
            /**
             * @brief Check whether joint current feedback is available.
             * @return true if the joint current port was allocated during constructPorts().
             */
            bool isJointCurrentAvailable() {return (feedback_ports.joint_current != nullptr);}
            /**
             * @brief Check whether Cartesian position feedback is available.
             * @return true if the Cartesian position port was allocated during constructPorts().
             */
            bool isCartesianPosAvailable() {return (feedback_ports.cartesian_pos != nullptr);}
            /**
             * @brief Check whether Cartesian orientation (quaternion) feedback is available.
             * @return true if the Cartesian quaternion port was allocated during constructPorts().
             */
            bool isCartesianQuatAvailable() {return (feedback_ports.cartesian_quat != nullptr);}
            /**
             * @brief Check whether Cartesian twist feedback is available.
             * @return true if the Cartesian twist port was allocated during constructPorts().
             */
            bool isCartesianTwistAvailable() {return (feedback_ports.cartesian_twist != nullptr);}
            /**
             * @brief Check whether Cartesian wrench feedback is available.
             * @return true if the Cartesian wrench port was allocated during constructPorts().
             */
            bool isCartesianWrenchAvailable() {return (feedback_ports.cartesian_wrench != nullptr);}
            /**
             * @brief Check whether base position feedback is available.
             * @return true if the base position port was allocated during constructPorts().
             */
            bool isBasePosAvailable() {return (feedback_ports.base_pos != nullptr);}
            /**
             * @brief Check whether base orientation (quaternion) feedback is available.
             * @return true if the base quaternion port was allocated during constructPorts().
             */
            bool isBaseQuatAvailable() {return (feedback_ports.base_quat != nullptr);}
            /**
             * @brief Check whether base twist feedback is available.
             * @return true if the base twist port was allocated during constructPorts().
             */
            bool isBaseTwistAvailable() {return (feedback_ports.base_twist != nullptr);}
    


            // virtual void* getSetpointJointPositionTripleBufferPtr() { return nullptr; }
            // virtual void* getSetpointJointVelocityBufferPtr() { return nullptr; }
            // virtual void* getJointPositionBufferPtr() { return nullptr; }

            // virtual void writeSetpointJointVelocity(const std::vector<float>& sp ) noexcept = 0;
            // virtual bool readFeedbackJointPosition(std::vector<float>& fb ) noexcept = 0;

            // -----------Functions for thread communicating with this driver -------------- 

            // void writeSetpointJointVelocity(const std::vector<float>& sp ) noexcept{

            //     // std::cout << "1 im writing joint velocity setpoints" << std::endl;

            //     static robotdrivers::FixedJointDataField<NUM_JOINTS> buff{};

            //     assert(sp.size() == buff.data.size());
                
            //     for (unsigned int i=0; i<buff.data.size(); ++i) {
            //         buff.data[i] = sp[i]; //copy
            //     }
                

            //     #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            //     buff.timestamp = std::chrono::steady_clock::now();
            //     #endif

            //     setpoint_ports.joint_vel->write(buff);

            // }

            // bool readFeedbackJointPosition(std::vector<float>& fb ) noexcept{
            //     // std::cout << "2 im reading joint position feedback" << std::endl;


            //     static robotdrivers::FixedJointDataField<NUM_JOINTS> joint_local_buff;
            //     bool ret = feedback_ports.joint_pos->read(joint_local_buff);

            //     assert(fb.size() == joint_local_buff.data.size());

            //     for (unsigned int i=0; i<joint_local_buff.data.size(); ++i) {
            //         fb[i] = joint_local_buff.data[i]; //copy
            //     }

            //     return ret;

            //     // #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            //     // buff.timestamp = std::chrono::steady_clock::now();
            //     //TODO: benchmark
            //     // #endif
            // } 

            /**
             * @brief Write a joint velocity setpoint into the triple buffer.
             *
             * Called by the Crospi control thread to send a velocity command to the robot driver.
             *
             * @param sp Joint velocity setpoint to publish.
             *
             * @warning This method must NEVER be called from within the derived driver class.
             *          It is intended to be called exclusively by the Crospi framework to push
             *          setpoints into the triple buffer. The driver should only read setpoints
             *          via readSetpointJointVelocity().
             */
            virtual void writeSetpointJointVelocity(const robotdrivers::DynamicJointDataField& sp ) noexcept{                

                #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
                sp.timestamp = std::chrono::steady_clock::now();
                #endif

                setpoint_ports.joint_vel->write(sp);

            }

            /**
             * @brief Read joint position feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a DynamicJointDataField to be filled with feedback data.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackJointPosition(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointPosAvailable() && feedback_ports.joint_pos->read(fb));}
            /**
             * @brief Read joint velocity feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a DynamicJointDataField to be filled with feedback data.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackJointVelocity(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointVelAvailable() && feedback_ports.joint_vel->read(fb));}
            /**
             * @brief Read joint torque feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a DynamicJointDataField to be filled with feedback data.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackJointTorque(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointTorqueAvailable() && feedback_ports.joint_torque->read(fb));}
            /**
             * @brief Read joint current feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a DynamicJointDataField to be filled with feedback data.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackJointCurrent(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointCurrentAvailable() && feedback_ports.joint_current->read(fb));}
            /**
             * @brief Read Cartesian position feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a Vector3Field to be filled with position (x, y, z).
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackCartesianPosition(robotdrivers::Vector3Field& fb ) noexcept{return (isCartesianPosAvailable() && feedback_ports.cartesian_pos->read(fb));}
            /**
             * @brief Read Cartesian orientation (quaternion) feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a QuaternionField to be filled with orientation (w, x, y, z).
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackCartesianQuaternion(robotdrivers::QuaternionField& fb ) noexcept{return (isCartesianQuatAvailable() && feedback_ports.cartesian_quat->read(fb));}
            /**
             * @brief Read Cartesian twist feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a ScrewField to be filled with linear and angular velocity.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackCartesianTwist(robotdrivers::ScrewField& fb ) noexcept{return (isCartesianTwistAvailable() && feedback_ports.cartesian_twist->read(fb));}
            /**
             * @brief Read Cartesian wrench feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a ScrewField to be filled with force and torque.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackCartesianWrench(robotdrivers::ScrewField& fb ) noexcept{return (isCartesianWrenchAvailable() && feedback_ports.cartesian_wrench->read(fb));}
            /**
             * @brief Read base position feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a Vector3Field to be filled with position (x, y, z).
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackBasePosition(robotdrivers::Vector3Field& fb ) noexcept{return (isBasePosAvailable() && feedback_ports.base_pos->read(fb));}
            /**
             * @brief Read base orientation (quaternion) feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a QuaternionField to be filled with orientation (w, x, y, z).
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackBaseQuaternion(robotdrivers::QuaternionField& fb ) noexcept{return (isBaseQuatAvailable() && feedback_ports.base_quat->read(fb));}
            /**
             * @brief Read base twist feedback from the lock-free triple buffer.
             * @param[out] fb Reference to a ScrewField to be filled with linear and angular velocity.
             * @return true if the port is available and a new value was read, false otherwise.
             */
            virtual bool readFeedbackBaseTwist(robotdrivers::ScrewField& fb ) noexcept{return (isBaseTwistAvailable() && feedback_ports.base_twist->read(fb));}


            




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
