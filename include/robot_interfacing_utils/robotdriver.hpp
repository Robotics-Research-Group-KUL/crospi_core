#pragma once

#include <memory>
#include <atomic>
#include <vector>
#include <array>
#include <chrono>
// #include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

// #include "feedback_struct.hpp"
// #include "robot_data_structures.hpp"
#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"

#include <boost/lockfree/spsc_value.hpp>
#include "robot_interfacing_utils/robot_data_structures.hpp"




// #include <expressiongraph/context.hpp>


namespace etasl {
    
    
    class RobotDriver {            

        protected:

        /**
         * @brief name of the robot
         * 
         */
        std::string name;

        typedef boost::lockfree::spsc_value< robotdrivers::DynamicJointDataField> triple_buffer_joint_type;
        // typedef boost::lockfree::spsc_value< robotdrivers::FixedJointDataField<NUM_JOINTS> > triple_buffer_joint_type;
        typedef boost::lockfree::spsc_value< robotdrivers::Vector3Field > triple_buffer_vector3_type;
        typedef boost::lockfree::spsc_value< robotdrivers::QuaternionField > triple_buffer_quaternion_type;
        typedef boost::lockfree::spsc_value< robotdrivers::ScrewField > triple_buffer_screw_type;


        
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

        AvailableFeedback available_feedback;

        struct SetpointTripleBuffers {
            std::shared_ptr<triple_buffer_joint_type> joint_vel;
        } setpoint_ports;
        
        
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

        virtual bool readSetpointJointVelocity(robotdrivers::DynamicJointDataField& sp ) noexcept{

            // std::cout << "3 im reading joint velocity setpoints" << std::endl;
            return setpoint_ports.joint_vel->read(sp);

            // #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            // buff.timestamp = std::chrono::steady_clock::now();
            //TODO: benchmark
            // #endif
        } 

        virtual void writeFeedbackJointPosition(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_pos->write(fb);}
        virtual void writeFeedbackJointVelocity(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_vel->write(fb);}
        virtual void writeFeedbackJointTorque(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_torque->write(fb);}
        virtual void writeFeedbackJointCurrent(const robotdrivers::DynamicJointDataField& fb ) noexcept{feedback_ports.joint_current->write(fb);}
        virtual void writeFeedbackCartesianPosition(const robotdrivers::Vector3Field& fb ) noexcept{feedback_ports.cartesian_pos->write(fb);}
        virtual void writeFeedbackCartesianQuaternion(const robotdrivers::QuaternionField& fb ) noexcept{feedback_ports.cartesian_quat->write(fb);}
        virtual void writeFeedbackCartesianTwist(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.cartesian_twist->write(fb);}
        virtual void writeFeedbackCartesianWrench(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.cartesian_wrench->write(fb);}
        virtual void writeFeedbackBasePosition(const robotdrivers::Vector3Field& fb ) noexcept{feedback_ports.base_pos->write(fb);}
        virtual void writeFeedbackBaseQuaternion(const robotdrivers::QuaternionField& fb ) noexcept{feedback_ports.base_quat->write(fb);}
        virtual void writeFeedbackBaseTwist(const robotdrivers::ScrewField& fb ) noexcept{feedback_ports.base_twist->write(fb);}

        

        public:
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

            bool isJointPosAvailable() {return (feedback_ports.joint_pos != nullptr);}
            bool isJointVelAvailable() {return (feedback_ports.joint_vel != nullptr);}
            bool isJointTorqueAvailable() {return (feedback_ports.joint_torque != nullptr);}
            bool isJointCurrentAvailable() {return (feedback_ports.joint_current != nullptr);}
            bool isCartesianPosAvailable() {return (feedback_ports.cartesian_pos != nullptr);}
            bool isCartesianQuatAvailable() {return (feedback_ports.cartesian_quat != nullptr);}
            bool isCartesianTwistAvailable() {return (feedback_ports.cartesian_twist != nullptr);}
            bool isCartesianWrenchAvailable() {return (feedback_ports.cartesian_wrench != nullptr);}
            bool isBasePosAvailable() {return (feedback_ports.base_pos != nullptr);}
            bool isBaseQuatAvailable() {return (feedback_ports.base_quat != nullptr);}
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

            virtual void writeSetpointJointVelocity(const robotdrivers::DynamicJointDataField& sp ) noexcept{                

                #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
                sp.timestamp = std::chrono::steady_clock::now();
                #endif

                setpoint_ports.joint_vel->write(sp);

            }

            virtual bool readFeedbackJointPosition(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointPosAvailable() && feedback_ports.joint_pos->read(fb));} 
            virtual bool readFeedbackJointVelocity(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointVelAvailable() && feedback_ports.joint_vel->read(fb));}
            virtual bool readFeedbackJointTorque(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointTorqueAvailable() && feedback_ports.joint_torque->read(fb));}
            virtual bool readFeedbackJointCurrent(robotdrivers::DynamicJointDataField& fb ) noexcept{return (isJointCurrentAvailable() && feedback_ports.joint_current->read(fb));}
            virtual bool readFeedbackCartesianPosition(robotdrivers::Vector3Field& fb ) noexcept{return (isCartesianPosAvailable() && feedback_ports.cartesian_pos->read(fb));}
            virtual bool readFeedbackCartesianQuaternion(robotdrivers::QuaternionField& fb ) noexcept{return (isCartesianQuatAvailable() && feedback_ports.cartesian_quat->read(fb));}
            virtual bool readFeedbackCartesianTwist(robotdrivers::ScrewField& fb ) noexcept{return (isCartesianTwistAvailable() && feedback_ports.cartesian_twist->read(fb));}
            virtual bool readFeedbackCartesianWrench(robotdrivers::ScrewField& fb ) noexcept{return (isCartesianWrenchAvailable() && feedback_ports.cartesian_wrench->read(fb));}
            virtual bool readFeedbackBasePosition(robotdrivers::Vector3Field& fb ) noexcept{return (isBasePosAvailable() && feedback_ports.base_pos->read(fb));}
            virtual bool readFeedbackBaseQuaternion(robotdrivers::QuaternionField& fb ) noexcept{return (isBaseQuatAvailable() && feedback_ports.base_quat->read(fb));}
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
