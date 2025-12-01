#pragma once

// #include <std>

#include <vector>
#include <array>
#include <mutex>
#include <chrono>
#include <algorithm>



// #include "crospi_utils/flowstatus.hpp"

using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;

// Define the macro to include timestamps in the data structures, for example for latency measure
// #define INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE

namespace robotdrivers {
    // using namespace KDL;


    // template <typename T> 
    // struct FeedbackField { 
    //     bool is_available = false; //To indicate if the robot has that type of data available
    //     T *data; // Template of data
        
    // }; 

    struct JointDataField { 
        // bool is_available = false; //To indicate if the robot has that type of data available
        #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        SteadyTimePoint timestamp;
        #endif

        JointDataField() {            
            #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            timestamp = std::chrono::steady_clock::now();
            #endif
        }
    };


    template<std::size_t NJOINTS>
    struct FixedJointDataField: public JointDataField { 
        // bool is_available = false; //To indicate if the robot has that type of data available
        std::array<float, NJOINTS> data; // std::vector of joint values

        FixedJointDataField(): JointDataField() {
            data.fill(0.0f);
        }
    };
    
    struct DynamicJointDataField: public JointDataField { 

        std::vector<float> data; // std::vector of joint values

        DynamicJointDataField() = default;

        DynamicJointDataField(int num_of_joints): JointDataField(){
            data.resize(num_of_joints,0.0);
        }

        static DynamicJointDataField Zeros(int num_of_joints) {
            DynamicJointDataField joint_data(num_of_joints);
            // joint_data.data.resize(num_of_joints, 0.0);
            return joint_data;
        }
        
    }; 

    struct Vector3Field { 
        float x;
        float y;
        float z;
        #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        SteadyTimePoint timestamp;
        #endif

        Vector3Field(): x(0.0), y(0.0), z(0.0)
        {
            #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            timestamp = std::chrono::steady_clock::now();
            #endif
        }
    }; 

    struct QuaternionField { 
        float qx;
        float qy;
        float qz;
        float qw;
        #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        SteadyTimePoint timestamp;
        #endif
        QuaternionField(): qx(0.0), qy(0.0), qz(0.0), qw(1.0)
        {
            #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            timestamp = std::chrono::steady_clock::now();
            #endif
        }
    }; 

    struct ScrewField { 
        struct Vector3Field linear; 
        struct Vector3Field angular;
        #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        SteadyTimePoint timestamp;
        #endif 

        ScrewField(): linear(), angular()
        {
            #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            timestamp = std::chrono::steady_clock::now();
            #endif
        }
    }; 


    /** @struct FeedbackMsg
     *  @brief This structure defines the supported feedback data types coming from a robot...
     *  @var FeedbackMsg::joint_pos 
     *  Member 'joint_pos' contains the measured joint positions of the robot
     *  @var FeedbackMsg::joint_vel 
     *  Member 'joint_vel' contains the measured joint velocities of the robot
     */
    struct FeedbackMsg {

        struct JointFeedback { 
            DynamicJointDataField          pos;
            DynamicJointDataField          vel;
            DynamicJointDataField          torque;
            DynamicJointDataField          current;

            bool is_pos_available;
            bool is_vel_available;
            bool is_torque_available;
            bool is_current_available;

            JointFeedback(int num_of_joints): pos(num_of_joints)
            ,vel(num_of_joints)
            ,torque(num_of_joints)
            ,current(num_of_joints),
            is_pos_available(false),
            is_vel_available(false),
            is_torque_available(false),
            is_current_available(false)
            {}

        } joint; 

        struct CartesianFeedback { 
            Vector3Field       pos;
            QuaternionField     quat;
            ScrewField          twist;
            ScrewField          wrench;

            bool is_pos_available;
            bool is_quat_available;
            bool is_twist_available;
            bool is_wrench_available;

            CartesianFeedback(): pos()
            ,quat()
            ,twist()
            ,wrench()
            ,is_pos_available(false)
            ,is_quat_available(false)
            ,is_twist_available(false)
            ,is_wrench_available(false)
            {}

         } cartesian; 

        struct BaseFeedback { 
            Vector3Field       pos;
            QuaternionField     quat;
            ScrewField          twist;

            bool is_pos_available;
            bool is_quat_available;
            bool is_twist_available;

            BaseFeedback(): pos()
            ,quat()
            ,twist()
            ,is_pos_available(false)
            ,is_quat_available(false)
            ,is_twist_available(false)
            {}

        } base; 

        struct  flags_s{
            bool    initialized;
            bool    configured;
            bool    connected;
            bool    started;
            bool    commanding_active;

            flags_s(): 
            initialized(false), 
            configured(false), 
            connected(false), 
            started(false), 
            commanding_active(false)
            {}
        }flags;

        // #ifdef PROTECT_DATA_STRUCTURES_WITH_MUTEX
        // std::mutex mtx; // Mutex to protect the data
        // #endif


        FeedbackMsg(int num_of_joints): joint(num_of_joints), cartesian(), base(), flags()
        {}
    };


    /** @struct SetpointMsg
     *  @brief This structure defines the supported setpoint data types coming from a robot...
     */
    struct SetpointMsg {

        DynamicJointDataField velocity;

        // #ifdef PROTECT_DATA_STRUCTURES_WITH_MUTEX
        // std::mutex mtx; // Mutex to protect the data
        // #endif

        SetpointMsg(int num_of_dofs): velocity(num_of_dofs)
        {}
    };




} // namespace robotdrivers