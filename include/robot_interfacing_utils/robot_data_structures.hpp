#pragma once

// #include <std>

#include <vector>
#include <array>
#include <mutex>
#include <chrono>
#include <algorithm>



// #include "etasl_task_utils/flowstatus.hpp"

using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;

// Define the macro to include timestamps in the data structures, for example for latency measure
#define INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE

namespace robotdrivers {
    // using namespace KDL;


    // template <typename T> 
    // struct FeedbackField { 
    //     bool is_available = false; //To indicate if the robot has that type of data available
    //     T *data; // Template of data
        
    // }; 


    template<std::size_t NJOINTS>
    struct JointData { 
        // bool is_available = false; //To indicate if the robot has that type of data available
        std::array<float, NJOINTS> data; // std::vector of joint values

        #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
        SteadyTimePoint timestamp;
        #endif

        JointData() {
            data.fill(0.0f);
            
            #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
            timestamp = std::chrono::steady_clock::now();
            #endif
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
            SteadyTimePoint timestamp;
            #endif
        }
    }; 

    struct ScrewField { 
        struct Vector3Field linear; 
        struct Vector3Field angular; 

        ScrewField(): linear(), angular()
        {}
    }; 


    // /** @struct FeedbackMsg
    //  *  @brief This structure defines the supported feedback data types coming from a robot...
    //  *  @var FeedbackMsg::joint_pos 
    //  *  Member 'joint_pos' contains the measured joint positions of the robot
    //  *  @var FeedbackMsg::joint_vel 
    //  *  Member 'joint_vel' contains the measured joint velocities of the robot
    //  */
    // struct FeedbackMsg {

    //     struct JointFeedback { 
    //         JointData<NJOINTS>          pos;
    //         JointData<NJOINTS>          vel;
    //         JointData<NJOINTS>          torque;
    //         JointData<NJOINTS>          current;

    //         JointFeedback(): pos(), vel(), torque(), current()
    //         {}

    //     } joint; 

    //     struct CartesianFeedback { 
    //         PositionField       pos;
    //         QuaternionField     quat;
    //         ScrewField          twist;
    //         ScrewField          wrench;

    //         CartesianFeedback(): pos(), quat(), twist(), wrench()
    //         {}

    //      } cartesian; 

    //     struct BaseFeedback { 
    //         PositionField       pos;
    //         QuaternionField     quat;
    //         ScrewField          twist;

    //         BaseFeedback(): pos(), quat(), twist()
    //         {}

    //     } base; 

    //     struct  flags_s{
    //         bool    initialized;
    //         bool    configured;
    //         bool    connected;
    //         bool    started;
    //         bool    commanding_active;

    //         flags_s(): 
    //         initialized(false), 
    //         configured(false), 
    //         connected(false), 
    //         started(false), 
    //         commanding_active(false)
    //         {}
    //     }flags;

    //     #ifdef PROTECT_DATA_STRUCTURES_WITH_MUTEX
    //     std::mutex mtx; // Mutex to protect the data
    //     #endif


    //     FeedbackMsg(int num_of_joints): joint(), cartesian(), base(), flags()
    //     {}
    // };




} // namespace robotdrivers