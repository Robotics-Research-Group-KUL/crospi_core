#pragma once

// #include <std>

#include <vector>
#include <mutex>

#include "etasl_task_utils/flowstatus.hpp"


namespace etasl {
    // using namespace KDL;


    // template <typename T> 
    // struct FeedbackField { 
    //     bool is_available = false; //To indicate if the robot has that type of data available
    //     T *data; // Template of data
        
    // }; 


    struct JointData { 
        bool is_available = false; //To indicate if the robot has that type of data available
        std::vector<double> data; // std::vector of joint values

        JointData(int num_of_joints){
            data.resize(num_of_joints,0.0);
        }
    }; 

    struct PositionField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        double x;
        double y;
        double z;
        PositionField(): x(0.0), y(0.0), z(0.0)
        {}
    }; 

    struct QuaternionField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        double qx;
        double qy;
        double qz;
        double qw;
        QuaternionField(): qx(0.0), qy(0.0), qz(0.0), qw(1.0)
        {}
    }; 




    struct ScrewField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        struct ForceField { 
            double x;
            double y;
            double z;
            ForceField(): x(0.0), y(0.0), z(0.0)
            {}
        } linear; 
    
        struct TorqueField { 
            double x;
            double y;
            double z;
            TorqueField(): x(0.0), y(0.0), z(0.0)
            {}
        } angular; 

        ScrewField(): linear(), angular()
        {}
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
            JointData          pos;
            JointData          vel;
            JointData          torque;
            JointData          current;

            JointFeedback(int num_of_joints): pos(num_of_joints), vel(num_of_joints), torque(num_of_joints), current(num_of_joints)
            {}

        } joint; 

        struct CartesianFeedback { 
            PositionField       pos;
            QuaternionField     quat;
            ScrewField          twist;
            ScrewField          wrench;

            CartesianFeedback(): pos(), quat(), twist(), wrench()
            {}

         } cartesian; 

        struct BaseFeedback { 
            PositionField       pos;
            QuaternionField     quat;
            ScrewField          twist;

            BaseFeedback(): pos(), quat(), twist()
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

        std::mutex mtx; // Mutex to protect the data


        FeedbackMsg(int num_of_joints): joint(num_of_joints), cartesian(), base(), flags()
        {}
    };



    /** @struct SetpointMsg
     *  @brief This structure defines the supported setpoint data types coming from a robot...
     */
    struct SetpointMsg {

        struct Setpoint_s { 
            FlowStatus fs;
            std::vector<double> data; // std::vector of joint values

            Setpoint_s(int num_of_joints): fs(NoData){
                data.resize(num_of_joints,0.0);
            }
        } velocity; 

        std::mutex mtx; // Mutex to protect the data

        SetpointMsg(int num_of_dofs): velocity(num_of_dofs)
        {}
    };
    

} // namespace etasl
