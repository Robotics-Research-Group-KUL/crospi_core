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

// #include <std>

#include <vector>
#include <mutex>

#include "crospi_utils/flowstatus.hpp"


namespace etasl {
    // using namespace KDL;


    // template <typename T> 
    // struct FeedbackField { 
    //     bool is_available = false; //To indicate if the robot has that type of data available
    //     T *data; // Template of data
        
    // }; 


    struct JointData { 
        bool is_available = false; //To indicate if the robot has that type of data available
        std::vector<float> data; // std::vector of joint values

        JointData(int num_of_joints){
            data.resize(num_of_joints,0.0);
        }
    }; 

    struct PositionField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        float x;
        float y;
        float z;
        PositionField(): x(0.0), y(0.0), z(0.0)
        {}
    }; 

    struct QuaternionField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        float qx;
        float qy;
        float qz;
        float qw;
        QuaternionField(): qx(0.0), qy(0.0), qz(0.0), qw(1.0)
        {}
    }; 




    struct ScrewField { 
        bool is_available = false; //To indicate if the robot has that type of data available
        struct ForceField { 
            float x;
            float y;
            float z;
            ForceField(): x(0.0), y(0.0), z(0.0)
            {}
        } linear; 
    
        struct TorqueField { 
            float x;
            float y;
            float z;
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
            std::vector<float> data; // std::vector of joint values

            Setpoint_s(int num_of_joints): fs(NoData){
                data.resize(num_of_joints,0.0);
            }
        } velocity; 

        std::mutex mtx; // Mutex to protect the data

        SetpointMsg(int num_of_dofs): velocity(num_of_dofs)
        {}
    };
    

} // namespace etasl
