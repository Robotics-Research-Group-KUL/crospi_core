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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp" // Include Float64 message header
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp" // Include Time message header


//#include <expressiongraph/solver_factory_hqp.hpp>
#include <string>
#include <algorithm>
#include <iomanip>
#include <boost/chrono.hpp>

#include "robot_interfacing_utils/robot_data_structures.hpp"
#include "robot_interfacing_utils/thread_manager.hpp"


// #include "triple_buffer/triple_buffer_wrapper.h"

// #include "buffer_utils/alloc.h"
// #include "buffer_common/roles.h"

#include <boost/lockfree/spsc_value.hpp>
// #include <boost/lockfree/spsc_queue.hpp>


// using namespace etasl;


typedef boost::lockfree::spsc_value< robotdrivers::JointData<7> > triple_buffer_setpoints_type;

class RobotControlNode : public rclcpp::Node
{

    public:
        RobotControlNode();
        void construct( double periodicity_val, triple_buffer_setpoints_type *triple_buffer_setpoints);
        void update();
        void consume_data();
        void safe_shutdown();


    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_time_;
        double periodicity; //Expressed in seconds
        double time;
        sensor_msgs::msg::JointState joint_state_msg;
        
        std::vector< std::string > jointnames;
        

        robotdrivers::JointData<7> setpoint_local_copy;

        std::vector< double> initial_joints; //Repeated in both classes for simplicity
        std::vector< double> joint_pos;
        std::vector< double> joint_vel;

        // triple_buffer_port_t *port_setpoints;
        // // triple_buffer_port_t *port_feedback;
        // triple_buffer_error_code_t ret;
        triple_buffer_setpoints_type *triple_buffer_setpoints_;
        
        long long sum_latency;
        long long max_latency;
        long long min_latency;
        long long num_samples;

        long long sum_time_acquire_release;
        long long max_time_acquire_release;
        long long min_time_acquire_release;
        long long num_samples_acquire_release;
    };
    
//     class Producer
//     {
        
//         public:
//         Producer( double periodicity_val, triple_buffer_setpoints_type *triple_buffer_setpoints);
//         void update(volatile std::atomic<bool>& stopFlag);
//         void produce_data();
//         void finalize();
        
        
//         private:
//         double periodicity; //Expressed in seconds
//         double time;
//         std::vector< double> initial_joints; //Repeated in both classes for simplicity
//         std::vector< double> joint_positions;
//         std::vector< double> joint_velocities;
        

//         robotdrivers::JointData<7> setpoint_local_copy;
        
//         //Trajectory sine
//         double amplitude;
//         double frequency;

//         // triple_buffer_port_t *port_setpoints;
//         // triple_buffer_error_code_t ret;
//         triple_buffer_setpoints_type *triple_buffer_setpoints_;
//         // triple_buffer_port_t *port_feedback;

//         long long sum_time_acquire_release;
//         long long max_time_acquire_release;
//         long long min_time_acquire_release;
//         long long num_samples_acquire_release;


// };