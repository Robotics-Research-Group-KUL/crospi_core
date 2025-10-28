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

#include "robot_interfacing_utils/shared_mem_struct_mutex.hpp"
#include "robot_interfacing_utils/thread_manager.hpp"


// #include "triple_buffer/triple_buffer_wrapper.h"

// #include "buffer_utils/alloc.h"
// #include "buffer_common/roles.h"



using namespace etasl;


class ConsumerNode : public rclcpp::Node
{

    public:
        ConsumerNode();
        void construct(FeedbackMsg* fb, SetpointMsg* sp, double periodicity_val);
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
        
        FeedbackMsg* feedback_ptr;
        SetpointMsg* setpoint_ptr;

        std::vector< double> initial_joints; //Repeated in both classes for simplicity
        std::vector< double> joint_pos;
        std::vector< double> joint_vel;

        long long sum_latency;
        long long max_latency;
        long long min_latency;
        long long num_samples;

        long long sum_time_acquire_release;
        long long max_time_acquire_release;
        long long min_time_acquire_release;
        long long num_samples_acquire_release;


    };
    
    class Producer
    {
        
        public:
        Producer(FeedbackMsg* fb, SetpointMsg* sp,  double periodicity_val);
        void update(volatile std::atomic<bool>& stopFlag);
        void produce_data();
        void finalize();
        
        
        private:
        double periodicity; //Expressed in seconds
        double time;
        std::vector< double> initial_joints; //Repeated in both classes for simplicity
        std::vector< double> joint_positions;
        std::vector< double> joint_velocities;
        
        FeedbackMsg* feedback_ptr;
        SetpointMsg* setpoint_ptr;
        
        //Trajectory sine
        double amplitude;
        double frequency;

        long long sum_time_acquire_release;
        long long max_time_acquire_release;
        long long min_time_acquire_release;
        long long num_samples_acquire_release;



};