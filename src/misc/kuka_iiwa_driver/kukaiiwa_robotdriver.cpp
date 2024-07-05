#include "kuka_iiwa_driver/kukaiiwa_robotdriver.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {

#define NUM_JOINTS 7
#define MAX_RETRIES 3000

KukaIiwaRobotDriver::KukaIiwaRobotDriver(
            std::string robot_name,
            FeedbackMsg* fb,
            SetpointMsg* sp,
            std::string p_ip_address,
            unsigned int p_fri_port)
    : periodicity(0.0),
    app(connection,client) ,
    control_mode(ControlMode::ControlMode::IDLE),
    ip_address(p_ip_address),
    fri_port(p_fri_port),
    iiwa_connected(false)
{
    feedback_ptr = fb; //defined in RobotDriver super class.
    setpoint_ptr = sp; //defined in RobotDriver super class.
    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of KukaIiwaRobotDriver class with name: " << name << std::endl;
    std::cout << "IP: " << ip_address << "  ,   fri_port: " << p_fri_port << std::endl;

}

bool KukaIiwaRobotDriver::initialize()
{
    // joint_pos = initial_joints;

    iiwa_connected = app.connect(fri_port, ip_address.c_str());
    // // TODO : Raise error if iiwa connected is false
    // // if (!iiwa_connected)
    // // {
    // //     fmt::print("Error: Could not connect to the robot\n");
    // //     return false;
    // // }

    // // TODO: Add timeout to the communication
    int num_retries = 0;
    while (client.current_session_state != MONITORING_READY)
    {
        if (num_retries > MAX_RETRIES)
        {
            fmt::print("Error: Could not connect to the robot\n");
            return false;
        }
        app.step(); //This is blocking
        client.getDiscreteState();
        client.getContinousState();
        num_retries++;
    }

    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    assert(feedback_ptr->joint.pos.data.size() == static_cast<int>(LBRState::NUMBER_OF_JOINTS));

    for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
        feedback_ptr->joint.pos.data[i] = client.meas_jnt_pos[i];
        // std::cout << "hello" << i << std::endl;
    }
    feedback_ptr->joint.pos.is_available = true;
    // Stablish communication with the robot

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    return true;
}

void KukaIiwaRobotDriver::update(volatile std::atomic<bool>& stopFlag)
{

    client.getContinousState();
	client.getDiscreteState();

    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    assert(feedback_ptr->joint.pos.data.size() == setpoint_ptr->velocity.data.size());

    if (client.current_session_state == COMMANDING_ACTIVE)
    {
        for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
            client.cmd_jnt_pos[i] += setpoint_ptr->velocity.data[i]*client.robotState().getSampleTime();
        }
    }

    for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
        feedback_ptr->joint.pos.data[i] = client.meas_jnt_pos[i];
    }
    // std::cout << "-----" << std::endl;
    // std::cout << client.cmd_jnt_pos[0] << " , " << client.cmd_jnt_pos[1] << " , "<< client.cmd_jnt_pos[2] << " , "<< std::endl;
    // std::cout << setpoint_ptr->velocity.data[0] << " , " << setpoint_ptr->velocity.data[1] << " , "<< setpoint_ptr->velocity.data[2] << " , "<< std::endl;
    setpoint_ptr->velocity.fs = etasl::OldData;

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    app.step();
    
    // print hola
}

void KukaIiwaRobotDriver::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void KukaIiwaRobotDriver::on_activate() 
{


}

void KukaIiwaRobotDriver::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void KukaIiwaRobotDriver::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void KukaIiwaRobotDriver::finalize() {
    std::cout << "finalize() called =======================" << std::endl;
    app.disconnect();

}



KukaIiwaRobotDriver::~KukaIiwaRobotDriver() {

};



} // namespace etasl
