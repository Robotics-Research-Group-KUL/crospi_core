#include "robot_interfacing_utils/simulationrobotdriver.hpp"
#include <fmt/format.h>
#include <iostream>

namespace etasl {


SimulationRobotDriver::SimulationRobotDriver()
    // : periodicity(periodicity_val)
    // , initial_joints(init_joints)
    // , joint_pos(init_joints)
{
}

void SimulationRobotDriver::construct(std::string robot_name, 
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
                        const Json::Value& config,
                        boost::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    periodicity = jsonchecker->asDouble(config, "periodicity");

    std::vector<double> init_joints;
    for (auto n : jsonchecker->asArray(config, "initial_joints")) {
        init_joints.push_back(jsonchecker->asDouble(n, ""));
    }

    initial_joints = init_joints;
    joint_pos = initial_joints;

    feedback_ptr = fb; //defined in RobotDriver super class.
    setpoint_ptr = sp; //defined in RobotDriver super class.
    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of SimulationRobotDriver class with name: " << name << std::endl;

}

bool SimulationRobotDriver::initialize()
{
    joint_pos = initial_joints;

    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    feedback_ptr->joint.pos.data = joint_pos;
    feedback_ptr->joint.pos.is_available = true;

    //TODO: This is only for test. Remove:
    feedback_ptr->cartesian.wrench.is_available = true;

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    std::cout << "***************************Initialized with cartesian wrench available!!!!!!!!!!" << name << std::endl;


    return true;
}


void SimulationRobotDriver::update(volatile std::atomic<bool>& stopFlag)
{
    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    assert(feedback_ptr->joint.pos.data.size() == setpoint_ptr->velocity.data.size());

    for (unsigned int i=0; i<feedback_ptr->joint.pos.data.size(); ++i) {
        joint_pos[i] += setpoint_ptr->velocity.data[i]*periodicity; //simple integration
        // joint_pos[i] += setpoint_ptr->velocity.data[i]*0.0000005; //simple integration
        // joint_pos[i] += 0.00000001; //test
        feedback_ptr->joint.pos.data[i] = joint_pos[i];
    }
    //TODO: This is only for test. Remove:
    feedback_ptr->cartesian.wrench.data[0] = 10.0;
    feedback_ptr->cartesian.wrench.data[1] = 11.0;
    feedback_ptr->cartesian.wrench.data[2] = 12.0;
    feedback_ptr->cartesian.wrench.data[3] = 13.0;
    feedback_ptr->cartesian.wrench.data[4] = 14.0;
    feedback_ptr->cartesian.wrench.data[5] = 15.0;

    setpoint_ptr->velocity.fs = etasl::OldData;
    // std::cout << "vel val:" << setpoint_ptr->velocity.data[0] << " , " << setpoint_ptr->velocity.data[1] << " , "<< setpoint_ptr->velocity.data[2] << std::endl;


    // std::cout << "Driver update has set all pos values to " << feedback_ptr->joint.pos.data[0] << std::endl;
    // std::cout << "Driver update has set all pos values to " << this->periodicity << std::endl;
    // std::cout << "Driver update has set all pos values to " << getName() << std::endl;

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    std::cout << "hihihi" << std::endl;

}

void SimulationRobotDriver::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void SimulationRobotDriver::on_activate() 
{


}

void SimulationRobotDriver::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void SimulationRobotDriver::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void SimulationRobotDriver::finalize() {
    std::cout << "finalize() called =======================" << std::endl;

}



SimulationRobotDriver::~SimulationRobotDriver() {

};



} // namespace etasl


// Uncomment this if want to make it a plugin:
// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(etasl::SimulationRobotDriver, etasl::RobotDriver)
