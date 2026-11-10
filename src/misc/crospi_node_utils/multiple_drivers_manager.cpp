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

#include "crospi_node_utils/multiple_drivers_manager.hpp"

namespace etasl {

    MultipleDriversManager::MultipleDriversManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const Json::Value config,
        std::shared_ptr<etasl::JsonChecker> jsonchecker,
        const bool& simulation,
        std::atomic<bool>* stopFlagPtr_p): 
        node_(node)
        ,parameters_(config)
        ,jsonchecker_(jsonchecker)
        ,simulation_(simulation)
        ,stopFlagPtr_(stopFlagPtr_p)
        , vector_inp_(KDL::Vector::Zero())
        , twist_inp_(KDL::Twist::Zero())
        , wrench_inp_(KDL::Wrench::Zero())
        , num_joints_in_all_drivers_(0)
        {

        Json::Value param_robot = parameters_["robot"];

        if(simulation_){
          param_robotdrivers_ = param_robot["robotsimulators"];
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("crospi_core", "etasl::RobotSimulator"); //This works because etasl::RobotSimulator inherits from etasl::RobotDriver
        }
        else{
          param_robotdrivers_ = param_robot["robotdrivers"];
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("crospi_core", "etasl::RobotDriver");
        }

    }

    MultipleDriversManager::~MultipleDriversManager() {
    }

    void MultipleDriversManager::construct_drivers(std::vector<float>& joint_positions_feedback){

      // ------- START: initialize robot joints and indices for demultiplexer and multiplexer of robot joints for different drivers. Already considers new structure of JSON      

      robot_joints_drivers_separated.clear();

      for (const auto& driver : param_robotdrivers_){
        std::vector<std::string> robot_joint_d;
        for (auto n : jsonchecker_->asArray(driver, "robot_joints")) {
          robot_joint_d.push_back(n.asString());
        }
        robot_joints_drivers_separated.push_back(robot_joint_d);
      }

      for(unsigned int i=0; i<robot_joints_drivers_separated.size(); ++i){
        int num_joints = robot_joints_drivers_separated.at(i).size();
        Json::Value current_driver_param = param_robotdrivers_[i];

        RCLCPP_INFO(node_->get_logger(), "Constructing driver %u of %u with %d joints.", i+1, param_robotdrivers_.size(), num_joints);
        RCLCPP_DEBUG(node_->get_logger(), "Driver configuration:\n%s", current_driver_param.toStyledString().c_str());
        robotdriver_manager_vector_.push_back(std::make_shared<etasl::RobotDriverManagerLockFree>(node_, current_driver_param, jsonchecker_, simulation_, stopFlagPtr_));
        robotdriver_manager_vector_.back()->construct_driver(num_joints, driver_loader_); //constructs and initializes communication with the robot

        // feedback_copies_vec.push_back(std::make_shared<robotdrivers::FeedbackMsg>(num_joints));
        joint_positions_drivers_.push_back(robotdrivers::DynamicJointDataField(num_joints)); //Initialize with zeros
        joint_velocities_setpoints_drivers_.push_back(Eigen::VectorXd::Zero(num_joints)); //Initialize with zeros
        num_joints_in_all_drivers_ += num_joints;

      }

      joint_positions_feedback.resize(num_joints_in_all_drivers_, 0.0); //Resize the output vector that will contain the feedback of all drivers

      // Copy in order
      robot_joints.resize(num_joints_in_all_drivers_);
      int offset = 0;
      for (const auto& field : robot_joints_drivers_separated) {
          std::copy(field.begin(), field.end(), robot_joints.begin() + offset);
          offset += field.size();
      }

      // ------- END: initialize robot joints and indices for demultiplexer and multiplexer of robot joints for different drivers. Already considers new structure of JSON

    }

    bool MultipleDriversManager::initialize(Context::Ptr ctx){

      assert(robotdriver_manager_vector_.size() == joint_positions_drivers_.size());

      bool all_initialized = true;
      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        all_initialized =  all_initialized && robotdriver_manager_vector_.at(i)->initialize(ctx);
        joint_positions_drivers_.at(i).data = robotdriver_manager_vector_.at(i)->get_position_feedback(); 
      }

      return all_initialized;
    }

    void MultipleDriversManager::update( std::vector<float>& joint_positions, const Eigen::VectorXd& jvel_etasl){

      this->demultiplexer(jvel_etasl, joint_velocities_setpoints_drivers_ );

      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        robotdriver_manager_vector_[i]->update(joint_positions_drivers_[i], joint_velocities_setpoints_drivers_[i]);
      }

      if(!this->multiplexer(joint_positions_drivers_, joint_positions)){
        std::string message = "The feedback of the different robotdrivers could not be multiplexed during the update due to a missmatch in size.";
        RCLCPP_ERROR(node_->get_logger(), message.c_str());
        auto transition = node_->shutdown(); //calls on_shutdown() hook
        return;
      }

    }

    void MultipleDriversManager::on_configure(Context::Ptr ctx){

      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        robotdriver_manager_vector_.at(i)->on_configure(ctx);
      }

    }

    std::vector<float> MultipleDriversManager::get_position_feedback(){

      std::vector<float> combined_jpos_current_vec(num_joints_in_all_drivers_, 0.0);
      std::vector<robotdrivers::DynamicJointDataField> separate_jpos_current_vec;

      separate_jpos_current_vec.resize(robotdriver_manager_vector_.size());

      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        separate_jpos_current_vec.at(i).data = robotdriver_manager_vector_.at(i)->get_position_feedback();
      }

      if(!this->multiplexer(separate_jpos_current_vec, combined_jpos_current_vec)){
        std::string message = "The position feedback of the different robotdrivers could not be multiplexed during the get_position_feedback() function due to a missmatch in size.";
        RCLCPP_ERROR(node_->get_logger(), message.c_str());
        auto transition = node_->shutdown(); //calls on_shutdown() hook
      }

        return combined_jpos_current_vec;
    }

    void MultipleDriversManager::on_activate(){
      for (const auto& driver_manager : robotdriver_manager_vector_){
        driver_manager->on_activate();
      }

    }

    void MultipleDriversManager::on_deactivate(){
      for (const auto& driver_manager : robotdriver_manager_vector_){
        driver_manager->on_deactivate();
      }
    }

    void MultipleDriversManager::on_cleanup(){
      for (const auto& driver_manager : robotdriver_manager_vector_){
        driver_manager->on_cleanup();
      }
    }

    void MultipleDriversManager::finalize(){

      for (const auto& driver_manager : robotdriver_manager_vector_){
        driver_manager->finalize();
      }

    }

    std::vector<std::string> MultipleDriversManager::get_robot_joints(){
      return robot_joints;
    }

    void MultipleDriversManager::demultiplexer(const Eigen::VectorXd& joint_vel_etasl, std::vector<Eigen::VectorXd>& jvel_etasl_separated_vector ){

      // Split in order
      Eigen::Index offset = 0;
      for (auto& v : jvel_etasl_separated_vector) {
          v = joint_vel_etasl.segment(offset, v.size());
          offset += v.size();
      }

      return;
    }

    bool MultipleDriversManager::multiplexer(const std::vector<robotdrivers::DynamicJointDataField>& separate_joint_pos, std::vector<float>& joint_positions){

      // Optional: check consistency
      if (joint_positions.size() != num_joints_in_all_drivers_) {
          return false;  // mismatch
      }

      // Copy in order
      int offset = 0;
      for (const auto& field : separate_joint_pos) {
          std::copy(field.data.begin(), field.data.end(), joint_positions.begin() + offset);
          offset += field.data.size();
      }

      return true;
    }

    std::vector<std::shared_ptr<t_manager::thread_t>> MultipleDriversManager::create_driver_threads_structures(std::atomic<bool> & stopFlag){

        int number_of_drivers = 0;
        std::vector<std::shared_ptr<t_manager::thread_t>> thread_str_drivers_vector_;

        const Json::Value& default_scheduling = parameters_["robot"]["thread_scheduling"]; // null if not defined

        for (const auto& driver_manager : robotdriver_manager_vector_){
          thread_str_drivers_vector_.push_back(driver_manager->create_thread_str(stopFlag, default_scheduling));
        }

        return thread_str_drivers_vector_;

      }

} // namespace etasl