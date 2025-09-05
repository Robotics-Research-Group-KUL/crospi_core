#include "robot_driver_manager_lockfree.hpp"
// #include "etasl_node_utils/robot_driver_manager.hpp"


namespace etasl {

    RobotDriverManagerLockFree::RobotDriverManagerLockFree(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
        , wrench_inp_(KDL::Wrench::Zero()){
        // Constructor implementation
        

        // Json::Value robot_param = board->getPath("/robot", false);

        // Json::Value param_iohandlers = parameters["iohandlers"];

        


    }



    RobotDriverManagerLockFree::~RobotDriverManagerLockFree() {
        // Destructor implementation
    }

    void RobotDriverManagerLockFree::construct_driver(int num_joints, std::shared_ptr<pluginlib::ClassLoader<etasl::RobotDriver>>  driver_loader){

        jvel_etasl_copy.data.resize(num_joints,0.0);

        
        std::string driver_name = "";


        // robotdriver = etasl::Registry<etasl::RobotDriverFactory>::create(param["robotsimulator"],jsonchecker_);
        for (const auto& key : parameters_.getMemberNames()) {
          if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
              driver_name = key.substr(3);
          }
        }
        
        if(driver_name == ""){
          RCLCPP_ERROR(node_->get_logger(), "Could not find any is- keyword in a robotsimulator/robotdriver field of the json configuration file. It must specify the type, e.g. is-ur10_e_driver_etasl = true (for robotdrivers) or is-simple_kinematic_simulator = true (for robotsimulators)");
          auto transition = node_->shutdown(); //calls on_shutdown() hook.
          return;
        }

        std::cout << parameters_.toStyledString() << std::endl;
        if(!parameters_.isMember("robot_joints")){
          std::string message = "Parameter robot_joints is currently missing in the robotdriver/robotsimulator " + driver_name + " and therefore the robot driver cannot be constructed.";
          RCLCPP_ERROR(node_->get_logger(), message.c_str());
          auto transition = node_->shutdown(); //calls on_shutdown() hook.
          return;
        }
    
        std::cout << "hellooooooooooooooooo000000" << std::endl;

        try
        {
          robotdriver_ = driver_loader->createSharedInstance("etasl::" + driver_name);  
        }
        catch(pluginlib::PluginlibException& ex)
        {
          std::string message = "The plugin failed to load. Error: \n" + std::string(ex.what());
          RCLCPP_ERROR(node_->get_logger(), message.c_str());
          auto transition = node_->shutdown(); //calls on_shutdown() hook.
          return;
        }
        
        robotdriver_->construct(driver_name, parameters_,jsonchecker_);
        

        robotdriver_->initialize();
    }


    bool RobotDriverManagerLockFree::initialize(std::shared_ptr<robotdrivers::FeedbackMsg> feedback_copy_ptr, Context::Ptr ctx){
            

      
            feedback_copy_ptr->joint.is_pos_available = robotdriver_->isJointPosAvailable();
            feedback_copy_ptr->joint.is_vel_available = robotdriver_->isJointVelAvailable();
            feedback_copy_ptr->joint.is_torque_available = robotdriver_->isJointTorqueAvailable();
            feedback_copy_ptr->joint.is_current_available = robotdriver_->isJointCurrentAvailable();

            feedback_copy_ptr->cartesian.is_pos_available = robotdriver_->isCartesianPosAvailable();
            feedback_copy_ptr->cartesian.is_quat_available = robotdriver_->isCartesianQuatAvailable();
            feedback_copy_ptr->cartesian.is_twist_available = robotdriver_->isCartesianTwistAvailable();
            feedback_copy_ptr->cartesian.is_wrench_available = robotdriver_->isCartesianWrenchAvailable();
            feedback_copy_ptr->base.is_pos_available = robotdriver_->isBasePosAvailable();
            feedback_copy_ptr->base.is_quat_available = robotdriver_->isBaseQuatAvailable();
            feedback_copy_ptr->base.is_twist_available = robotdriver_->isBaseTwistAvailable();


      
            if (!feedback_copy_ptr->joint.is_pos_available){
              std::string message = "The position feedback is not available in the used robot driver. This is required to run eTaSL.";
              RCLCPP_ERROR(node_->get_logger(), message.c_str());
              auto transition = node_->shutdown(); //calls on_shutdown() hook.
            }
            

            robotdriver_->readFeedbackJointPosition(feedback_copy_ptr->joint.pos);
      
            // --------- Check if the requested feedback is available in the robot driver ---------------
            // Json::Value param_robot = board->getPath("/robot", false);
            // Json::Value param_robot = parameters_["robot"];

            feedback_report["joint_vel"] = jsonchecker_->is_member(parameters_, "name_expr_joint_vel") && feedback_copy_ptr->joint.is_vel_available;
            feedback_report["joint_torque"] = jsonchecker_->is_member(parameters_, "name_expr_joint_torque") && feedback_copy_ptr->joint.is_torque_available;
            feedback_report["joint_current"] = jsonchecker_->is_member(parameters_, "name_expr_joint_current") && feedback_copy_ptr->joint.is_current_available;
            feedback_report["cartesian_pos"] = jsonchecker_->is_member(parameters_, "name_expr_cartesian_pos") && feedback_copy_ptr->cartesian.is_pos_available;
            feedback_report["cartesian_quat"] = jsonchecker_->is_member(parameters_, "name_expr_cartesian_quat") && feedback_copy_ptr->cartesian.is_quat_available;
            feedback_report["cartesian_twist"] = jsonchecker_->is_member(parameters_, "name_expr_cartesian_twist") && feedback_copy_ptr->cartesian.is_twist_available;
            feedback_report["cartesian_wrench"] = jsonchecker_->is_member(parameters_, "name_expr_cartesian_wrench") && feedback_copy_ptr->cartesian.is_wrench_available;
            feedback_report["base_pos"] = jsonchecker_->is_member(parameters_, "name_expr_base_pos") && feedback_copy_ptr->base.is_pos_available;
            feedback_report["base_quat"] = jsonchecker_->is_member(parameters_, "name_expr_base_quat") && feedback_copy_ptr->base.is_quat_available;
            feedback_report["base_twist"] = jsonchecker_->is_member(parameters_, "name_expr_base_twist") && feedback_copy_ptr->base.is_twist_available;

      

            input_channels_feedback.joint_vel.clear();
            input_channels_feedback.joint_torque.clear();
            input_channels_feedback.joint_current.clear();
      
            input_channels_feedback.joint_vel.resize(feedback_copy_ptr->joint.pos.data.size(), nullptr);
            input_channels_feedback.joint_torque.resize(feedback_copy_ptr->joint.pos.data.size(),nullptr);
            input_channels_feedback.joint_current.resize(feedback_copy_ptr->joint.pos.data.size(),nullptr);



            //The following checks if the user is requesting feedback that is not available in the robot driver
            for (const auto& pair : feedback_report) {
                std::string key = pair.first;
                bool value = pair.second;
        
                if(jsonchecker_->is_member(parameters_, "name_expr_" + key)){
        
      
                  if(!value){
                    std::string message = "The requested " + key + " feedback is not available in the used robot driver. Delete the input value name_expr_"+ key +" from the setup.json file or fix the robot driver to report it.";
                    RCLCPP_ERROR(node_->get_logger(), message.c_str());
                    auto transition = node_->shutdown(); //calls on_shutdown() hook.
                    return false;
                  }
        
                //   Json::Value param_iohandlers = parameters_["iohandlers"];

                //   for (const auto& input_h : param_iohandlers["inputhandlers"]){ //Check that the user is not requesting a topic with the same name as the expression variable for the driver
                //     if(input_h["varname"].asString() == jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key)){
                //       std::string message = "The name `" + input_h["varname"].asString() + "` cannot be used within the setup.json file for both the name_expr_"+ key + " of robotdriver field and an input handler.";
                //       RCLCPP_ERROR(node_->get_logger(), message.c_str());
                //       auto transition = node_->shutdown(); //calls on_shutdown() hook.
                //       return false;
                //     } 
                //   }

                }
            }
            return true;
    }
                

    void RobotDriverManagerLockFree::update( std::shared_ptr<robotdrivers::FeedbackMsg> feedback_copy_ptr, const Eigen::VectorXd& jvel_etasl){


    
        assert(jvel_etasl_copy.data.size() == jvel_etasl.size());

        // static std::vector<float> jvel_etasl_copy(jvel_etasl.size(), 0.0);
        for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
          jvel_etasl_copy.data[i] = jvel_etasl[i];
        }
        
        robotdriver_->writeSetpointJointVelocity(jvel_etasl_copy);
    
        if (feedback_copy_ptr->joint.is_pos_available){
          robotdriver_->readFeedbackJointPosition(feedback_copy_ptr->joint.pos);
        }
        else{
          RCLCPP_ERROR(node_->get_logger(), "The position feedback is not available in the used robot driver. This is required to run eTaSL.");
          auto transition = node_->shutdown();
        }
    
    
        //Read joint velocities
        if (feedback_report["joint_vel"]){
          robotdriver_->readFeedbackJointVelocity(feedback_copy_ptr->joint.vel);
          for (unsigned int i=0; i<feedback_copy_ptr->joint.vel.data.size(); ++i) { //Check that the input channel exists (i.e. was declared in the task specification)
            if(input_channels_feedback.joint_vel[i]){      
                input_channels_feedback.joint_vel[i]->setValue(feedback_copy_ptr->joint.vel.data[i]);
            }
          }
        }
    
        //Read joint torques
        if (feedback_report["joint_torque"]){
          robotdriver_->readFeedbackJointTorque(feedback_copy_ptr->joint.torque);
        }
    
        //Read joint currents
        if (feedback_report["joint_current"]){
          robotdriver_->readFeedbackJointCurrent(feedback_copy_ptr->joint.current);
        }
    
        // -------- Cartesian feedback -----------------
    
        //Read Cartesian position
        if (feedback_report["cartesian_pos"]){
          robotdriver_->readFeedbackCartesianPosition(feedback_copy_ptr->cartesian.pos);
        }
    
        //Read Cartesian orientation in quaternion format
        if (feedback_report["cartesian_quat"]){
          robotdriver_->readFeedbackCartesianQuaternion(feedback_copy_ptr->cartesian.quat);
        }
    
        //Read Cartesian twist
        if (feedback_report["cartesian_twist"]){
          robotdriver_->readFeedbackCartesianTwist(feedback_copy_ptr->cartesian.twist);
        }
    
        //Read Cartesian wrench
        if (feedback_report["cartesian_wrench"]){
          robotdriver_->readFeedbackCartesianWrench(feedback_copy_ptr->cartesian.wrench);
        }
    
        //Read base position
        if (feedback_report["base_pos"]){
          robotdriver_->readFeedbackBasePosition(feedback_copy_ptr->base.pos);
        }
    
        //Read base orientation in quaternion format
        if (feedback_report["base_quat"]){
          robotdriver_->readFeedbackBaseQuaternion(feedback_copy_ptr->base.quat);
        }
    
        //Read base twist
        if (feedback_report["base_twist"]){
          robotdriver_->readFeedbackBaseTwist(feedback_copy_ptr->base.twist);
        }
    
    
        //TODO: Delete feedback_copy_ptr and instead write directly to input_channels_feedback pointers as below:
        // Write in the input handler the joint velocities
        // if (feedback_report["joint_vel"]){
        //     for (unsigned int i=0; i<jvel_etasl.size(); ++i) { //Check that the input channel exists (i.e. was declared in the task specification)
        //         if(input_channels_feedback.joint_vel[i]){      
        //             input_channels_feedback.joint_vel[i]->setValue(feedback_copy_ptr->joint.vel.data[i]);
        //         }
        //     }
        // }
    
        //Write in the input handler the joint torques
        if (feedback_report["joint_torque"]){
            for (unsigned int i=0; i<jvel_etasl.size(); ++i) { //Check that the input channel exists (i.e. was declared in the task specification)
                if(input_channels_feedback.joint_torque[i]){      
                    input_channels_feedback.joint_torque[i]->setValue(feedback_copy_ptr->joint.torque.data[i]);
                }
            }
        }
    
        //Write in the input handler the joint currents
        if (feedback_report["joint_current"]){
            for (unsigned int i=0; i<jvel_etasl.size(); ++i) { //Check that the input channel exists (i.e. was declared in the task specification)
                if(input_channels_feedback.joint_current[i]){      
                    input_channels_feedback.joint_current[i]->setValue(feedback_copy_ptr->joint.current.data[i]);
                }
            }
        }
    
        // -------- Cartesian feedback -----------------
    
        //Write in the input handler the Cartesian position
        if (feedback_report["cartesian_pos"]){
            if(input_channels_feedback.cartesian_pos){
                vector_inp_[0] = feedback_copy_ptr->cartesian.pos.x;
                vector_inp_[1] = feedback_copy_ptr->cartesian.pos.y;
                vector_inp_[2] = feedback_copy_ptr->cartesian.pos.z;

                input_channels_feedback.cartesian_pos->setValue(vector_inp_);
            }
        }
    
        //Write in the input handler the Cartesian orientation in quaternion format
        //TODO: Check the quaternion format in KDL and also how to transform it to rotation
        if (feedback_report["cartesian_quat"]){
            if(input_channels_feedback.cartesian_quat){
                input_channels_feedback.cartesian_quat->setValue(KDL::Rotation::Quaternion(feedback_copy_ptr->cartesian.quat.qw, feedback_copy_ptr->cartesian.quat.qx, feedback_copy_ptr->cartesian.quat.qy, feedback_copy_ptr->cartesian.quat.qz));
            }
        }
    
        //Write in the input handler the Cartesian twist
        if (feedback_report["cartesian_twist"]){
            if(input_channels_feedback.cartesian_twist){
                twist_inp_.vel[0] = feedback_copy_ptr->cartesian.twist.linear.x;
                twist_inp_.vel[1] = feedback_copy_ptr->cartesian.twist.linear.y;
                twist_inp_.vel[2] = feedback_copy_ptr->cartesian.twist.linear.z;
                twist_inp_.rot[0] = feedback_copy_ptr->cartesian.twist.angular.x;
                twist_inp_.rot[1] = feedback_copy_ptr->cartesian.twist.angular.y;
                twist_inp_.rot[2] = feedback_copy_ptr->cartesian.twist.angular.z;
                input_channels_feedback.cartesian_twist->setValue(twist_inp_);
            }
        }
    
        //Write in the input handler the Cartesian wrench
        if (feedback_report["cartesian_wrench"]){
            if(input_channels_feedback.cartesian_wrench){
                wrench_inp_.force[0] = feedback_copy_ptr->cartesian.wrench.linear.x;
                wrench_inp_.force[1] = feedback_copy_ptr->cartesian.wrench.linear.y;
                wrench_inp_.force[2] = feedback_copy_ptr->cartesian.wrench.linear.z;
                wrench_inp_.torque[0] = feedback_copy_ptr->cartesian.wrench.angular.x;
                wrench_inp_.torque[1] = feedback_copy_ptr->cartesian.wrench.angular.y;
                wrench_inp_.torque[2] = feedback_copy_ptr->cartesian.wrench.angular.z;

                input_channels_feedback.cartesian_wrench->setValue(wrench_inp_);
            }
        }
    
        //Write in the input handler the base position
        if (feedback_report["base_pos"]){
            if(input_channels_feedback.base_pos){
            vector_inp_[0] = feedback_copy_ptr->base.pos.x;
            vector_inp_[1] = feedback_copy_ptr->base.pos.y;
            vector_inp_[2] = feedback_copy_ptr->base.pos.z;
            input_channels_feedback.base_pos->setValue(vector_inp_);
            }
        }
    
        //Write in the input handler the base orientation in quaternion format
        if (feedback_report["base_quat"]){
            if(input_channels_feedback.base_quat){
            input_channels_feedback.base_quat->setValue(KDL::Rotation::Quaternion(feedback_copy_ptr->base.quat.qw, feedback_copy_ptr->base.quat.qx, feedback_copy_ptr->base.quat.qy, feedback_copy_ptr->base.quat.qz));
            }
        }
    
        //Write in the input handler the base twist
        if (feedback_report["base_twist"]){
            if(input_channels_feedback.base_twist){
                twist_inp_.vel[0] = feedback_copy_ptr->base.twist.linear.x;
                twist_inp_.vel[1] = feedback_copy_ptr->base.twist.linear.y;
                twist_inp_.vel[2] = feedback_copy_ptr->base.twist.linear.z;
                twist_inp_.rot[0] = feedback_copy_ptr->base.twist.angular.x;
                twist_inp_.rot[1] = feedback_copy_ptr->base.twist.angular.y;
                twist_inp_.rot[2] = feedback_copy_ptr->base.twist.angular.z;
                input_channels_feedback.base_twist->setValue(twist_inp_);
            }
        }


    }


    void RobotDriverManagerLockFree::on_configure(Context::Ptr ctx){


        for (const auto& pair : feedback_report) {
            std::string key = pair.first;
            bool value = pair.second;
      
            if(value){
              //Check types and get Input Channels accordingly. Joint values are vectors of pointers, and the rest are pointers. Careful!
              if (key == "joint_vel") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_vel.size(); ++i) {
                  input_channels_feedback.joint_vel[i] = ctx->getInputChannel<float>(jsonchecker_->asString(parameters_,  "name_expr_" + key) + "_" + std::to_string(i));
                }
              } 
              else if (key == "joint_torque") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_torque.size(); ++i) {
                  input_channels_feedback.joint_torque[i] = ctx->getInputChannel<float>(jsonchecker_->asString(parameters_,  "name_expr_" + key) + "_" + std::to_string(i));
                }
              }
              else if (key == "joint_current") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_current.size(); ++i) {
                  input_channels_feedback.joint_current[i] = ctx->getInputChannel<float>(jsonchecker_->asString(parameters_,  "name_expr_" + key) + "_" + std::to_string(i));
                }
              }
              else if (key == "cartesian_pos") {
                  input_channels_feedback.cartesian_pos = ctx->getInputChannel<KDL::Vector>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "cartesian_quat") {
                  input_channels_feedback.cartesian_quat = ctx->getInputChannel<KDL::Rotation>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "cartesian_twist") {
                  input_channels_feedback.cartesian_twist = ctx->getInputChannel<KDL::Twist>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "cartesian_wrench") {
                  input_channels_feedback.cartesian_wrench = ctx->getInputChannel<KDL::Wrench>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "base_pos") {
                  input_channels_feedback.base_pos = ctx->getInputChannel<KDL::Vector>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "base_quat") {
                  input_channels_feedback.base_quat = ctx->getInputChannel<KDL::Rotation>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
              else if (key == "base_twist") {
                  input_channels_feedback.base_twist = ctx->getInputChannel<KDL::Twist>(jsonchecker_->asString(parameters_,  "name_expr_" + key));
              }
            }
      
          }

    }

    std::vector<float> RobotDriverManagerLockFree::get_position_feedback(){
        robotdrivers::DynamicJointDataField jpos_current_vec;

        robotdriver_->readFeedbackJointPosition(jpos_current_vec);

        return jpos_current_vec.data;
    }

    void RobotDriverManagerLockFree::on_activate(){

    }


    void RobotDriverManagerLockFree::on_deactivate(){

    }


    void RobotDriverManagerLockFree::on_cleanup(){

    }

    void RobotDriverManagerLockFree::finalize(){

        if (robotdriver_!=nullptr){
            robotdriver_->finalize();
          }
    }

    std::shared_ptr<t_manager::thread_t> RobotDriverManagerLockFree::create_thread_str(std::atomic<bool> & stopFlag){
            
        double periodicity = jsonchecker_->asDouble(parameters_, "periodicity");
        thread_str_driver = std::make_shared<t_manager::thread_t>();
    
        
        thread_str_driver->periodicity = std::chrono::nanoseconds(static_cast<long long>(periodicity * 1E9)); //*1E9 to convert seconds to nanoseconds
        thread_str_driver->update_hook = std::bind(&etasl::RobotDriver::update, robotdriver_, std::ref(stopFlag));
        thread_str_driver->finalize_hook = std::bind(&etasl::RobotDriver::finalize, robotdriver_);
        
    
        return thread_str_driver;
    
      }



} // namespace etasl