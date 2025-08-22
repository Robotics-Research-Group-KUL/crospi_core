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

    void RobotDriverManagerLockFree::construct_driver(int num_joints){

        feedback_shared_ptr = std::make_shared<etasl::FeedbackMsg>(num_joints);
        setpoint_shared_ptr = std::make_shared<etasl::SetpointMsg>(num_joints);

        std::string driver_name = "";
        std::string robotplugintype = "";
        
        if (simulation_){
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("etasl_ros2", "etasl::RobotSimulator"); //This works because etasl::RobotSimulator inherits from etasl::RobotDriver
          robotplugintype = "robotsimulator";
        }
        else{
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("etasl_ros2", "etasl::RobotDriver");
          robotplugintype = "robotdriver";
        }
        Json::Value robot_params = parameters_["robot"];
        Json::Value driver_params = robot_params[robotplugintype];
        // robotdriver = etasl::Registry<etasl::RobotDriverFactory>::create(param["robotsimulator"],jsonchecker_);
        for (const auto& key : driver_params.getMemberNames()) {
          if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
              driver_name = key.substr(3);
          }
        }
        
        if(driver_name == ""){
          RCLCPP_ERROR(node_->get_logger(), "Could not find any is- keyword in the %s field of the json configuration file. It must specify the type, e.g. is-ur10_e_driver_etasl = true or is-simple_kinematic_simulator = true", robotplugintype.c_str());
          auto transition = node_->shutdown(); //calls on_shutdown() hook.
          return;
        }
    
        
        try
        {
          robotdriver_ = driver_loader_->createSharedInstance("etasl::" + driver_name);  
        }
        catch(pluginlib::PluginlibException& ex)
        {
          std::string message = "The plugin failed to load. Error: \n" + std::string(ex.what());
          RCLCPP_ERROR(node_->get_logger(), message.c_str());
          auto transition = node_->shutdown(); //calls on_shutdown() hook.
          return;
        }
        robotdriver_->construct(driver_name, feedback_shared_ptr.get(), setpoint_shared_ptr.get(), driver_params,jsonchecker_);

        robotdriver_->initialize();
    }


    bool RobotDriverManagerLockFree::initialize(std::shared_ptr<etasl::FeedbackMsg> feedback_copy_ptr, Context::Ptr ctx){
            

            // feedback_shared_ptr->mtx.lock();
      
            feedback_copy_ptr->joint.pos.is_available = true;
            // feedback_copy_ptr->joint.vel.is_available = feedback_shared_ptr->joint.vel.is_available;
            // feedback_copy_ptr->joint.torque.is_available = feedback_shared_ptr->joint.torque.is_available;
            // feedback_copy_ptr->joint.current.is_available = feedback_shared_ptr->joint.current.is_available;
      
            // feedback_copy_ptr->cartesian.pos.is_available = feedback_shared_ptr->cartesian.pos.is_available;
            // feedback_copy_ptr->cartesian.quat.is_available = feedback_shared_ptr->cartesian.quat.is_available;
            // feedback_copy_ptr->cartesian.twist.is_available = feedback_shared_ptr->cartesian.twist.is_available;
            // feedback_copy_ptr->cartesian.wrench.is_available = feedback_shared_ptr->cartesian.wrench.is_available;
            
            // feedback_copy_ptr->base.pos.is_available = feedback_shared_ptr->base.pos.is_available;
            // feedback_copy_ptr->base.quat.is_available = feedback_shared_ptr->base.quat.is_available;
            // feedback_copy_ptr->base.twist.is_available = feedback_shared_ptr->base.twist.is_available;
      
            if (!feedback_copy_ptr->joint.pos.is_available){
              std::string message = "The position feedback is not available in the used robot driver. This is required to run eTaSL.";
              RCLCPP_ERROR(node_->get_logger(), message.c_str());
              auto transition = node_->shutdown(); //calls on_shutdown() hook.
            }
            
            // for(unsigned int i = 0; i < feedback_shared_ptr->joint.pos.data.size(); ++i){
            // //   jpos_init_vec[i] = feedback_shared_ptr->joint.pos.data[i];
            //   feedback_copy_ptr->joint.pos.data[i] = feedback_shared_ptr->joint.pos.data[i];
            // }

            robotdriver_->readFeedbackJointPosition(feedback_copy_ptr->joint.pos.data);
      
            // feedback_shared_ptr->mtx.unlock();

            // --------- Check if the requested feedback is available in the robot driver ---------------
            // Json::Value param_robot = board->getPath("/robot", false);
            Json::Value param_robot = parameters_["robot"];

            feedback_report["joint_vel"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_joint_vel") && feedback_copy_ptr->joint.vel.is_available;
            feedback_report["joint_torque"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_joint_torque") && feedback_copy_ptr->joint.torque.is_available;
            feedback_report["joint_current"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_joint_current") && feedback_copy_ptr->joint.current.is_available;
            feedback_report["cartesian_pos"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_cartesian_pos") && feedback_copy_ptr->cartesian.pos.is_available;
            feedback_report["cartesian_quat"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_cartesian_quat") && feedback_copy_ptr->cartesian.quat.is_available;
            feedback_report["cartesian_twist"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_cartesian_twist") && feedback_copy_ptr->cartesian.twist.is_available;
            feedback_report["cartesian_wrench"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_cartesian_wrench") && feedback_copy_ptr->cartesian.wrench.is_available;
            feedback_report["base_pos"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_base_pos") && feedback_copy_ptr->base.pos.is_available;
            feedback_report["base_quat"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_base_quat") && feedback_copy_ptr->base.quat.is_available;
            feedback_report["base_twist"] = jsonchecker_->is_member(param_robot, "robotdriver/name_expr_base_twist") && feedback_copy_ptr->base.twist.is_available;
      


            input_channels_feedback.joint_vel.clear();
            input_channels_feedback.joint_torque.clear();
            input_channels_feedback.joint_current.clear();
      
            input_channels_feedback.joint_vel.resize(feedback_copy_ptr->joint.pos.data.size(), nullptr);
            input_channels_feedback.joint_torque.resize(feedback_copy_ptr->joint.pos.data.size(),nullptr);
            input_channels_feedback.joint_current.resize(feedback_copy_ptr->joint.pos.data.size(),nullptr);



            for (const auto& pair : feedback_report) {
                std::string key = pair.first;
                bool value = pair.second;
        
                if(jsonchecker_->is_member(param_robot, "robotdriver/name_expr_" + key)){
        
        
                  if(!value){
                    std::string message = "The requested " + key + " feedback is not available in the used robot driver. Delete the input value name_expr_"+ key +" from the setup.json file or fix the robot driver to report it.";
                    RCLCPP_ERROR(node_->get_logger(), message.c_str());
                    auto transition = node_->shutdown(); //calls on_shutdown() hook.
                    return false;
                  }
        
                  Json::Value param_iohandlers = parameters_["iohandlers"];

                  for (const auto& input_h : param_iohandlers["inputhandlers"]){ //Check that the user is not requesting a topic with the same name as the expression variable for the driver
                    if(input_h["varname"].asString() == jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key)){
                      std::string message = "The name `" + input_h["varname"].asString() + "` cannot be used within the setup.json file for both the name_expr_"+ key + " of robotdriver field and an input handler.";
                      RCLCPP_ERROR(node_->get_logger(), message.c_str());
                      auto transition = node_->shutdown(); //calls on_shutdown() hook.
                      return false;
                    } 
                  }
                }
            }
            return true;
    }
                

    void RobotDriverManagerLockFree::update( std::shared_ptr<etasl::FeedbackMsg> feedback_copy_ptr, const Eigen::VectorXd& jvel_etasl){

        // feedback_shared_ptr->mtx.lock();
        // setpoint_shared_ptr->mtx.lock();
    
        assert(feedback_shared_ptr->joint.pos.data.size() == jvel_etasl.size());
        // assert(setpoint_shared_ptr->velocity.data.size() == jvel_etasl.size());

        
        // setpoint_shared_ptr->velocity.fs = etasl::NewData;
        // for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
        //   setpoint_shared_ptr->velocity.data[i] = jvel_etasl[i];
        // }

        static std::vector<float> jvel_etasl_copy(jvel_etasl.size(), 0.0);
        for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
          jvel_etasl_copy[i] = jvel_etasl[i];
        }
        
        robotdriver_->writeSetpointJointVelocity(jvel_etasl_copy);
        robotdriver_->readFeedbackJointPosition(feedback_copy_ptr->joint.pos.data);
    
        // if (feedback_copy_ptr->joint.pos.is_available){
        //   for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
        //     feedback_copy_ptr->joint.pos.data[i] = feedback_shared_ptr->joint.pos.data[i];
        //   }
        // }
    
    
        // //Copy joint velocities
        // if (feedback_report["joint_vel"]){
        //   for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
        //     feedback_copy_ptr->joint.vel.data[i] = feedback_shared_ptr->joint.vel.data[i];
        //   }
        // }
    
        // //Copy joint torques
        // if (feedback_report["joint_torque"]){
        //   for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
        //     feedback_copy_ptr->joint.torque.data[i] = feedback_shared_ptr->joint.torque.data[i];
        //   }
        // }
    
        // //Copy joint currents
        // if (feedback_report["joint_current"]){
        //   for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
        //     feedback_copy_ptr->joint.current.data[i] = feedback_shared_ptr->joint.current.data[i];
        //   }
        // }
    
        // // -------- Cartesian feedback -----------------
    
        // //Copy Cartesian position
        // if (feedback_report["cartesian_pos"]){
        //   feedback_copy_ptr->cartesian.pos.x = feedback_shared_ptr->cartesian.pos.x;
        //   feedback_copy_ptr->cartesian.pos.y = feedback_shared_ptr->cartesian.pos.y;
        //   feedback_copy_ptr->cartesian.pos.z = feedback_shared_ptr->cartesian.pos.z;
        // }
    
        // //Copy Cartesian orientation in quaternion format
        // if (feedback_report["cartesian_quat"]){
        //   feedback_copy_ptr->cartesian.quat.qx = feedback_shared_ptr->cartesian.quat.qx;
        //   feedback_copy_ptr->cartesian.quat.qy = feedback_shared_ptr->cartesian.quat.qy;
        //   feedback_copy_ptr->cartesian.quat.qz = feedback_shared_ptr->cartesian.quat.qz;
        //   feedback_copy_ptr->cartesian.quat.qw = feedback_shared_ptr->cartesian.quat.qw;
        // }
    
        // //Copy Cartesian twist
        // if (feedback_report["cartesian_twist"]){
        //   feedback_copy_ptr->cartesian.twist.linear.x = feedback_shared_ptr->cartesian.twist.linear.x;
        //   feedback_copy_ptr->cartesian.twist.linear.y = feedback_shared_ptr->cartesian.twist.linear.y;
        //   feedback_copy_ptr->cartesian.twist.linear.z = feedback_shared_ptr->cartesian.twist.linear.z;
        //   feedback_copy_ptr->cartesian.twist.angular.x = feedback_shared_ptr->cartesian.twist.angular.x;
        //   feedback_copy_ptr->cartesian.twist.angular.y = feedback_shared_ptr->cartesian.twist.angular.y;
        //   feedback_copy_ptr->cartesian.twist.angular.z = feedback_shared_ptr->cartesian.twist.angular.z;
        // }
    
        // //Copy Cartesian wrench
        // if (feedback_report["cartesian_wrench"]){
        //   feedback_copy_ptr->cartesian.wrench.linear.x = feedback_shared_ptr->cartesian.wrench.linear.x;
        //   feedback_copy_ptr->cartesian.wrench.linear.y = feedback_shared_ptr->cartesian.wrench.linear.y;
        //   feedback_copy_ptr->cartesian.wrench.linear.z = feedback_shared_ptr->cartesian.wrench.linear.z;
        //   feedback_copy_ptr->cartesian.wrench.angular.x = feedback_shared_ptr->cartesian.wrench.angular.x;
        //   feedback_copy_ptr->cartesian.wrench.angular.y = feedback_shared_ptr->cartesian.wrench.angular.y;
        //   feedback_copy_ptr->cartesian.wrench.angular.z = feedback_shared_ptr->cartesian.wrench.angular.z;
        // }
    
        // //Copy base position
        // if (feedback_report["base_pos"]){
        //   feedback_copy_ptr->base.pos.x = feedback_shared_ptr->base.pos.x;
        //   feedback_copy_ptr->base.pos.y = feedback_shared_ptr->base.pos.y;
        //   feedback_copy_ptr->base.pos.z = feedback_shared_ptr->base.pos.z;
        // }
    
        // //Copy base orientation in quaternion format
        // if (feedback_report["base_quat"]){
        //   feedback_copy_ptr->base.quat.qx = feedback_shared_ptr->base.quat.qx;
        //   feedback_copy_ptr->base.quat.qy = feedback_shared_ptr->base.quat.qy;
        //   feedback_copy_ptr->base.quat.qz = feedback_shared_ptr->base.quat.qz;
        //   feedback_copy_ptr->base.quat.qw = feedback_shared_ptr->base.quat.qw;
        // }
    
        // //Copy base twist
        // if (feedback_report["base_twist"]){
        //   feedback_copy_ptr->base.twist.linear.x = feedback_shared_ptr->base.twist.linear.x;
        //   feedback_copy_ptr->base.twist.linear.y = feedback_shared_ptr->base.twist.linear.y;
        //   feedback_copy_ptr->base.twist.linear.z = feedback_shared_ptr->base.twist.linear.z;
        //   feedback_copy_ptr->base.twist.angular.x = feedback_shared_ptr->base.twist.angular.x;
        //   feedback_copy_ptr->base.twist.angular.y = feedback_shared_ptr->base.twist.angular.y;
        //   feedback_copy_ptr->base.twist.angular.z = feedback_shared_ptr->base.twist.angular.z;
        // }
    
        // feedback_shared_ptr->mtx.unlock();
        // setpoint_shared_ptr->mtx.unlock();
    
    
        // std::cout << jpos_etasl[6] << std::endl;
    
        // for (unsigned int i=0; i<jpos_etasl.size(); ++i) {
        //   std::cout << jpos_etasl[i] << " , ";
        // }
        // std::endl;
    
        //TODO: Delete feedback_copy_ptr and instead write directly to input_channels_feedback pointers as below:
        // Write in the input handler the joint velocities
        if (feedback_report["joint_vel"]){
            for (unsigned int i=0; i<jvel_etasl.size(); ++i) { //Check that the input channel exists (i.e. was declared in the task specification)
                if(input_channels_feedback.joint_vel[i]){      
                    input_channels_feedback.joint_vel[i]->setValue(feedback_copy_ptr->joint.vel.data[i]);
                }
            }
        }
    
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

        Json::Value param_robot = parameters_["robot"];

        for (const auto& pair : feedback_report) {
            std::string key = pair.first;
            bool value = pair.second;
      
            if(value){
              //Check types and get Input Channels accordingly. Joint values are vectors of pointers, and the rest are pointers. Careful!
              if (key == "joint_vel") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_vel.size(); ++i) {
                  input_channels_feedback.joint_vel[i] = ctx->getInputChannel<float>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key) + "_" + std::to_string(i));
                }
              } 
              else if (key == "joint_torque") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_torque.size(); ++i) {
                  input_channels_feedback.joint_torque[i] = ctx->getInputChannel<float>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key) + "_" + std::to_string(i));
                }
              }
              else if (key == "joint_current") {
                for (unsigned int i = 0; i < input_channels_feedback.joint_current.size(); ++i) {
                  input_channels_feedback.joint_current[i] = ctx->getInputChannel<float>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key) + "_" + std::to_string(i));
                }
              }
              else if (key == "cartesian_pos") {
                  input_channels_feedback.cartesian_pos = ctx->getInputChannel<KDL::Vector>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "cartesian_quat") {
                  input_channels_feedback.cartesian_quat = ctx->getInputChannel<KDL::Rotation>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "cartesian_twist") {
                  input_channels_feedback.cartesian_twist = ctx->getInputChannel<KDL::Twist>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "cartesian_wrench") {
                  input_channels_feedback.cartesian_wrench = ctx->getInputChannel<KDL::Wrench>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "base_pos") {
                  input_channels_feedback.base_pos = ctx->getInputChannel<KDL::Vector>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "base_quat") {
                  input_channels_feedback.base_quat = ctx->getInputChannel<KDL::Rotation>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
              else if (key == "base_twist") {
                  input_channels_feedback.base_twist = ctx->getInputChannel<KDL::Twist>(jsonchecker_->asString(param_robot, "robotdriver/name_expr_" + key));
              }
            }
      
          }

    }

    std::vector<float> RobotDriverManagerLockFree::get_position_feedback(){
        std::vector<float> jpos_init_vec;
        // feedback_shared_ptr->mtx.lock();
        jpos_init_vec.resize(6,0.0);
        // for(unsigned int i = 0; i < feedback_shared_ptr->joint.pos.data.size(); ++i){
        //   jpos_init_vec[i] = feedback_shared_ptr->joint.pos.data[i];
        // }
        robotdriver_->readFeedbackJointPosition(jpos_init_vec);

        // feedback_shared_ptr->mtx.unlock();

        return jpos_init_vec;
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
    
        double periodicity;
        Json::Value param_robot = parameters_["robot"];

        if(simulation_){
          periodicity = jsonchecker_->asDouble(param_robot, "robotsimulator/periodicity");
        }
        else{
          periodicity = jsonchecker_->asDouble(param_robot, "robotdriver/periodicity");
        }
        
        thread_str_driver = std::make_shared<t_manager::thread_t>();
    
        
        thread_str_driver->periodicity = std::chrono::nanoseconds(static_cast<long long>(periodicity * 1E9)); //*1E9 to convert seconds to nanoseconds
        thread_str_driver->update_hook = std::bind(&etasl::RobotDriver::update, robotdriver_, std::ref(stopFlag));
        thread_str_driver->finalize_hook = std::bind(&etasl::RobotDriver::finalize, robotdriver_);
        
    
        return thread_str_driver;
    
      }



} // namespace etasl