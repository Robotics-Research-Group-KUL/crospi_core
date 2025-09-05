#include "multiple_drivers_manager.hpp"


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
        // Constructor implementation

        Json::Value param_robot = parameters_["robot"];

        Json::Value def_robot_spec_params = parameters_["default_robot_specification"];
        for (auto n : jsonchecker_->asArray(def_robot_spec_params, "robot_joints")) {
          jnames_all_.push_back(n.asString());
        }

        for (unsigned int i=0;i<jnames_all_.size();++i) {
          robot_joint_names_ndx_map[ jnames_all_[i]]  =i;
        }

        
        if(simulation_){
          // periodicity = jsonchecker_->asDouble(param_robot, "robotsimulator/periodicity");
          param_robotdrivers_ = param_robot["robotsimulators"];
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("etasl_ros2", "etasl::RobotSimulator"); //This works because etasl::RobotSimulator inherits from etasl::RobotDriver

        }
        else{
          param_robotdrivers_ = param_robot["robotdrivers"];
          driver_loader_ = std::make_shared<pluginlib::ClassLoader<etasl::RobotDriver>>("etasl_ros2", "etasl::RobotDriver");
          // periodicity = jsonchecker_->asDouble(param_robot, "robotdriver/periodicity");
        }

        // Json::Value robot_param = board->getPath("/robot", false);

        // Json::Value param_iohandlers = parameters["iohandlers"];

        


    }



    MultipleDriversManager::~MultipleDriversManager() {
        // Destructor implementation
    }

    void MultipleDriversManager::construct_drivers(){

      std::cout << "beginning of MultipleDriversManager::construct_drivers()" << std::endl;


      // ------- START: initialize robot joints and indices for demultiplexer and multiplexer of robot joints for different drivers. Already considers new structure of JSON      
      
      separated_jnames_all_.clear();
      separated_jindices_in_expr_.clear();
      
      
      for (const auto& driver : param_robotdrivers_){
        std::vector<std::string> separated_jnames_element;
        for (auto n : jsonchecker_->asArray(driver, "robot_joints")) {
          separated_jnames_element.push_back(n.asString());
        }
        separated_jnames_all_.push_back(separated_jnames_element);
      }
      
      separated_jindices_in_expr_.resize(separated_jnames_all_.size());
      
      
      for(unsigned int i=0; i<separated_jnames_all_.size(); ++i){
        int num_joints = separated_jnames_all_.at(i).size();
        separated_jindices_in_expr_.at(i).resize(num_joints);
        Json::Value current_driver_param = param_robotdrivers_[i];
        robotdriver_manager_vector_.push_back(std::make_shared<etasl::RobotDriverManagerLockFree>(node_, current_driver_param, jsonchecker_, simulation_, stopFlagPtr_));
        robotdriver_manager_vector_.back()->construct_driver(num_joints, driver_loader_); //constructs and initializes communication with the robot
        
        feedback_copies_vec.push_back(std::make_shared<robotdrivers::FeedbackMsg>(num_joints));
        jvel_etasl_copies_vec.push_back(Eigen::VectorXd::Zero(num_joints)); //Initialize with zeros
        feedback_separate_joint_pos_.push_back(robotdrivers::DynamicJointDataField(num_joints));
        num_joints_in_all_drivers_ += num_joints;
      }
      
      // ------- END: initialize robot joints and indices for demultiplexer and multiplexer of robot joints for different drivers. Already considers new structure of JSON

      // for (const auto& drivers : param_robotdrivers_){

      //   // double periodicity = jsonchecker_->asDouble(drivers, "periodicity");
      //   // thread_str_drivers_vector_.push_back(std::make_shared<t_manager::thread_t>());
      //   // std::shared_ptr<t_manager::thread_t> thread_str_driver = thread_str_drivers_vector_.back(); //Takes the last element

      //   // robotdriver_manager = std::make_shared<etasl::RobotDriverManagerLockFree>(node_, param_root, jsonchecker, simulation, stopFlagPtr);
      //   // robotdriver_manager->construct_driver(jointnames.size());  //constructs and initializes communication with the robot

      //   robotdriver_manager_vector_.push_back(std::make_shared<etasl::RobotDriverManagerLockFree>(node_, parameters_, jsonchecker_, simulation_, stopFlagPtr_));
      //   robotdriver_manager_vector_.back()->construct_driver(num_joints); //constructs and initializes communication with the robot

      //   //TODO: iMPORTANT! CHANGE num_joints to each driver number of joints
      //   feedback_copies_vec.push_back(std::make_shared<robotdrivers::FeedbackMsg>(num_joints));
      //   jvel_etasl_copies_vec.push_back(Eigen::VectorXd::Zero(num_joints));
        
      // }

      std::cout << "end of MultipleDriversManager::construct_drivers()" << std::endl;
      

    }


    bool MultipleDriversManager::initialize(Context::Ptr ctx){

      std::cout << "beginning of MultipleDriversManager::initialize()" << std::endl;



        //The following checks if the user is requesting intputs both as an input handler and also through one of the drivers (repeader)

        // Json::Value param_iohandlers = parameters_["iohandlers"];

        // std::vector<std::string>> list_of_exp_names = {
        //   "joint_vel",
        //   "joint_torque",
        //   "joint_current",
        //   "cartesian_pos",
        //   "cartesian_quat",
        //   "cartesian_twist",
        //   "cartesian_wrench",
        //   "base_pos",
        //   "base_quat",
        //   "base_twist"
        // };

        // //Check that the user is not requesting a topic with the same name as the expression variable for the driver
        // for (const auto& input_h : param_iohandlers["inputhandlers"]){ 
        //   for (const auto& driver_p : param_robotdrivers_){
        //     for(unsigned int it=0; it<list_of_exp_names.size(); ++it){
        //       if(jsonchecker_->is_member(driver_p, "name_expr_" + list_of_exp_names.at(i)) && input_h["varname"].asString() == jsonchecker_->asString(driver_p, "name_expr_" + list_of_exp_names.at(i))){
        //         std::string message = "The name `" + input_h["varname"].asString() + "` cannot be used within the setup.json file for both the name_expr_"+ list_of_exp_names.at(i) + " of a robotdriver field and an input handler.";
        //         RCLCPP_ERROR(node_->get_logger(), message.c_str());
        //         auto transition = node_->shutdown(); //calls on_shutdown() hook.
        //         return false;
        //       } 
        //     }
        //   }
        // }


      assert(robotdriver_manager_vector_.size() == feedback_copies_vec.size());

      // for (const auto& driver_managers : robotdriver_manager_vector_){
      bool all_initialized = true;
      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        all_initialized =  all_initialized && robotdriver_manager_vector_.at(i)->initialize(feedback_copies_vec[i], ctx);
        feedback_separate_joint_pos_.at(i).data = feedback_copies_vec.at(i)->joint.pos.data; 
      }

      // bool MultipleDriversManager::multiplexer(feedback_separate_joint_pos_, combined_joint_pos_);
      // combined_joint_pos_.data.resize(feedback_copy_ptr->joint.pos.data.size(),0.0);
      // combined_joint_pos_.data = feedback_copy_ptr->joint.pos.data; //Initialize with the current values of the robot

      // update_joint_indices(robot_joint_names_ndx_map);
      

      // if(!this->multiplexer(feedback_separate_joint_pos_, feedback_copy_ptr->joint.pos)){
      //   std::string message = "The feedback of the different robotdrivers could not be multiplexed at initialization.";
      //   RCLCPP_ERROR(node_->get_logger(), message.c_str());
      //   auto transition = node_->shutdown(); //calls on_shutdown() hook
      //   return false;
      // }
            
      std::cout << "end of MultipleDriversManager::initialize()" << std::endl;

      return all_initialized;
    }

    void MultipleDriversManager::update( std::vector<float>& joint_positions, const Eigen::VectorXd& jvel_etasl){

      // std::cout <<"The size of the separated_jindices_in_expr_:" << separated_jindices_in_expr_.size() << std::endl;

      // for(const auto& vec : separated_jindices_in_expr_){
      //   for(const auto& val : vec){
      //     std::cout << val << ", ";
      //   }
      //   std::cout << std::endl;
      // }


      this->demultiplexer(jvel_etasl, jvel_etasl_copies_vec );
      // void MultipleDriversManager::demultiplexer(const Eigen::VectorXd& joint_vel_etasl, std::vector<Eigen::VectorXd>& jvel_etasl_separated_vector ){

      // for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
      //   std::cout << jvel_etasl_copies_vec[0][i] << "=?" << jvel_etasl[i] << ", ";
      // }
      // std::cout << std::endl;

      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        robotdriver_manager_vector_[i]->update(feedback_copies_vec[i], jvel_etasl_copies_vec[i]);
      }


      //After reading feedback in the update I am copying data in the right format.
      //TODO: Delete the feedback_copy_ptr and avoid so many copies!!!! I just need position stuff for this steps
      for (unsigned int i=0; i<feedback_copies_vec.size(); ++i) {
        feedback_separate_joint_pos_[i].data = feedback_copies_vec[i]->joint.pos.data; 
      }


      // if(!this->multiplexer(feedback_copies_vec, feedback_copy_ptr)){
      if(!this->multiplexer(feedback_separate_joint_pos_, feedback_copy_ptr->joint.pos)){
        std::string message = "The feedback of the different robotdrivers could not be multiplexed during the update.";
        RCLCPP_ERROR(node_->get_logger(), message.c_str());
        auto transition = node_->shutdown(); //calls on_shutdown() hook
        return;
      }

      // for (unsigned int i=0; i<feedback_copy_ptr->joint.pos.data.size(); ++i) {
      //     std::cout << feedback_separate_joint_pos_[0].data[i] << "=?" << feedback_copy_ptr->joint.pos.data[i] << ", ";
      // }
      // std::cout << std::endl;

      // for(unsigned int i=0; i<feedback_copy_ptr->joint.pos.data.size(); ++i) {
      //   std::cout << feedback_copy_ptr->joint.pos.data[i] << ", ";
      // }
      // std::cout << std::endl;

    }

    void MultipleDriversManager::update_joint_indices(const std::map<std::string,int>& robot_joint_names_ndx_map){
      for (unsigned int g = 0; g < separated_jnames_all_.size(); ++g) {
        for (unsigned int i = 0; i < separated_jnames_all_.at(g).size(); ++i) {
          auto it = robot_joint_names_ndx_map.find(separated_jnames_all_.at(g).at(i));
          if (it == robot_joint_names_ndx_map.end()) { //Joint name not found in the etasl expressions
            separated_jindices_in_expr_.at(g).at(i) = -1; // Joint not found
            // std::string message = "The joint name " + separated_jnames_all_.at(g).at(i) + "specified in the configuration of the driver/simulator was not found in the robot_joints provided to etasl in the provided .setup.json file";
            // RCLCPP_ERROR(node_->get_logger(), message.c_str());
            // auto transition = node_->shutdown(); //calls on_shutdown() hook
          }
          else{
            separated_jindices_in_expr_.at(g).at(i) = it->second;  // Assigns the index of the joint
          }
        }
      } 
    }


    //TODO: change to unordered_map instead of map
    void MultipleDriversManager::on_configure(Context::Ptr ctx){

      std::cout << "beginning of MultipleDriversManager::on_configure()" << std::endl;

      update_joint_indices(robot_joint_names_ndx_map); //TODO: do this once at initialization since now we always give to the etasl_node all the joints of all the drivers (without taking into account what is in the etasl expressions)


      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        robotdriver_manager_vector_.at(i)->on_configure(ctx);
      }

      std::cout << "end of MultipleDriversManager::on_configure()" << std::endl;

    }


    std::vector<float> MultipleDriversManager::get_position_feedback(){

      std::cout << "beginning of MultipleDriversManager::get_position_feedback()" << std::endl;

      
      robotdrivers::DynamicJointDataField combined_jpos_current_vec(num_joints_in_all_drivers_);
      std::vector<robotdrivers::DynamicJointDataField> separate_jpos_current_vec;

      separate_jpos_current_vec.resize(robotdriver_manager_vector_.size());

      std::cout << "ciaoo 111" << std::endl;

      for (unsigned int i=0; i<robotdriver_manager_vector_.size(); ++i) {
        separate_jpos_current_vec.at(i).data = robotdriver_manager_vector_.at(i)->get_position_feedback();
      }
      std::cout << "ciaoo 222" << std::endl;

      for (unsigned int i=0; i<separate_jpos_current_vec.at(0).data.size(); ++i) {
        std::cout << separate_jpos_current_vec.at(0).data.at(i) << ", ";
      }
      std::cout << std::endl;
      std::cout << "/////////////////////////////////////////////////////////hereee!!!! abovee" << std::endl;


      if(!this->multiplexer(separate_jpos_current_vec, combined_jpos_current_vec)){
        std::string message = "The position feedback of the different robotdrivers could not be multiplexed during the get_position_feedback() function.";
        RCLCPP_ERROR(node_->get_logger(), message.c_str());
        auto transition = node_->shutdown(); //calls on_shutdown() hook
      }
      std::cout << "end of MultipleDriversManager::get_position_feedback()" << std::endl;

        return combined_jpos_current_vec.data;
    }

    void MultipleDriversManager::on_activate(){

    }


    void MultipleDriversManager::on_deactivate(){

    }


    void MultipleDriversManager::on_cleanup(){

    }

    void MultipleDriversManager::finalize(){

      for (const auto& driver_manager : robotdriver_manager_vector_){
        driver_manager->finalize();
      }

    }

    // void MultipleDriversManager::demultiplexer(const std::vector<std::vector<std::string>>& separated_jnames_all_, const Eigen::VectorXd& joint_vel_etasl, std::vector<Eigen::VectorXd>& jvel_etasl_separated_vector ){
    void MultipleDriversManager::demultiplexer(const Eigen::VectorXd& joint_vel_etasl, std::vector<Eigen::VectorXd>& jvel_etasl_separated_vector ){

      for (unsigned int g = 0; g < jvel_etasl_separated_vector.size(); ++g) {
        for (unsigned int i=0;i<jvel_etasl_separated_vector[g].size();++i) {
          if (separated_jindices_in_expr_[g][i] >= 0){ //!= -1
            jvel_etasl_separated_vector[g][i] = joint_vel_etasl[separated_jindices_in_expr_[g][i]];
          }
          else{
            jvel_etasl_separated_vector[g][i] = 0; //Set to zero velocities that are not considered by eTaSL in the current task
          }
        }
      }

      //TODO
      return;
    }

    // bool MultipleDriversManager::multiplexer(const std::vector<std::shared_ptr<robotdrivers::FeedbackMsg>>& separate_feedback_copies_vec, std::shared_ptr<robotdrivers::FeedbackMsg>& combined_feedback_copy_ptr){
    //   //TODO
    //   return true;
    // }
    bool MultipleDriversManager::multiplexer(const std::vector<robotdrivers::DynamicJointDataField>& separate_joint_pos, robotdrivers::DynamicJointDataField& combined_joint_pos){
      
      // std::cout << "separate_joint_pos.size(): " << separate_joint_pos.size() << std::endl;
      // std::cout << "separate_joint_pos[0].data.size(): " << separate_joint_pos[0].data.size() << std::endl;

      for (unsigned int g = 0; g < separate_joint_pos.size(); ++g) {
        for (unsigned int i=0;i<separate_joint_pos[g].data.size();++i) {
          if (separated_jindices_in_expr_[g][i] >= 0){ //!= -1
            std::cout << "separated_jindices_in_expr_[g][i]" << separated_jindices_in_expr_[g][i] << std::endl;
            std::cout << "combined_joint_pos.data.size()" << combined_joint_pos.data.size() << std::endl;

            assert(separated_jindices_in_expr_[g][i] < combined_joint_pos.data.size()); //TODO: Comment this out
            combined_joint_pos.data[separated_jindices_in_expr_[g][i]] = separate_joint_pos[g].data[i];
            // std::cout << separate_joint_pos[g].data[i] << ", ";
          }
          //Else: don't do anything. Just maintain the previous positions (e.g. in case that the joints are not being considered in the current etasl task)
        }
        // std::cout << std::endl;
      }

      return true;
    }


    std::vector<std::shared_ptr<t_manager::thread_t>> MultipleDriversManager::create_driver_threads_structures(std::atomic<bool> & stopFlag){
    
        
        // std::vector<double> periodicities_vector;
        int number_of_drivers = 0;
        std::vector<std::shared_ptr<t_manager::thread_t>> thread_str_drivers_vector_;


        for (const auto& driver_manager : robotdriver_manager_vector_){
          thread_str_drivers_vector_.push_back(driver_manager->create_thread_str(stopFlag));
        }

        return thread_str_drivers_vector_;
    
      }



} // namespace etasl