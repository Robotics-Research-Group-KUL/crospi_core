#include "etasl_node.hpp"
#include "IO_handlers_deleteme.hpp"
#include "port_observer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp> 

#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/flowstatus.hpp"
#include <etasl_task_utils/string_interpolate.hpp> //To e.g. specify $[etasl_ros2]/launch/etc... and other functionalities in strings

#include <jsoncpp/json/json.h>

#include <fmt/format.h>

// For real-time control loop
#include <chrono>
#include <thread>
#include <vector>


using namespace std::chrono_literals;
using namespace KDL;
using namespace Eigen;


etaslNode::etaslNode(const std::string & node_name, bool intra_process_comms = false): rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
, time(0.0)
, event_msg(std_msgs::msg::String())
, event_postfix(get_name())
, first_time_configured(false)
, is_configured(false)
{

  // Lambda function to handle errors when reading a JSON element from the configuration file
  std::function<void(const std::string&)> error_callback = [&](const std::string& error_msg) {
    RCUTILS_LOG_ERROR_NAMED(get_name(), error_msg.c_str());
    rclcpp::shutdown(); 
    exit(1); //1 indicates It indicates abnormal termination of a program as a result a minor problem.
  };
  jsonchecker = std::make_shared<etasl::JsonChecker>(error_callback);

  srv_etasl_console_ = create_service<std_srvs::srv::Empty>("etasl_node/etasl_console", std::bind(&etaslNode::etasl_console, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationFile_ = create_service<etasl_interfaces::srv::TaskSpecificationFile>("etasl_node/readTaskSpecificationFile", std::bind(&etaslNode::readTaskSpecificationFile, this, std::placeholders::_1, std::placeholders::_2));
  srv_readRobotSpecification_ = create_service<etasl_interfaces::srv::TaskSpecificationFile>("etasl_node/readRobotSpecification", std::bind(&etaslNode::readRobotSpecification, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationString_ = create_service<etasl_interfaces::srv::TaskSpecificationString>("etasl_node/readTaskSpecificationString", std::bind(&etaslNode::readTaskSpecificationString, this, std::placeholders::_1, std::placeholders::_2));
  
  srv_readTaskParameters_ = create_service<etasl_interfaces::srv::TaskSpecificationString>("etasl_node/readTaskParameters", std::bind(&etaslNode::readTaskParameters, this, std::placeholders::_1, std::placeholders::_2));

  std::string configFilePath = this->declare_parameter<std::string>("config_file", "");
  bool simulation_param = this->declare_parameter<bool>("simulation", true);



// Checks if the path is empty. i.e. the parameter was not defined.
  if (configFilePath.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No directory path provided. Use the 'config_file' ros parameter as follows: \n ros2 run etasl_ros2 etasl_node --ros-args -p config_file:=/path/to/directory");
      rclcpp::shutdown();
      return;
  }

  // Checks if the path is in the appropriate format.
  static const std::regex pattern_file(R"(^\$\[([a-zA-Z0-9_]+)\]\/.+\.json$)");
  std::smatch matched_package;
  std::string package_name;
  
  bool matches_file_pattern = std::regex_match(configFilePath,matched_package, pattern_file);
  if(!matches_file_pattern){
    std::string message = "The format of the config_file:=" + configFilePath + " provided as --ros-arg: is not valid.\n It should contain a valid ROS2 package based on the etasl_ros2_application_template, and it should be a JSON file as follows: $[my_ros2_package]/applications/application_example_ur10/application_example_ur10.setup.json";
    RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
    rclcpp::shutdown();
    return;
  }
  else{
    package_name = matched_package[1];
  }

 // Checks if the package exists when performing the string interpolation. It does not check if the file exists! 
//  Necessary because BlackBoard::load called in BlackBoard::load_process_and_validate does not handle the exception thrown by string_interpolate 
  std::string file_path;
  try {
      file_path = etasl::string_interpolate(configFilePath);
  } catch (const std::exception& e) {
    // Catches any other exceptions derived from std::exception
      std::string message = "Exception caught when reading the config_file:=" + configFilePath + " provided as --ros-arg: \n Exception e.what():" + std::string(e.what());
      RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
      rclcpp::shutdown();
      return;
  }

  std::filesystem::path path(file_path);

    // Check if the file path exists
  if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
      std::string message = "The config_file:=" + file_path + " which was provided through --ros-arg is not a valid file directory.";
      RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
      rclcpp::shutdown();
      return;
  } 


  reinitialize_data_structures();

  board = std::make_shared<etasl::BlackBoard>(1); //if -1, all parts of the local path in the URI are returned.  if >=0, then
  // *                             the last path_components_used parts are retained.
  // etasl::BlackBoard board(1);
  std::cout << " loading blackboard" << std::endl;

  std::string location_of_bundled_schema = etasl::string_interpolate(fmt::format("$[{}]/schemas/generated", package_name));

  //print parent_path
  std::cout << "parent_path: " << path.parent_path() << std::endl;
  // board->setSearchPath(path.parent_path());
  // board->setSearchPath(fmt::format("{}:$[etasl_ros2]/scripts/schema", path.parent_path().c_str()));
  // board->setSearchPath("$[etasl_ros2]/scripts/schema:$[etasl_ros2]/scripts/schema/tasks");
  // board->setSearchPath("$[etasl_ros2]/scripts/schema");
  // board->setSearchPath("https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_json_schemas/raw/main/schemas");
  // board->setSearchPath("$[etasl_ros2_application_template]/schemas:https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_json_schemas/raw/main/schemas");
  board->setSearchPath(location_of_bundled_schema); //Selects search path based on the package provided in the config file
  // board->load_process_and_validate("$[etasl_ros2]/scripts/json/blackboard.json");
  
  board->load_process_and_validate(configFilePath); // Also checks if it is a valid JSON file
  // fmt::print("{:->80}\n", "-");


  Json::Value etasl_param = board->getPath("/etasl", false);
  periodicity_ms = 1000*jsonchecker->asDouble(etasl_param, "general/sample_time"); //Expressed in milliseconds

  // TODO: change the quality of service to transient local and reliable:
  rclcpp::QoS qos_profile( rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

  qos_profile
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE) // Ensures every message is delivered (may retransmit if lost)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
    .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST) //Keeps the last msgs received in case buffer is fulll
    .keep_last(20); //Buffer size


  events_pub_ = this->create_publisher<std_msgs::msg::String>(jsonchecker->asString(etasl_param, "general/event_topic"), qos_profile); 

  std::string message = "\n+++++++++++++++++++++++++++++++++++++++++++++++++\nThe following configuration file was loaded correctly:" + file_path + "\n+++++++++++++++++++++++++++++++++++++++++++++++++";
  RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());


  this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(std::bind( &etaslNode::safe_shutdown, this)); // Adds safe_shutdown as callback before shutting down, e.g. with ctrl+c. This methods returns rclcpp::OnShutdownCallbackHandle shutdown_cb_handle
  // this->get_node_base_interface()->get_context()->add_on_shutdown_callback(std::bind( &etaslNode::safe_shutdown, this)); //Can be used to add callback during shutdown (not pre-shutdown, so publishers and others are no longer available)
  // rclcpp::on_shutdown(std::bind( &etaslNode::safe_shutdown, my_etasl_node)); //Alternative to add_on_shutdown_callback (don't know the difference)

  Json::Value robot_param = board->getPath("/robot", false);


  if(simulation_param){
    if(robot_param.isMember("robotsimulators") && robot_param["robotsimulators"].size() > 0){
      simulation = true;
      RCUTILS_LOG_INFO_NAMED(get_name(), "Executing in simulation mode!!!.");
      
    }
    else{
      RCUTILS_LOG_ERROR_NAMED(get_name(), "No robotsimulators were specified in the configuration file and therefore the etasl cannot run in simulation (i.e. simulation=true is not valid). Please either set simulation to false or provide a valid robotsimulators field in the configuration file.");
      rclcpp::shutdown();
      return;
    }
  }
  else{
    if(robot_param.isMember("robotdrivers") && robot_param["robotdrivers"].size() > 0){
      simulation = false;
      RCUTILS_LOG_INFO_NAMED(get_name(), "Executing in real robot (i.e. simulation=false)!!!.");
    }
    else{
      RCUTILS_LOG_ERROR_NAMED(get_name(), "No robotdrivers were specified in the configuration file and therefore etasl can only run in simulation (i.e. simulation=true). Please either set simulation to true or provide a valid robotdrivers field in the configuration file.");
      rclcpp::shutdown();
      return;
    }
  }


  // if (simulation_param || (robot_param.isMember("robotdrivers") && robot_param["robotdrivers"].isMember("is-no_driver") && robot_param["robotdrivers"]["is-no_driver"].isBool()) ) {
  //   simulation = true;
  // }
  // else {
  //   simulation = false;
  // }
  

}


bool etaslNode::etasl_console(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response>  response) {

    if (this->get_current_state().label() == "unconfigured" || this->get_current_state().label() == "finalized") {
            RCUTILS_LOG_ERROR_NAMED(get_name(), "etasl_console not allowed while the node is unconfigured or finalized");
            return false;
    } 
    else if (this->get_current_state().label() == "active") {
            RCUTILS_LOG_INFO_NAMED(get_name(), "Node automatically deactivating and entering to the etasl_console...");
            auto transition = this->deactivate();
            int retval = LUA->call_console(); 
            return retval==0;
    }
    else{
            RCUTILS_LOG_INFO_NAMED(get_name(), "Entering to the etasl_console...");
            int retval = LUA->call_console(); 
            return retval==0;
    }

    return false;
}

bool etaslNode::readTaskSpecificationFile(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Response>  response) {

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service etasl_node/readTaskSpecificationFile can only be read in unconfigured state");
		response->success = false;
		return false;
	}

  std::string file_path = etasl::string_interpolate(request->file_path);
	
	try{
		// Read eTaSL specification:
		int retval = LUA->executeFile(std::string(file_path));
		// int retval = LUA->executeFile("/workspaces/colcon_ws/install/etasl_ros2/share/etasl_ros2/etasl/move_cartesianspace.lua");
		std::cout << "--------read lua file " <<std::endl; 
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing task specification file. Please provide a valid task specification file");
			response->success = false;
      auto transition = this->shutdown(); //calls on_shutdown() hook.
      // this->safe_shutdown();
			return false;
		}
	} catch (const char* msg) {
		// can be thrown by file/string errors during reading
		// by lua_bind during reading
		// by expressiongraph during reading ( expressiongraph will not throw when evaluating)
		std::string message = "The following error was thrown while reading the task specification: " + std::string(msg);
		RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
		response->success = false;
    auto transition = this->shutdown(); //calls on_shutdown() hook.
    // this->safe_shutdown();
		return false;
	}


	response->success = true;

	return true;
}

bool etaslNode::readRobotSpecification(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Response>  response) {

  Json::Value param_original = board->getPath("/robot", false);
  Json::Value param = param_original; //Copy

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service etasl_node/readTaskSpecificationFile can only be read in unconfigured state");
		response->success = false;
		return false;
	}

  // std::cout << "+++++++++++++++++++++++++++++" << std::endl;
  // std::cout << "+++++++++++++++++++++++++++++" << std::endl;
  // std::cout << "THE REQUESTED ROBOT SPECIFICATION IS: " << request->file_path << std::endl;
  // std::cout << "+++++++++++++++++++++++++++++" << std::endl;


  if (request->file_path != "") { // If a file_path is provided, then we override the default_robot_specification to use the provided lua file
    std::string file_path = etasl::string_interpolate(request->file_path);
    Json::Value robot_joints = param["default_robot_specification"]["robot_joints"];
    param.removeMember("default_robot_specification");
    param["default_robot_specification"]["is-lua_robotspecification"] = true;
    param["default_robot_specification"]["file_path"] = file_path;
    param["default_robot_specification"]["robot_joints"] = robot_joints;
    RCUTILS_LOG_WARN_NAMED(get_name(), "The default robot specification is being overridden by passing non empty lua file_path in the request of readRobotSpecification");
  }

  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";  // Optional: adjust to control whitespace in output
  // Create a StreamWriterBuilder to serialize the JSON value
  std::string robot_spec = "_JSON_ROBOTSPECIFICATION_STRING = '" + Json::writeString(writer, param["default_robot_specification"]) + "'";
  try{
    // Read eTaSL specification:
    int retval = LUA->executeString(robot_spec);
    if (retval !=0) {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing the following specificed string command in LUA while loading the robot specification ");
      RCUTILS_LOG_ERROR_NAMED(get_name(), robot_spec.c_str());
      response->success = false;
      rclcpp::shutdown(); 
      return false;
    }
  } catch (const char* msg) {
    // can be thrown by file/string errors during reading
    // by lua_bind during reading
    // by expressiongraph during reading ( expressiongraph will not throw when evaluating)
    std::string message = "The following error was thrown while loading the robot specification: " + std::string(msg);
    RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
    response->success = false;
    rclcpp::shutdown(); 
    return false;
  }

	response->success = true;

	return true;
}


bool etaslNode::readTaskSpecificationString(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Response>  response) {
	
	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service etasl_node/readTaskSpecificationString can only be read in unconfigured state");
		response->success = false;
		return false;
	}

	try{
		// Read eTaSL specification:
		int retval = LUA->executeString(request->str);
		// int retval = LUA->executeFile("/workspaces/colcon_ws/install/etasl_ros2/share/etasl_ros2/etasl/move_cartesianspace.lua");
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing the following specificed string command in LUA within the etasl_node/readTaskSpecificationString service: ");
			RCUTILS_LOG_ERROR_NAMED(get_name(), request->str.c_str());

			response->success = false;
      auto transition = this->shutdown(); //calls on_shutdown() hook.
			return false;
		}
	} catch (const char* msg) {
		// can be thrown by file/string errors during reading
		// by lua_bind during reading
		// by expressiongraph during reading ( expressiongraph will not throw when evaluating)
		std::string message = "The following error was thrown while reading the task specification string: " + std::string(msg);
		RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
		response->success = false;
    auto transition = this->shutdown(); //calls on_shutdown() hook.
		return false;
	}


	response->success = true;

	return true;

}


bool etaslNode::readTaskParameters(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Response>  response) {
	
	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service etasl_node/readTaskParameters can only be read in unconfigured state");
		response->success = false;
		return false;
	}

	try{
		// Read eTaSL specification:
    std::string param_request = "_JSON_TASK_SPECIFICATION_PARAMETERS_STRING='" + request->str + "'"; 
    
    std::cout << "The received parameters are:" << std::endl;
    std::cout << request->str << std::endl;

		int retval = LUA->executeString(param_request);
		// int retval = LUA->executeFile("/workspaces/colcon_ws/install/etasl_ros2/share/etasl_ros2/etasl/move_cartesianspace.lua");
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error interpreting the following string as a JSON string in LUA after calling the etasl_node/readTaskParameters service: ");
			RCUTILS_LOG_ERROR_NAMED(get_name(), request->str.c_str());

			response->success = false;
      auto transition = this->shutdown(); //calls on_shutdown() hook.
			return false;
		}
	} catch (const char* msg) {
		// can be thrown by file/string errors during reading
		// by lua_bind during reading
		// by expressiongraph during reading ( expressiongraph will not throw when evaluating)
		std::string message = "The following error was thrown while reading the task specification: " + std::string(msg);
		RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
		response->success = false;
    auto transition = this->shutdown(); //calls on_shutdown() hook.
		return false;
	}


	response->success = true;

	return true;

}


// inspiration found in https://etasl.pages.gitlab.kuleuven.be/etasl-api-doc/api/etasl-rtt/solver__state_8hpp_source.html
void etaslNode::update_controller_output(Eigen::VectorXd const& jvalues_solver){
    for (unsigned int i=0;i<jnames_in_expr.size();++i) {
      //TODO: is name_ndx really necessary? I counld just do jpos_ros[i] = jnames_in_expr[i] and get rid of name_ndx
      std::map<std::string,int>::iterator it = name_ndx.find(jnames_in_expr[i]);
      if (it!=name_ndx.end()) {
          jpos_ros[it->second] = jvalues_solver[i]; //joints that will be used for ros topic
      }
    }
}


void etaslNode::update_controller_input(Eigen::VectorXd const& jvalues_meas){
      for (unsigned int i=0;i<jnames_in_expr.size();++i) {
      std::map<std::string,int>::iterator p = jindex.find(jnames_in_expr[i]);
      if (p!=jindex.end()) {
          jpos_etasl[p->second] = jvalues_meas[i]; //joints that will be used for etasl
      }
  } 
}

void etaslNode::solver_configuration(){
      // Create registry and register known solvers: 
    Json::Value param = board->getPath("/etasl", false);

    solver_registry = boost::make_shared<SolverRegistry>();
    registerSolverFactory_qpOases(solver_registry, "qpoases");
    //registerSolverFactory_hqp(R, "hqp");
    

    // we can get the properties from the solver from the context and specify these properties in eTaSL or
    // create a parameter plist (instead of ctx->solver_property) :
        // parameters of the solver:

        // TODO: Delete the following after handling solvers with register factory (same as with IO handlers)
        if (!jsonchecker->asBool(param, "solver/is-qpoasessolver")){
          RCUTILS_LOG_ERROR_NAMED(get_name(), "is-qpoasessolver should be true in the JSON definition since currently only qpoases solver is supported.");
          // this->safe_shutdown();
          auto transition = this->shutdown(); //calls on_shutdown() hook.
          return;
        }

        std::string solver_name             = "qpoases" ;
        ParameterList plist;
        plist["nWSR"]                  = jsonchecker->asDouble(param, "solver/nWSR");
        plist["regularization_factor"] = jsonchecker->asDouble(param, "solver/regularization_factor");
        plist["cputime"] = jsonchecker->asDouble(param, "solver/cputime");
        // parameters of the initialization procedure:
        plist["initialization_full"]                  = int(jsonchecker->asBool(param, "initializer/full")); // == true
        plist["initialization_duration"]              = jsonchecker->asDouble(param, "initializer/duration");
        plist["initialization_sample_time"]           = jsonchecker->asDouble(param, "initializer/sample_time");
        plist["initialization_convergence_criterion"] = jsonchecker->asDouble(param, "initializer/convergence_criterion");
        plist["initialization_weightfactor"]          = jsonchecker->asDouble(param, "initializer/weightfactor");


    int result = solver_registry->createSolver(solver_name, plist, true, false, slv); 
    if (result!=0) {
        std::string message = "Failed to create the solver " + solver_name;
        RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
        auto transition = this->shutdown(); //calls on_shutdown() hook.
        // this->safe_shutdown();
        return;
    }
    ctx->setSolverProperty("sample_time", periodicity_ms/1000.0);


    // if (ctx->getSolverProperty("verbose",0.0)>0) {
        std::stringstream message;
        message << "Solver and initialization properties : \n";
        // int counter = 0;
        for (auto const& it : ctx->solver_property) {
            message << "\t" << it.first << ": " << it.second << " \n";
            // std::cout << counter << std::endl;
            // counter++;
        }
        RCUTILS_LOG_INFO_NAMED(get_name(), (message.str()).c_str());
    // }
  
}


void etaslNode::initialize_joints(){
    jpos_etasl  = VectorXd::Zero(slv->getNrOfJointStates());   
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "============The jointnames are:   ");


    for (unsigned int i=0;i<jointnames.size();++i) {
        std::map<std::string,int>::iterator p = jindex.find(jointnames[i]);
        if (p!=jindex.end()) {
            jnames_in_expr.push_back(jointnames[i]);
            std::string message = "Joint " + std::to_string(i+1) + ": " + jointnames[i];
            // RCUTILS_LOG_INFO_NAMED(get_name(), message);
            RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());
        }
    } 
    jpos_ros  = VectorXd::Zero(jnames_in_expr.size()); 
    // jpos_etasl = VectorXd::Zero(jnames_in_expr.size()); 
    std::string message = "Number of matching jointnames with robot variables in exp. graph: "+ std::to_string(jnames_in_expr.size());
    RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());

    name_ndx.clear();
    for (unsigned int i=0;i<jnames_in_expr.size();++i) {
        name_ndx[ jnames_in_expr[i]]  =i;
    }

    




    // for (int i = 0; i < jointnames_etasl.size(); ++i) {
    //   name_ndx[jointnames_etasl[i]] = i;
    // }

    driver_to_etasl.resize(jointnames_drivers.size(), -1);
    for (size_t i = 0; i < jointnames_drivers.size(); ++i) {
        auto it = name_ndx.find(jointnames_drivers[i]);
        if (it != name_ndx.end()) {
            driver_to_etasl[i] = it->second;
        }// Else, it remains -1
    }

    // build the reverse mapping too
    etasl_to_driver.resize(jnames_in_expr.size(), -1);
    for (size_t i = 0; i < driver_to_etasl.size(); ++i) {
        if (driver_to_etasl[i] != -1) {
            etasl_to_driver[driver_to_etasl[i]] = i;
        }
    }


    // RCUTILS_LOG_INFO_NAMED(get_name(), "holaaa");

    // Read initial joint positions from the robot feedback
    std::vector<float> jpos_init_vec = multiple_robotdriver_managers->get_position_feedback();
    VectorXd jpos_init;
    jpos_init  = VectorXd::Zero(jnames_in_expr.size()); 


    // assert(jpos_init_vec.size() == jpos_init.size());


    for (unsigned int i=0;i<jointnames_drivers.size();++i) {
      std::map<std::string,int>::iterator it = name_ndx.find(jointnames_drivers[i]);
      if (it!=name_ndx.end()) {
        jpos_init[it->second] = jpos_init_vec[i];
      }
    }

    std::cout << "--------------------------------------------------The initial joint positions read from the robot drivers are: " << std::endl;

    for(unsigned int i=0;i<jpos_init_vec.size();++i) {
      std::cout << jpos_init_vec[i] << ", ";
    }
    std::cout << std::endl;

    for(unsigned int i=0;i<jpos_init.size();++i) {
      std::cout << jpos_init[i] << ", ";
    }
    std::cout << std::endl;

    

    update_controller_input(jpos_init);
        // RCUTILS_LOG_INFO_NAMED(get_name(), "holaaaaaaaaa");


    if(jnames_in_expr.size()==0){
        RCUTILS_LOG_ERROR_NAMED(get_name(), "None of the robot_joints specified in the JSON configuration correspond the joints defined in the eTaSL robot expression graph.");
    }
    else if(jnames_in_expr.size() != jointnames.size()){
      RCUTILS_LOG_WARN_NAMED(get_name(), "The number of robot_joints specified in the JSON configuration do not correspond to all the joints defined in the eTaSL robot expression graph.");
      RCUTILS_LOG_WARN_NAMED(get_name(), "The jointnames that do not correspond are ignored and not published");
    }

}


void etaslNode::initialize_feature_variables(){
    // initialization of robot and feature variables: 
    FeatureVariableInitializer::Ptr initializer = createFeatureVariableInitializer(slv, ctx, ctx->solver_property);
    int retval = initializer->prepareSolver();
    if (retval!=0) {
        RCUTILS_LOG_ERROR_NAMED(get_name(), (initializer->errorMessage(retval)).c_str());
        auto transition = this->shutdown(); //calls on_shutdown() hook.
        // this->safe_shutdown();
        return;
    }


    slv->prepareExecution(ctx);
    slv->getJointNameToIndex(jindex);
    this->initialize_joints();
    
    fpos_etasl = VectorXd::Zero(slv->getNrOfFeatureStates()); // choose the initial feature variables for the initializer, i.e.
    // std::cout <<  "Before initialization\njpos = " << jpos_etasl.transpose() << "\nfpos_etasl = " << fpos_etasl.transpose() << std::endl;
    initializer->setRobotState(jpos_etasl);
    // initializer->setFeatureState(fpos_etasl); //TODO: should I leave it out? leave this out if you want to use the initial values in the task specification.
    retval = initializer->initialize_feature_variables();
    if (retval!=0) {
        RCUTILS_LOG_ERROR_NAMED(get_name(), (initializer->errorMessage(retval)).c_str());
        auto transition = this->shutdown(); //calls on_shutdown() hook.
        // this->safe_shutdown();
        return; 
    }

    fpos_etasl = initializer->getFeatureState();
    // std::cout <<  "After initialization\njpos = " << jpos_etasl.transpose() << "\nfpos_etasl = " << fpos_etasl.transpose() << std::endl;
    // now both jpos_etasl and fpos_etasl are properly initiallized
    std::vector<int> fndx;
    ctx->getScalarsOfType("feature", fndx);
    // ctx->getScalarsOfType("robot", jndx);
    for (size_t i = 0; i < fndx.size(); ++i) {
        auto s = ctx->getScalarStruct(fndx[i]);
        fnames.push_back(s->name);
    }
  
}


void etaslNode::configure_etasl(){
    
 
    /**
     * read task specification and creation of solver and handlers for monitor, inputs, outputs
     */


    // reset all monitors:
    ctx->resetMonitors();

    /****************************************************
     * Solver configuration based with parameters
     ***************************************************/
    this->solver_configuration();

    /****************************************************
     * Initialization
     ***************************************************/    
    // initial input (e.g. robot joints) is used for initialization.                
    this->initialize_feature_variables();

    /****************************************************
     * Update input handlers in the configure_input_handlers (i.e. read values for solver initialization)
     ***************************************************/
    io_handler_manager->configure_input_handlers(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);
    io_handler_manager->configure_output_handlers(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);


    // Prepare the solver for execution, define output variables for both robot joints and feature states: 
    slv->setTime(0.0);
    slv->setFeatureStates(fpos_etasl);
    slv->setJointStates(jpos_etasl);

    // create observers for monitoring events:       
    Observer::Ptr obs = create_port_observer( ctx, events_pub_, "portevent","",false,false,LUA);
    obs = create_port_observer( ctx, events_pub_, "event",event_postfix,false, false, LUA, obs);
    obs = create_port_observer( ctx, events_pub_, "exit", event_postfix,true, false, LUA, obs);
    obs = create_port_observer( ctx, events_pub_, "debug", event_postfix,true, true, LUA, obs);

    //obs = create_default_observer(ctx,"exit",obs);
    ctx->addDefaultObserver(obs);

    // fvelocities.resize( fnames.size() ); 
    // fvelocities = Eigen::VectorXd::Zero( fnames.size() );
    jvel_etasl = VectorXd::Zero(slv->getNrOfJointStates());
    fvel_etasl  = VectorXd::Zero(slv->getNrOfFeatureStates());

    {
        // not really necessary, getting the full state such that we can print out the total number of opt. vars:
        Eigen::VectorXd state;
        slv->getState(state);

        std::stringstream message;
        message << "The size of the initial state of the solver is: " <<  std::to_string(state.size());
        RCUTILS_LOG_INFO_NAMED(get_name(), (message.str()).c_str());

        std::stringstream message2;
        message2 << "The initial state of the solver is: " << state.transpose();
        RCUTILS_LOG_INFO_NAMED(get_name(), (message2.str()).c_str());
    }

}






void etaslNode::update()
{       

        // if(stopFlagPtr->load()) //This checks if the stopFlag has been triggered, e.g. by an error in the robot driver
        // {
        //   rclcpp::shutdown();
        //   return;
        // }
        // gets inputs, this can includes joint values in jpos,
        // which will be overwritten if used.
        io_handler_manager->update_input_handlers(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);

        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            RCUTILS_LOG_INFO_NAMED(get_name(), "Finishing execution because exit monitor was triggered.");
            // event_msg.data = "etasl_finished";
            // events_pub_->publish(event_msg);
            auto transition = this->deactivate();
            RCUTILS_LOG_INFO_NAMED(get_name(), "===============================================================");
            if (transition.label() == "inactive") {
              RCUTILS_LOG_INFO_NAMED(get_name(), "Node automatically deactivated due to triggered exit monitor");
            }
            else {
              RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to automatically deactivate node when the etasl monitor was triggered");
            }
              RCUTILS_LOG_INFO_NAMED(get_name(), "===============================================================");
            return;
            // break;
        }

      update_controller_output(jpos_etasl);
       
         
        // set states:
        slv->setTime(time);
        slv->setJointStates(jpos_etasl);
        slv->setFeatureStates(fpos_etasl);
        // solve
        int c = slv->solve();
        if (c!=0) {

            std::string message = "The solver encountered the following error : \n" + slv->errorMessage(c);
            RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
            auto transition = this->shutdown(); //calls on_shutdown() hook.
            // this->safe_shutdown();
            return;
            // break;
        }
        // get outputs:
        slv->getJointVelocities(jvel_etasl);
        slv->getFeatureVelocities(fvel_etasl);

        update_robot_status();

        io_handler_manager->update_output_handlers(jnames_in_expr, jpos_ros, jvel_etasl, fnames, fvel_etasl, fpos_etasl);
}

void etaslNode::update_robot_status(){

    // jpos_etasl += jvel_etasl*(periodicity_ms/1000.0);  // or replace with reading joint positions from real robot
    fpos_etasl += fvel_etasl*(periodicity_ms/1000.0);  // you always integrate feature variables yourself
    time += (periodicity_ms/1000.0);


    //construct jvel_all_drivers from jvel_etasl

    //TODO: The search does not need to be done at every iteration. It can be done once in the configuration phase
    // for (unsigned int i=0;i<jointnames_drivers.size();++i) {
    //   std::map<std::string,int>::iterator it = name_ndx.find(jointnames_drivers[i]);
    //   if (it!=name_ndx.end()) {
    //     jvel_all_drivers[i] = jvel_etasl[it->second]; //joints that will be used for ros topic
    //   }
    //   else{
    //     jvel_all_drivers[i] = 0.0; //if the joint is not used in the expression graph, then we set its velocity to zero
    //   }
    // }

    // Fill jvel_all_drivers from jvel_etasl
    for (size_t i = 0; i < driver_to_etasl.size(); ++i) {
      if (driver_to_etasl[i] != -1) {
          jvel_all_drivers[i] = jvel_etasl[driver_to_etasl[i]];
      } else {
          jvel_all_drivers[i] = 0.0; // joint not used in ETaSL
      }
    }

    multiple_robotdriver_managers->update(joint_positions_feedback, jvel_all_drivers);


    //TODO: The search does not need to be done at every iteration. It can be done once in the configuration phase
    // for (unsigned int i=0;i<jointnames_drivers.size();++i) {
    //   std::map<std::string,int>::iterator it = name_ndx.find(jointnames_drivers[i]);
    //   if (it!=name_ndx.end()) {
    //     jpos_etasl[it->second] = joint_positions_feedback[i];
    //   }
    // }

    for (size_t etasl_idx = 0; etasl_idx < etasl_to_driver.size(); ++etasl_idx) {
        if (etasl_to_driver[etasl_idx] != -1) {
            jpos_etasl[etasl_idx] = joint_positions_feedback[etasl_to_driver[etasl_idx]];
        }
    }
    
    // if (feedback_copy_ptr->joint.is_pos_available){
      // for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
      //   jpos_etasl[i] = feedback_copy_ptr->joint.pos.data[i]; //required only for positions
      // }
    // }
    
}

void etaslNode::reinitialize_data_structures() {
    ctx = create_context();
    slv.reset();
    ctx->addType("robot");
    ctx->addType("feature");
    LUA = std::make_shared<LuaContext>();
    // define a variable that only depends on time
    LUA->initContext(ctx);

    solver_registry.reset();
    // registerSolverFactory_qpOases(solver_registry, "qpoases");


    time = 0.0;
    fpos_etasl = VectorXd::Zero(0);
    jvel_etasl = VectorXd::Zero(0);
    fvel_etasl = VectorXd::Zero(0);

    // jointnames.clear();
    jnames_in_expr.clear();

    jindex.clear();
    name_ndx.clear();
    fnames.clear();
    
}


void etaslNode::construct_node(std::atomic<bool>* stopFlagPtr_p){

  //This method is necessary becaus shared_from_this() does not work in the constructor of the class (as it returns something that does not yet exist)
    //shared_from_this() is used for all objects that need a reference to the ROS2 node.
    stopFlagPtr = stopFlagPtr_p;

      // Json::Value param_root = board->getPath(".", false);
    Json::Value param_root = board->getLocal();
    // std::cout << "param_root: " << param_root.toStyledString() << std::endl;
    io_handler_manager = std::make_shared<etasl::IOHandlerManager>(shared_from_this(), param_root, jsonchecker);
    std::cout << "constructing input handlers" << std::endl;

    Json::Value param = board->getPath("/robot", false);

    jointnames.clear();
    for (auto n : jsonchecker->asArray(param, "default_robot_specification/robot_joints")) {
        jointnames.push_back(jsonchecker->asString(n, ""));
    }



    /****************************************************
    * Adding Robot Driver
    ***************************************************/  
    multiple_robotdriver_managers = std::make_shared<etasl::MultipleDriversManager>(shared_from_this(), param_root, jsonchecker, simulation, stopFlagPtr);
    multiple_robotdriver_managers->construct_drivers(joint_positions_feedback);  //constructs and initializes communication with the robot, and resizes joint_positions_feedback
    jvel_all_drivers = VectorXd::Zero(joint_positions_feedback.size()); //After construct_drivers(joint_positions_feedback) is called, joint_positions_feedback has the correct size
    jointnames_drivers = multiple_robotdriver_managers->get_robot_joints();


    /****************************************************
    * Adding input and output handlers from the read JSON file 
    ***************************************************/
    io_handler_manager->construct_input_handlers();
    io_handler_manager->construct_output_handlers();


}


 /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  lifecycle_return etaslNode::on_configure(const rclcpp_lifecycle::State & state)
  {
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initialize and configure our publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.



 // This still needs to be here and cannot be moved to construct_node() function because of the routine for verifying the joints in the expression. That routine should change 
    if(!first_time_configured){

      bool are_robot_drivers_initialized = multiple_robotdriver_managers->initialize(ctx);

      if(!are_robot_drivers_initialized){
        return lifecycle_return::ERROR; //The message log and shutdown is done inside the multiple_robotdriver_managers
      }


      timer_ = this->create_wall_timer(std::chrono::milliseconds(periodicity_ms), std::bind(&etaslNode::update, this));

      // std::vector<double> jpos_init_vec; //TODO: This variable is no longer needed and can be replaced by feedback_copy_ptr->joint.pos.data
      // jpos_init_vec.resize(feedback_copy_ptr->joint.pos.data.size(),0.0);
      // for(unsigned int i = 0; i < feedback_copy_ptr->joint.pos.data.size(); ++i){
      //   jpos_init_vec[i] = feedback_copy_ptr->joint.pos.data[i];
      // }

      // jpos_init = VectorXd::Zero(jpos_init_vec.size());
      // for (unsigned int i = 0; i < jpos_init_vec.size(); ++i) {
      //   jpos_init[i] = jpos_init_vec[i];
      // }

      io_handler_manager->initialize_input_handlers(ctx, jnames_in_expr, fnames, jpos_ros, fpos_etasl);
      io_handler_manager->initialize_output_handlers(ctx, jnames_in_expr, fnames);

    }
    else{
      // jpos_init = jpos_etasl;

      // std::vector<float> jpos_init_vec = multiple_robotdriver_managers->get_position_feedback(); //This method can only be used after the first time configuration

      // // jpos_init = VectorXd::Zero(jpos_init_vec.size());
      // assert(jpos_init_vec.size() == jpos_init.size());

      // for (unsigned int i = 0; i < jpos_init_vec.size(); ++i) {
      //   jpos_init[i] = jpos_init_vec[i];
      // }

    }


  
    
    timer_->cancel();


    this->configure_etasl();

    multiple_robotdriver_managers->on_configure(ctx);





    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    
    first_time_configured = true;
    is_configured = true;

    return lifecycle_return::SUCCESS;
  }

  /// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  lifecycle_return etaslNode::on_activate(const rclcpp_lifecycle::State & state)
  {
    // We explicitly activate the lifecycle publisher.
    // Starting from this point, all messages are no longer
    // ignored but sent into the network.

    if (!is_configured){
          RCUTILS_LOG_ERROR_NAMED(get_name(), "The node be activated immediatly after calling lifecycle deactivate or lifecycle cleanup. The node must be configured with lifecycle configure first.");
          return lifecycle_return::FAILURE;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "Entering on activate.");

    timer_->reset();

    multiple_robotdriver_managers->on_activate();

    // TODO: Handle erros in activate and return lifecycle_return::FAILURE instead
    io_handler_manager->activate_input_handlers(ctx, jnames_in_expr, fnames, slv);
    io_handler_manager->activate_output_handlers(ctx, jnames_in_expr, fnames, slv);


    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");


    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }

  /// Transition callback for state deactivating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  lifecycle_return etaslNode::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    // We explicitly deactivate the lifecycle publisher.
    // Starting from this point, all messages are no longer
    // sent into the network.
    
    timer_->cancel();

    jvel_etasl.setZero(); //Sets joint velocities to zero
    fvel_etasl.setZero(); //Sets feature variables to zero
    jvel_all_drivers.setZero(); //Sets joint velocities (for robot drivers) to zero

    update_robot_status(); //Updates structures that the robot thread reads from shared memory, ensuring zero velocities

    multiple_robotdriver_managers->on_deactivate();
    io_handler_manager->deactivate_input_handlers(ctx);
    io_handler_manager->deactivate_output_handlers(ctx);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    is_configured = false; //To indicate that it has not being configured after deactivating node

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }

  /// Transition callback for state cleaningup
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  lifecycle_return etaslNode::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    jvel_etasl.setZero(); //Sets joint velocities to zero
    fvel_etasl.setZero(); //Sets feature variables to zero

    multiple_robotdriver_managers->on_cleanup();

    io_handler_manager->cleanup_input_handlers(ctx);
    io_handler_manager->cleanup_output_handlers(ctx);

    timer_->cancel();
    this->reinitialize_data_structures();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    is_configured = false; //To indicate that it has not being configured after cleanup node
    // Json::Value param = board->getPath("/default-etasl", false);

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }

  // on_shutdown is only called when calling shutdown() method and not when closing program e.g. with ctrl+c.
  // Its kind of useless...

  /// Transition callback for state shutting down
  /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  lifecycle_return etaslNode::on_shutdown(const rclcpp_lifecycle::State & state)
  {
    
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());

    this->safe_shutdown();
    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }


  /**
   * @brief Performs a safe shutdown, mainly calling all finalize methods of IO handlers and robot drviers
   * This function is passed as a callback function to 
   * @return (void)
   */
  void etaslNode::safe_shutdown(){
    // RCUTILS_LOG_INFO_NAMED(get_name(), "Program shutting down safely.");

    std::cout << "Program shutting down safely." << std::endl;
    stopFlagPtr->store(true); //Stops execution of driver_thread after executor e.g. when interrupted with ctr+c signal    

    multiple_robotdriver_managers->finalize();

    io_handler_manager->finalize_input_handlers();
    io_handler_manager->finalize_output_handlers();
    
  }


  std::vector<std::shared_ptr<t_manager::thread_t>> etaslNode::create_driver_threads_structures(std::atomic<bool> & stopFlag){

    return multiple_robotdriver_managers->create_driver_threads_structures(stopFlag);

  }



int main(int argc, char * argv[])
{

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize node
    rclcpp::init(argc, argv);


    std::shared_ptr<etaslNode> my_etasl_node = std::make_shared<etaslNode>("etasl_node");
    // auto my_etasl_node = std::make_shared<etaslNode>("etasl_node");

    std::atomic<bool> stopFlag(false); 
    
    my_etasl_node->construct_node(&stopFlag);



    // std::shared_ptr<t_manager::thread_t> thread_str_driver = my_etasl_node->create_thread_str(stopFlag);

    std::vector<std::shared_ptr<t_manager::thread_t>> thread_str_drivers_vec =  my_etasl_node->create_driver_threads_structures(stopFlag);

    std::vector<std::thread> driver_threads_vec;
    driver_threads_vec.resize(thread_str_drivers_vec.size());

    for (unsigned int i = 0; i < thread_str_drivers_vec.size(); ++i) {
      std::shared_ptr<t_manager::thread_t> thread_str_driver = thread_str_drivers_vec[i];
      driver_threads_vec[i] = std::thread(t_manager::do_thread_loop, thread_str_driver, std::ref(stopFlag));
      t_manager::setScheduling(driver_threads_vec[i], SCHED_FIFO, 90);
    }

    // std::thread driver_thread(t_manager::do_thread_loop, thread_str_driver, std::ref(stopFlag));
    // t_manager::setScheduling(driver_thread, SCHED_FIFO, 90);
    // driver_thread.detach();// Avoids the main thread to block. See spin() + stopFlag mechanism below.


    rclcpp::ExecutorOptions options;
    options.context = my_etasl_node->get_node_base_interface()->get_context(); //necessary to avoid unexplainable segmentation fault from the ros rclcpp library!!!
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(my_etasl_node->get_node_base_interface());
    executor.spin(); //This method blocks until shutdown!

    stopFlag.store(true); //Stops execution of driver_thread after executor is interrupted with ctr+c signal

    for (auto& thr : driver_threads_vec) {
      thr.join(); //Blocks until each driver_thread stops, after the stopFlag is set to true
    }

    // driver_thread.join();//Blocks until driver_thread stops, after the stopFlag is set to true

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Needed for the robotdriver thread to stop properly before calling shutdown. Otherwise segmentation fault is observed. This is because shutdown deletes something from the robotdriver as now ros2 pluginlib is being used.
    
    executor.remove_node(my_etasl_node->get_node_base_interface());

    rclcpp::shutdown();

    RCUTILS_LOG_INFO("Program terminated correctly.");
  return 0;
}