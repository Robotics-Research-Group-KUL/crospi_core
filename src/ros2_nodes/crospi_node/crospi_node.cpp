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

#include "crospi_node.hpp"
#include "port_observer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp> 

#include "crospi_utils/etasl_error.hpp"
#include "crospi_utils/flowstatus.hpp"
#include <crospi_utils/string_interpolate.hpp> //To e.g. specify $[crospi_core]/launch/etc... and other functionalities in strings

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

  srv_etasl_console_ = create_service<std_srvs::srv::Empty>("crospi_node/etasl_console", std::bind(&etaslNode::etasl_console, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationFile_ = create_service<crospi_interfaces::srv::TaskSpecificationFile>("crospi_node/readTaskSpecificationFile", std::bind(&etaslNode::readTaskSpecificationFile, this, std::placeholders::_1, std::placeholders::_2));
  srv_readRobotSpecification_ = create_service<crospi_interfaces::srv::TaskSpecificationFile>("crospi_node/readRobotSpecification", std::bind(&etaslNode::readRobotSpecification, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationString_ = create_service<crospi_interfaces::srv::TaskSpecificationString>("crospi_node/readTaskSpecificationString", std::bind(&etaslNode::readTaskSpecificationString, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskParameters_ = create_service<crospi_interfaces::srv::TaskSpecificationString>("crospi_node/readTaskParameters", std::bind(&etaslNode::readTaskParameters, this, std::placeholders::_1, std::placeholders::_2));

  std::string configFilePath = this->declare_parameter<std::string>("config_file", "");
  bool simulation_param = this->declare_parameter<bool>("simulation", true);

// Checks if the path is empty. i.e. the parameter was not defined.
  if (configFilePath.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No directory path provided. Use the 'config_file' ros parameter as follows: \n ros2 run crospi_core crospi_node --ros-args -p config_file:=/path/to/directory");
      rclcpp::shutdown();
      return;
  }

  // Checks if the path is in the appropriate format.
  static const std::regex pattern_file(R"(^\$\[([a-zA-Z0-9_]+)\]\/.+\.json$)");
  std::smatch matched_package;
  std::string package_name;

  bool matches_file_pattern = std::regex_match(configFilePath,matched_package, pattern_file);
  if(!matches_file_pattern){
    std::string message = "The format of the config_file:=" + configFilePath + " provided as --ros-arg: is not valid.\n It should contain a valid ROS2 package based on the crospi_application_template, and it should be a JSON file as follows: $[my_ros2_package]/applications/application_example_ur10/application_example_ur10.setup.json";
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

  // path_components_used: if -1, all parts of the local path in the URI are returned. If >=0, only the last path_components_used parts are retained.
  board = std::make_shared<etasl::BlackBoard>(1);

  std::string location_of_bundled_schema = etasl::string_interpolate(fmt::format("$[{}]/schemas/generated", package_name));

  board->setSearchPath(location_of_bundled_schema); //Selects search path based on the package provided in the config file

  board->load_process_and_validate(configFilePath); // Also checks if it is a valid JSON file

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

bool etaslNode::readTaskSpecificationFile(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Response>  response) {

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service crospi_node/readTaskSpecificationFile can only be read in unconfigured state");
		response->success = false;
		return false;
	}

  std::string file_path = etasl::string_interpolate(request->file_path);

	try{
		// Read eTaSL specification:
		int retval = LUA->executeFile(std::string(file_path));
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing task specification file. Please provide a valid task specification file");
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

bool etaslNode::readRobotSpecification(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Response>  response) {

  Json::Value param_original = board->getPath("/robot", false);
  Json::Value param = param_original; //Copy

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service crospi_node/readTaskSpecificationFile can only be read in unconfigured state");
		response->success = false;
		return false;
	}

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

bool etaslNode::readTaskSpecificationString(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Response>  response) {

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service crospi_node/readTaskSpecificationString can only be read in unconfigured state");
		response->success = false;
		return false;
	}

	try{
		// Read eTaSL specification:
		int retval = LUA->executeString(request->str);
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing the following specificed string command in LUA within the crospi_node/readTaskSpecificationString service: ");
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

bool etaslNode::readTaskParameters(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Response>  response) {

	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service crospi_node/readTaskParameters can only be read in unconfigured state");
		response->success = false;
		return false;
	}

	try{
		// Read eTaSL specification:
    std::string param_request = "_JSON_TASK_SPECIFICATION_PARAMETERS_STRING='" + request->str + "'"; 

    RCUTILS_LOG_INFO_NAMED(get_name(), "The received task parameters are:\n%s", request->str.c_str());

		int retval = LUA->executeString(param_request);
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error interpreting the following string as a JSON string in LUA after calling the crospi_node/readTaskParameters service: ");
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

    // we can get the properties from the solver from the context and specify these properties in eTaSL or
    // create a parameter plist (instead of ctx->solver_property) :
        // parameters of the solver:

        // TODO: Delete the following after handling solvers with register factory (same as with IO handlers)
        if (!jsonchecker->asBool(param, "solver/is-qpoasessolver")){
          RCUTILS_LOG_ERROR_NAMED(get_name(), "is-qpoasessolver should be true in the JSON definition since currently only qpoases solver is supported.");
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
        return;
    }
    ctx->setSolverProperty("sample_time", periodicity_ms/1000.0);

    std::stringstream message;
    message << "Solver and initialization properties : \n";
    for (auto const& it : ctx->solver_property) {
        message << "\t" << it.first << ": " << it.second << " \n";
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), (message.str()).c_str());
}

void etaslNode::initialize_joints(){
    jpos_etasl  = VectorXd::Zero(slv->getNrOfJointStates());   

    RCUTILS_LOG_INFO_NAMED(get_name(), "============The jointnames are:   ");

    for (unsigned int i=0;i<jointnames.size();++i) {
        std::map<std::string,int>::iterator p = jindex.find(jointnames[i]);
        if (p!=jindex.end()) {
            jnames_in_expr.push_back(jointnames[i]);
            std::string message = "Joint " + std::to_string(i+1) + ": " + jointnames[i];
            RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());
        }
    } 
    jpos_ros  = VectorXd::Zero(jnames_in_expr.size()); 
    std::string message = "Number of matching jointnames with robot variables in exp. graph: "+ std::to_string(jnames_in_expr.size());
    RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());

    name_ndx.clear();
    for (unsigned int i=0;i<jnames_in_expr.size();++i) {
        name_ndx[ jnames_in_expr[i]]  =i;
    }

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

    // Read initial joint positions from the robot feedback
    std::vector<float> jpos_init_vec = multiple_robotdriver_managers->get_position_feedback();
    VectorXd jpos_init;
    jpos_init  = VectorXd::Zero(jnames_in_expr.size());

    for (unsigned int i=0;i<jointnames_drivers.size();++i) {
      std::map<std::string,int>::iterator it = name_ndx.find(jointnames_drivers[i]);
      if (it!=name_ndx.end()) {
        jpos_init[it->second] = jpos_init_vec[i];
      }
    }

    std::stringstream jpos_init_msg;
    jpos_init_msg << "The initial joint positions read from the robot drivers are:\n\tdriver order: ";
    for(unsigned int i=0;i<jpos_init_vec.size();++i) {
      jpos_init_msg << jpos_init_vec[i] << ", ";
    }
    jpos_init_msg << "\n\teTaSL order:  ";
    for(unsigned int i=0;i<jpos_init.size();++i) {
      jpos_init_msg << jpos_init[i] << ", ";
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "%s", jpos_init_msg.str().c_str());

    update_controller_input(jpos_init);

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
        return;
    }

    slv->prepareExecution(ctx);
    slv->getJointNameToIndex(jindex);
    this->initialize_joints();

    fpos_etasl = VectorXd::Zero(slv->getNrOfFeatureStates()); // choose the initial feature variables for the initializer, i.e.
    initializer->setRobotState(jpos_etasl);
    // The feature state is intentionally not set here, so that the initial values from the task specification are used.
    retval = initializer->initialize_feature_variables();
    if (retval!=0) {
        RCUTILS_LOG_ERROR_NAMED(get_name(), (initializer->errorMessage(retval)).c_str());
        auto transition = this->shutdown(); //calls on_shutdown() hook.
        return; 
    }

    fpos_etasl = initializer->getFeatureState();
    // now both jpos_etasl and fpos_etasl are properly initiallized
    std::vector<int> fndx;
    ctx->getScalarsOfType("feature", fndx);
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

    ctx->addDefaultObserver(obs);

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

        // gets inputs, this can includes joint values in jpos,
        // which will be overwritten if used.
        io_handler_manager->update_input_handlers(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);

        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            RCUTILS_LOG_INFO_NAMED(get_name(), "Finishing execution because exit monitor was triggered.");
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
            return;
        }
        // get outputs:
        slv->getJointVelocities(jvel_etasl);
        slv->getFeatureVelocities(fvel_etasl);

        update_robot_status();

        io_handler_manager->update_output_handlers(jnames_in_expr, jpos_ros, jvel_etasl, fnames, fvel_etasl, fpos_etasl);
}

void etaslNode::update_robot_status(){

    fpos_etasl += fvel_etasl*(periodicity_ms/1000.0);  // you always integrate feature variables yourself
    time += (periodicity_ms/1000.0);

    //construct jvel_all_drivers from jvel_etasl

    // Fill jvel_all_drivers from jvel_etasl
    for (size_t i = 0; i < driver_to_etasl.size(); ++i) {
      if (driver_to_etasl[i] != -1) {
          jvel_all_drivers[i] = jvel_etasl[driver_to_etasl[i]];
      } else {
          jvel_all_drivers[i] = 0.0; // joint not used in ETaSL
      }
    }

    multiple_robotdriver_managers->update(joint_positions_feedback, jvel_all_drivers);

    for (size_t etasl_idx = 0; etasl_idx < etasl_to_driver.size(); ++etasl_idx) {
        if (etasl_to_driver[etasl_idx] != -1) {
            jpos_etasl[etasl_idx] = joint_positions_feedback[etasl_to_driver[etasl_idx]];
        }
    }

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

    time = 0.0;
    fpos_etasl = VectorXd::Zero(0);
    jvel_etasl = VectorXd::Zero(0);
    fvel_etasl = VectorXd::Zero(0);

    jnames_in_expr.clear();

    jindex.clear();
    name_ndx.clear();
    fnames.clear();

}

void etaslNode::construct_node(std::atomic<bool>* stopFlagPtr_p){

  //This method is necessary becaus shared_from_this() does not work in the constructor of the class (as it returns something that does not yet exist)
    //shared_from_this() is used for all objects that need a reference to the ROS2 node.
    stopFlagPtr = stopFlagPtr_p;

    Json::Value param_root = board->getLocal();
    io_handler_manager = std::make_shared<etasl::IOHandlerManager>(shared_from_this(), param_root, jsonchecker);

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

 // This still needs to be here and cannot be moved to construct_node() function because of the routine for verifying the joints in the expression. That routine should change 
    if(!first_time_configured){

      bool are_robot_drivers_initialized = multiple_robotdriver_managers->initialize(ctx);

      if(!are_robot_drivers_initialized){
        return lifecycle_return::ERROR; //The message log and shutdown is done inside the multiple_robotdriver_managers
      }

      timer_ = this->create_wall_timer(std::chrono::milliseconds(periodicity_ms), std::bind(&etaslNode::update, this));

      io_handler_manager->initialize_input_handlers(ctx, jnames_in_expr, fnames, jpos_ros, fpos_etasl);
      io_handler_manager->initialize_output_handlers(ctx, jnames_in_expr, fnames);
    }

    timer_->cancel();

    this->configure_etasl();

    multiple_robotdriver_managers->on_configure(ctx);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

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
    jvel_etasl.setZero(); //Sets joint velocities to zero
    fvel_etasl.setZero(); //Sets feature variables to zero

    multiple_robotdriver_managers->on_cleanup();

    io_handler_manager->cleanup_input_handlers(ctx);
    io_handler_manager->cleanup_output_handlers(ctx);

    timer_->cancel();
    this->reinitialize_data_structures();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    is_configured = false; //To indicate that it has not being configured after cleanup node

    return lifecycle_return::SUCCESS;
  }

  // on_shutdown is only called when calling shutdown() method and not when closing program e.g. with ctrl+c.

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

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called from state %s.", state.label().c_str());

    this->safe_shutdown();
    return lifecycle_return::SUCCESS;
  }

  /**
   * @brief Performs a safe shutdown, mainly calling all finalize methods of IO handlers and robot drviers
   * This function is passed as a callback function to 
   * @return (void)
   */
  void etaslNode::safe_shutdown(){

    RCUTILS_LOG_INFO_NAMED(get_name(), "Program shutting down safely.");
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

    std::shared_ptr<etaslNode> my_crospi_node = std::make_shared<etaslNode>("crospi_node");

    std::atomic<bool> stopFlag(false); 

    my_crospi_node->construct_node(&stopFlag);

    std::vector<std::shared_ptr<t_manager::thread_t>> thread_str_drivers_vec =  my_crospi_node->create_driver_threads_structures(stopFlag);

    std::vector<std::thread> driver_threads_vec;
    driver_threads_vec.resize(thread_str_drivers_vec.size());

    for (unsigned int i = 0; i < thread_str_drivers_vec.size(); ++i) {
      std::shared_ptr<t_manager::thread_t> thread_str_driver = thread_str_drivers_vec[i];
      driver_threads_vec[i] = std::thread(t_manager::do_thread_loop, thread_str_driver, std::ref(stopFlag));
      t_manager::setScheduling(driver_threads_vec[i], SCHED_FIFO, 90);
    }

    rclcpp::ExecutorOptions options;
    options.context = my_crospi_node->get_node_base_interface()->get_context(); //necessary to avoid unexplainable segmentation fault from the ros rclcpp library!!!
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(my_crospi_node->get_node_base_interface());
    executor.spin(); //This method blocks until shutdown!

    stopFlag.store(true); //Stops execution of driver_thread after executor is interrupted with ctr+c signal

    for (auto& thr : driver_threads_vec) {
      thr.join(); //Blocks until each driver_thread stops, after the stopFlag is set to true
    }

    executor.remove_node(my_crospi_node->get_node_base_interface());

    rclcpp::shutdown();

    RCUTILS_LOG_INFO("Program terminated correctly.");
  return 0;
}