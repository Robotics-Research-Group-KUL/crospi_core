#include "etasl_node.hpp"
#include "IO_handlers_deleteme.hpp"
#include "port_observer.hpp"

# include <ament_index_cpp/get_package_share_directory.hpp> 

// #include "etasl_task_utils/blackboard.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/flowstatus.hpp"



#include <jsoncpp/json/json.h>


// For real-time control loop
#include <chrono>
#include <thread>
#include <vector>





using namespace std::chrono_literals;
using namespace KDL;
using namespace Eigen;
// using namespace etasl;



/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


etaslNode::etaslNode(const std::string & node_name, bool intra_process_comms = false): rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
, periodicity_param(10) //Expressed in milliseconds
, time(0.0)
, event_msg(std_msgs::msg::String())
, event_postfix(get_name())
, first_time_configured(false)
, is_configured(false)
{
  //Used unless the ROS parameters are modified externally (e.g. through terminal or launchfile)


  // this->declare_parameter("task_specification_file",  rclcpp::PARAMETER_STRING);
  // this->declare_parameter("jointnames",  rclcpp::PARAMETER_STRING_ARRAY);

  // fname = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/taskspec2.lua";
  // this->declare_parameter("outpfilename",  outpfilename); //outpfilename as default val
  // this->declare_parameter("task_specification_file",  fname); //fname as default val

  // this->create_service<lifecycle_msgs::srv::ChangeState>("configure", &srv_configure);
//   test_service_ = create_service<lifecycle_msgs::srv::ChangeState>("etasl_node/configure", std::bind(&etaslNode::srv_configure, this, std::placeholders::_1, std::placeholders::_2));

  srv_etasl_console_ = create_service<std_srvs::srv::Empty>("etasl_node/etasl_console", std::bind(&etaslNode::etasl_console, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationFile_ = create_service<etasl_interfaces::srv::TaskSpecificationFile>("etasl_node/readTaskSpecificationFile", std::bind(&etaslNode::readTaskSpecificationFile, this, std::placeholders::_1, std::placeholders::_2));

  srv_readTaskSpecificationString_ = create_service<etasl_interfaces::srv::TaskSpecificationString>("etasl_node/readTaskSpecificationString", std::bind(&etaslNode::readTaskSpecificationString, this, std::placeholders::_1, std::placeholders::_2));

  events_pub_ = this->create_publisher<std_msgs::msg::String>("fsm/events", 10); 

  reinitialize_data_structures();

  board = boost::make_shared<etasl::BlackBoard>(1);
  // etasl::BlackBoard board(1);
  std::cout << " loading blackboard" << std::endl;
  board->setSearchPath("$[etasl_ros2]/scripts/schema:$[etasl_ros2]/scripts/schema/tasks");
  board->load_process_and_validate("$[etasl_ros2]/scripts/json/blackboard.json");
  fmt::print("{:->80}\n", "-");


  this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(std::bind( &etaslNode::safe_shutdown, this)); // Adds safe_shutdown as callback before shutting down, e.g. with ctrl+c. This methods returns rclcpp::OnShutdownCallbackHandle shutdown_cb_handle
  // this->get_node_base_interface()->get_context()->add_on_shutdown_callback(std::bind( &etaslNode::safe_shutdown, this)); //Can be used to add callback during shutdown (not pre-shutdown, so publishers and others are no longer available)
  // rclcpp::on_shutdown(std::bind( &etaslNode::safe_shutdown, my_etasl_node)); //Alternative to add_on_shutdown_callback (don't know the difference)

}

// bool etaslNode::srv_configure(const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request, std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>  response)
//       {
//           std::cout << "Request id: "<< request->transition.id << std::endl;
//           std::cout << "Request label: "<< request->transition.label << std::endl;
//           std::this_thread::sleep_for(5s);
//           std::cout << "Finish service call: " << std::endl;
//           response->success = true;
//           return true;

//       }

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

	// std::stringstream file_path;
	// file_path << "Solver and initialization properties : \n";
	// file_path.str()).c_str()
	std::string file_path = "";

	if(request->rel_shared_dir){
		std::string shared_dir = ament_index_cpp::get_package_share_directory("etasl_ros2");
		file_path = shared_dir + "/etasl/" + request->file_path;
	}
	else{
		file_path = request->file_path;
	}
	std::cout << file_path <<std::endl; 
	// std::cout << "response:" <<request->file_path <<std::endl; 

	try{
		// Read eTaSL specification:
		int retval = LUA->executeFile(std::string(file_path));
		// int retval = LUA->executeFile("/home/santiregui/ros2_ws/install/etasl_ros2/share/etasl_ros2/etasl/move_cartesianspace.lua");
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


bool etaslNode::readTaskSpecificationString(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Response>  response) {
	
	if(this->get_current_state().label() != "unconfigured"){
		RCUTILS_LOG_ERROR_NAMED(get_name(), "Service etasl_node/readTaskSpecificationString can only be read in unconfigured state");
		response->success = false;
		return false;
	}

	try{
		// Read eTaSL specification:
		int retval = LUA->executeString(request->str);
		// int retval = LUA->executeFile("/home/santiregui/ros2_ws/install/etasl_ros2/share/etasl_ros2/etasl/move_cartesianspace.lua");
		if (retval !=0) {
			RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing specificed string command in LUA within the etasl_node/readTaskSpecificationString service. ");
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


void etaslNode::update_controller_output(Eigen::VectorXd const& jvalues_solver){
    for (unsigned int i=0;i<jnames_in_expr.size();++i) {
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
    Json::Value param = board->getPath("/default-etasl", false);

    solver_registry = boost::make_shared<SolverRegistry>();
    registerSolverFactory_qpOases(solver_registry, "qpoases");
    //registerSolverFactory_hqp(R, "hqp");

    // we can get the properties from the solver from the context and specify these properties in eTaSL or
    // create a parameter plist (instead of ctx->solver_property) :
        // parameters of the solver:

        // TODO: Delete the following after handling solvers with register factory (same as with IO handlers)
        if (!param["solver"]["is-qpoasessolver"].asBool()){
          RCUTILS_LOG_ERROR_NAMED(get_name(), "is-qpoasessolver should be true in the JSON definition since currently only qpoases solver is supported.");
          // this->safe_shutdown();
          auto transition = this->shutdown(); //calls on_shutdown() hook.
          return;
        }

        std::string solver_name             = "qpoases" ;
        ParameterList plist;
        plist["nWSR"]                  = param["solver"]["nWSR"].asDouble();
        plist["regularization_factor"] = param["solver"]["regularization_factor"].asDouble();
        plist["cputime"] = param["solver"]["cputime"].asDouble();
        // parameters of the initialization procedure:
        plist["initialization_full"]                  = int(param["initializer"]["full"].asBool()); // == true
        plist["initialization_duration"]              = param["initializer"]["duration"].asDouble();
        plist["initialization_sample_time"]           = param["initializer"]["sample_time"].asDouble();
        plist["initialization_convergence_criterion"] = param["initializer"]["convergence_criterion"].asDouble();
        plist["initialization_weightfactor"]          = param["initializer"]["weightfactor"].asDouble();

        // std::cout << "(((((((((((((((((())))))))))))))))))" << std::endl;
        // std::cout <<"nWSR: " << plist["nWSR"] << ". full: " << plist["initialization_full"] << std::endl;
        // std::string solver_name             = "qpoases" ;
        // ParameterList plist;
        // plist["nWSR"]                  = 100;
        // plist["regularization_factor"] = 1E-5;
        // plist["cputime"] = 10.0;
        // // parameters of the initialization procedure:
        // plist["initialization_full"]                  = 1.0; // == true
        // plist["initialization_duration"]              = 3.0;
        // plist["initialization_sample_time"]           = 0.01;
        // plist["initialization_convergence_criterion"] = 1E-4;
        // plist["initialization_weightfactor"]          = 100;

        // param["etasl"]["use_sim_time"].asBool()
        

    // std::string solver_name = ctx->getSolverStringProperty("solver", "qpoasis");  // 2nd arg is the default value if nothing is specified.
    // int result = solver_registry->createSolver(solver_name, ctx->solver_property, true, false, slv); 

    int result = solver_registry->createSolver(solver_name, plist, true, false, slv); 
    if (result!=0) {
        std::string message = "Failed to create the solver " + solver_name;
        RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
        auto transition = this->shutdown(); //calls on_shutdown() hook.
        // this->safe_shutdown();
        return;
    }
    ctx->setSolverProperty("sample_time", periodicity_param/1000.0);
    // double dt = ctx->getSolverProperty("sample_time", periodicity_param/1000.0);
    // std::cout<<"sample_time:" << dt << std::endl;


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

    update_controller_input(jpos_init);

    if(jnames_in_expr.size()==0){
        RCUTILS_LOG_WARN_NAMED(get_name(), "None of the joint_names specified in the JSON configuration correspond the joints defined in the eTaSL robot expression graph.");
    }
    else if(jnames_in_expr.size() != jointnames.size()){
      RCUTILS_LOG_WARN_NAMED(get_name(), "The number of joint_names specified in the JSON configuration do not correspond to all the joints defined in the eTaSL robot expression graph.");
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
  
    // fmt::print("blackboard/default-etasl : ");
    Json::Value param = board->getPath("/default-etasl", false);
    // fmt::print("After processing and validating:\n{}", param);
    // fmt::print("{:->80}\n", "-");

    // Read configuration ROS parameters
    // jointnames = this->get_parameter("jointnames").as_string_array();

    jointnames.clear();
    for (auto n : param["robotdriver"]["joint_names"]) {
        jointnames.push_back(n.asString());
    }


    // std::cout << jointnames[0] << "," << jointnames[1] <<std::endl;
 
    /**
     * read task specification and creation of solver and handlers for monitor, inputs, outputs
     */

    // Setup context for robot control problem:
    // ctx->addType("robot");
    // ctx->addType("feature");


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
     * Update input handlers (i.e. read values for solver initialization)
     ***************************************************/
    for (auto h : inputhandlers) {
        h->update(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);
    }


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

    // initialize our outputhandler:
    // std::ofstream outpfile(outpfilename);

    // std::vector<std::string> hello;
    // slv->getJointNameVector(hello);
    //   std::cout << "------------" << "THe jointnames are: "<< std::endl;
    //   for (unsigned int i=0;i<hello.size();++i) {
    //     std::cout << "------------" << hello[i] << std::endl;
    // } 

}



int etaslNode::get_periodicity_param()
{
  return periodicity_param;
}





// TODO: use the following to map jointnames and jvalues. Found in https://etasl.pages.gitlab.kuleuven.be/etasl-api-doc/api/etasl-rtt/solver__state_8hpp_source.html
// void etaslNode::setJointValues(const std::vector<double>& jval, const std::vector<std::string>& jvalnames) {
//     assert( jval.size() == jvalnames.size() );
//     for (int i=1;i<jvalnames.size();++i) {
//         std::map< std::string, int>::iterator it=jindex.find( jvalnames[i] );
//         if (it!=jindex.end()) {
//             jvalues[it->second] = jval[i];
//         } 
//     }     
// }

void etaslNode::update()
{       
        // gets inputs, this can includes joint values in jpos,
        // which will be overwritten if used.
        for (auto& h : inputhandlers) {
            // TODO: Check if jpos_ros or jpos_etasl should be used
            h->update(time, jnames_in_expr, jpos_ros, fnames, fpos_etasl);
        }

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
         // integrate previous outputs:
        //  For real robot, instead of integrating use the following function:
        // etaslNode::update_controller_input(Eigen::VectorXd const& jvalues_meas);


      // for (unsigned int i=0;i<jnames_in_expr.size();++i) {
      //     std::map<std::string,int>::iterator p = jindex.find(jnames_in_expr[i]);
      //     // std::cout << "p->second:  " << p->second << std::endl;
      //     if (p!=jindex.end() && i<jpos_etasl.size()) {
      //         jpos_ros[p->second] = jpos_etasl[i];
      //     }
      // } 
      update_controller_output(jpos_etasl);
        // std::cout << jpos_ros[0] << std::endl;
       
         
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


        for (auto& h : outputhandlers) {
            // TODO: Check if jpos_ros or jpos_etasl should be used
            h->update(jnames_in_expr, jpos_ros, jvel_etasl, fnames, fpos_etasl, fvel_etasl);
        }
        // std::cout << "jointpos:" << jpos_etasl.transpose() << std::endl;
}

void etaslNode::update_robot_status(){

    // jpos_etasl += jvel_etasl*(periodicity_param/1000.0);  // or replace with reading joint positions from real robot
    fpos_etasl += fvel_etasl*(periodicity_param/1000.0);  // you always integrate feature variables yourself
    time += (periodicity_param/1000.0);       // idem.

    feedback_shared_ptr->mtx.lock();
    setpoint_shared_ptr->mtx.lock();

    assert(feedback_shared_ptr->joint.pos.data.size() == jvel_etasl.size());
    assert(setpoint_shared_ptr->velocity.data.size() == jvel_etasl.size());

    setpoint_shared_ptr->velocity.fs = etasl::NewData;
    for (unsigned int i=0; i<jvel_etasl.size(); ++i) {
      setpoint_shared_ptr->velocity.data[i] = jvel_etasl[i];
      jpos_etasl[i] = feedback_shared_ptr->joint.pos.data[i];
    // TODO: retrieve other feedback values into a structure saved locally in this thread
    }


    feedback_shared_ptr->mtx.unlock();
    setpoint_shared_ptr->mtx.unlock();
}

void etaslNode::reinitialize_data_structures() {
    ctx = create_context();
    slv.reset();
    ctx->addType("robot");
    ctx->addType("feature");
    LUA = boost::make_shared<LuaContext>();
    // define a variable that only depends on time
    LUA->initContext(ctx);

    solver_registry.reset();
    // registerSolverFactory_qpOases(solver_registry, "qpoases");


    time = 0.0;
    fpos_etasl = VectorXd::Zero(0);
    jvel_etasl = VectorXd::Zero(0);
    fvel_etasl = VectorXd::Zero(0);

    jointnames.clear();
    jnames_in_expr.clear();

    jindex.clear();
    name_ndx.clear();
    fnames.clear();

    // inputhandlers.clear();
    // outputhandlers.clear();
    // ih_initialized.clear();

    
}

bool etaslNode::initialize_input_handlers(){
    // RCUTILS_LOG_INFO_NAMED(get_name(), "Initializing input handlers...");
    // // initial input is used for initialization.
    // for (size_t i = 0; i < inputhandlers.size(); ++i) {
    //     if (!ih_initialized[i]) {
    //         std::string message = "Initializing input handler: "+ inputhandlers[i]->getName();
    //         RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());
    //         ih_initialized[i] = inputhandlers[i]->initialize(ctx, jnames_in_expr, fnames, jpos_ros, fpos_etasl);
    //         if (!ih_initialized[i]) {
    //             std::string message = "Could not initialize input handler : " + inputhandlers[i]->getName();
    //             RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
    //             this->safe_shutdown();
    //             return false;
    //         }
    //     }
    // }

    RCUTILS_LOG_INFO_NAMED(get_name(), "Initializing input handlers...");
    for (auto& h : inputhandlers) {
        std::stringstream message;
        message << "Initializing input handler:" <<  h->getName();
        RCUTILS_LOG_INFO_NAMED(get_name(), (message.str()).c_str());

        h->initialize(ctx, jnames_in_expr, fnames, jpos_ros, fpos_etasl);
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "finished initializing input handlers");

    return true;
}


bool etaslNode::initialize_output_handlers(){
    // initialize output-handlers
    RCUTILS_LOG_INFO_NAMED(get_name(), "Initializing output handlers...");
    for (auto& h : outputhandlers) {
        std::stringstream message;
        message << "Initializing output handler:" <<  h->getName();
        RCUTILS_LOG_INFO_NAMED(get_name(), (message.str()).c_str());
        RCUTILS_LOG_INFO_NAMED(get_name(), "helloo1");

        h->initialize(ctx, jnames_in_expr, fnames);
        RCUTILS_LOG_INFO_NAMED(get_name(), "helloo2");
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "finished initializing output handlers");
    return true;
}

void etaslNode::configure_node(){
    

    Json::Value param = board->getPath("/default-etasl", false);

    jointnames.clear();
    for (auto n : param["robotdriver"]["joint_names"]) {
        jointnames.push_back(n.asString());
    }

    feedback_shared_ptr = boost::make_shared<etasl::FeedbackMsg>(jointnames.size());
    setpoint_shared_ptr = boost::make_shared<etasl::SetpointMsg>(jointnames.size());

    /****************************************************
    * Registering factories
    ***************************************************/
   register_factories();

    /****************************************************
    * Adding Robot Driver
    ***************************************************/
    RCUTILS_LOG_INFO_NAMED(get_name(), "register_output_handler");
    // TODO: add info about the output handler added (e.g. p[is-...] and p[topic-name])
    robotdriver = etasl::Registry<etasl::RobotDriverFactory>::create(param["robotdriver"]);
    


    /****************************************************
    * Adding input and output handlers from the read JSON file 
    ***************************************************/
    for (const auto& p : param["outputhandlers"]) {
        RCUTILS_LOG_INFO_NAMED(get_name(), "register_output_handler");
        outputhandlers.push_back(etasl::Registry<etasl::OutputHandlerFactory>::create(p));
    }
    for (const auto& p : param["inputhandlers"]) {
        RCUTILS_LOG_INFO_NAMED(get_name(), "register_input_handler");
        inputhandlers.push_back(etasl::Registry<etasl::InputHandlerFactory>::create(p));
        ih_initialized.push_back(false);
    }  
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


 // This still needs to be here and cannot be moved to configure_node() function because of the routine for verifying the joints in the expression. That routine should change 
    if(!first_time_configured){
      timer_ = this->create_wall_timer(std::chrono::milliseconds(periodicity_param), std::bind(&etaslNode::update, this));
      Json::Value param = board->getPath("/default-etasl", false);

      std::vector<double> jpos_init_vec;
      for (auto n : param["robotdriver"]["initial_joints"]) {
          jpos_init_vec.push_back(n.asDouble());
      }

      jpos_init = VectorXd::Zero(jpos_init_vec.size());
      for (size_t i = 0; i < jpos_init_vec.size(); ++i) {
        jpos_init[i] = jpos_init_vec[i];
      }
      
      // jpos_init << 180.0/180.0*3.1416, -90.0/180.0*3.1416, 90.0/180.0*3.1416, -90.0/180.0*3.1416, -90.0/180.0*3.1416, 0.0/180.0*3.1416;
      
      robotdriver->initialize();
      this->initialize_input_handlers();
      this->initialize_output_handlers();

    }
    else{
      jpos_init = jpos_etasl;
    }

  
    
    timer_->cancel();


    this->configure_etasl();

    // std::cout << "The time is:" << std::endl;
    // auto timeee = ctx->getOutputExpression<double>("time");
    // std::cout << timeee->value() << std::endl;



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

    robotdriver->on_activate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "Entering on activate for input handlers.");
    for (auto& h : inputhandlers) {
        h->on_activate(ctx, jnames_in_expr, fnames);
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "Entering on activate for output handlers.");
    for (auto& h : outputhandlers) {
        h->on_activate(ctx, jnames_in_expr, fnames);
    }

    //     std::cout << "hello2" << std::endl;
    // std::cout <<"The current state label is:" << state.label() << std::endl;
    // std::cout <<"The current state id is:" << state.id() << std::endl;

    // etaslNode::on_activate(state);

    // std::cout << "hello3" << std::endl;

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    // std::this_thread::sleep_for(2s);


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
    robotdriver->on_deactivate();
    for (auto& h : inputhandlers) {
        h->on_deactivate(ctx);
    }
    for (auto& h : outputhandlers) {
        h->on_deactivate(ctx);
    }

    // for (auto& h : inputhandlers) {
    //     h->finalize();
    // }
    // for (auto& h : outputhandlers) {
    //     h->finalize();
    // }


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

    robotdriver->on_cleanup();
    for (auto& h : inputhandlers) {
        h->on_cleanup(ctx);
    }
    for (auto& h : outputhandlers) {
        h->on_cleanup(ctx);
    }

    timer_->cancel();
    this->reinitialize_data_structures();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    is_configured = false; //To indicate that it has not being configured after cleanup node

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

    robotdriver->finalize();
    for (auto& h : inputhandlers) {
        h->finalize();
    }
    for (auto& h : outputhandlers) {
        h->finalize();
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // rclcpp::shutdown(); 
  }

  void etaslNode::register_factories(){
        // The following registers each factory. If you don't declare this, the program will not be able to create objects from
    // such factory when specified in the JSON files.
    // etasl::registerQPOasesSolverFactory();
    etasl::registerTopicOutputHandlerFactory(shared_from_this()); 
    // etasl::registerFileOutputHandlerFactory();
    etasl::registerJointStateOutputHandlerFactory(shared_from_this());
    // etasl::registerTopicInputHandlerFactory(shared_from_this());
    // etasl::registerTFOutputHandlerFactory(shared_from_this());
    etasl::registerTwistInputHandlerFactory(shared_from_this());
    etasl::registerSimulationRobotDriverFactory(feedback_shared_ptr.get(), setpoint_shared_ptr.get());
  }

  boost::shared_ptr<t_manager::thread_t> etaslNode::create_thread_str(std::atomic<bool> & stopFlag){
    

    Json::Value param = board->getPath("/default-etasl", false);
    double periodicity = param["robotdriver"]["periodicity"].asDouble();

    thread_str_driver = boost::make_shared<t_manager::thread_t>();

    thread_str_driver->periodicity = std::chrono::nanoseconds(static_cast<long long>(periodicity * 1E9)); //*1E9 to convert seconds to nanoseconds
    // thread_str_driver->periodicity = std::chrono::milliseconds(10);
    thread_str_driver->update_hook = std::bind(&etasl::RobotDriver::update, robotdriver, std::ref(stopFlag));
    thread_str_driver->finalize_hook = std::bind(&etasl::RobotDriver::finalize, robotdriver);

    return thread_str_driver;

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


    my_etasl_node->configure_node();


    std::atomic<bool> stopFlag(false); 

    boost::shared_ptr<t_manager::thread_t> thread_str_driver = my_etasl_node->create_thread_str(stopFlag);

    std::thread driver_thread(t_manager::do_thread_loop, thread_str_driver, std::ref(stopFlag));
    driver_thread.detach();// Avoids the main thread to block. See spin() + stopFlag mechanism below.


    

    // executor.add_node(my_etasl_node->get_node_base_interface());
    // rclcpp::spin(my_etasl_node->get_node_base_interface());
    // executor.spin();

    rclcpp::ExecutorOptions options;
    options.context = my_etasl_node->get_node_base_interface()->get_context();
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(my_etasl_node->get_node_base_interface());
    executor.spin(); //This method blocks!

    stopFlag.store(true); //Stops execution of driver_thread after executor is interrupted with ctr+c signal

    executor.remove_node(my_etasl_node->get_node_base_interface());


    // The following gives a segmentation fault after the on_configure() call:
    // rclcpp::executors::StaticSingleThreadedExecutor executor;
    // executor.add_node(my_etasl_node->get_node_base_interface());
    // executor.spin();


    // std::string cmd_filename = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/json/test_io_handlers.json";
    // Json::Value cmd = etasl::loadJSONFile(cmd_filename);
    // if (!cmd) {
    //     throw etasl::etasl_error(etasl::etasl_error::FAILED_TO_LOAD, "Cannot find file '{}'", cmd_filename);
    //     return -1;
    // }

    // etasl::registerTopicInputHandlerFactory(my_etasl_node);
    // etasl::registerTopicOutputHandlerFactory(my_etasl_node);

    // std::vector<etasl::OutputHandler::SharedPtr> outputhandlers;


    // for (const auto& p : cmd["etasl"]["outputhandlers"]) {
    //     // etasl::add_output_handler(
    //     //     etasl::Registry<etasl::OutputHandlerFactory>::create(p));
    //       outputhandlers.push_back(etasl::Registry<etasl::OutputHandlerFactory>::create(p));
    // }

    // Preparation for control loop
    // int periodicity_param = my_etasl_node->get_periodicity_param();
    // const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(periodicity_param);
    // std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;


    /****************************************************
    * Control loop 
    ***************************************************/
    // while (rclcpp::ok()) {
 
    //     my_etasl_node->update();
    //     // if you need to send output to a robot, do it here, using jvel_etasl or jpos_etasl
    //     my_etasl_node->publishJointState();
    //     // rclcpp::spin(my_etasl_node);
    //     // rclcpp::spin_some(my_etasl_node);//TODO: change to executor::spin_some()
    //     executor.spin_some();//TODO: change to executor::spin_some()

    //     std::this_thread::sleep_until(end_time_sleep);
    //     // TODO: The following while might not be needed. Now it never gets executed.
    //     while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
    //         errno = 0;
    //         std::this_thread::sleep_until(end_time_sleep);
    //         std::cout << "Needed some extra time"<< std::endl; //Temporarily placed for debugging
    //     }
    //     end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds periodicity
    // }

    rclcpp::shutdown();
    // my_etasl_node->safe_shutdown();

    RCUTILS_LOG_INFO("Program terminated correctly.");
  return 0;
}