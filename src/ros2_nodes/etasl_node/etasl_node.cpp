#include "etasl_node.hpp"
#include "IO_handlers.hpp"

// For real-time control loop
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace KDL;
using namespace Eigen;



/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


etaslNode::etaslNode(const std::string & node_name, bool intra_process_comms = false): rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
, periodicity_param(10) //Expressed in milliseconds
, time(0.0)
, first_time_configured(false)
{
  //Used unless the ROS parameters are modified externally (e.g. through terminal or launchfile)

  outpfilename = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/log_test.csv";
  this->declare_parameter("outpfilename",  outpfilename);
  this->declare_parameter("task_specification_file",  rclcpp::PARAMETER_STRING);
  this->declare_parameter("jointnames",  rclcpp::PARAMETER_STRING_ARRAY);

  // fname = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/taskspec2.lua";
  // this->declare_parameter("outpfilename",  outpfilename); //outpfilename as default val
  // this->declare_parameter("task_specification_file",  fname); //fname as default val

  // this->create_service<lifecycle_msgs::srv::ChangeState>("configure", &srv_configure);
  test_service_ = create_service<lifecycle_msgs::srv::ChangeState>("etasl_node/configure", std::bind(&etaslNode::srv_configure, this, std::placeholders::_1, std::placeholders::_2));
  
  events_pub_ = this->create_publisher<std_msgs::msg::String>("fsm/events", 10); 
  event_msg = std_msgs::msg::String();

}

bool etaslNode::srv_configure(const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request, std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>  response)
      {
          std::cout << "Request id: "<< request->transition.id << std::endl;
          std::cout << "Request label: "<< request->transition.label << std::endl;
          std::this_thread::sleep_for(5s);
          std::cout << "Finish service call: " << std::endl;
          response->success = true;
          return true;

      }


void etaslNode::publishJointState() {
    // auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

    // if(this->get_current_state()){

    // }

      this->update();

      joint_state_msg.header.stamp = this->now(); // Set the timestamp to the current time
      // joint_state_msg.name = jointnames; //No need to update if defined in configuration
      
      for (unsigned int i=0;i<jnames_in_expr.size();++i) {
      // Populate the joint state message according to your robot configuration
      joint_state_msg.position[i]  = jpos_ros[i];
      joint_state_msg.velocity[i]  = 0.0;
      joint_state_msg.effort[i]  = 0.0;
      } 

      // std::cout << "publishing joints" << std::endl;
    // Print the current state for demo purposes
    // if (!joint_pub_->is_activated()) {
    //   RCLCPP_INFO(
    //     get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    // } 

    // We independently from the current state call publish on the lifecycle
    // publisher.
    // Only if the publisher is in an active state, the message transfer is
    // enabled and the message actually published.
    joint_pub_->publish(joint_state_msg);
}

typename Observer::Ptr create_PrintObserver(
        typename boost::shared_ptr<solver> _slv, 
        const std::string& _action_name, 
        const std::string& _message, 
        typename Observer::Ptr _next ) 
{
    PrintObserver::Ptr r( new PrintObserver(_slv, _action_name, _message,_next) );
    return r; 
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
    solver_registry = boost::make_shared<SolverRegistry>();
    registerSolverFactory_qpOases(solver_registry, "qpoases");
    //registerSolverFactory_hqp(R, "hqp");

    // we can get the properties from the solver from the context and specify these properties in eTaSL or
    // create a parameter plist (instead of ctx->solver_property) :
        // parameters of the solver:
        std::string solver_name             = "qpoases" ;
        ParameterList plist;
        plist["nWSR"]                  = 100;
        plist["regularization_factor"] = 1E-5;
        plist["cputime"] = 1.0;
        // parameters of the initialization procedure:
        plist["initialization_full"]                  = 1.0; // == true
        plist["initialization_duration"]              = 3.0;
        plist["initialization_sample_time"]           = 0.01;
        plist["initialization_convergence_criterion"] = 1E-4;
        plist["initialization_weightfactor"]          = 100;
        

    // std::string solver_name = ctx->getSolverStringProperty("solver", "qpoasis");  // 2nd arg is the default value if nothing is specified.
    // int result = solver_registry->createSolver(solver_name, ctx->solver_property, true, false, slv); 

    int result = solver_registry->createSolver(solver_name, plist, true, false, slv); 
    if (result!=0) {
        std::string message = "Failed to create the solver " + solver_name;
        RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
        rclcpp::shutdown();
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

    if(jnames_in_expr.size() != jointnames.size()){
      RCUTILS_LOG_WARN_NAMED(get_name(), "The number of jointnames specified in ROS param do not correspond to all the joints defined in the eTaSL robot expression graph.");
      RCUTILS_LOG_WARN_NAMED(get_name(), "The jointnames that do not correspond are ignored and not published");
    }

}


void etaslNode::initialize_feature_variables(){
    // initialization of robot and feature variables: 
    FeatureVariableInitializer::Ptr initializer = createFeatureVariableInitializer(slv, ctx, ctx->solver_property);
    int retval = initializer->prepareSolver();
    if (retval!=0) {
        RCUTILS_LOG_ERROR_NAMED(get_name(), (initializer->errorMessage(retval)).c_str());
        rclcpp::shutdown();
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
        rclcpp::shutdown();
        return; 
    }

    fpos_etasl = initializer->getFeatureState();
    // std::cout <<  "After initialization\njpos = " << jpos_etasl.transpose() << "\nfpos_etasl = " << fpos_etasl.transpose() << std::endl;
    // now both jpos_etasl and fpos_etasl are properly initiallized
  
}

void etaslNode::configure_jointstate_msg(){
    joint_state_msg.name = jnames_in_expr;
    joint_state_msg.position.resize(jnames_in_expr.size(),0.0); 
    joint_state_msg.velocity.resize(jnames_in_expr.size(),0.0);
    joint_state_msg.effort.resize(jnames_in_expr.size(),0.0);
}

void etaslNode::configure_etasl(){


    ctx = create_context(); //Create context used for etasl

    // Read configuration ROS parameters
    fname = this->get_parameter("task_specification_file").as_string();
    jointnames = this->get_parameter("jointnames").as_string_array();
    // std::cout << jointnames[0] << "," << jointnames[1] <<std::endl;
 
    /**
     * read task specification and creation of solver and handlers for monitor, inputs, outputs
     */

    // Setup context for robot control problem:
    ctx->addType("robot");
    ctx->addType("feature");

    std::string message = "The task specification file used is: "+ fname;
    RCUTILS_LOG_INFO_NAMED(get_name(), message.c_str());

    try{
        // Read eTaSL specification:
        LUA = boost::make_shared<LuaContext>();
        LUA->initContext(ctx);
        int retval = LUA->executeFile(fname);
        if (retval !=0) {
            RCUTILS_LOG_ERROR_NAMED(get_name(), "Error executing task specification file");
            rclcpp::shutdown();
            return;
        }
    } catch (const char* msg) {
        // can be thrown by file/string errors during reading
        // by lua_bind during reading
        // by expressiongraph during reading ( expressiongraph will not throw when evaluating)
        std::string message = "The following error was throuwn while reading the task specification: " + std::string(msg);
        RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
        rclcpp::shutdown();
        return;
    }

    // oh( ctx);     // or   std::vector<std::string> varnames =  {"q","f","dx","dy"};
                                      //      eTaSL_OutputHandler oh( ctx, varnames); for only specific outputs
    // create the OutputHandler:
    oh = boost::make_shared<eTaSL_OutputHandler>(ctx);
        // create the Inputhandler:
    ih = boost::make_shared<eTaSL_InputHandler>(ctx,"sine_input", 0.2,0.1,0.0);
    // ih->update(time);                                              
    ih->update(0.0);    

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

    // Prepare the solver for execution, define output variables for both robot joints and feature states: 
    slv->setTime(0.0);
    slv->setFeatureStates(fpos_etasl);
    slv->setJointStates(jpos_etasl);

    // create observers for monitoring events:       
    // only now because our PrintObserver monitor uses slv 
    // (prepared for execution, not prepared for initialization):
    // observers are chained together:
    Observer::Ptr obs1     = create_default_observer(ctx, "exit");
    Observer::Ptr obs2     = create_PrintObserver(slv, "print", "PrintObserver was triggered ", obs1);
    ctx->addDefaultObserver(obs2);

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

    outpfilename = this->get_parameter("outpfilename").as_string();
    outpfile_ptr = boost::make_shared<std::ofstream>(outpfilename);
    
    
    oh->printHeader(*outpfile_ptr);

    // std::vector<std::string> hello;
    // slv->getJointNameVector(hello);
    //   std::cout << "------------" << "THe jointnames are: "<< std::endl;
    //   for (unsigned int i=0;i<hello.size();++i) {
    //     std::cout << "------------" << hello[i] << std::endl;
    // } 

}



// Get methods

Context::Ptr etaslNode::get_ctx()
{
  return ctx;
}
boost::shared_ptr<solver> etaslNode::get_slv()
{
  return slv;
}
boost::shared_ptr<eTaSL_OutputHandler> etaslNode::get_output_handler()
{
  return oh;
}
boost::shared_ptr<eTaSL_InputHandler> etaslNode::get_input_handler()
{
  return ih;
}

VectorXd etaslNode::get_fpos_etasl()
{
  return fpos_etasl;
}

VectorXd etaslNode::get_jpos_etasl()
{
  return jpos_etasl;
}

double etaslNode::get_time()
{
  return time;
}

int etaslNode::get_periodicity_param()
{
  return periodicity_param;
}

std::string etaslNode::get_outpfilename()
{
  return outpfilename;
}

std::string etaslNode::get_etasl_fname()
{
  return fname;
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

        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            RCUTILS_LOG_INFO_NAMED(get_name(), "Finishing execution because exit monitor was triggered.");
            event_msg.data = "etasl_finished";
            events_pub_->publish(event_msg);
            auto transition = this->deactivate();
            // if (transition.id() != lifecycle_msgs::msg::Transition::SUCCESS) {
            //     RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to transition node to deactivate state when the etasl monitor was triggered");
            // }
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
       

        // get inputs and store them into the context ctx:
        ih->update(time);
        // handle outputs and display them:
        oh->printOutput(*outpfile_ptr);            
        // set states:
        slv->setTime(time);
        slv->setJointStates(jpos_etasl);
        slv->setFeatureStates(fpos_etasl);
        // solve
        int c = slv->solve();
        if (c!=0) {

            std::string message = "The solver encountered the following error : \n" + slv->errorMessage(c);
            RCUTILS_LOG_ERROR_NAMED(get_name(), message.c_str());
            rclcpp::shutdown();
            return;
            // break;
        }
        // get outputs:
        slv->getJointVelocities(jvel_etasl);
        slv->getFeatureVelocities(fvel_etasl);

        jpos_etasl += jvel_etasl*(periodicity_param/1000.0);  // or replace with reading joint positions from real robot
        fpos_etasl += fvel_etasl*(periodicity_param/1000.0);  // you always integrate feature variables yourself
        time += (periodicity_param/1000.0);       // idem.

        // std::cout << "jointpos:" << jpos_etasl.transpose() << std::endl;

}

void etaslNode::reinitialize_data_structures() {
    ctx = create_context();
    slv.reset();
    // ctx->addType("robot");
    // ctx->addType("feature");
    LUA = boost::make_shared<LuaContext>();
    // define a variable that only depends on time
    LUA->initContext(ctx);

    solver_registry.reset();
    // registerSolverFactory_qpOases(solver_registry, "qpoases");
    outpfile_ptr.reset();
    ih.reset();
    oh.reset();

    time = 0.0;
    fpos_etasl = VectorXd::Zero(0);
    jvel_etasl = VectorXd::Zero(0);
    fvel_etasl = VectorXd::Zero(0);

    jointnames.clear();
    jnames_in_expr.clear();
    outpfile_ptr.reset();

    jindex.clear();
    name_ndx.clear();
    
    


    // install observers:
    // Observer::Ptr obs = create_port_observer( ctx, &eventPort, "portevent","",false);
    // obs = create_port_observer( ctx, &eventPort, "event",event_postfix,false);
    // obs = create_port_observer( ctx, &eventPort, "exit", event_postfix,true, obs);
    //obs = create_default_observer(ctx,"exit",obs);
    // ctx->addDefaultObserver(obs);
    // initialized=false;
    // etaslread=false;
    // controller_input_defined=false;
    // controller_output_defined=false;
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
  lifecycle_return etaslNode::on_configure(const rclcpp_lifecycle::State &)
  {
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initialize and configure our publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.




    joint_state_msg = sensor_msgs::msg::JointState();
    if(!first_time_configured){
      joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1); //queue_size 1 so that it sends the latest always
      timer_ = this->create_wall_timer(std::chrono::milliseconds(periodicity_param), std::bind(&etaslNode::publishJointState, this));
      jpos_init = VectorXd::Zero(6);
      jpos_init << 180.0/180.0*3.1416, -90.0/180.0*3.1416, 90.0/180.0*3.1416, -90.0/180.0*3.1416, -90.0/180.0*3.1416, 0.0/180.0*3.1416;
    }
    else{
      jpos_init = jpos_etasl;
    }

  
    
    timer_->cancel();
    this->configure_etasl();
    this->configure_jointstate_msg();


    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    
    first_time_configured = true;
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

    // TODO: Only allow transitions from configure, and not from deactivate

    // std::cout << "hello1" << std::endl;
    timer_->reset();
    joint_pub_->on_activate();
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
    joint_pub_->on_deactivate();
    timer_->cancel();


    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

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
  lifecycle_return etaslNode::on_cleanup(const rclcpp_lifecycle::State &)
  {
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_->cancel();
    // joint_pub_.reset();
    this->reinitialize_data_structures();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }

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
    timer_->cancel();
    // joint_pub_->cancel();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return lifecycle_return::SUCCESS;
  }



int main(int argc, char * argv[])
{

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    std::shared_ptr<etaslNode> my_etasl_node = std::make_shared<etaslNode>("etasl_node");
    
    executor.add_node(my_etasl_node->get_node_base_interface());


    
    // my_etasl_node->configure_etasl();
    // my_etasl_node->configure_jointstate_msg();


    // Preparation for control loop
    // int periodicity_param = my_etasl_node->get_periodicity_param();
    // const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(periodicity_param);
    // std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;


    executor.spin();
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

    RCUTILS_LOG_INFO_NAMED(my_etasl_node->get_name(), "Program terminated correctly.");
    rclcpp::shutdown();

  return 0;
}