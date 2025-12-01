#include "simple_crospi_node.hpp"
#include "IO_handlers_deleteme.hpp"

// For real-time control loop
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace KDL;
using namespace Eigen;



/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


etaslNode::etaslNode(): Node("simple_crospi_node")
, count_(0)
, periodicity_param(10) //Expressed in milliseconds
, time(0.0)
{
  //Used unless the ROS parameters are modified externally (e.g. through terminal or launchfile)

  outpfilename = "/workspaces/colcon_ws/src/crospi_core/etasl/log_test.csv";
  this->declare_parameter("outpfilename",  outpfilename);
  this->declare_parameter("task_specification_file",  rclcpp::PARAMETER_STRING);
  this->declare_parameter("jointnames",  rclcpp::PARAMETER_STRING_ARRAY);

  // fname = "/workspaces/colcon_ws/src/crospi_core/etasl/taskspec2.lua";
  // this->declare_parameter("outpfilename",  outpfilename); //outpfilename as default val
  // this->declare_parameter("task_specification_file",  fname); //fname as default val

  ctx = create_context(); //Create context used for etasl

  
  joint_state_msg = sensor_msgs::msg::JointState();
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1); //queue_size 1 so that it sends the latest always
  // timer_ = this->create_wall_timer(100ms, std::bind(&etaslNode::publishJointState, this));
}


void etaslNode::publishJointState() {
    // auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

      joint_state_msg.header.stamp = this->now(); // Set the timestamp to the current time
      // joint_state_msg.name = jointnames; //No need to update if defined in configuration
      
      for (unsigned int i=0;i<jnames_in_expr.size();++i) {
      // Populate the joint state message according to your robot configuration
      joint_state_msg.position[i]  = jpos_ros[i];
      joint_state_msg.velocity[i]  = 0.0;
      joint_state_msg.effort[i]  = 0.0;
      } 

    publisher_->publish(joint_state_msg);
}

typename Observer::Ptr 
create_PrintObserver(
        typename std::shared_ptr<solver> _slv, 
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
      std::cout << "p->second:  " << p->second << std::endl;
      if (p!=jindex.end()) {
          jpos_etasl[p->second] = jvalues_meas[i]; //joints that will be used for etasl
      }
  } 
}

void etaslNode::solver_configuration(){
      // Create registry and register known solvers: 
    solver_registry = std::make_shared<SolverRegistry>();
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
        std::cerr << "Failed to create the solver " << solver_name << std::endl;
        rclcpp::shutdown();
        return;
    }
    ctx->setSolverProperty("sample_time", periodicity_param/1000.0);
    // double dt = ctx->getSolverProperty("sample_time", periodicity_param/1000.0);
    // std::cout<<"sample_time:" << dt << std::endl;


    if (ctx->getSolverProperty("verbose",0.0)>0) {
        std::cerr << "Solver and initialization properties : " << std::endl;
        for (auto const& it : ctx->solver_property) {
            std::cerr << "\t" << it.first << ": " << it.second << std::endl;
        }
    }
  
}


void etaslNode::initialize_joints(){
    jpos_etasl  = VectorXd::Zero(slv->getNrOfJointStates());   

    for (unsigned int i=0;i<jointnames.size();++i) {
        std::map<std::string,int>::iterator p = jindex.find(jointnames[i]);
        if (p!=jindex.end()) {
            jnames_in_expr.push_back(jointnames[i]);
            std::cout << "============The jointnames are:   " << jointnames[i] << std::endl;
        }
    } 
    jpos_ros  = VectorXd::Zero(jnames_in_expr.size()); 
    // jpos_etasl = VectorXd::Zero(jnames_in_expr.size()); 
    std::cout << "Number of matching jointnames with robot variables in exp. graph: " << jnames_in_expr.size() << std::endl;
    std::cout << "Number of joints:" << slv->getNrOfJointStates() << std::endl;

    name_ndx.clear();
    for (unsigned int i=0;i<jnames_in_expr.size();++i) {
        name_ndx[ jnames_in_expr[i]]  =i;
    }

    VectorXd jpos_init = VectorXd::Zero(slv->getNrOfJointStates());
    jpos_init << 180.0/180.0*3.1416, -90.0/180.0*3.1416, 90.0/180.0*3.1416, -90.0/180.0*3.1416, -90.0/180.0*3.1416, 0.0/180.0*3.1416;
    update_controller_input(jpos_init);

    if(jnames_in_expr.size() != jointnames.size()){
      std::cerr << "The number of jointnames specified in ROS param do not correspond to all the joints defined in the eTaSL robot expression graph."<< std::endl;
      std::cerr << "The jointnames that do not correspond are ignored and not published"<< std::endl;
    }

}


void etaslNode::initialize_feature_variables(){
    // initialization of robot and feature variables: 
    FeatureVariableInitializer::Ptr initializer = createFeatureVariableInitializer(slv, ctx, ctx->solver_property);
    int retval = initializer->prepareSolver();
    if (retval!=0) {
        std::cerr << initializer->errorMessage(retval) << std::endl;
        rclcpp::shutdown();
        return;
    }


    slv->prepareExecution(ctx);
    slv->getJointNameToIndex(jindex);
    this->initialize_joints();
    
    fpos_etasl = VectorXd::Zero(slv->getNrOfFeatureStates()); // choose the initial feature variables for the initializer, i.e.

    std::cout <<  "Before initialization\njpos = " << jpos_etasl.transpose() << "\nfpos_etasl = " << fpos_etasl.transpose() << std::endl;
    initializer->setRobotState(jpos_etasl);
    // initializer->setFeatureState(fpos_etasl); //TODO: should I leave it out? leave this out if you want to use the initial values in the task specification.
    retval = initializer->initialize_feature_variables();
    if (retval!=0) {
        std::cerr << initializer->errorMessage(retval) << std::endl;
        rclcpp::shutdown();
        return; 
    }

    fpos_etasl = initializer->getFeatureState();
    std::cout <<  "After initialization\njpos = " << jpos_etasl.transpose() << "\nfpos_etasl = " << fpos_etasl.transpose() << std::endl;
    // now both jpos_etasl and fpos_etasl are properly initiallized
  
}

void etaslNode::configure_jointstate_msg(){
    joint_state_msg.name = jnames_in_expr;
    joint_state_msg.position.resize(jnames_in_expr.size(),0.0); 
    joint_state_msg.velocity.resize(jnames_in_expr.size(),0.0);
    joint_state_msg.effort.resize(jnames_in_expr.size(),0.0);
}

void etaslNode::configure_etasl(){

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

    std::cout << "FIle name is: " << fname << std::endl;
    try{
        // Read eTaSL specification:
        LuaContext LUA;
        LUA.initContext(ctx);
        int retval = LUA.executeFile(fname);
        if (retval !=0) {
            std::cerr << "Error executing task specification file" << std::endl;
            rclcpp::shutdown();
            return;
        }
    } catch (const char* msg) {
        // can be thrown by file/string errors during reading
        // by lua_bind during reading
        // by expressiongraph during reading ( expressiongraph will not throw when evaluating)
        std::cout << "error thrown: " << msg << std::endl;
    }

    // oh( ctx);     // or   std::vector<std::string> varnames =  {"q","f","dx","dy"};
                                      //      eTaSL_OutputHandler oh( ctx, varnames); for only specific outputs
    // create the OutputHandler:
    oh = std::make_shared<eTaSL_OutputHandler>(ctx);
        // create the Inputhandler:
    ih = std::make_shared<eTaSL_InputHandler>(ctx,"sine_input", 0.2,0.1,0.0);
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
        std::cerr << "size of the initial state " << state.size() << std::endl;
        std::cerr << "the initial state " << state.transpose() << std::endl;
    }

    // initialize our outputhandler:
    // std::ofstream outpfile(outpfilename);

    outpfilename = this->get_parameter("outpfilename").as_string();
    outpfile_ptr = std::make_shared<std::ofstream>(outpfilename);
    
    
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
std::shared_ptr<solver> etaslNode::get_slv()
{
  return slv;
}
std::shared_ptr<eTaSL_OutputHandler> etaslNode::get_output_handler()
{
  return oh;
}
std::shared_ptr<eTaSL_InputHandler> etaslNode::get_input_handler()
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
            std::cerr << "finishing because exit monitor was triggered" << std::endl;
            rclcpp::shutdown();
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
            std::cerr << "solved encountered error : \n";
            std::cerr << slv->errorMessage(c)  << std::endl;
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


}


int main(int argc, char * argv[])
{

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    std::shared_ptr<etaslNode> my_crospi_node = std::make_shared<etaslNode>();

    executor.add_node(my_crospi_node);
    
    my_crospi_node->configure_etasl();
    my_crospi_node->configure_jointstate_msg();


    // Preparation for control loop
    int periodicity_param = my_crospi_node->get_periodicity_param();
    const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(periodicity_param);
    std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;

    /****************************************************
    * Control loop 
    ***************************************************/
    while (rclcpp::ok()) {
 
        my_crospi_node->update();
        // if you need to send output to a robot, do it here, using jvel_etasl or jpos_etasl
        my_crospi_node->publishJointState();
        // rclcpp::spin(my_crospi_node);
        // rclcpp::spin_some(my_crospi_node);//TODO: change to executor::spin_some()
        executor.spin_some();//TODO: change to executor::spin_some()

        std::this_thread::sleep_until(end_time_sleep);
        // TODO: The following while might not be needed. Now it never gets executed.
        while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
            errno = 0;
            std::this_thread::sleep_until(end_time_sleep);
            std::cout << "Needed some extra time"<< std::endl; //Temporarily placed for debugging
        }
        end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds periodicity
    }

    std::cout << "Program terminated correctly." << std::endl;
    rclcpp::shutdown();

  return 0;
}