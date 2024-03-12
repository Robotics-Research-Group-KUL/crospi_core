#include "etasl_node.hpp"
#include "IO_handlers.hpp"

// For real-time control loop
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace KDL;
using namespace Eigen;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


etaslNode::etaslNode(): Node("etasl_node")
, count_(0)
, angle(0.0)

{
  // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // timer_ = this->create_wall_timer(500ms, std::bind(&etaslNode::timer_callback, this));

  joint_state_msg = sensor_msgs::msg::JointState();
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&etaslNode::publishJointState, this));
}


void etaslNode::publishJointState() {
    // auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

    
    // Populate the joint state message according to your robot configuration
    // For example:
    joint_state_msg.header.stamp = this->now(); // Set the timestamp to the current time
    joint_state_msg.name = {"joint1"};
    joint_state_msg.position = {angle};
    joint_state_msg.velocity = {0.0};
    joint_state_msg.effort = {0.0};
    // angle = angle + 0.01;
    // joint_state_msg.name = {"joint1", "joint2", "joint3"};
    // joint_state_msg.position = {1.0, 2.0, 3.0};
    // joint_state_msg.velocity = {0.1, 0.2, 0.3};
    // joint_state_msg.effort = {10.0, 20.0, 30.0};

    publisher_->publish(joint_state_msg);
}


void etaslNode::setAngle(double p_angle){
  angle = p_angle;
}

typename Observer::Ptr 
create_PrintObserver(
        typename boost::shared_ptr<solver> _slv, 
        const std::string& _action_name, 
        const std::string& _message, 
        typename Observer::Ptr _next ) 
{
    PrintObserver::Ptr r( new PrintObserver(_slv, _action_name, _message,_next) );
    return r; 
}




int main(int argc, char * argv[])
{

      /**
     * Program arguments
     */   
    std::string outpfilename = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/log_test.csv";
    std::string fname = "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/taskspec2.lua";
    // if (argc!=3) {
    //     std::cerr << "C++ driver to run eTaSL"<< std::endl;
    //     std::cerr << "Usage:\n " << argv[0] << " etasl_script.lua  outputfile.csv" << std::endl;
    //     return -1;
    // } else {
    //     fname = argv[1];
    //     outpfilename = argv[2];
    // }

    /**
     * read task specification and creation of solver and handlers for monitor, inputs, outputs
     */
    // Create registry and register known solvers: 
    SolverRegistry::Ptr R = boost::make_shared<SolverRegistry>();
    registerSolverFactory_qpOases(R, "qpoases");
    //registerSolverFactory_hqp(R, "hqp");

    // Create context and set-up for robot control problem:
    Context::Ptr ctx = create_context();
    ctx->addType("robot");
    ctx->addType("feature");

    try{
        // Read eTaSL specification:
        LuaContext LUA;
        LUA.initContext(ctx);
        int retval = LUA.executeFile(fname);
        if (retval !=0) {
            std::cerr << "Error executing task specification file" << std::endl;
            return -2;
        }
    } catch (const char* msg) {
        // can be thrown by file/string errors during reading
        // by lua_bind during reading
        // by expressiongraph during reading ( expressiongraph will not throw when evaluating)
        std::cout << "error thrown: " << msg << std::endl;
    }

    // create the OutputHandler and solver:
    eTaSL_OutputHandler oh( ctx);     // or   std::vector<std::string> varnames =  {"q","f","dx","dy"};
                                     //      eTaSL_OutputHandler oh( ctx, varnames); for only specific outputs
    boost::shared_ptr<solver> slv;

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
        
    int periodicity_param = 10; //Expressed in milliseconds

    // std::string solver_name = ctx->getSolverStringProperty("solver", "qpoasis");  // 2nd arg is the default value if nothing is specified.
    // int result = R->createSolver(solver_name, ctx->solver_property, true, false, slv); 

    int result = R->createSolver(solver_name, plist, true, false, slv); 
    if (result!=0) {
        std::cerr << "Failed to create the solver " << solver_name << std::endl;
        return -1;
    }
    ctx->setSolverProperty("sample_time", periodicity_param/1000.0);
    double dt = ctx->getSolverProperty("sample_time", periodicity_param/1000.0);
    std::cout<<"sample_time:" << dt << std::endl;


    if (ctx->getSolverProperty("verbose",0.0)>0) {
        std::cerr << "Solver and initialization properties : " << std::endl;
        for (auto const& it : ctx->solver_property) {
            std::cerr << "\t" << it.first << ": " << it.second << std::endl;
        }
    }

    // create the Inputhandler:
    eTaSL_InputHandler ih(ctx,"sine_input", 0.5,0.1,0.0);
 

    /****************************************************
     * Initialization
     ***************************************************/
    int retval;

    // initialize time:
    double time = 0.0;


    // reset all monitors:
    ctx->resetMonitors();
    
    // initialize our outputhandler:
    std::ofstream outpfile(outpfilename);
    oh.printHeader(outpfile);
    
    // initial input is used for initialization.
    ih.update(time);                                              
    
    // initialization of robt and feature variables: 
    auto initializer = createFeatureVariableInitializer(slv, ctx, ctx->solver_property);
    retval = initializer->prepareSolver();
    if (retval!=0) {
        std::cerr << initializer->errorMessage(retval) << std::endl;
        return -1; 
    }
    VectorXd jpos  = VectorXd::Zero(slv->getNrOfJointStates());   // choose the joint robot state to start with
    std::cout << "Number of joints:" << slv->getNrOfJointStates() << std::endl;
    jpos[0] = 0.2;
    // jpos[1] = -M_PI/2;
    VectorXd fpos = VectorXd::Zero(slv->getNrOfFeatureStates()); // choose the initial feature variables for the initializer, i.e.
    std::cout <<  "Before initialization\njpos = " << jpos.transpose() << "\nfpos = " << fpos.transpose() << std::endl;
    initializer->setRobotState(jpos);
    initializer->setFeatureState(fpos); // leave this out if you want to use the initial values in the task specification.
    retval = initializer->initialize_feature_variables();
    if (retval!=0) {
        std::cerr << initializer->errorMessage(retval) << std::endl;
        return -1; 
    }

    fpos = initializer->getFeatureState();
    std::cout <<  "After initialization\njpos = " << jpos.transpose() << "\nfpos = " << fpos.transpose() << std::endl;
    // now both jpos and fpos are properly initiallized
  


    /****************************************************
     * Execution 
     ***************************************************/

    // Prepare the solver for execution, define output variables for both robot joints and feature states: 
    slv->prepareExecution(ctx);
    slv->setJointStates(jpos);
    slv->setFeatureStates(fpos);
    VectorXd jvel = VectorXd::Zero(slv->getNrOfJointStates());
    VectorXd fvel  = VectorXd::Zero(slv->getNrOfFeatureStates());
       {
        // not really necessary, getting the full state such that we can print out the total number of opt. vars:
        Eigen::VectorXd state;
        slv->getState(state);
        std::cerr << "size of the initial state " << state.size() << std::endl;
        std::cerr << "the initial state " << state.transpose() << std::endl;
    }

    // create observers for monitoring events:       
    // only now because our PrintObserver monitor uses slv 
    // (prepared for execution, not prepared for initialization):
    // observers are chained together:
    Observer::Ptr obs1     = create_default_observer(ctx, "exit");
    Observer::Ptr obs2     = create_PrintObserver(slv, "print", "PrintObserver was triggered ", obs1);
    ctx->addDefaultObserver(obs2);


    // For real-time loop
    const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(periodicity_param);
    std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;

    rclcpp::init(argc, argv);
    auto my_etasl_node = std::make_shared<etaslNode>();


      // control loop :
    while (rclcpp::ok()) {
        // integrate previous outputs:
        jpos += jvel*(periodicity_param/1000.0);  // or replace with reading joint positions from real robot
        fpos += fvel*(periodicity_param/1000.0);  // you always integrate feature variables yourself
        time += (periodicity_param/1000.0);       // idem.
        // std::cout << time << std::endl;
       
        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            std::cerr << "finishing..." << std::endl;
            rclcpp::shutdown();
            break;
        }
        // get inputs and store them into the context ctx:
        ih.update(time);
        // handle outputs and display them:
        oh.printOutput(outpfile);            
        // set states:
        slv->setTime(time);
        slv->setJointStates(jpos);
        slv->setFeatureStates(fpos);
        // solve
        int c = slv->solve();
        if (c!=0) {
            std::cerr << "solved encountered error : \n";
            std::cerr << slv->errorMessage(c)  << std::endl;
            break;
        }
        // get outputs:
        slv->getJointVelocities(jvel);
        slv->getFeatureVelocities(fvel);

        my_etasl_node->setAngle(jpos[0]);
        // etaslNode::publishJointState();
        rclcpp::spin_some(my_etasl_node);
        // if you need to send output to a robot, do it here, using jvel or jpos

        while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
            errno = 0;
            std::this_thread::sleep_until(end_time_sleep);
        }
        end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds_one_milisecond
    }


  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<etaslNode>());


  return 0;
}