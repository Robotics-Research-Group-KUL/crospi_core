/**
 * Example C++ file that reads an eTaSL specification and runs the controller
 *
 * This example includes all the interactions needed:
 *  - using the solver factory to get a solver
 *  - outputs
 *  - inputs
 *  - getting outputs that could be send to a robot system, and including joint data from the robot
 *  - dealing with monitors
 *
 * Notes:
 *   - in a real application, it is not adviced to directly write to a file in a control loop.
 *   - do not forget to set the LUAPATH to the eTaSL lua files
 *     (needed to find the eTaSL LUA libraries)
 *   - you can slow down everything by adapting the integration in the control loop for time,joint pos, feature pos.
 *     (also slower time constant!)
 *   - alternatively, you can only slow down time by only adapting the integration of the time variable.
 *
 * E. Aertbelien, 2022
 */
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/solver_registry.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <expressiongraph/defaultobserver.hpp>
//#include <expressiongraph/solver_factory_hqp.hpp>
#include <string>
#include <algorithm>
#include <iomanip>
#include <boost/chrono.hpp>
#include "featurevariableinitializer.hpp"

// For real-time control loop
#include <chrono>
#include <thread>


// ROS2 Stuff
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"




namespace KDL {

    /**
     * Example of a class that handles output suitable for simple, unbuffered, csv output.
     * It gets the output values from the context and outputs the results in a
     * tab-delimited file with a header containing the names of the signals
     *
     * Only handles double-expressions !
     * 
     * This class is just an example, you can choose the structure of the class and its methods
     * yourself.
     */
    class eTaSL_OutputHandler {
        private:
            std::vector< Expression<double>::Ptr > outp; 
            Context::Ptr ctx;
            std::vector<std::string>   varnames;
            std::vector<double>        values;
            std::vector<std::string>   not_found;
        public:

            /**
             * aux. function to cut out the "global.", if it exists
             * (one day this will probably become obsolete)
             */
            std::string cut_global( const std::string& name ) const {
                if (name.substr(0,7)=="global.") {
                    return name.substr(7);
                } else {
                    return name;
                }
            } 

            /** 
             * Gets specific outputexpressions as output.
             * - has_warnings() will be true if not all names are found.
             * - Zero will be returned for names that are not found.
             * - getUnknownVariables() will return the names that were not found.
             */
            eTaSL_OutputHandler( 
                    Context::Ptr    _ctx,
                    const std::vector<std::string>& _varnames
            ): ctx(_ctx), varnames(_varnames), values(_varnames.size()), outp(_varnames.size())
            {
                std::fill(values.begin(), values.end(), 0.0);
                for (auto i = 0; i < varnames.size(); ++i) {
                    outp[i] = ctx->getOutputExpression<double>(varnames[i]); 
                    if (!outp[i]) {
                        not_found.push_back( varnames[i]);
                        outp[i] = Constant(0.0);
                    }
                }
            }
            /**
             * Gets all (DOUBLE-expr) outputexpressions as output
             */
            eTaSL_OutputHandler(
                    Context::Ptr   _ctx
                    ):ctx(_ctx) {
                for (auto ov : _ctx->output_vars) {
                    auto ptr = ctx->getOutputExpression<double>(ov.first);
                    if (ptr!=0) {
                        varnames.push_back(cut_global(ov.first));
                        outp.push_back(ptr);
                        values.push_back(0.0);
                    }  
                }
            }
      
            /**
             * prints a header with the names of the output expressions
             */ 
            void printHeader(std::ofstream& os) {
                unsigned int L = varnames.size();
                for (unsigned int i=0;i<L;++i) {
                    if (i!=0) {
                        os << "\t";
                    }
                    os << "\"" << varnames[i] << "\"";
                }
                os << std::endl;
                os << std::scientific;
            }

            /**
             * updates the values for the output and prints them in a tab-delimited format
             */
            void printOutput(std::ofstream& os) {
                unsigned int L = outp.size();
                for (unsigned int i=0;i<L;++i) {
                    values[i] = outp[i]->value();
                    if (i!=0) {
                        os << "\t";
                    }
                    os << std::setprecision(17) << values[i];
                }
                os << std::endl;
            }
    }; // eTaSL_OutputHandler


    /**
     * example of a class that handles input
     * It generates a sine (with time derivatives) and connects it to a given
     * input channel
     */ 
    class eTaSL_InputHandler {
            VariableType<double>::Ptr inp;
            double amplitude;
            double omega;
            double phase;
            int    time_ndx;
        public:
        /**
         * @brief Construct a new eTaSL InputHandler object to generate a sine wave signal
         * 
         * @param ctx : context
         * @param name: name of the input channel
         * @param _amplitude : amplitude of the input
         * @param _frequency : frequency of the input
         * @param _phase     : phase of the input
         * 
         */
            eTaSL_InputHandler(Context::Ptr ctx, const std::string& channel_name, double _amplitude, double _frequency, double _phase=0.0):
                amplitude(_amplitude),
                omega(2*M_PI*_frequency),
                phase(_phase)
             {
                inp = ctx->getInputChannel<double>(channel_name);     // this gives back a reference to the variable where the input is stored.
                if (!inp) {
                    std::cerr << "Warning: input channel does not exists, ignored" << std::endl;
                }
                time_ndx = ctx->getScalarNdx("time");
             }

            /**
             * @brief update
             * 
             * Update the input and adapts the context to this input.
             * @param time : compute the input for this time
             */
            void update(double time) {
                double f  = amplitude*sin( omega*time + phase);
                double df = amplitude*omega*cos( omega*time + phase);
                if (inp) {
                    inp->setValue( f );
                    inp->setJacobian( time_ndx, df );
                }
            }
    };
    

    /** 
     * A monitor is a construct in eTaSL that specifies conditions that can
     * trigger events.  All observers are called when an event is triggered.
     * An observer reacts to these events, typically by checking the
     * "action_name" to see whether it is relevant to the observer.  Monitors
     * are "edge triggered", i.e. when their condition is triggered, they call
     * the appropriate observer ONCE.
     *
     * This class is an example of a custom observer monitoring events, the
     * action_name is passed in the arguments.  It just prints a message and
     * the current joint and feature variables to the console
     *
     * This class has to have Observer as a base-class
     */
    class PrintObserver: public Observer {
        boost::shared_ptr<solver> slv;
        Observer::Ptr next;
        Eigen::VectorXd jpos;
        Eigen::VectorXd fpos;
        std::string action_name;
        std::string message;  
    public:
        typedef boost::shared_ptr< PrintObserver > Ptr;
        /**
         * \param _slv  solver, where we will get joint and feature values 
         * \param _message  an additional message to pass.
         * \param _next next observer to check.  
         */
        PrintObserver(boost::shared_ptr<solver> _slv,
                         const std::string& _action_name,
                         const std::string& _message,
                         Observer::Ptr _next ):
            slv(_slv),
            next(_next) ,
            jpos( Eigen::VectorXd::Zero(_slv->getNrOfJointStates()) ),
            fpos( Eigen::VectorXd::Zero(_slv->getNrOfFeatureStates())),
            action_name(_action_name),
            message(_message)
        {
        }

      
        virtual void monitor_activated(const  MonitorScalar& mon) {
            if (mon.action_name.compare(action_name)==0) {
                std::string msg = message + "(argument=" + mon.argument + ")";
                slv->getJointStates(jpos);
                slv->getFeatureStates(fpos);
                std::cout << msg << std::endl;
                std::cout << "robot variables   : " << jpos << std::endl;
                std::cout << "feature variables : " << fpos << std::endl;
            }
            // for this observer, we always pass it on to the next observer, even when this monitor was activated
            // for other observers this could be different.
            if (next) {
                next->monitor_activated(mon);
            }
        }

        virtual ~PrintObserver() {
            // a virtual destructor is necessary, even when empty, for proper 
            // destruction when using base class pointers.
        }
    };


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


} // namespace KDL

using namespace std;
using namespace KDL;
using namespace Eigen;




int main(int argc, char* argv[]) {
    /**
     * Program arguments
     */   
    string outpfilename;
    string fname;
    if (argc!=3) {
        cerr << "C++ driver to run eTaSL"<< endl;
        cerr << "Usage:\n " << argv[0] << " etasl_script.lua  outputfile.csv" << endl;
        return -1;
    } else {
        fname = argv[1];
        outpfilename = argv[2];
    }

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
            cerr << "Error executing task specification file" << endl;
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
        string solver_name             = "qpoases" ;
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
    // int result = R->createSolver(solver_name, ctx->solver_property, true, false, slv); 

    int result = R->createSolver(solver_name, plist, true, false, slv); 
    if (result!=0) {
        std::cerr << "Failed to create the solver " << solver_name << std::endl;
        return -1;
    }
    double dt = ctx->getSolverProperty("sample_time", 0.01);
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
    ofstream outpfile(outpfilename);
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
    std::cout << slv->getNrOfJointStates() << std::endl;
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
    const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(10);
    std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;

    // ROS2 Stuff
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    
    rclcpp::Node my_node("my_robot_publisher");
    auto publisher = my_node.create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    
    // my_node("my_robot_publisher");

    // rclcpp::Node::create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // rclcpp::Node::

    joint_state_msg->name = {"joint1"};
    joint_state_msg->position = {jpos[0]};
    joint_state_msg->velocity = {0.0};
    joint_state_msg->effort = {0.0};

    publisher->publish(joint_state_msg);

    // control loop :
    while (true) {
        // integrate previous outputs:
        jpos += jvel*dt;  // or replace with reading joint positions from real robot
        fpos += fvel*dt;  // you always integrate feature variables yourself
        time += dt;       // idem.
       
        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            cerr << "finishing..." << endl;
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
            cerr << "solved encountered error : \n";
            cerr << slv->errorMessage(c)  << endl;
            break;
        }
        // get outputs:
        slv->getJointVelocities(jvel);
        slv->getFeatureVelocities(fvel);
        // if you need to send output to a robot, do it here, using jvel or jpos

        while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
            errno = 0;
            std::this_thread::sleep_until(end_time_sleep);
        }
        end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds_one_milisecond
    }
    return 0;
}

