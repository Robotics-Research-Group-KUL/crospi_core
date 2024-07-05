#ifndef ETASL_ROS2_SIMPLE_NODE_HPP
#define ETASL_ROS2_SIMPLE_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp" // Include Time message header

// For lifecycle state machine node:
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"


// For logging in ros2
#include "rcutils/logging_macros.h"


// For eTaSL
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/solver_registry.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <expressiongraph/defaultobserver.hpp>
//#include <expressiongraph/solver_factory_hqp.hpp>

#include <algorithm>
#include <iomanip>
#include <boost/chrono.hpp>

#include "featurevariableinitializer.hpp"
#include "IO_handlers_deleteme.hpp"
#include "etasl_interfaces/srv/task_specification_string.hpp" 
#include "etasl_interfaces/srv/task_specification_file.hpp" 


// #include "etasl_node_utils/rostask.hpp"
#include "etasl_task_utils/blackboard.hpp"
#include "etasl_task_utils/registry.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/topicinputhandlerfactory.hpp"

#include "etasl_task_utils/outputhandler.hpp"
#include "etasl_task_utils/outputhandlerfactory.hpp"
#include "etasl_node_utils/jointstateoutputhandlerfactory.hpp"
#include "etasl_node_utils/topicoutputhandlerfactory.hpp"
#include "etasl_task_utils/fileoutputhandlerfactory.hpp"
#include "etasl_node_utils/tfoutputhandlerfactory.hpp"
#include "etasl_node_utils/twistinputhandlerfactory.hpp"


#include "robot_interfacing_utils/robotdriverfactory.hpp"
#include "robot_interfacing_utils/simulationrobotdriverfactory.hpp"
#include "robot_interfacing_utils/feedback_struct.hpp"
#include "robot_interfacing_utils/thread_manager.hpp"







using namespace KDL;
using namespace Eigen;
using namespace std::chrono_literals;
// using namespace etasl;

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  lifecycle_return;


class etaslNode : public rclcpp_lifecycle::LifecycleNode
{

    public:
        explicit etaslNode(const std::string & node_name, bool intra_process_comms);
        
        lifecycle_return on_configure(const rclcpp_lifecycle::State & state);
        lifecycle_return on_activate(const rclcpp_lifecycle::State & state);
        lifecycle_return on_deactivate(const rclcpp_lifecycle::State & state);    
        lifecycle_return on_cleanup(const rclcpp_lifecycle::State & state);
        lifecycle_return on_shutdown(const rclcpp_lifecycle::State & state);


        void configure_etasl();
        void update();
        void reinitialize_data_structures();
        bool initialize_input_handlers();
        bool initialize_output_handlers();
        void safe_shutdown();

        // void setJointValues(const std::vector<double>& jval, const std::vector<std::string>& jvalnames);

        int get_periodicity_param();


        void update_controller_output(Eigen::VectorXd const& jvalues);
        void update_controller_input(Eigen::VectorXd const& jvalues);

        void solver_configuration();
        void initialize_joints();
        void initialize_feature_variables();
        void configure_node();

        void register_factories();
        void update_robot_status();
        boost::shared_ptr<t_manager::thread_t> create_thread_str(std::atomic<bool> & stopFlag);
        





        // bool srv_configure(const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
        //   std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>  response);

        bool etasl_console(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response>  response);
        bool readTaskSpecificationFile(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationFile::Response>  response);
        bool readTaskSpecificationString(const std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<etasl_interfaces::srv::TaskSpecificationString::Response>  response);
    
    private:
        std::shared_ptr<rclcpp::TimerBase> timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;


        int periodicity_param; //Expressed in milliseconds
        double time;
        std_msgs::msg::String event_msg;
        std::string event_postfix;

        Context::Ptr ctx;
        boost::shared_ptr<solver> slv;
        boost::shared_ptr<LuaContext> LUA;
        SolverRegistry::Ptr solver_registry;

        // boost::shared_ptr<eTaSL_OutputHandler> oh;
        // boost::shared_ptr<eTaSL_InputHandler> ih;
        // boost::shared_ptr<std::ofstream > outpfile_ptr;

        std::vector<etasl::OutputHandler::SharedPtr> outputhandlers;
        std::vector<etasl::InputHandler::SharedPtr> inputhandlers;
        etasl::RobotDriver::SharedPtr               robotdriver;

        std::vector<bool> ih_initialized;

        std::vector< std::string > jointnames;
        std::vector<std::string> jnames_in_expr;
        std::vector< std::string > fnames;

        boost::shared_ptr<etasl::BlackBoard> board;


        boost::shared_ptr<etasl::FeedbackMsg> feedback_shared_ptr;
        boost::shared_ptr<etasl::SetpointMsg> setpoint_shared_ptr;
        boost::shared_ptr<t_manager::thread_t> thread_str_driver;
        


        // eTaSL_OutputHandler oh;
        // eTaSL_InputHandler ih;

        VectorXd fpos_etasl;
        VectorXd jpos_etasl;
        VectorXd jpos_ros;
        // VectorXd jpos;
        VectorXd jvel_etasl;
        VectorXd fvel_etasl;

        VectorXd jpos_init;




          
        std::string outpfilename;
        std::string fname;

        std::map< std::string, int> jindex;
        std::map<std::string,int>  name_ndx;

        bool first_time_configured;
        bool is_configured;


        rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr test_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_etasl_console_;
        rclcpp::Service<etasl_interfaces::srv::TaskSpecificationString>::SharedPtr srv_readTaskSpecificationString_;
        rclcpp::Service<etasl_interfaces::srv::TaskSpecificationFile>::SharedPtr srv_readTaskSpecificationFile_;




        





};




#endif