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
#include <string>
#include <algorithm>
#include <iomanip>
#include <boost/chrono.hpp>
#include "featurevariableinitializer.hpp"

#include "IO_handlers.hpp"


using namespace KDL;
using namespace Eigen;

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  lifecycle_return;


class etaslNode : public rclcpp_lifecycle::LifecycleNode
{

    public:
        explicit etaslNode(const std::string & node_name, bool intra_process_comms);
        
        lifecycle_return on_configure(const rclcpp_lifecycle::State &);
        lifecycle_return on_activate(const rclcpp_lifecycle::State &);
        lifecycle_return on_deactivate(const rclcpp_lifecycle::State &);    
        lifecycle_return on_cleanup(const rclcpp_lifecycle::State &);
        lifecycle_return on_shutdown(const rclcpp_lifecycle::State & state);


        void publishJointState();
        void configure_etasl();
        void update();
        void reinitialize_data_structures();

        // void setJointValues(const std::vector<double>& jval, const std::vector<std::string>& jvalnames);

        Context::Ptr get_ctx();
        boost::shared_ptr<solver> get_slv();
        boost::shared_ptr<eTaSL_OutputHandler>  get_output_handler();
        boost::shared_ptr<eTaSL_InputHandler> get_input_handler();
        int get_periodicity_param();
        VectorXd get_fpos_etasl();
        VectorXd get_jpos_etasl();
        double get_time();
        std::string get_outpfilename();
        std::string get_etasl_fname();

        void update_controller_output(Eigen::VectorXd const& jvalues);
        void update_controller_input(Eigen::VectorXd const& jvalues);

        void solver_configuration();
        void initialize_joints();
        void initialize_feature_variables();
        void configure_jointstate_msg();

        bool srv_configure(const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
          std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>  response);

        bool srv_etasl_console(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response>  response);
        
    


    
    private:
        std::shared_ptr<rclcpp::TimerBase> timer_;
        // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>> joint_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;


        int periodicity_param; //Expressed in milliseconds
        double time;
        sensor_msgs::msg::JointState joint_state_msg;
        std_msgs::msg::String event_msg;
        std::string event_postfix;

        Context::Ptr ctx;
        boost::shared_ptr<solver> slv;
        boost::shared_ptr<LuaContext> LUA;
        SolverRegistry::Ptr solver_registry;

        boost::shared_ptr<eTaSL_OutputHandler> oh;
        boost::shared_ptr<eTaSL_InputHandler> ih;
        boost::shared_ptr<std::ofstream > outpfile_ptr;

        std::vector< std::string > jointnames;
        std::vector<std::string> jnames_in_expr;

        


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


        rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr test_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_etasl_console_;



        





};




#endif