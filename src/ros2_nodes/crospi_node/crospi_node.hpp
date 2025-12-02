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

#ifndef crospi_core_SIMPLE_NODE_HPP
#define crospi_core_SIMPLE_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <variant>


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
#include "crospi_interfaces/srv/task_specification_string.hpp" 
#include "crospi_interfaces/srv/task_specification_file.hpp" 


#include "crospi_utils/blackboard.hpp"
#include "crospi_utils/registry.hpp"
#include <crospi_utils/inputhandler.hpp>


#include "crospi_node_utils/io_handler_manager.hpp"
// #include "crospi_node_utils/robot_driver_manager.hpp"
// #include "crospi_node_utils/robot_driver_manager_lockfree.hpp"
#include "crospi_node_utils/multiple_drivers_manager.hpp"


// #include "robot_interfacing_utils/feedback_struct.hpp"
#include "robot_interfacing_utils/robot_data_structures.hpp"

// #include "robot_interfacing_utils/thread_manager.hpp"

#include "crospi_utils/json_checker.hpp"



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
        void safe_shutdown();

        void update_controller_output(Eigen::VectorXd const& jvalues);
        void update_controller_input(Eigen::VectorXd const& jvalues);

        void solver_configuration();
        void initialize_joints();
        void initialize_feature_variables();
        void construct_node(std::atomic<bool>* stopFlagPtr_p);

        void update_robot_status();
        std::vector<std::shared_ptr<t_manager::thread_t>> create_driver_threads_structures(std::atomic<bool> & stopFlag);

        bool etasl_console(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response>  response);
        bool readTaskSpecificationFile(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Response>  response);
        bool readRobotSpecification(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationFile::Response>  response);
        bool readTaskSpecificationString(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Response>  response);
        bool readTaskParameters(const std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Request> request, std::shared_ptr<crospi_interfaces::srv::TaskSpecificationString::Response>  response);
    
    private:
        std::shared_ptr<rclcpp::TimerBase> timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;


        int periodicity_ms; //Expressed in milliseconds
        double time;
        std_msgs::msg::String event_msg;
        std::string event_postfix;

        Context::Ptr ctx;
        boost::shared_ptr<solver> slv;
        std::shared_ptr<LuaContext> LUA;
        SolverRegistry::Ptr solver_registry;

        std::atomic<bool>* stopFlagPtr;

        
        etasl::IOHandlerManager::SharedPtr               io_handler_manager;


        // etasl::RobotDriverManagerLockFree::SharedPtr robotdriver_manager;
        etasl::MultipleDriversManager::SharedPtr multiple_robotdriver_managers;

        std::vector< std::string > jointnames;
        std::vector< std::string > jointnames_drivers;
        std::vector<std::string> jnames_in_expr;
        std::vector< std::string > fnames;

        std::vector<int> driver_to_etasl; // Maps driver index -> etasl index, or -1 if not used
        std::vector<int> etasl_to_driver; // Maps etasl index -> driver index
        

        std::shared_ptr<etasl::BlackBoard> board;


        std::shared_ptr<etasl::JsonChecker> jsonchecker;
        

        std::vector<float> joint_positions_feedback; //Feedback of all joints in all drivers

        VectorXd fpos_etasl;
        VectorXd jpos_etasl;
        VectorXd jpos_ros;
        // VectorXd jpos;
        VectorXd jvel_etasl;
        VectorXd jvel_all_drivers;
        VectorXd fvel_etasl;


        std::string outpfilename;
        std::string fname;

        std::map< std::string, int> jindex; //Indexes of joints according to eTaSL task specification
        std::map<std::string,int>  name_ndx; //TODO: change to unordered_map instead of map

        bool first_time_configured;
        bool is_configured;

        bool simulation;


        rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr test_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_etasl_console_;
        rclcpp::Service<crospi_interfaces::srv::TaskSpecificationString>::SharedPtr srv_readTaskSpecificationString_;
        rclcpp::Service<crospi_interfaces::srv::TaskSpecificationFile>::SharedPtr srv_readTaskSpecificationFile_;
        rclcpp::Service<crospi_interfaces::srv::TaskSpecificationFile>::SharedPtr srv_readRobotSpecification_;
        rclcpp::Service<crospi_interfaces::srv::TaskSpecificationString>::SharedPtr srv_readTaskParameters_;


};




#endif