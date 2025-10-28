#include "etasl_node_utils/rostask.hpp"
#include "etasl_task_utils/blackboard.hpp"
#include "etasl_task_utils/string_interpolate.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fmt/format.h>
#include <string>
#include <thread>
#include <unordered_map>
// factories:
#include "etasl_task_utils/qpoasessolverfactory.hpp"
#include "etasl_node_utils/topicinputhandlerfactory.hpp"
#include "etasl_node_utils/jointstateoutputhandlerfactory.hpp"
#include "etasl_node_utils/topicoutputhandlerfactory.hpp"
#include "etasl_task_utils/fileoutputhandlerfactory.hpp"
#include "etasl_node_utils/tfoutputhandlerfactory.hpp"

#include <fmt/format.h>
#include <sstream>
#include <deque>
#include "rclcpp_action/rclcpp_action.hpp"

#include "etasl_task_utils/outputhandlerfactory.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"

#include "etasl_interfaces/action/run_task.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace KDL;

using namespace std::chrono_literals;

namespace etasl {


// int main(int argc, char* argv[])
// {
//     using namespace etasl;
//     using namespace std::string_literals;

//     rclcpp::init(argc, argv);
//     rclcpp::NodeOptions options;
//     options.allow_undeclared_parameters(true);
//     auto blackboard_fn = "$[etasl_node_utils]/scripts/json/blackboard.json";
//     auto searchpath = "$[etasl_node_utils]/scripts/schema:$[etasl_node_utils]/scripts/schema/tasks";
//     BlackBoard board(1);
//     board.setSearchPath(searchpath);
//     board.load_process_and_validate(blackboard_fn);
//     Json::Value robotconfig = board.getPath("/robot-configuration", false);

//     // rclcpp::executors::MultiThreadedExecutor executor;
//     // executor.add_node(etasl_node_utils);
//     // executor.spin();
//     // rclcpp::shutdown();

//     return 0;
// }