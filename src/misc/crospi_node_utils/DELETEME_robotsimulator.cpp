#include "crospi_node_utils/rostask.hpp"
#include "crospi_utils/blackboard.hpp"
#include "crospi_utils/string_interpolate.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fmt/format.h>
#include <string>
#include <thread>
#include <unordered_map>
// factories:
#include "crospi_utils/qpoasessolverfactory.hpp"
#include "crospi_node_utils/topicinputhandlerfactory.hpp"
#include "crospi_node_utils/jointstateoutputhandlerfactory.hpp"
#include "crospi_node_utils/topicoutputhandlerfactory.hpp"
#include "crospi_utils/fileoutputhandlerfactory.hpp"
#include "crospi_node_utils/tfoutputhandlerfactory.hpp"

#include <fmt/format.h>
#include <sstream>
#include <deque>
#include "rclcpp_action/rclcpp_action.hpp"

#include "crospi_utils/outputhandlerfactory.hpp"
#include "crospi_utils/inputhandlerfactory.hpp"

#include "crospi_interfaces/action/run_task.hpp"
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
//     auto blackboard_fn = "$[crospi_node_utils]/scripts/json/blackboard.json";
//     auto searchpath = "$[crospi_node_utils]/scripts/schema:$[crospi_node_utils]/scripts/schema/tasks";
//     BlackBoard board(1);
//     board.setSearchPath(searchpath);
//     board.load_process_and_validate(blackboard_fn);
//     Json::Value robotconfig = board.getPath("/robot-configuration", false);

//     // rclcpp::executors::MultiThreadedExecutor executor;
//     // executor.add_node(crospi_node_utils);
//     // executor.spin();
//     // rclcpp::shutdown();

//     return 0;
// }