//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Authors: Santiago Iregui and Erwin Aertbeliën
//  emails: <santiago.iregui@kuleuven.be> and <erwin.aertbelien@kuleuven.be>
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


#include "crospi_node_utils/rostask.hpp"
#include "crospi_utils/blackboard.hpp"
#include "crospi_utils/string_interpolate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <chrono>
#include <fmt/format.h>
#include <string>
#include <thread>

// factories:
#include "crospi_utils/qpoasessolverfactory.hpp"

#include "crospi_node_utils/topicinputhandlerfactory.hpp"

#include "crospi_node_utils/jointstateoutputhandlerfactory.hpp"
#include "crospi_node_utils/jointstateinputhandlerfactory.hpp"
#include "crospi_node_utils/topicoutputhandlerfactory.hpp"
#include "crospi_utils/fileoutputhandlerfactory.hpp"
#include "crospi_node_utils/tfoutputhandlerfactory.hpp"

#include <fmt/format.h>
#include <sstream>
#include <queue>

using namespace KDL;

using namespace std::chrono_literals;

namespace etasl {
/*
class RosTaskRunner : public rclcpp::Node {
    std::queue<RosTask::SharedPtr> taskq;
    std::mutex taskmtx;
    std::string blackboard_fn;
    std::string searchpath;

    BlackBoard board;
    Json::Value defparam;
    double sample_time;
    bool running;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::CallbackGroup::SharedPtr low_prior_cbg;

public:
    RosTaskRunner(
        const std::string& _blackboard_fn,
        const std::string& _searchpath,
        const rclcpp::NodeOptions& options)
        : Node("RosTaskRunner", options)
        , blackboard_fn(_blackboard_fn)
        , searchpath(_searchpath)
        , board(1)
    {
        board.setSearchPath(searchpath);
        board.load_process_and_validate(blackboard_fn);
        defparam = board.getPath("/default-etasl", false);
        sample_time = defparam["sample_time"].asDouble();

        low_prior_cbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer = this->create_wall_timer(
            std::chrono::duration<double>(sample_time),
            std::bind(&RosTaskRunner::onTimer, this));
        running = false;
    }

    void pushTask(RosTask::SharedPtr tsk)
    {
        taskq.push(tsk);
    }

    rclcpp::CallbackGroup::SharedPtr get_high_priority_cbg()
    {
        return get_node_base_interface()->get_default_callback_group();
    }
    rclcpp::CallbackGroup::SharedPtr get_low_priority_cbg()
    {
        return low_prior_cbg;
    }

    void onTimer()
    {
        RosTask::SharedPtr currenttask;
        if (!running) {
            if (!taskq.empty()) {
                currenttask = taskq.front();
                currenttask->load();
                currenttask->initialize();
                currenttask->startLoop();
            } else {
                return;
            }
        }
        bool finished = currenttask->onTimer();
        if (finished) {
            currenttask->finalize();
            taskq.pop();
            running = false;
        }
    }

    ~RosTaskRunner()
    {
    }
};*/

} // namespace etasl

int main(int argc, char* argv[])
{
    using namespace etasl;
    using namespace std::string_literals;

    rclcpp::init(argc, argv);
    //rclcpp::NodeOptions options;
    //options.allow_undeclared_parameters(true);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("crospi_node");

    BlackBoard board(1);
    std::cout << " loading blackboard" << std::endl;
    board.setSearchPath("$[crospi_core]/scripts/schema:$[crospi_core]/scripts/schema/tasks");

    std::cout << " +++++++++++++" << std::endl;
    board.load_process_and_validate("$[crospi_core]/scripts/json/blackboard.json");
    std::cout << " +++++++++++++---------------------" << std::endl;
    fmt::print("{:->80}\n", "-");
    fmt::print("blackboard/default-etasl : ");
    Json::Value param = board.getPath("/default-etasl", false);
    fmt::print("After processing and validating:\n{}", param);
    fmt::print("{:->80}\n", "-"); //Prints 80 times -


    std::vector<double> outputhandlers = {1,2,3};
    for (auto h : outputhandlers) {
        std::cout<< "entered for loop" << std::endl;
        std::cout<< "contains: " << h << std::endl;
    }

    // const std::string cmd_filename = string_interpolate("$[crospi_core]/scripts/etasl/move_circle.json");
    // Json::Value cmd = loadJSONFile(cmd_filename);
    // if (!cmd) {
    //     throw etasl_error(etasl_error::FAILED_TO_LOAD, "Cannot find file '{}'", cmd_filename);
    //     return -1;
    // }
    // cmd = board.process_and_validate("", cmd, "cmd");
    // fmt::print("Command after validation and processing: \n{}\n", cmd);
    // addIfNotExists(cmd["etasl"], param);

    // fmt::print("Command after fusion: \n{}\n", cmd);
    // saveJSONFILE(cmd, "tst_hello.json");
    // fmt::print("{:->80}\n", "-");
    // double freq = 1.0 / cmd["etasl"]["sample_time"].asDouble();
    // rclcpp::Rate rate(freq);
    // int counter = 0;

    // registerQPOasesSolverFactory();
    // registerTopicOutputHandlerFactory(node);
    // registerFileOutputHandlerFactory();
    // registerJointStateOutputHandlerFactory(node);
    // registerTopicInputHandlerFactory(node);
    // registerTFOutputHandlerFactory(node);
    // registerJointStateInputHandlerFactory(node);

    // RosTask task(node, cmd);
    // task.load();

    // task.resetInitialization();
    // while (!task.initialize()) {
    //     rclcpp::spin_some(node);
    //     if (!param["etasl"]["use_sim_time"].asBool()) {
    //         rate.sleep();
    //     }        
    // }
    // task.startLoop();
    // bool finished = false;
    // while (rclcpp::ok() && (!finished)) {
    //     // RCLCPP_INFO(node->get_logger(), "Hello %d", counter++);
    //     finished = task.onTimer();
    //     rclcpp::spin_some(node);
    //     if (!param["etasl"]["use_sim_time"].asBool()) {
    //         rate.sleep();
    //     }
    // }
    // task.finalize();

    std::cout << "hellooooooooooooo" << std::endl;
    rclcpp::shutdown();
    return 0;
}
