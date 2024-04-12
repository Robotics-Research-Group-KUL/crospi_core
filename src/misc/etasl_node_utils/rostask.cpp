/**
 * @todo:
 *    remove solver parameter mechanism from the eTaSL part and the context scripting.
 */

#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/solver_registry.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <string>
#include <algorithm>
#include <iomanip>
#include <fmt/format.h>
#include "etasl_node_utils/rostask.hpp"
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
#include "etasl_node_utils/rosloghandler.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace etasl {

static int l_get_package_share_directory(lua_State* L)
{
    std::string package = luaL_checkstring(L, 1);
    try {
        std::string share_directory = ament_index_cpp::get_package_share_directory(package);
        lua_pushstring(L, share_directory.c_str());
        return 1; /* number of results*/
    } catch (ament_index_cpp::PackageNotFoundError& err) {
        luaL_error(L, fmt::format("package {} not found", err.package_name).c_str());
        // the above statement returns the function
        return 0;
    }
}

RosTask::RosTask(rclcpp_lifecycle::LifecycleNode::SharedPtr _node, Json::Value& param)
    : node(_node)
    , Task(param)
{
    Task::set_log_handler(RosLogHandler::create(node));
    lua_pushcfunction(LUA.L, l_get_package_share_directory);
    lua_setglobal(LUA.L, "get_package_share_directory");
}

// void RosTask::setNode(rclcpp_lifecycle::LifecycleNode::SharedPtr _node) {
//     node = _node;
// }

// void RosTask::load(const std::string& filename) {
//     Task::load(filename);
// }
//
// void RosTask::initialize() {
//     Task::initialize();
// }
//
// void RosTask::startLoop() {
//     Task::startLoop();
// }

// bool RosTask::onTimer()
// {

//     bool retval = Task::onTimer();

//     if (timestats.duration_since_reset() > 1.0 - 1E-3) {
//         log->log(INFO, timestats.getStatistics(time));
//         timestats.reset(dt);
//     }
//     if (retval) {
//         log->log(INFO, fmt::format("finishing... (time={})", time));
//     }
//     return retval;
// }

} // namespace etasl
