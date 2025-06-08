#include "etasl_node_utils/inputhandlermanager.hpp"


//  #include <expressiongraph/context.hpp>
//  #include <expressiongraph/context_scripting.hpp>
//  #include <expressiongraph/solver_registry.hpp>
//  #include <expressiongraph/solver_factory_qpoases.hpp>
//  #include <expressiongraph/defaultobserver.hpp>
//  #include <string>
//  #include <algorithm>
//  #include <iomanip>
//  #include <fmt/format.h>
//  #include "lauxlib.h"
//  #include "lua.h"
//  #include "lualib.h"
//  #include "etasl_node_utils/rosloghandler.hpp"
//  #include "ament_index_cpp/get_package_prefix.hpp"
//  #include "ament_index_cpp/get_package_share_directory.hpp"
 
 namespace etasl {
 
 InputHandlerManager::InputHandlerManager(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node, 
    Json::Value& param)
     : node(_node)
     , Task(param)
 {


 }
 
 InputHandlerManager::construct_input_handlers()
 {
    
 }

 
 // void InputHandlerManager::startLoop() {
 //     Task::startLoop();
 // }
 

 } // namespace etasl
 