#include "etasl_node_utils/io_handler_manager.hpp"


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
 
 IOHandlerManager::IOHandlerManager(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node, 
    const Json::Value& _param,
    std::shared_ptr<etasl::JsonChecker> _jsonchecker)
     : node(_node)
     , parameters(_param)
     , jsonchecker(_jsonchecker)
 {
    std::cout << "IOHandlerManager constructor called" << std::endl;
    
    inputhandler_loader = std::make_shared<pluginlib::ClassLoader<etasl::InputHandler>>("etasl_ros2", "etasl::InputHandler");
    outputhandler_loader = std::make_shared<pluginlib::ClassLoader<etasl::OutputHandler>>("etasl_ros2", "etasl::OutputHandler");
    // std::cout << "param_root: " << parameters.toStyledString() << std::endl;

 }
 
 void IOHandlerManager::construct_input_handlers()

 {
    // std::cout << "param_root: " << parameters.toStyledString() << std::endl;

    // TODO: Check that none of the varnames are repeated, otherwise shutdown and inform
    if (jsonchecker->is_member(parameters, "iohandlers/inputhandlers")) //inputhandlers field is optional
    {
      // RCLCPP_INFO(node->get_logger(), "json param detected^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
      
      Json::Value param_iohandlers = parameters["iohandlers"];
      for (const auto& par : param_iohandlers["inputhandlers"]) {
          std::string handler_name = "";
          for (const auto& key : par.getMemberNames()) {
            if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
              handler_name = key.substr(3);
            }
          }
          // std::string message = "handler_name: " + handler_name;
          // RCLCPP_INFO(node->get_logger(), message.c_str());

          if(handler_name == ""){
            std::string message = "Could not find any is- keyword in the inputhandler field of the json configuration file. It must specify the type, e.g. is-TwistInputHandler = true";
            RCLCPP_ERROR(node->get_logger(), message.c_str());
            auto transition = node->shutdown(); //calls on_shutdown() hook.
            return;
          }
  
          etasl::InputHandler::SharedPtr handler_ptr = nullptr;
          try
          {
            handler_ptr = inputhandler_loader->createSharedInstance("etasl::" + handler_name);
          }
          catch(pluginlib::PluginlibException& ex)
          {
            std::string message = "The plugin failed to load. Error: \n" + std::string(ex.what());
            RCLCPP_ERROR(node->get_logger(), message.c_str());
            auto transition = node->shutdown(); //calls on_shutdown() hook.
            return;
          }
  
          if (handler_ptr){
            RCLCPP_INFO(node->get_logger(), "register_input_handler");
            inputhandlers.push_back(handler_ptr);
            bool is_constructed = handler_ptr->construct(handler_name, node, par, jsonchecker);
            if(!is_constructed){
              std::string message = "The input handler " + handler_name + " could not be constructed. Shutting down.";
              RCLCPP_ERROR(node->get_logger(), message.c_str());
              auto transition = node->shutdown(); //calls on_shutdown() hook.
              return;
            }
            RCLCPP_INFO(node->get_logger(), "Input handler %s constructed successfully.", handler_name.c_str());
          }
        }  
        RCLCPP_INFO(node->get_logger(), "Constructed Input Handlers successfully.");

    }
    else{
      std::string message = "The iohandlers/inputhandlers field is not present in the json configuration file.";
      RCLCPP_WARN(node->get_logger(), message.c_str());
      return;
    }

 }
 
 void IOHandlerManager::construct_output_handlers()
 {
    // Json::Value param_iohandlers = parameters["iohandlers"];

    // for (const auto& p : param_iohandlers["outputhandlers"]) {
    //     RCLCPP_INFO(node->get_logger(), "register_output_handler");
    //     outputhandlers.push_back(etasl::Registry<etasl::OutputHandlerFactory>::create(p, jsonchecker));
    // }

    // TODO: Check that none of the varnames are repeated, otherwise shutdown and inform
    if (jsonchecker->is_member(parameters, "iohandlers/outputhandlers")) //outputhandlers field is optional
    {
      // RCLCPP_INFO(node->get_logger(), "json param detected^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
      
      Json::Value param_iohandlers = parameters["iohandlers"];
      for (const auto& par : param_iohandlers["outputhandlers"]) {
          std::string handler_name = "";
          for (const auto& key : par.getMemberNames()) {
            if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
              handler_name = key.substr(3);
            }
          }
          // std::string message = "handler_name: " + handler_name;
          // RCLCPP_INFO(node->get_logger(), message.c_str());

          if(handler_name == ""){
            std::string message = "Could not find any is- keyword in the outputhandler field of the json configuration file. It must specify the type, e.g. is-JointStateOutputHandler = true";
            RCLCPP_ERROR(node->get_logger(), message.c_str());
            auto transition = node->shutdown(); //calls on_shutdown() hook.
            return;
          }
  
          etasl::OutputHandler::SharedPtr handler_ptr = nullptr;
          try
          {
            handler_ptr = outputhandler_loader->createSharedInstance("etasl::" + handler_name);
          }
          catch(pluginlib::PluginlibException& ex)
          {
            std::string message = "The plugin failed to load. Error: \n" + std::string(ex.what());
            RCLCPP_ERROR(node->get_logger(), message.c_str());
            auto transition = node->shutdown(); //calls on_shutdown() hook.
            return;
          }
  
          if (handler_ptr){
            RCLCPP_INFO(node->get_logger(), "register_output_handler");
            outputhandlers.push_back(handler_ptr);
            bool is_constructed = handler_ptr->construct(handler_name, node, par, jsonchecker);
            if(!is_constructed){
              std::string message = "The output handler " + handler_name + " could not be constructed. Shutting down.";
              RCLCPP_ERROR(node->get_logger(), message.c_str());
              auto transition = node->shutdown(); //calls on_shutdown() hook.
              return;
            }
            RCLCPP_INFO(node->get_logger(), "Output handler %s constructed successfully.", handler_name.c_str());
          }
        }  
        RCLCPP_INFO(node->get_logger(), "Constructed Output Handlers successfully.");

    }
    else{
      std::string message = "The iohandlers/outputhandlers field is not present in the json configuration file.";
      RCLCPP_WARN(node->get_logger(), message.c_str());
      return;
    }
 }

 void IOHandlerManager::initialize_input_handlers(
  Context::Ptr ctx,
  const std::vector<std::string>& jnames,
  const std::vector<std::string>& fnames,
  Eigen::VectorXd& jpos,
  Eigen::VectorXd& fpos)
  {
    // TODO
    RCLCPP_INFO(node->get_logger(), "Initializing input handlers...");
    for (auto& h : inputhandlers) {
        std::stringstream message;
        message << "Initializing input handler:" <<  h->getName();
        RCLCPP_INFO(node->get_logger(), (message.str()).c_str());

        h->initialize(ctx, jnames, fnames, jpos, fpos);
        // h->initialize(ctx, jnames_in_expr, fnames, jpos_ros, fpos_etasl);
    }
    RCLCPP_INFO(node->get_logger(), "finished initializing input handlers");

  }

void IOHandlerManager::initialize_output_handlers(
  Context::Ptr ctx,
  const std::vector<std::string>& jnames,
  const std::vector<std::string>& fnames)
  {
    // TODO
    RCLCPP_INFO(node->get_logger(), "Initializing output handlers...");
    for (auto& h : outputhandlers) {
        std::stringstream message;
        message << "Initializing output handler:" <<  h->getName();
        RCLCPP_INFO(node->get_logger(), (message.str()).c_str());
        h->initialize(ctx, jnames, fnames);
    }
    RCLCPP_INFO(node->get_logger(), "finished initializing output handlers");
  }

void IOHandlerManager::configure_input_handlers(
  double time,
  const std::vector<std::string>& jnames,
  Eigen::VectorXd& jpos,
  const std::vector<std::string>& fnames,
  Eigen::VectorXd& fpos)
  {
    //TODO: For now it only updates, but I should inplement an on_configure method in InputHandler class and call all with a for loop
    // for (auto h : inputhandlers) {
    //   h->configure(time, jnames, jpos, fnames, fpos);
    // }

    this->update_input_handlers(time, jnames, jpos, fnames, fpos);
    
  }

void IOHandlerManager::configure_output_handlers(
  double time,
  const std::vector<std::string>& jnames,
  Eigen::VectorXd& jpos,
  const std::vector<std::string>& fnames,
  Eigen::VectorXd& fpos)
  {
    //TODO: For now it only updates, but I should inplement an on_configure method in OutputHandler class and call all with a for loop
    // for (auto h : outputhandlers) {
    //   h->configure(time, jnames, jpos, fnames, fpos);
    // }

    //No need to update output handlers at configuration (only input handlers!)
  }

void IOHandlerManager::update_input_handlers(
  double time,
  const std::vector<std::string>& jnames,
  Eigen::VectorXd& jpos,
  const std::vector<std::string>& fnames,
  Eigen::VectorXd& fpos)
  {
    for (auto h : inputhandlers) {
      h->update(time, jnames, jpos, fnames, fpos);
    }
  }

void IOHandlerManager::update_output_handlers(
  const std::vector<std::string>& jnames,
  const Eigen::VectorXd& jpos,
  const Eigen::VectorXd& jvel,
  const std::vector<std::string>& fnames,
  const Eigen::VectorXd& fvel,
  const Eigen::VectorXd& fpos)
  {
    for (auto& h : outputhandlers) {
      // TODO: Check if jpos or jvel should be used
      h->update(jnames, jpos, jvel, fnames, fvel, fpos);
    }
  }

void IOHandlerManager::activate_input_handlers(
  Context::Ptr ctx,    
  const std::vector<std::string>& jnames,
  const std::vector<std::string>& fnames)
  {
    RCLCPP_INFO(node->get_logger(), "Entering on activate for input handlers.");
    for (auto& h : inputhandlers) {
        h->on_activate(ctx, jnames, fnames);
        //TODO: Handle boolean output of on_activate

    }
    RCLCPP_INFO(node->get_logger(), "Activated input handlers.");

  }

void IOHandlerManager::activate_output_handlers(
  Context::Ptr ctx,    
  const std::vector<std::string>& jnames,
  const std::vector<std::string>& fnames)
  {
    RCLCPP_INFO(node->get_logger(), "Entering on activate for output handlers.");
    for (auto& h : outputhandlers) {
        h->on_activate(ctx, jnames, fnames);
        //TODO: Handle boolean output of on_activate
    }
    RCLCPP_INFO(node->get_logger(), "Activated output handlers.");

  }


void IOHandlerManager::deactivate_input_handlers(Context::Ptr ctx)
{
  for (auto& h : inputhandlers) {
    h->on_deactivate(ctx);
  }
}
void IOHandlerManager::deactivate_output_handlers(Context::Ptr ctx)
{
    for (auto& h : outputhandlers) {
        h->on_deactivate(ctx);
    }
}


void IOHandlerManager::cleanup_input_handlers(Context::Ptr ctx)
{
  for (auto& h : inputhandlers) {
    h->on_cleanup(ctx);
  }
}
void IOHandlerManager::cleanup_output_handlers(Context::Ptr ctx)
{
  for (auto& h : outputhandlers) {
    h->on_cleanup(ctx);
}
}

void IOHandlerManager::finalize_input_handlers()
{
  for (auto& h : inputhandlers) {
    h->finalize();
  }
}
void IOHandlerManager::finalize_output_handlers()
{
  for (auto& h : outputhandlers) {
    h->finalize();
  }
}
 
 // void IOHandlerManager::startLoop() {
 //     Task::startLoop();
 // }
 

 } // namespace etasl
 