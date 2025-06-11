#ifndef EXPRESSIONGRAPH_PORT_OBSERVER_ROS_HPP
#define EXPRESSIONGRAPH_PORT_OBSERVER_ROS_HPP
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace KDL;


/**
 * Creates an observer that maps a trigger of a monitor in eTaSL to
 * a string on an orocos outputport.
 * \param _ctx [in] context, when exit_when_triggered==true, then setFinishStatus() is called 
 *             on _ctx when an event is triggered.
 * \param _outp [in] the ROS topic to push the event to.
 * \param _action_name [in] should match the  action_name that is specified in the monitor 
 * \param _exit_when_triggered [in] when true, the triggered monitor will also cause the component to 
 *        stop.
 *\param _activate_console [in] when true, it activates the etasl_console for debugging purposes. 
 *          If true, the value of _exit_when_triggered will be taken as true always. 
 *\param _LUA [in] shared_ptr of the LuaContext
 * \param _next  [in] points to the next handler for an observer.
 */
Observer::Ptr create_port_observer(
    Context::Ptr _ctx,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _outp,
    const std::string& _action_name,
    const std::string& _event_postfix,
    bool  exit_when_triggered,
    bool _activate_console,
    std::shared_ptr<LuaContext> _LUA,
    Observer::Ptr _next = Observer::Ptr() 
);


#endif
