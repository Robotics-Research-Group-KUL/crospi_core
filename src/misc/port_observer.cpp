#include <expressiongraph/context.hpp>
#include "port_observer.hpp"
#include <string>

using namespace KDL;

/**
 * Observer that puts events on an rFSM event queue 
 */
class PortObserver: public Observer {
    Context::Ptr ctx;
    const std::string portname;
    Observer::Ptr   next;                 ///< the next observer you want to react to monitors.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr outp;
    std_msgs::msg::String event_msg;
    const std::string action_name;
    const std::string event_postfix;
    bool  exit_when_triggered;
    bool activate_console;
    std::shared_ptr<LuaContext> LUA;
    std::string            ename;
 
public:
    typedef boost::shared_ptr< PortObserver > Ptr; //Not compatible with std::shared_ptr, so we use boost::shared_ptr here

    PortObserver(
            Context::Ptr _ctx,
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _outp,
            const std::string& _action_name,
            const std::string& _event_postfix,
            bool  _exit_when_triggered,
            bool _activate_console,
            std::shared_ptr<LuaContext> _LUA,
            Observer::Ptr _next 
    ):   ctx(_ctx),
         outp(_outp),
         event_msg(std_msgs::msg::String()),
         action_name(_action_name),
         event_postfix(_event_postfix),
         exit_when_triggered(_exit_when_triggered),
         activate_console(_activate_console),
         LUA(_LUA),
         next(_next) 
    {
        ename.reserve(512);
    }


    /**
     * The solver will call this when MonitoringScalar is activated.
     * \param [in] mon the monitor that was activated.
     */
    virtual void monitor_activated(const  MonitorScalar& mon) {
        if (mon.action_name.compare(action_name)==0) {
            std::stringstream sstr(ename);
            if (mon.argument.size()==0) {
                sstr <<"e_finished";
            } else {
                sstr << mon.argument;
            }
            if (event_postfix.size()!=0) {
                sstr << "@" << event_postfix;
            } 
            if(activate_console){
                ctx->setFinishStatus();
                int retval = LUA->call_console();
            }
            else if (exit_when_triggered) {
                ctx->setFinishStatus();
            }
            event_msg.data = sstr.str();
            outp->publish(event_msg);
        } 
        else {
            if (next) {
                next->monitor_activated(mon);
            }
        }
    }

    virtual ~PortObserver() {}
}; 

Observer::Ptr create_port_observer(
    Context::Ptr _ctx,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _outp,
    const std::string& _action_name,
    const std::string& _event_postfix,
    bool  _exit_when_triggered,
    bool _activate_console,
    std::shared_ptr<LuaContext> _LUA,
    Observer::Ptr _next
) {
    
    PortObserver::Ptr r( new PortObserver(_ctx,_outp, _action_name, _event_postfix, _exit_when_triggered, _activate_console, _LUA, _next) );
    return r;
}

