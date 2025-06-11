#ifndef PRINTOBSERVER_HPP_15A4BF
#define PRINTOBSERVER_HPP_15A4BF

#include <cstdio>

#include <expressiongraph/context.hpp>
#include <expressiongraph/solver.hpp>

namespace etasl {
using namespace KDL;
/**
 * A monitor is a construct in eTaSL that specifies conditions that can
 * trigger events.  All observers are called when an event is triggered.
 * An observer reacts to these events, typically by checking the
 * "action_name" to see whether it is relevant to the observer.  Monitors
 * are "edge triggered", i.e. when their condition is triggered, they call
 * the appropriate observer ONCE.
 *
 * This class is an example of a custom observer monitoring events, the
 * action_name is passed in the arguments.  It just prints a message and
 * the current joint and feature variables to the console
 *
 * This class has to have Observer as a base-class
 */
class PrintObserver : public Observer {
    std::shared_ptr<solver> slv;
    Observer::Ptr next;
    Eigen::VectorXd jpos;
    Eigen::VectorXd fpos;
    std::string action_name;
    std::string message;

public:
    typedef boost::shared_ptr<PrintObserver> Ptr;
    /**
     * \param _slv  solver, where we will get joint and feature values
     * \param _message  an additional message to pass.
     * \param _next next observer to check.
     */
    PrintObserver(std::shared_ptr<solver> _slv,
        const std::string& _action_name,
        const std::string& _message,
        Observer::Ptr _next);

    virtual void monitor_activated(const MonitorScalar& mon);

    virtual ~PrintObserver();
};

typename Observer::Ptr
create_PrintObserver(
    typename std::shared_ptr<solver> _slv,
    const std::string& _action_name,
    const std::string& _message,
    typename Observer::Ptr _next);

}

#endif