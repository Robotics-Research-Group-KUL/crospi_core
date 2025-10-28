#include "etasl_task_utils/printobserver.hpp"
#include <string>



namespace etasl {
 
        PrintObserver::PrintObserver(std::shared_ptr<solver> _slv,
                         const std::string& _action_name,
                         const std::string& _message,
                         Observer::Ptr _next ) :
            slv(_slv),
            next(_next) ,
            jpos( Eigen::VectorXd::Zero(_slv->getNrOfJointStates()) ),
            fpos( Eigen::VectorXd::Zero(_slv->getNrOfFeatureStates())),
            action_name(_action_name),
            message(_message)
        {
        }

      
        void PrintObserver::monitor_activated(const  MonitorScalar& mon) {
            if (mon.action_name.compare(action_name)==0) {
                std::string msg = message + "(argument=" + mon.argument + ")";
                slv->getJointStates(jpos);
                slv->getFeatureStates(fpos);
                std::cout << msg << std::endl;
                std::cout << "robot variables   : " << jpos << std::endl;
                std::cout << "feature variables : " << fpos << std::endl;
                slv->printMatrices(std::cout);
            }
            // for this observer, we always pass it on to the next observer, even when this monitor was activated
            // for other observers this could be different.
            if (next) {
                next->monitor_activated(mon);
            }
        }

        PrintObserver::~PrintObserver() {
            // a virtual destructor is necessary, even when empty, for proper 
            // destruction when using base class pointers.
        }


    typename Observer::Ptr 
    create_PrintObserver(
            typename std::shared_ptr<solver> _slv, 
            const std::string& _action_name, 
            const std::string& _message, 
            typename Observer::Ptr _next ) 
    {
        PrintObserver::Ptr r( new PrintObserver(_slv, _action_name, _message,_next) );
        return r; 
    }
};