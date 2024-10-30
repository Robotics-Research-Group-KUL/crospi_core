#ifndef ETASL_ROS2_IO_HANDLERS_SIMPLE_NODE_HPP
#define ETASL_ROS2_IO_HANDLERS_SIMPLE_NODE_HPP



#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/solver_registry.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <expressiongraph/defaultobserver.hpp>


using namespace KDL;

/**
    * Example of a class that handles output suitable for simple, unbuffered, csv output.
    * It gets the output values from the context and outputs the results in a
    * tab-delimited file with a header containing the names of the signals
    *
    * Only handles double-expressions !
    * 
    * This class is just an example, you can choose the structure of the class and its methods
    * yourself.
    */
class eTaSL_OutputHandler
{

    public:

            /** 
        * Gets specific outputexpressions as output.
        * - has_warnings() will be true if not all names are found.
        * - Zero will be returned for names that are not found.
        * - getUnknownVariables() will return the names that were not found.
        */
        eTaSL_OutputHandler(Context::Ptr _ctx, const std::vector<std::string>& _varnames);

        /**
            * Gets all (DOUBLE-expr) outputexpressions as output
            */
        eTaSL_OutputHandler(Context::Ptr _ctx);

        /**
            * prints a header with the names of the output expressions
            */ 
        void printHeader(std::ofstream& os);


        /**
            * updates the values for the output and prints them in a tab-delimited format
            */
        void printOutput(std::ofstream& os);


            /**
        * aux. function to cut out the "global.", if it exists
        * (one day this will probably become obsolete)
        */
        std::string cut_global( const std::string& name ) const;
    
    private:
        std::vector< Expression<double>::Ptr > outp; 
        Context::Ptr ctx;
        std::vector<std::string>   varnames;
        std::vector<double>        values;
        std::vector<std::string>   not_found;


};


class eTaSL_InputHandler
{
    public:

        /**
        * @brief Construct a new eTaSL InputHandler object to generate a sine wave signal
        * 
        * @param ctx : context
        * @param name: name of the input channel
        * @param _amplitude : amplitude of the input
        * @param _frequency : frequency of the input
        * @param _phase     : phase of the input
        * 
        */
        eTaSL_InputHandler(Context::Ptr ctx, const std::string& channel_name, double _amplitude, double _frequency, double _phase);


         /**
            * @brief update
            * 
            * Update the input and adapts the context to this input.
            * @param time : compute the input for this time
            */
        void update(double time);


    private:
        VariableType<double>::Ptr inp;
        double amplitude;
        double omega;
        double phase;
        int    time_ndx;

};


class PrintObserver: public Observer {

    public:
        typedef boost::shared_ptr< PrintObserver > Ptr;

        /**
        * \param _slv  solver, where we will get joint and feature values 
        * \param _message  an additional message to pass.
        * \param _next next observer to check.  
        */
        PrintObserver(boost::shared_ptr<solver> _slv, const std::string& _action_name, const std::string& _message, Observer::Ptr _next );

        virtual void monitor_activated(const  MonitorScalar& mon) {
            if (mon.action_name.compare(action_name)==0) {
                std::string msg = message + "(argument=" + mon.argument + ")";
                slv->getJointStates(jpos);
                slv->getFeatureStates(fpos);
                std::cout << msg << std::endl;
                std::cout << "robot variables   : " << jpos << std::endl;
                std::cout << "feature variables : " << fpos << std::endl;
            }
            // for this observer, we always pass it on to the next observer, even when this monitor was activated
            // for other observers this could be different.
            if (next) {
                next->monitor_activated(mon);
            }
        }

        virtual ~PrintObserver() {
            // a virtual destructor is necessary, even when empty, for proper 
            // destruction when using base class pointers.
        }
    private:
        boost::shared_ptr<solver> slv;
        Observer::Ptr next;
        Eigen::VectorXd jpos;
        Eigen::VectorXd fpos;
        std::string action_name;
        std::string message;  

};



#endif