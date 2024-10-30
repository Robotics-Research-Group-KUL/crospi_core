#include "IO_handlers_deleteme.hpp"

using namespace KDL;


/** 
    * Gets specific outputexpressions as output.
    * - has_warnings() will be true if not all names are found.
    * - Zero will be returned for names that are not found.
    * - getUnknownVariables() will return the names that were not found.
    */
eTaSL_OutputHandler::eTaSL_OutputHandler(Context::Ptr    _ctx, const std::vector<std::string>& _varnames):
ctx(_ctx)
,varnames(_varnames)
,values(_varnames.size())
,outp(_varnames.size())
{
    std::fill(values.begin(), values.end(), 0.0);
    for (auto i = 0; i < varnames.size(); ++i) {
        outp[i] = ctx->getOutputExpression<double>(varnames[i]); 
        if (!outp[i]) {
            not_found.push_back( varnames[i]);
            outp[i] = Constant(0.0);
        }
    }
}

/**
    * Gets all (DOUBLE-expr) outputexpressions as output
    */
eTaSL_OutputHandler::eTaSL_OutputHandler(Context::Ptr   _ctx):ctx(_ctx) 
{
    for (auto ov : _ctx->output_vars) {
        auto ptr = ctx->getOutputExpression<double>(ov.first);
        if (ptr!=0) {
            varnames.push_back(cut_global(ov.first));
            outp.push_back(ptr);
            values.push_back(0.0);
        }  
    }
}



/**
    * aux. function to cut out the "global.", if it exists
    * (one day this will probably become obsolete)
    */
std::string eTaSL_OutputHandler::cut_global( const std::string& name ) const {
    if (name.substr(0,7)=="global.") {
        return name.substr(7);
    } else {
        return name;
    }
} 




/**
    * prints a header with the names of the output expressions
    */ 
void eTaSL_OutputHandler::printHeader(std::ofstream& os) {
    unsigned int L = varnames.size();
    for (unsigned int i=0;i<L;++i) {
        if (i!=0) {
            os << "\t";
        }
        os << "\"" << varnames[i] << "\"";
    }
    os << std::endl;
    os << std::scientific;
}

/**
    * updates the values for the output and prints them in a tab-delimited format
    */
void eTaSL_OutputHandler::printOutput(std::ofstream& os) {
    unsigned int L = outp.size();
    for (unsigned int i=0;i<L;++i) {
        values[i] = outp[i]->value();
        if (i!=0) {
            os << "\t";
        }
        os << std::setprecision(17) << values[i];
    }
    os << std::endl;
}



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
    eTaSL_InputHandler::eTaSL_InputHandler(Context::Ptr ctx, const std::string& channel_name, double _amplitude, double _frequency, double _phase=0.0):
        amplitude(_amplitude),
        omega(2*M_PI*_frequency),
        phase(_phase)
        {
        inp = ctx->getInputChannel<double>(channel_name);     // this gives back a reference to the variable where the input is stored.
        if (!inp) {
            std::cerr << "Warning: input channel does not exists, ignored" << std::endl;
        }
        time_ndx = ctx->getScalarNdx("time");
        }

/**
    * @brief update
    * 
    * Update the input and adapts the context to this input.
    * @param time : compute the input for this time
    */
void eTaSL_InputHandler::update(double time) {
    double f  = amplitude*sin( omega*time + phase);
    double df = amplitude*omega*cos( omega*time + phase);
    if (inp) {
        inp->setValue( f );
        inp->setJacobian( time_ndx, df );
    }
}



/**
    * \param _slv  solver, where we will get joint and feature values 
    * \param _message  an additional message to pass.
    * \param _next next observer to check.  
    */
PrintObserver::PrintObserver(boost::shared_ptr<solver> _slv,
                    const std::string& _action_name,
                    const std::string& _message,
                    Observer::Ptr _next ):
    slv(_slv),
    next(_next) ,
    jpos( Eigen::VectorXd::Zero(_slv->getNrOfJointStates()) ),
    fpos( Eigen::VectorXd::Zero(_slv->getNrOfFeatureStates())),
    action_name(_action_name),
    message(_message)
{
    
}






