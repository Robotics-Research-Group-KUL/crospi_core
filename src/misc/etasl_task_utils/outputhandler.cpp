#include "etasl_task_utils/outputhandler.hpp"

namespace etasl {
    using namespace KDL;

    std::string cut_global( const std::string& name ) 
    { 
        if (name.substr(0,7)=="global.") {
            return name.substr(7);
        } else {
            return name;
        }
    } 
}