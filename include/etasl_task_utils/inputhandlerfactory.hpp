#pragma once

#include "jsoncpp/json/json.h"
#include <etasl_task_utils/inputhandler.hpp>
#include <memory>
#include <unordered_map>

namespace etasl {



/**
 * This (abstract) class describes a Factory class for InPutHandlers
 * 
 * 
*/
class InputHandlerFactory {

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;
    typedef InputHandler::SharedPtr ProductSharedPtr;
    // typedef OutputHandler::SharedPtr SharedProductPtr; // Base-class of the Product class

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema() = 0;

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName() = 0;

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual InputHandler::SharedPtr create(const Json::Value& parameters) = 0;

    virtual ~InputHandlerFactory() { }
};

} // etasl