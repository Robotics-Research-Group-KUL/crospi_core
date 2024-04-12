#pragma once

#include "jsoncpp/json/json.h"
#include <etasl_task_utils/outputhandler.hpp>
#include <memory>
#include <unordered_map>

namespace etasl {



/**
 * This (abstract) class describes a Factory class for OutPutHandlers
 * 
 * A Factory can create a specific instance of an OutputHandler based on
 * its input parameters.
 * 
 * Each implementation of an OutputHandler will define a register.... () function 
 * in a ....factory.hpp header file that only contains a register... function that
 * registers this implementation under a name given by getName().
 * 
 * The application code can check the validity of the parameters by using the schema
 * returned by getSchema().  Typically, there is a copy of this in the schema directory.
 * 
 * Uses will ask the registry to create an OutputHandler instance using the
 *   Registry<OutputHandlerFactory>::create( json ) method.
 * This will call the create method of the appropriate OutputHandlerFactory.
 * 
 * By convention there is a boolean property with the name `is-<name>` where name is the same name
 * as returned by the getName() method.  The registry uses this to call the appropriate Factory.
 * 
*/
class OutputHandlerFactory {

public:
    typedef std::shared_ptr<OutputHandlerFactory> SharedPtr;
    typedef OutputHandler::SharedPtr ProductSharedPtr;
    // typedef OutputHandler::SharedPtr SharedProductPtr; // Base-class of the Product class

    /**
     * @brief gets the schema for the parameters of this factory.  You typically find a duplicate of
     * this schema in the ./scripts/schema directory.
     * 
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
    virtual OutputHandler::SharedPtr create(const Json::Value& parameters) = 0;

    virtual ~OutputHandlerFactory() { }
};

} // etasl