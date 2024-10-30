#pragma once

/**
 * file: etasl/solver_factory_qpoases
 *
 * purpose:
 *   improved version of solver_factory_qpoases
 *
 * author:
 *   Erwin Aertbelien, 2023
 *
 * part of eTaSL
 */

#include <etasl_task_utils/solverregistry.hpp>
#include "jsoncpp/json/json.h"

namespace etasl {

class SolverFactoryQPOases: public SolverFactory {

public:

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema();

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName() ;

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual KDL::solver::Ptr create(const Json::Value& parameters);

    virtual ~SolverFactoryQPOases() { }
};

void registerSolverFactoryQPOases();

} // namespace etasl


