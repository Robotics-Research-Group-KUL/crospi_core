#pragma once

#include "jsoncpp/json/json.h"
#include <expressiongraph/solver.hpp>
#include <memory>
#include <unordered_map>
#include "etasl_task_utils/json_checker.hpp"

namespace etasl {


class SolverFactory {

public:
    typedef std::shared_ptr<SolverFactory> SharedPtr;
    typedef KDL::solver::Ptr ProductSharedPtr;
    

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
    virtual ProductSharedPtr create(const Json::Value& parameters, boost::shared_ptr<JsonChecker> jsonchecker) = 0;

    virtual ~SolverFactory() { }
};

} // etasl