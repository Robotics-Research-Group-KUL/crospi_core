//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Erwin Aertbeliën
//  email: <erwin.aertbelien@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.


#include "crospi_utils/solverfactory.hpp"
#include "crospi_utils/qpoasessolverfactory.hpp"
#include <expressiongraph/qpoases_solver.hpp>
#include "crospi_utils/etasl_error.hpp"
#include "crospi_utils/registry.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create TopicOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class QPOasesSolverFactory : public SolverFactory {

public:
    typedef boost::shared_ptr<QPOasesSolverFactory> SharedPtr;

    QPOasesSolverFactory()
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
            {
                "$id": "qpoasessolver.json",
                "title":"QPOases Parameters",
                "type":"object",
                "description" : "eTaSL constraints are translated into a quadratic problem.  This specifies that the QPOases solver will be used to solve this numeric problem",
                "properties" : {
                    "is-qpoasessolver" : {
                        "description":"key to indicate the QPOases solver will be used (boolean, but value is of no importance) ",
                        "type":"boolean"
                    },
                    "nWSR" : {
                        "description":"maximum number of working set recalcuations to be performed during initial homiotopy",
                        "type":"number",
                        "default":100
                    },
                    "regularization_factor": {
                        "description":"regularization factor used in the transcription to a numeric problem.  A higher value lowers joint velocities in the neighbourhoud of singularities, but decreases the adherence to the soft constraints",
                        "type":"number",
                        "default":1E-4
                    },
                    "cputime" : {
                        "description":"maximum allowed CPU time in seconds for the initialisation of QPOases",
                        "type" : "number",
                        "default":100.0
                    }
                },
                "required": ["nWSR","regularization_factor","cputime"],
                "additionalProperties" : false
            }
        )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "qpoasessolver";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual KDL::solver::Ptr create(const Json::Value& parameters, std::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        int nWSR =  jsonchecker->asInt(parameters, "nWSR");
        double regularization_factor =  jsonchecker->asDouble(parameters, "regularization_factor");;
        double cputime =  jsonchecker->asDouble(parameters, "cputime");
        return boost::make_shared<KDL::qpOASESSolver>(nWSR, cputime, regularization_factor);
    }

    virtual ~QPOasesSolverFactory() { }
};

void registerQPOasesSolverFactory()
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<SolverFactory>::registerFactory(boost::make_shared<QPOasesSolverFactory>());
}

} // namespace