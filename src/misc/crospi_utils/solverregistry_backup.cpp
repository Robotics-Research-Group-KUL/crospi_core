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


#include <crospi_utils/etasl_error.hpp>
#include <crospi_utils/solverregistry.hpp>
#include <fmt/format.h>
namespace etasl {

SolverRegistry& SolverRegistry::instance()
{
    // it **is** thread-safe in C++11.
    static SolverRegistry myInstance;
    return myInstance;
}

/**
 * @brief register a factory
 * @param factory
 */
void SolverRegistry::registerFactory(SolverFactory::SharedPtr factory)
{
    m[factory->getName()] = factory;
}

KDL::solver::Ptr SolverRegistry::create(const std::string& name, const Json::Value& parameters)
{
    auto r = m.find(name);
    if (r == m.end()) {
        throw etasl_error(etasl_error::UNKNOWN_SOLVER_REQUESTED, fmt::format("Solver with name '{}' is not registered", name));
    }
    return r->second->create(parameters);
}

KDL::solver::Ptr createSolver(const std::string& name, const Json::Value& parameters)
{
    return SolverRegistry::instance().create(name, parameters);
}

}; // namespace etasl