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

#pragma once

#include <crospi_utils/solverregistry.hpp>
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


