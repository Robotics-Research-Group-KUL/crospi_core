
//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Authors: Santiago Iregui and Erwin Aertbeliën
//  emails: <santiago.iregui@kuleuven.be> and <erwin.aertbelien@kuleuven.be>
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

#include "jsoncpp/json/json.h"
#include "robot_interfacing_utils/robotdriver.hpp"
#include <memory>
#include <unordered_map>
#include "crospi_utils/json_checker.hpp"


namespace etasl {



/**
 * This (abstract) class describes a Factory class for RobotDriver
 * 
 * 
*/
class RobotDriverFactory {

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;
    typedef RobotDriver::SharedPtr ProductSharedPtr;
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
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters, std::shared_ptr<JsonChecker> jsonchecker) = 0;

    virtual ~RobotDriverFactory() { }
};

} // etasl