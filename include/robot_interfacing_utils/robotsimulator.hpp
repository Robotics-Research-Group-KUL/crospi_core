//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
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

#include "robot_interfacing_utils/robotdriver.hpp"

namespace etasl {

/**
 * @brief Specialization of RobotDriver for simulated robots.
 *
 * RobotSimulator is a lightweight subclass of RobotDriver intended for use
 * with simulated robot drivers (e.g., simple kinematic visualization (RVIZ), drivers that interface with Gazebo,
 * MuJoCo, or other simulators instead of real hardware). It inherits all
 * triple-buffered communication ports and lifecycle methods from RobotDriver
 * without adding any new members or methods.
 *
 * The only distinction from RobotDriver is semantic: this class signals to
 * the Crospi plugin loader and the rest of the framework that the driver is
 * intended for simulation rather than real hardware. All derived simulated
 * driver plugins (e.g., a Gazebo robot driver) should inherit from
 * RobotSimulator instead of RobotDriver directly.
 *
 * @see RobotDriver
 */
class RobotSimulator : public RobotDriver {
public:
    /**
     * @brief Default virtual destructor.
     */
    virtual ~RobotSimulator() = default;
    // No new methods — inherits everything from RobotDriver
};

} // namespace etasl
