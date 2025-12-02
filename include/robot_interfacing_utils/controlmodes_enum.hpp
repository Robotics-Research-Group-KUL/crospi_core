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


namespace ControlMode {

    enum ControlMode
    {
        IDLE                = 0, //Robot stays still while reporting feedback
        JOINT_POSITION		= 1, 
        JOINT_VELOCITY		= 2,
        JOINT_IMPEDANCE		= 3,
        JOINT_TORQUE		= 4,
        CARTESIAN_IMPEDANCE	= 5,
        CARTESIAN_WRENCH	= 6,
        CARTESIAN_POSE		= 7
    };

}  // namespace ControlMode