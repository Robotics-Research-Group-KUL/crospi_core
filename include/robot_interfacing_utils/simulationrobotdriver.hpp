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

#include "robotdriver.hpp"
#include "controlmodes_enum.hpp"
#include <string>




namespace etasl {

class SimulationRobotDriver : public RobotDriver {
    public:
        typedef std::shared_ptr<SimulationRobotDriver> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        double periodicity;
        ControlMode::ControlMode control_mode;

        std::vector<double> initial_joints;
        std::vector<double> joint_pos;
        

    public:
        SimulationRobotDriver();

        virtual void construct(std::string robot_name, 
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        // virtual const std::string& getName() const override;

        virtual ~SimulationRobotDriver();
};

} // namespace etasl
