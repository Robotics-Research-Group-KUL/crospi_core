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

#ifndef EXPRESSIONGRAPH_FEATUREVARIABLEINITIALIZER_HPP
#define EXPRESSIONGRAPH_FEATUREVARIABLEINITIALIZER_HPP

#include <Eigen/Dense>
#include <expressiongraph/context.hpp>
#include <expressiongraph/solver.hpp>
#include <jsoncpp/json/json.h>

namespace etasl {
    using namespace KDL;

    /**
     * Abstract interface to a class that initializes the feature variables in an eTaSL 
     * specification.
     * createFeatureVariableInitializer creates a default initializer, suitable for
     * most eTaSL applications.
     */
    class FeatureVariableInitializer {
        public:
            enum ErrorCode {
                DISPLAY_ERRORS     = 5
            };

            typedef typename std::shared_ptr<FeatureVariableInitializer> Ptr;

            /**
             * gets the current value of the feature variable state
             */
            virtual const Eigen::VectorXd& getFeatureState() const = 0;

            /**
             * prepare the solver for initialization
             * among others, this determines the size of the robot and feature
             * variable states.
             * 
             * @warning can throw an etasl_error
             */
            virtual void prepareSolver() = 0;

            /**
             * Sets the values for the feature variables (before initialization)
             */
            virtual void setFeatureState( const Eigen::VectorXd& fpos) = 0;

            /**
             * Set the values for the robot joints. Typically correspond to the actual
             * joint values of the robot when starting up.
             */
            virtual void setRobotState( const Eigen::VectorXd& jpos) = 0;

            /**
             * Performs initialization and returns an error code.
             * Returns 0 if success.  
             *
             * The initialization performs an optimization in _only_ the feature variables
             * (i.e. keeping the robot variables fixed).
             *
             * You can read the results of the initialization using getFeatureState().
             *
             * @warning
             *  (1) initial robot state and feature state are extracted from the context of
             *    the task specification.
             *  (2) But for a physical robot, you'll want to set the robot state using
             *    setRobotState() before initializing.
             *  (3) can throw an etasl_error
             */
            virtual void initialize_feature_variables() = 0;

            virtual ~FeatureVariableInitializer() {}; 
    };

    FeatureVariableInitializer::Ptr 
    createFeatureVariableInitializer(solver::Ptr _slvr, Context::Ptr _ctx, const Json::Value& param);

} // namespace etasl

#endif
