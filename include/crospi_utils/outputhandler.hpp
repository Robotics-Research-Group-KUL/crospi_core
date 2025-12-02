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

#include <memory>
#include <expressiongraph/context.hpp>
#include <expressiongraph/solver.hpp>
#include "crospi_utils/etasl_error.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "crospi_utils/json_checker.hpp"



namespace etasl {
    using namespace KDL;

    class OutputHandler {
        public:
            typedef std::shared_ptr< OutputHandler > SharedPtr;

            /** 
             * @brief Construct the plugin. Since the constructor of a ROS2 plugin cannot have arguments, the following method is used to pass those arguments.
             * This member function should be called right after the constructor of the super class is called.
             * @param oh_name the name of the outputhandler
             * @param parameters additional configuration parameters coming from the JSON configuration
             * @param jsonchecker a pointer to the jsonchecker that can be used to check the validity of the configuration
             * @return bool true if constructed correctly, false otherwise
             */
            virtual bool construct(
                std::string _name, 
                rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value& _parameters,
                std::shared_ptr<etasl::JsonChecker> _jsonchecker
            ) = 0;
            
            /**
             * @brief Initialize the output handler
             * @param ctx etasl context
             * @param jnames vector with joint names
             * @param fnames vector with feature variable names
             * @return true if initialized, false if not initialized, e.g. because some data is not yet available. 
             * @details can be used to initialize the handler, or can be used to only change jpos/fpos
             *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
             *           feature variables)
             */
            [[nodiscard]] virtual bool initialize(
                Context::Ptr ctx,
                const std::vector< std::string >& jnames,
                const std::vector< std::string >& fnames
            ) = 0;
            /**
             * updates the values for the output and prints them in a tab-delimited 
             * called during the
             */
            virtual void update(
                const std::vector<std::string>& jnames,
                const Eigen::VectorXd& jpos,
                const Eigen::VectorXd& jvel,
                const std::vector<std::string>& fnames,
                const Eigen::VectorXd& fvel,
                const Eigen::VectorXd& fpos
                ) = 0;
            /**
             * @brief finalize
             * Handles finalization of the outputhandler, such as writing and closing files.
             */
            virtual void finalize() {};

            /**
             * @brief on_activate
             * Handles the activation of the outputhandler
             */
            virtual void on_activate(Context::Ptr ctx,    
                                    const std::vector<std::string>& jnames,
                                    const std::vector<std::string>& fnames,
                                    boost::shared_ptr<solver> slv) {};

            /**
             * @brief on_deactivate
             * Handles the deactivation of the outputhandler
             */
            virtual void on_deactivate(Context::Ptr ctx) {};

            /**
             * @brief on_cleanup
             * Handles the cleanup of the outputhandler
             */
            virtual void on_cleanup(Context::Ptr ctx) {};

            /**
             * Returns a name for an instance of this handler
            */
            virtual const std::string& getName() const = 0;

            virtual ~OutputHandler() {};

    }; // TopicOutputHandler

    /**
     * aux. routine that cuts off the "global." in front of names.
    */
    std::string cut_global( const std::string& name );


} // namespace KDL
