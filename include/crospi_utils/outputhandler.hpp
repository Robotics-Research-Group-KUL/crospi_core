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

    /**
     * @brief Abstract base class for output handlers in the Crospi framework.
     *
     * OutputHandler defines the interface for plugins that publish computed data from
     * the eTaSL expression graph to external destinations. Output handlers are
     * responsible for reading output channel values from the eTaSL context and
     * transmitting them as continuous data streams (e.g., publishing to ROS2 topics
     * or other communication channels).
     *
     * The lifecycle of an OutputHandler follows these states:
     * - construct(): Parse configuration and allocate resources.
     * - initialize(): Set up communication with external data sinks (e.g., create publishers).
     * - on_activate(): Create eTaSL output channels and start publishing.
     * - update(): Called each control cycle to read and publish output channel values.
     * - on_deactivate(): Clear data structures and deactivate lifecycle publishers.
     * - on_cleanup(): Hook called during the Crospi node's cleanup phase.
     * - finalize(): Safely shut down and release resources.
     */
    class OutputHandler {
        public:
            /**
             * @brief Shared pointer alias for OutputHandler.
             */
            typedef std::shared_ptr< OutputHandler > SharedPtr;

            /**
             * @brief Construct the output handler with the given configuration.
             *
             * Called after the default constructor to pass configuration parameters.
             * This separation exists because ROS2 plugin constructors cannot take arguments.
             *
             * @param _name The name of the output handler.
             * @param _node Shared pointer to the lifecycle node owning this handler.
             * @param _parameters JSON configuration for this output handler.
             * @param _jsonchecker Pointer to a JsonChecker for validating configuration values.
             * @return true if constructed successfully, false otherwise.
             */
            virtual bool construct(
                std::string _name,
                rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value& _parameters,
                std::shared_ptr<etasl::JsonChecker> _jsonchecker
            ) = 0;

            /**
             * @brief Initialize the output handler and set up communication with external data sinks.
             *
             * Called after construct() and before activate() and update(). The implementation should
             * set up the necessary communication channels (e.g., create ROS2 lifecycle publishers)
             * to send output data to external destinations.
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             * @param jnames Vector of joint names in the order expected by eTaSL.
             * @param fnames Vector of feature variable names in the order expected by eTaSL.
             * @return true if initialization succeeded, false otherwise (e.g., failed to
             *         create publisher or required data is not yet available).
             */
            [[nodiscard]] virtual bool initialize(
                Context::Ptr ctx,
                const std::vector< std::string >& jnames,
                const std::vector< std::string >& fnames
            ) = 0;

            /**
             * @brief Update the output data for one time-step of the control cycle.
             *
             * Called periodically by the Crospi framework (once per control cycle) to
             * read the latest computed values from the eTaSL output channels and publish
             * them to the external data sinks.
             *
             * @param jnames Vector of joint names in the order expected by eTaSL.
             * @param jpos Current joint positions computed by eTaSL.
             * @param jvel Current joint velocities computed by eTaSL.
             * @param fnames Vector of feature variable names in the order expected by eTaSL.
             * @param fpos Current feature variable positions computed by eTaSL.
             * @param fvel Current feature variable velocities computed by eTaSL.
             *
             * @warning This method runs in the Crospi control thread and must NOT contain
             *          any blocking code (e.g., network I/O that waits indefinitely,
             *          sleeps, etc.). Blocking calls will disrupt the real-time control
             *          loop and cause timing jitter or missed control cycles.
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
             * @brief Safely finalize the output handler.
             *
             * Called when the output handler is being destroyed (after the control loop
             * has stopped). Override to release resources, close files, destroy publishers,
             * or perform cleanup.
             *
             * If the output handler publishes safety-critical data (e.g., zero joint
             * velocities on shutdown), the implementation should ensure the message is
             * received before destroying the publisher. For ROS2 reliable publishers,
             * this can be done using:
             * @code
             * pub->wait_for_all_acked(std::chrono::milliseconds(500));
             * @endcode
             * before deactivating and destroying down the publisher.
             */
            virtual void finalize() {};

            /**
             * @brief Callback invoked when the Crospi node enters the active state.
             *
             * The implementation should activate the lifecycle publishers (in case of ROS2 topics) andcreate the eTaSL output channels that this handler
             * will read data from. Output channels are obtained from the eTaSL context
             * using the appropriate type, e.g.:
             * @code
             * auto ptr = ctx->getOutputExpression<double>("variable_name");
             * @endcode
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             * @param jnames Vector of joint names.
             * @param fnames Vector of feature variable names.
             * @param slv Shared pointer to the eTaSL solver.
             */
            virtual void on_activate(Context::Ptr ctx,
                                    const std::vector<std::string>& jnames,
                                    const std::vector<std::string>& fnames,
                                    boost::shared_ptr<solver> slv) {};

            /**
             * @brief Callback invoked when the Crospi node exits the active state.
             *
             * The implementation should set all previously created output channel pointers
             * to nullptr. This releases the reference to the eTaSL output channels and
             * prevents dangling pointers during the inactive or cleanup phases.
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             */
            virtual void on_deactivate(Context::Ptr ctx) {};

            /**
             * @brief Callback invoked when the Crospi node enters the cleanup state.
             *
             * This method is normally left empty by derived classes since real cleanup is handled at finalize state. It exists as a hook
             * in case custom logic needs to be executed during the Crospi node's cleanup
             * phase.
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             */
            virtual void on_cleanup(Context::Ptr ctx) {};

            /**
             * @brief Get the name of the output handler.
             * @return Constant reference to the handler's name string.
             */
            virtual const std::string& getName() const = 0;

            /**
             * @brief Destroy the OutputHandler object.
             */
            virtual ~OutputHandler() {};

    }; // TopicOutputHandler

    /**
     * @brief Remove the "global." prefix from an eTaSL variable name.
     *
     * eTaSL variable names are often prefixed with "global." when created in the
     * global context. This auxiliary routine strips that prefix for use in output
     * formatting or ROS2 topic naming.
     *
     * @param name The variable name, possibly prefixed with "global.".
     * @return The name without the "global." prefix, or the original string if
     *         the prefix is not present.
     */
    std::string cut_global( const std::string& name );


} // namespace KDL
