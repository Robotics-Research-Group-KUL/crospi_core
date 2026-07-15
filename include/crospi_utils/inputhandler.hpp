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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "crospi_utils/json_checker.hpp"


namespace etasl {
    using namespace KDL;
    
    /**
     * @brief Abstract base class for input handlers in the Crospi framework.
     *
     * InputHandler defines the interface for plugins that feed external data into
     * the eTaSL expression graph. Input handlers are responsible for reading external continuous
     * data streams (e.g. ROS2 topics with specific msg types or other communication channels) and writing them into the eTaSL input channels.
     *
     * The lifecycle of an InputHandler follows these states:
     * - construct(): Parse configuration and allocate resources.
     * - initialize(): Initialize communication with external data sources (e.g. subscribe to topics).
     * - on_activate(): Hook called when the Crospi node enters the active state.
     * - update(): Called each control cycle to refresh input channel values.
     * - on_deactivate(): Hook called when the Crospi node exits the active state.
     * - on_cleanup(): Hook called during the Crospi node's cleanup phase.
     * - finalize(): Safely shut down and release resources.
     */
    class InputHandler {
        public:
            /**
             * @brief Shared pointer alias for InputHandler.
             */
            typedef std::shared_ptr< InputHandler > SharedPtr;

            /**
             * @brief Construct the input handler with the given configuration.
             *
             * Called after the default constructor to pass configuration parameters.
             * This separation exists because ROS2 plugin constructors cannot take arguments.
             *
             * @param name The name of the input handler.
             * @param _node Shared pointer to the lifecycle node owning this handler.
             * @param _parameters JSON configuration for this input handler.
             * @param _jsonchecker Pointer to a JsonChecker for validating configuration values.
             * @return true if constructed successfully, false otherwise.
             */
            virtual bool construct(
                std::string name,
                rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value& _parameters,
                std::shared_ptr<etasl::JsonChecker> _jsonchecker
            ) = 0;

            /**
             * @brief Initialize the input handler to initialize communication with external data sources.
             *
             * Called after construct() and before activate() and update(). 
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             * @param jnames Vector of joint names in the order expected by eTaSL.
             * @param fnames Vector of feature variable names in the order expected by eTaSL.
             * @param[out] jpos (Normally not used) Access to Eigen joint positions.
             * @param[out] fpos (Normally not used) Access to Eigen feature variable positions.
             * @return true if initialization succeeded, false otherwise (e.g., required data
             *         is not yet available).
             */
            [[nodiscard]] virtual bool initialize(
                Context::Ptr ctx,
                const std::vector< std::string >& jnames,
                const std::vector< std::string >& fnames,
                Eigen::VectorXd& jpos,
                Eigen::VectorXd& fpos                
            ) = 0;
            
            /**
             * @brief Update the input channel values for one time-step of the control cycle.
             *
             * Called periodically by the Crospi framework (once per control cycle) to
             * refresh the values of the eTaSL input channels. The implementation should
             * read the latest sensor data or compute new input values and write them into
             * the jpos and fpos vectors.
             *
             * @param time Current time stamp (in seconds). Can be used for time-based
             *             expressions or automatic differentiation within the expression graph.
             * @param jnames Vector of joint names in the order expected by eTaSL.
             * @param jpos (Normally not used) Access to Eigen joint positions.
             * @param fnames Vector of feature variable names in the order expected by eTaSL.
             * @param fpos (Normally not used) Access to Eigen feature variable positions.
             *
             * @warning This method runs in the Crospi control thread and must NOT contain
             *          any blocking code (e.g., network I/O that waits indefinitely,
             *          sleeps, etc). Blocking calls will disrupt
             *          the real-time control loop and cause timing jitter or missed control
             *          cycles. All data retrieval should be non-blocking.
             */
            virtual void update(
                double time,
                const std::vector<std::string>& jnames,
                Eigen::VectorXd& jpos,
                const std::vector<std::string>& fnames,
                Eigen::VectorXd& fpos ) = 0;

            /**
             * @brief Get the name of the input handler.
             * @return Constant reference to the handler's name string.
             */
            virtual const std::string& getName() const = 0;

            /**
             * @brief Safely finalize the input handler.
             *
             * Called when the input handler is being destroyed. Override to release
             * resources, close connections, or perform cleanup.
             */
            virtual void finalize() {}

            /**
             * @brief Callback invoked when the Crospi node enters the active state.
             *
             * The implementation should create the eTaSL input channels that this handler
             * will feed data into. Input channels are obtained from the eTaSL context
             * using the appropriate type, e.g.:
             * @code
             * input_channel = ctx->getInputChannel<KDL::Twist>("variable_name");
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
             * The implementation should set all previously created input channel pointers
             * to nullptr. This releases the reference to the eTaSL input channels and
             * prevents dangling pointers during the inactive or cleanup phases.
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             */
            virtual void on_deactivate(Context::Ptr ctx) {};

            /**
             * @brief Callback invoked when the Crospi node enters the cleanup state.
             *
             * This method is normally left empty by derived classes. It exists as a hook
             * in case custom logic needs to be executed during the Crospi node's cleanup
             * phase.
             *
             * @param ctx Shared pointer to the eTaSL expression graph context.
             */
            virtual void on_cleanup(Context::Ptr ctx) {};

            /**
             * @brief Destroy the InputHandler object.
             */
            virtual ~InputHandler() {};

    }; // InputHandler

} // namespace etasls
