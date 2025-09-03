#pragma once

#include <memory>
#include <expressiongraph/context.hpp>
#include <expressiongraph/solver.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "etasl_task_utils/json_checker.hpp"


namespace etasl {
    using namespace KDL;
    
    class InputHandler {
        public:
            typedef std::shared_ptr< InputHandler > SharedPtr;

            /** 
             * @brief Construct the plugin. Since the constructor of a ROS2 plugin cannot have arguments, the following method is used to pass those arguments.
             * This member function should be called right after the constructor of the super class is called.
             * @param ih_name the name of the inputhandler
             * @param parameters additional configuration parameters coming from the JSON configuration
             * @param jsonchecker a pointer to the jsonchecker that can be used to check the validity of the configuration
             * @return bool true if constructed correctly, false otherwise
             */
            virtual bool construct(
                std::string name, 
                rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
                const Json::Value& _parameters,
                std::shared_ptr<etasl::JsonChecker> _jsonchecker
            ) = 0;

            /**
             * @brief Initialize the input handler
             * @param ctx etasl context
             * @param jnames vector with joint names
             * @param fnames vector with feature variable names
             * @param jpos Eigen vector with joint positions
             * @param fpos Eigen vector with feature variable positions
             * @return true if initialized, false if not initialized, e.g. because some data is not yet available. 
             * @details can be used to initialize the handler, or can be used to only change jpos/fpos
             *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
             *           feature variables)
             */
            [[nodiscard]] virtual bool initialize(
                Context::Ptr ctx,
                const std::vector< std::string >& jnames,
                const std::vector< std::string >& fnames,
                Eigen::VectorXd& jpos,
                Eigen::VectorXd& fpos                
            ) = 0;
            
            /**
             * gets new values for this input and use them to fill in the input channels defined
             * within eTaSL.
             * time is an input argument that can be used to lookup an appropriate value for the input
             * (e.g. when the inputhandler stores a whole trajectory)
             */
            virtual void update(
                double time, 
                const std::vector<std::string>& jnames,
                Eigen::VectorXd& jpos, 
                const std::vector<std::string>& fnames,
                Eigen::VectorXd& fpos ) = 0;

            /**
             * name of the handler
            */
            virtual const std::string& getName() const = 0;

            /**
             * finalize the input handler.
            */
            virtual void finalize() {}

            /**
             * @brief on_activate
             * Handles the deactivation of the inputhandler
             */
            virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames,
                            boost::shared_ptr<solver> slv) {};

            /**
             * @brief on_activate
             * Handles the deactivation of the inputhandler
             */
            virtual void on_deactivate(Context::Ptr ctx) {};

            /**
             * @brief on_cleanup
             * Handles the cleanup of the inputhandler
             */
            virtual void on_cleanup(Context::Ptr ctx) {};


            virtual ~InputHandler() {};

    }; // InputHandler

} // namespace etasls
