#pragma once

#include <memory>
#include <expressiongraph/context.hpp>
#include "etasl_task_utils/etasl_error.hpp"

namespace etasl {
    using namespace KDL;

    class OutputHandler {
        public:
            typedef std::shared_ptr< OutputHandler > SharedPtr;
            
            /**
             * is called during the initialization of the eTaSL task
            */
            virtual void initialize(
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
            virtual void on_activate(Context::Ptr ctx) {};

            /**
             * @brief on_deactivate
             * Handles the deactivation of the outputhandler
             */
            virtual void on_deactivate(Context::Ptr ctx) {};

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
