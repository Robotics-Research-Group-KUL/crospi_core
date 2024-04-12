#ifndef ABSTRACT_TASK_HPP_134DE3
#define ABSTRACT_TASK_HPP_134DE3

#include <string>
#include "etasl_task_utils/outputhandler.hpp"
#include "etasl_task_utils/inputhandler.hpp"
#include "etasl_task_utils/loghandler.hpp"
#include "jsoncpp/json/json.h"

namespace etasl {
    /**
     * @brief AbstractTask describes the interface of an eTaSL task
     * 
     * Typically implementations will inherit from the implementation in task.hpp, but in
     * some cases it could be better to reimplement more specifically and directly inherit
     * from this Abstract Base class
     */
    class AbstractTask {
        public:
            /**
             * @brief creates a new eTaSL context and solver 
             */
            virtual double getSampleTime() = 0;
            virtual void load() = 0;
            virtual void add_output_handler(OutputHandler::SharedPtr h) = 0;
            virtual void add_input_handler(InputHandler::SharedPtr h) = 0;
            virtual void set_log_handler(LogHandler::SharedPtr h) = 0;
            virtual void resetInitialization() = 0;
            [[nodiscard]] virtual bool initialize() = 0;
            virtual void startLoop() = 0;
            /**
             * To be called at each sample period
             * returns true if task is finished, otherwise false.
            */
            virtual bool onTimer() = 0;
            virtual void finalize() = 0;
            virtual ~AbstractTask() {}
    };





}; // namespace etasl

#endif