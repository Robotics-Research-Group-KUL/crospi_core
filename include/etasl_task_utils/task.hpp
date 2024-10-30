#ifndef TASK_HPP_234DEA1
#define TASK_HPP_234DEA1

#include <cstdio>

#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/printobserver.hpp"
#include "etasl_task_utils/timestatistics.hpp"
#include <etasl_task_utils/abstracttask.hpp>
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <string>

namespace etasl {

using namespace KDL;

class Task : public AbstractTask {
    // void debug_etasl_parameters(const etaslParameters& p);
protected:
    Context::Ptr ctx;
    LuaContext LUA;
    boost::shared_ptr<solver> slv;
    std::vector<std::string> jnames;
    std::vector<std::string> fnames;
    Eigen::VectorXd jpos;
    Eigen::VectorXd fpos;
    Eigen::VectorXd jvel;
    Eigen::VectorXd fvel;
    double dt;
    double time;
    std::string name;
    Json::Value param;
    std::vector<OutputHandler::SharedPtr> outputhandlers;
    std::vector<InputHandler::SharedPtr> inputhandlers;
    std::vector<bool> ih_initialized;
    LogHandler::SharedPtr log;
    TimeStatistics timestats;
    double reporting_interval;

public:

    const std::string getName() const {
        return name;
    }

    /**
     * @brief Constructor
     * @param _param parameters of the task.  This contains eTaSL related parameters 
     * using the "etasl" key (see schema/etasl.json),  the eTaSL lua file to read under 
     * the "etaslfile" key, parameters for this eTaSL specification under the "task" key.
     * Creates a lua interpreter, the output-handlers, input-handlers, and solver according
     * to the parameteter specification.  It introduces the parameters under the "task" key
     * to the Lua context using the global variable "parameterstring".
     * @warning can throw etasl_error 
     */
    explicit Task(const Json::Value& _param);

    /**
     * @brief gets the sample time according to the parameters and set as sample time for this task.
     * @return  sample time as a double (in seconds)
     */
    virtual double getSampleTime() override;

    /**
     * @brief loads and interpretes the etasl file in the key "etaslfile".  The file can use
     * the global variable "parameterstring" to access parameters for the task.
     * @warning can throw etasl_error, in case of error reading the task file.
     */
    virtual void load() override;

    virtual void add_output_handler(OutputHandler::SharedPtr h) override;
    virtual void add_input_handler(InputHandler::SharedPtr h) override;
    virtual void set_log_handler(LogHandler::SharedPtr h) override;

    virtual void resetInitialization() override;

    /**
     * @brief Initializes eTaSL feature variables by solving the initial non-convex optimization 
     * problem.  Initializes and reads once from the inputhandlers.  This includes the
     * robot joint variables if they are available. After initialization, it creates all
     * the observers and initializes the output-handlers.
     * Returns false if this routine needs to be called again, after handling possible inputs.
     * (because some required inputs from the input-handlers are missing.)
     */
    [[nodiscard]] virtual bool initialize() override;

    /**
     * starts the solver for execution. 
     * @warning can allocate memory.
    */
    virtual void startLoop() override;

    /**
     * Reads input handlers, executes the controller for one sample period, and 
     * writes output-handlers.  
     */
    virtual bool onTimer() override;
    virtual void finalize() override;
};

}; // namespace etasl
#endif
