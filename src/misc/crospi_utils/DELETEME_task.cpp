#include "crospi_utils/task.hpp"
#include "crospi_utils/outputhandlerfactory.hpp"
#include "crospi_utils/inputhandlerfactory.hpp"
#include "crospi_utils/featurevariableinitializer.hpp"
#include "crospi_utils/solverfactory.hpp"
#include "crospi_utils/registry.hpp"
#include <fmt/format.h>

namespace etasl {
using namespace KDL;

Task::Task(const Json::Value& _param)
    : param(_param)
{
    log = std::make_shared<DefaultLogHandler>();
    if (param["title"]) {
        name = param["title"].asString();
    } else {
        name = "task-without-name";
    }
    log->log(INFO, "{} : constructor ", name);
    ctx = create_context();
    ctx->addType("robot");
    ctx->addType("feature");
    LUA.initContext(ctx);
    Json::FastWriter writer;
    std::string parameters = writer.write(param["task"]);
    lua_pushstring(LUA.L, parameters.c_str());
    lua_setglobal(LUA.L, "parameterstring");

    dt = param["etasl"]["sample_time"].asDouble();
    reporting_interval = param["etasl"]["reporting_interval"].asDouble();
    // create outputhandlers:
    for (const auto& p : param["etasl"]["outputhandlers"]) {
        add_output_handler(Registry<OutputHandlerFactory>::create(p));
    }
    // create inputhandlers:
    for (const auto& p : param["etasl"]["inputhandlers"]) {
        add_input_handler(Registry<InputHandlerFactory>::create(p));
        ih_initialized.push_back(false);
    }
    slv = Registry<SolverFactory>::create(param["etasl"]["solver"]);
}

double Task::getSampleTime()
{
    return dt;
}

void Task::add_output_handler(OutputHandler::SharedPtr h)
{
    log->log(INFO, "{} : register_output_handler", name);
    outputhandlers.push_back(h);
}

void Task::add_input_handler(InputHandler::SharedPtr h)
{
    log->log(INFO, "{} : register_input_handler", name);
    inputhandlers.push_back(h);
}

void Task::set_log_handler(LogHandler::SharedPtr h)
{
    log = h;
}

void Task::load()
{
    const std::string filename = param["etaslfile"].asString();
    log->log(INFO, "{} : load '{}' ", name, filename);
    try {
        int retval = LUA.executeFile(filename);
        if (retval != 0) {
            throw etasl_error(
                etasl_error::FAILED_TO_LOAD,
                fmt::format("failed to read lua file : {}", filename));
        }
    } catch (const char* msg) {
        throw etasl_error(etasl_error::LUA_PARSER_ERROR, msg);
    }
}

void Task::resetInitialization()
{
    log->log(INFO, "{} : Task::resetInitialization", name);
    std::vector<int> jndx;
    std::vector<int> fndx;
    ctx->getScalarsOfType("robot", jndx);
    ctx->getScalarsOfType("feature", fndx);

    jpos = Eigen::VectorXd::Zero(jndx.size());
    fpos = Eigen::VectorXd::Zero(fndx.size());
    jvel = Eigen::VectorXd::Zero(jpos.size());
    fvel = Eigen::VectorXd::Zero(fpos.size());

    // get initial values from eTaSL specification:
    jnames.clear();
    for (size_t i = 0; i < jndx.size(); ++i) {
        auto s = ctx->getScalarStruct(jndx[i]);
        jpos[i] = s->initial_value;
        jnames.push_back(s->name);
    }
    fnames.clear();
    for (size_t i = 0; i < fndx.size(); ++i) {
        auto s = ctx->getScalarStruct(fndx[i]);
        fpos[i] = s->initial_value;
        fnames.push_back(s->name);
    }
    // overwrite with initial values form the JSON parameters:
    // @todo remove this?
    if (param["etasl"]["initial_values"]) {
        const auto& iv = param["etasl"]["initial_values"];
        for (size_t i = 0; i < jndx.size(); ++i) {
            if (iv.isMember(jnames[i])) {
                jpos[i] = iv[jnames[i]].asDouble();
            }
        }
        for (size_t i = 0; i < fndx.size(); ++i) {
            if (iv.isMember(fnames[i])) {
                fpos[i] = iv[fnames[i]].asDouble();
            }
        }
    }

    log->log(INFO, "{} : Initial values for joint variables : ", name);
    for (size_t i = 0; i < jnames.size(); ++i) {
        log->log(INFO, "    {:20} : {}", jnames[i].c_str(), jpos[i]);
    }
    log->log(INFO, "{} : Initial values for feature variables :", name);
    for (size_t i = 0; i < fnames.size(); ++i) {
        log->log(INFO, "    {:20} : {}", fnames[i].c_str(), fpos[i]);
    }

    for (size_t i = 0; i < inputhandlers.size(); ++i) {
        ih_initialized[i] = false;
    }
}

bool Task::initialize()
{
    log->log(INFO, "{} : Task::initialize", name);

    // initialize time:
    time = 0.0;
    log->log(INFO, "{} : Initialize input handlers:", name);
    // initial input is used for initialization.
    bool fail = false;
    for (size_t i = 0; i < inputhandlers.size(); ++i) {
        log->log(INFO, "    |- {}", inputhandlers[i]->getName());
        if (!ih_initialized[i]) {
            ih_initialized[i] = inputhandlers[i]->initialize(ctx, jnames, fnames, jpos, fpos);
            if (!ih_initialized[i]) {
                fail = true;
            }
        }
    }
    if (fail) {
        return false;
    }
    for (auto h : inputhandlers) {
        h->update(time, jnames, jpos, fnames, fpos);
    }

    log->log(INFO, "{} : Prepare to Initialize feature variables:", name);
    // initialization of robt and feature variables:
    auto initializer = createFeatureVariableInitializer(slv, ctx, param["etasl"]["initializer"]);
    initializer->prepareSolver();

    log->log(INFO, "{} : Initialize feature variables:", name);
    initializer->setRobotState(jpos);
    initializer->setFeatureState(fpos); // leave this out if you want to use the initial values in the task specification.
    initializer->initialize_feature_variables();

    fpos = initializer->getFeatureState();
    // now both jpos and fpos are properly initialized
    log->log(INFO, "{} : Joint variables : ", name);
    for (size_t i = 0; i < jnames.size(); ++i) {
        log->log(INFO, "    {:20} : {}", jnames[i].c_str(), jpos[i]);
    }
    log->log(INFO, "{} : Feature variables :", name);
    for (size_t i = 0; i < fnames.size(); ++i) {
        log->log(INFO, "    {:20} : {}", fnames[i].c_str(), fpos[i]);
    }
   // create observers:
    Observer::Ptr obs1 = create_default_observer(ctx, "exit");
    Observer::Ptr obs2 = create_PrintObserver(slv, "print", "PrintObserver was triggered ", obs1);
    ctx->addDefaultObserver(obs2);
    // reset all monitors:
    ctx->resetMonitors();

    // initialize output-handlers
    log->log(INFO, "{} : Initialize output handlers:", name);
    for (auto& h : outputhandlers) {
        log->log(INFO, "    |- {}", h->getName());
        h->initialize(ctx, jnames, fnames);
    }
    return true;
}

void Task::startLoop()
{
    // Prepare the solver for execution, define output variables for both robot joints and feature states:
    log->log(INFO, "{} : startLoop started", name);
    slv->prepareExecution(ctx);
    slv->setJointStates(jpos);
    slv->setFeatureStates(fpos);
    jvel = Eigen::VectorXd::Zero(slv->getNrOfJointStates());
    fvel = Eigen::VectorXd::Zero(slv->getNrOfFeatureStates());
    timestats.reset(dt);
}

bool Task::onTimer()
{
    try {
        // measure timing
        timestats.update();
        // integrate previous outputs:


        // gets inputs, this can includes joint values in jpos,
        // which will be overwritten if used.
        for (auto& h : inputhandlers) {
            h->update(time, jnames, jpos, fnames, fpos);
        }

        // check monitors:
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            log->log(INFO, timestats.getStatistics(time));
            timestats.reset(dt);
            log->log(INFO, fmt::format("finishing... (time={})", time));
            return true;
        }
        // set states:
        slv->setTime(time);
        slv->setJointStates(jpos);
        slv->setFeatureStates(fpos);
        // solve
        int c = slv->solve();
        if (c != 0) {
            // @todo not a good way to handle this
            throw etasl_error(etasl_error::SOLVER_FAILED_DURING_RUN, slv->errorMessage(c));
        }
        // get outputs:
        slv->getJointVelocities(jvel);
        slv->getFeatureVelocities(fvel);
    } catch (std::exception& e) {
        log->log(WARN, "Exception was thrown : {}", e.what());
        jvel.setZero();
        fvel.setZero();
        for (auto& h : outputhandlers) {
            h->update(jnames, jpos, jvel, fnames, fpos, fvel);
        }
        throw;
    }
    // if you need to send output to a robot, do it here, using jvel or jpos
    for (auto& h : outputhandlers) {
        h->update(jnames, jpos, jvel, fnames, fpos, fvel);
    }
    timestats.computing_finished();
    if (timestats.duration_since_reset() > reporting_interval - 1E-3) {
        log->log(INFO, timestats.getStatistics(time));
        timestats.reset(dt);
    }

    jpos += jvel * dt; // or replace with reading joint positions from real robot
    fpos += fvel * dt; // you always integrate feature variables yourself
    time += dt; // idem.

    return false;
}

void Task::finalize()
{
    log->log(INFO, "{} : Finalize called", name);
    jvel.setZero();
    fvel.setZero();
    for (auto& h : inputhandlers) {
        h->finalize();
    }
    for (auto& h : outputhandlers) {
        h->finalize();
    }
}



} // namespace etasl