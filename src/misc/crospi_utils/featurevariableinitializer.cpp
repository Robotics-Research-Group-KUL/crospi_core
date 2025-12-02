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


#include "crospi_utils/featurevariableinitializer.hpp"
#include "crospi_utils/etasl_error.hpp"
#include <Eigen/Dense>
#include <iomanip>
#include <sstream>
#include <fmt/format.h>

using namespace Eigen;

namespace etasl {

// void variables_from_context(
//     Context::Ptr ctx,
//     const std::vector<std::string>& jnames,
//     Eigen::VectorXd& jpos)
// {
//     assert(jnames.size() == jpos.size() /* size of list of names and joint vector is not the same */);
//     for (size_t i = 0; i < jnames.size(); ++i) {
//         VariableScalar* vs = ctx->getScalarStruct(jnames[i]);
//         assert(vs != 0 /* non existing variable?*/);
//         jpos[i] = vs->initial_value;
//     }
// }

// void variables_to_context(
//         Context::Ptr ctx,
//         const std::vector<std::string>& jnames,
//         const Eigen::VectorXd& jpos
// ) {
//     assert( jnames.size() == jpos.size() /* size of list of names and joint vector is not the same */);
//     for (size_t i=0;i<jnames.size();++i) {
//         VariableScalar* vs=ctx->getScalarStruct(jnames[i]);
//         assert(vs!=0 /* non existing variable?*/);
//         vs->initial_value = jpos[i];
//     }
// }

// VariableRemapper::VariableRemapper(
//         const std::vector<std::string>& src_names,
//         const std::vector<std::string>& tgt_names
// ):
//     tgt_to_src(tgt_names.size())
// {
//     for (size_t i = 0;i < tgt_names.size();++i) {
//         tgt_to_src[i] = -1;
//         for (size_t j = 0;j < src_names.size();++j) {
//             if (src_names[i] == tgt_names[i]) {
//                 tgt_to_src[i] = j;
//                 break;
//             }
//         }
//         assert( tgt_to_src[i]!=-1 /* tgt_name does not exist in src */ );
//     }
// }

// void
// VariableRemapper::set(
//         const Eigen::VectorXd& src,
//         Eigen::VectorXd& tgt
// ) {
//     for (size_t i = 0; i< tgt_to_src.size();++i) {
//         tgt[i] = src[tgt_to_src[i]];
//     }
// }

// void
// VariableRemapper::get(
//         Eigen::VectorXd& src,
//         const Eigen::VectorXd& tgt
// ) {
//         for (size_t i = 0;i< tgt_to_src.size();++i) {
//             src[tgt_to_src[i]] = tgt[i];
//         }
// }

class FeatureVariableInitializerImpl : public FeatureVariableInitializer {

    enum ErrorCode {
        BEFORE_ERRORS = -1,
        SUCCESS = 0,
        PREPARE_FAILED = 1,
        FULL_NOT_SUPPORTED = 2,
        INIT_SOLVER_ERROR = 3,
        NOT_CONVERGED = 4,
        AFTER_ERRORS = 5
    };

    solver::Ptr slvr;
    Context::Ptr ctx;

    // parameters:
    bool full;
    double sample_time;
    double duration;
    double convergence_criterion;
    double weight_factor;

    bool prepared;

    // initialized by prepareSolver(...)
    Eigen::VectorXd jointstate;
    Eigen::VectorXd featstate;
    Eigen::VectorXd featvel;
    int solver_error; // solver status in last initialization
    int count; // number of steps in last initialization
    double norm; // norm of step in last initialization
    double cputime; // duration of the initialization procedure

public:
    FeatureVariableInitializerImpl(
        solver::Ptr _slvr,
        Context::Ptr _ctx,
        bool _full,
        double _sample_time,
        double _duration,
        double _convergence_criterion,
        double _weight_factor);
    virtual void prepareSolver();
    virtual const Eigen::VectorXd& getFeatureState() const;
    virtual void setFeatureState(const Eigen::VectorXd& fpos);
    virtual void setRobotState(const Eigen::VectorXd& jpos);
    //virtual int getError() const;
    //virtual std::string errorMessage(int code);
    virtual void initialize_feature_variables();
    virtual ~FeatureVariableInitializerImpl() {};
};

FeatureVariableInitializerImpl::FeatureVariableInitializerImpl(
    solver::Ptr _slvr,
    Context::Ptr _ctx,
    bool _full,
    double _sample_time,
    double _duration,
    double _convergence_criterion,
    double _weight_factor)
    : slvr(_slvr)
    , ctx(_ctx)
    , full(_full)
    , sample_time(_sample_time)
    , duration(_duration)
    , convergence_criterion(_convergence_criterion)
    , weight_factor(_weight_factor)
    , prepared(false)
    , solver_error(0)
    , count(0)
    , norm(0)
    , cputime(0)
{
}

void FeatureVariableInitializerImpl::prepareSolver()
{
    if (full) {
        solver_error = slvr->prepareInitializationFull(ctx, weight_factor);
    } else {
        solver_error = slvr->prepareInitialization(ctx);
    }
    if (solver_error != 0) {
        throw etasl_error( etasl_error::INITIALIZER_SOLVER_ERROR, 
            "Error while preparing the solver during initialization : "+slvr->errorMessage(solver_error));
    }
    jointstate = VectorXd::Zero(slvr->getNrOfJointStates());
    featstate = VectorXd::Zero(slvr->getNrOfFeatureStates());
    featvel = VectorXd::Zero(slvr->getNrOfFeatureStates());
    prepared = true;
}

void FeatureVariableInitializerImpl::setFeatureState(const Eigen::VectorXd& fpos)
{
    assert(prepared /* prepareSolver should be called first */);
    featstate = fpos;
}

void FeatureVariableInitializerImpl::setRobotState(const Eigen::VectorXd& jpos)
{
    assert(prepared /* prepareSolver should be called first */);
    jointstate = jpos;
}

const Eigen::VectorXd&
FeatureVariableInitializerImpl::getFeatureState() const
{
    assert(prepared /* prepareSolver should be called first */);
    return featstate;
}

/**
 * CAVEATS:
 *  - Assumes that the input channels of the context are set appropriately.
 *  - Assumes that the robot and feature variables contain appropriate initial values ,
 *    with the following semantics:
 *      + robot variables jointstate: initial position of the robot, will remain fixed
 *      + feature variables featstate: initial values used for the optimization during initialization, will be
 *        adapted in this procedure.
 */
void FeatureVariableInitializerImpl::initialize_feature_variables()
{
    using namespace std;

    assert(prepared /* prepareSolver should be called first */);

    count = 0;
    solver_error = 0;
    norm = 0;

    // initialization:
    slvr->setJointStates(jointstate);
    slvr->setTime(0.0);
    if (slvr->getNrOfFeatureStates() > 0) {
        // feature variables exist:
        double t = 0.0;
        while (t < duration) {
            count++;
            slvr->setFeatureStates(featstate);
            int solver_error = slvr->solve();
            if (solver_error != 0) {
                throw etasl_error( etasl_error::INITIALIZER_SOLVER_ERROR, 
                    "Error in solver during initialization loop : "+slvr->errorMessage(solver_error));
            }
            slvr->getFeatureVelocities(featvel);
            featstate += featvel * sample_time;
            norm = featvel.norm(); // new semantics, without sample time !!! * sample_time;

            if (norm <= convergence_criterion) {
                return;
            }
            t += sample_time;
        }
        throw etasl_error(etasl_error::INITIALIZER_NOT_CONVERGED,
            fmt::format("Initalization loop did not convergence with remaining error {}", norm));
    }
}

FeatureVariableInitializer::Ptr
createFeatureVariableInitializer(solver::Ptr _slvr, Context::Ptr _ctx, const Json::Value& param)
{
    // for now there is only one initializer:
    bool full = param["full"].asBool();
    double sample_time = param["sample_time"].asDouble();
    double duration = param["duration"].asDouble();
    double convergence_criterion = param["convergence_criterion"].asDouble();
    double weight_factor = param["weightfactor"].asDouble();
    return std::make_shared<FeatureVariableInitializerImpl>(
        _slvr,
        _ctx,
        full,
        sample_time,
        duration,
        convergence_criterion,
        weight_factor);
}


} // namespace etasl
