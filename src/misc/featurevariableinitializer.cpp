#include "featurevariableinitializer.hpp"
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>

using namespace Eigen;

namespace KDL {

void variables_from_context( 
        Context::Ptr ctx, 
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos
) {
    assert( jnames.size() == jpos.size() /* size of list of names and joint vector is not the same */);  
    for (int i=0;i<jnames.size();++i) {
        VariableScalar* vs=ctx->getScalarStruct(jnames[i]);
        assert(vs!=0 /* non existing variable?*/);
        jpos[i] = vs->initial_value;
    }
}

void variables_to_context( 
        Context::Ptr ctx, 
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos
) {
    assert( jnames.size() == jpos.size() /* size of list of names and joint vector is not the same */);  
    for (int i=0;i<jnames.size();++i) {
        VariableScalar* vs=ctx->getScalarStruct(jnames[i]);
        assert(vs!=0 /* non existing variable?*/);
        vs->initial_value = jpos[i];
    }
}



VariableRemapper::VariableRemapper( 
        const std::vector<std::string>& src_names, 
        const std::vector<std::string>& tgt_names 
):
    tgt_to_src(tgt_names.size()) 
{
    for (int i = 0;i < tgt_names.size();++i) {
        tgt_to_src[i] = -1;
        for (int j = 0;j < src_names.size();++j) {
            if (src_names[i] == tgt_names[i]) {
                tgt_to_src[i] = j;
                break;
            }
        }
        assert( tgt_to_src[i]!=-1 /* tgt_name does not exist in src */ );
    }
}


void 
VariableRemapper::set( 
        const Eigen::VectorXd& src, 
        Eigen::VectorXd& tgt 
) {
    for (int i = 0; i< tgt_to_src.size();++i) {
        tgt[i] = src[tgt_to_src[i]];
    }
}


void 
VariableRemapper::get( 
        Eigen::VectorXd& src, 
        const Eigen::VectorXd& tgt 
) {
        for (int i = 0;i< tgt_to_src.size();++i) {
            src[tgt_to_src[i]] = tgt[i];
        }
}


class FeatureVariableInitializerImpl: public FeatureVariableInitializer {

    enum ErrorCode {
        BEFORE_ERRORS      = -1,
        SUCCESS            = 0,
        PREPARE_FAILED     = 1,
        FULL_NOT_SUPPORTED = 2,     
        INIT_SOLVER_ERROR  = 3,
        NOT_CONVERGED      = 4,
        AFTER_ERRORS       = 5
    }; 


    solver::Ptr           slvr;
    Context::Ptr          ctx;

    // parameters:
    bool                  full;
    double                sample_time;
    double                duration;
    double                convergence_criterion;
    double                weight_factor;

    bool                  prepared;


    // initialized by prepareSolver(...) 
    Eigen::VectorXd       jointstate; 
    Eigen::VectorXd       featstate;
    Eigen::VectorXd       featvel;
    int                   solver_error;         // solver status in last initialization
    int                   count;                // number of steps in last initialization
    double                norm;                 // norm of step in last initialization
    double                cputime;              // duration of the initialization procedure

    public:
        FeatureVariableInitializerImpl( 
              solver::Ptr _slvr, 
              Context::Ptr _ctx,
              bool _full, 
              double _sample_time,
              double _duration,
              double _convergence_criterion,
              double _weight_factor
              );
        virtual int prepareSolver();
        virtual const Eigen::VectorXd& getFeatureState() const; 
        virtual void setFeatureState( const Eigen::VectorXd& fpos);
        virtual void setRobotState( const Eigen::VectorXd& jpos);
        virtual int getError() const;
        virtual std::string errorMessage(int code);
        virtual int initialize_feature_variables();
        virtual ~FeatureVariableInitializerImpl() {};
}; 

FeatureVariableInitializerImpl::FeatureVariableInitializerImpl( 
              solver::Ptr _slvr, 
              Context::Ptr _ctx,
              bool _full, 
              double _sample_time,
              double _duration,
              double _convergence_criterion,
              double _weight_factor
        ):
            slvr(_slvr),
            ctx(_ctx),
            full(_full),
            sample_time(_sample_time),
            duration(_duration),
            convergence_criterion(_convergence_criterion),
            weight_factor(_weight_factor),
            prepared(false),
            solver_error( 0 ),
            count(0),
            norm(0),
            cputime(0)
       {}

int FeatureVariableInitializerImpl::prepareSolver() {
    if (full) {
        solver_error = slvr->prepareInitializationFull(ctx, weight_factor);
    } else {
        solver_error = slvr->prepareInitialization(ctx);
    }
    if (solver_error!=0) {
        return solver_error;
    }
    jointstate = VectorXd::Zero(slvr->getNrOfJointStates());
    featstate  = VectorXd::Zero(slvr->getNrOfFeatureStates());
    featvel    = VectorXd::Zero(slvr->getNrOfFeatureStates());
    prepared   = true;
    return ErrorCode::SUCCESS;
}

void FeatureVariableInitializerImpl::setFeatureState( const Eigen::VectorXd& fpos) {
    assert( prepared /* prepareSolver should be called first */ );
    featstate = fpos;
}

void FeatureVariableInitializerImpl::setRobotState( const Eigen::VectorXd& jpos) {
    assert( prepared /* prepareSolver should be called first */ );
    jointstate = jpos;
}

const Eigen::VectorXd& 
FeatureVariableInitializerImpl::getFeatureState() const {
    assert( prepared /* prepareSolver should be called first */ );
    return featstate;
}

int FeatureVariableInitializerImpl::getError() const {
    return solver_error;
}



std::string
FeatureVariableInitializerImpl::errorMessage(int code)  {
      switch (code) {
          case ErrorCode::SUCCESS:
            return "No error";
          case ErrorCode::PREPARE_FAILED:
            return "Preparation failed because specification contains more priority levels than the solver supports"; 
          case ErrorCode::FULL_NOT_SUPPORTED:
            return "Full initialization is not supported by this solver";
          case ErrorCode::INIT_SOLVER_ERROR:
            {
                std::stringstream ss;
                ss << "Solver returned error \n" << slvr->errorMessage(solver_error);
                return ss.str();
            }
          case ErrorCode::NOT_CONVERGED:
            return "The feature values did not converge within the interval given by convergence_criterion";
          case FeatureVariableInitializer::ErrorCode::DISPLAY_ERRORS:
            {
                std::stringstream ss;
                ss << "Error codes of FeatureVariableInitializer" << std::endl;
                ss << "---------------------------------------------" << std::endl;
                ss << std::setw(5) << FeatureVariableInitializer::ErrorCode::DISPLAY_ERRORS << std::setw(0) 
                   << "\tUsed to display all all error messages for a FeatureVariableInitializer" << std::endl; 
                ss << "\nError codes of FeatureVariableInitializerImpl" << std::endl;
                ss << "---------------------------------------------" << std::endl;
                for (int i=ErrorCode::BEFORE_ERRORS+1; i<ErrorCode::AFTER_ERRORS;++i) {
                    ss << std::setw(5) << i << std::setw(0) << "\t" << errorMessage(i) << std::endl; 
                }
                ss << "\n" << slvr->errorMessage(solver::ErrorCode::DISPLAY_ERRORS);
                return ss.str();
            }
         default:
            // pass it to the solver object:
            return "Solver returned error during initialization:\n\t" +slvr->errorMessage(code);
      }
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
int FeatureVariableInitializerImpl::initialize_feature_variables() {
    using namespace std;

    assert( prepared /* prepareSolver should be called first */ );

    count        = 0;
    solver_error = 0;
    norm         = 0;
   
    // initialization:
    slvr->setJointStates(jointstate);
    slvr->setTime(0.0);
     if (slvr->getNrOfFeatureStates() > 0 ) {
        // feature variables exist:
        for (double t=0;t<duration;t+=sample_time) {
            count++;
            slvr->setFeatureStates(featstate);
            int solver_error = slvr->solve();
            if (solver_error!=0) {
                return solver_error;
            }
            slvr->getFeatureVelocities(featvel);
            featstate += featvel * sample_time;
            norm = featvel.norm(); // new semantics, without sample time !!! * sample_time;

            if (norm <= convergence_criterion) {
                return ErrorCode::SUCCESS;
            }
        }
        return ErrorCode::NOT_CONVERGED;
    } else {
        // no feature variables:
        return ErrorCode::SUCCESS;
    }
}




FeatureVariableInitializer::Ptr 
createFeatureVariableInitializer(solver::Ptr _slvr, Context::Ptr _ctx, ParameterList& plist) {
    // for now there is only one initializer:
    bool full                     = true;
    double sample_time            = 0.01;
    double duration               = 3.0;
    double convergence_criterion  = 1E-4;
    double weight_factor          = 100;
    for (auto it = plist.begin();it!=plist.end();++it) {
        if (it->first.compare("initialization_full")==0) {
            full = it->second;
        } else if (it->first.compare("initialization_sample_time")==0) {
            sample_time = it->second;
        } else if (it->first.compare("initialization_duration")==0) {
            duration  = it->second;
        } else if (it->first.compare("initialization_convergence_criterion")==0) {
            convergence_criterion = it->second;
        } else if (it->first.compare("initialization_weight_factor")==0) {
            weight_factor = it->second;
        }
    }
    return std::make_shared<FeatureVariableInitializerImpl>(
            _slvr, 
            _ctx,
            full, 
            sample_time, 
            duration, 
            convergence_criterion, 
            weight_factor);
}



} // namespace KDL
