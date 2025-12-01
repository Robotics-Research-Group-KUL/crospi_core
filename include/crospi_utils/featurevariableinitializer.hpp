#ifndef EXPRESSIONGRAPH_FEATUREVARIABLEINITIALIZER_HPP
#define EXPRESSIONGRAPH_FEATUREVARIABLEINITIALIZER_HPP

#include <Eigen/Dense>
#include <expressiongraph/context.hpp>
#include <expressiongraph/solver.hpp>
#include <jsoncpp/json/json.h>

//#include <expressiongraph/solver_factory.hpp>

//#include <crospi_utils/slvregistry.hpp>

namespace etasl {
    using namespace KDL;

// /**
//  * gets the values for a list of variables from the context
//  *
//  * REMARK:
//  *    - This routine a list of names to determine the order in which these joints are returned.
//  *    - This order is determined by the solver.
//  *
//  *    - solver->getJointNameVector(..) can be used to obtain this list for robot joint variables 
//  *    - solver->getFeatureNameVector(..) can be used to obtain this list for feature variables
//  * CAVEAT:
//  *    - Uses asserts because this is an internal routine, never exposed to user errors.  
//  */
// void variables_from_context( 
//         Context::Ptr ctx, 
//         const std::vector<std::string>& jnames,
//         Eigen::VectorXd& jpos
// );

// /**
//  * sets the values for a list of variables to the context
//  *
//  * REMARK:
//  *    - This routine a list of names to determine the order in which these joints are returned.
//  *    - This order is determined by the solver.
//  *
//  *    - solver->getJointNameVector(..) can be used to obtain this list for robot joint variables 
//  *    - solver->getFeatureNameVector(..) can be used to obtain this list for feature variables
//  * CAVEAT:
//  *    - Uses asserts because this is an internal routine, never exposed to user errors.  
//  */
// void variables_to_context( 
//         Context::Ptr ctx, 
//         const std::vector<std::string>& jnames,
//         const Eigen::VectorXd& jpos
// );



//     /**
//      *
//      * Given a list of variables and a list of names for a source
//      * Given a list of variables and a list of names for a target 
//      *
//      * This class provides an effecient way to SET ALL variables of the target to
//      * the corresponding value in the source.  This class provides an effecient way
//      * to GET ALL variables of the target and put it at corresponding place in the
//      * source.
//      * 
//      *
//      * CAVEAT:
//      *   - All variable names of the target are considered.  Only variables with a
//      *   name occuring in the target are considered for the source, i.e. the source
//      *   can contain more variables than the target. No duplicate names allowed.
//      *   (this is not checked).
//      */
//     class VariableRemapper {
//         std::vector<int> tgt_to_src;
//     public:
//        VariableRemapper( const std::vector<std::string>& src_names, const std::vector<std::string>& tgt_names );

//         /**
//          * set target to the values of src
//          */
//         void set( const Eigen::VectorXd& src, Eigen::VectorXd& tgt );

//         /**
//          * gets the values of tgt into src
//          */
//         void get( Eigen::VectorXd& src, const Eigen::VectorXd& tgt );
//     };

    /**
     * Abstract interface to a class that initializes the feature variables in an eTaSL 
     * specification.
     * createFeatureVariableInitializer creates a default initializer, suitable for
     * most eTaSL applications.
     */
    class FeatureVariableInitializer {
        public:
            enum ErrorCode {
                DISPLAY_ERRORS     = 5
            };
     
            typedef typename std::shared_ptr<FeatureVariableInitializer> Ptr;

            /**
             * gets the current value of the feature variable state
             */
            virtual const Eigen::VectorXd& getFeatureState() const = 0;

            /**
             * prepare the solver for initialization
             * among others, this determines the size of the robot and feature
             * variable states.
             * 
             * @warning can throw an etasl_error
             */
            virtual void prepareSolver() = 0;

            /**
             * Sets the values for the feature variables (before initialization)
             */
            virtual void setFeatureState( const Eigen::VectorXd& fpos) = 0;


            /**
             * Set the values for the robot joints. Typically correspond to the actual
             * joint values of the robot when starting up.
             */
            virtual void setRobotState( const Eigen::VectorXd& jpos) = 0;
    

            /**
             * gets the last occurring error
             */
            //virtual int getError() const = 0;

            /*
             * gets a string description of the error (including underlying solver errors
             * (if called with ErrorCode::DISPLAY_ERRORS, returns string with description
             *  of all errors)
             */
            //virtual std::string errorMessage(int code) = 0;

            /**
             * Performs initialization and returns an error code.
             * Returns 0 if success.  
             *
             * The initialization performs an optimization in _only_ the feature variables
             * (i.e. keeping the robot variables fixed).
             *
             * You can read the results of the initialization using getFeatureState().
             *
             * @warning
             *  (1) initial robot state and feature state are extracted from the context of
             *    the task specification.
             *  (2) But for a physical robot, you'll want to set the robot state using
             *    setRobotState() before initializing.
             *  (3) can throw an etasl_error
             */
            virtual void initialize_feature_variables() = 0;

            virtual ~FeatureVariableInitializer() {}; 
    };

    //FeatureVariableInitializer::Ptr 
    //createFeatureVariableInitializer(solver::Ptr _slvr, Context::Ptr _ctx, ParameterList& plist);

    FeatureVariableInitializer::Ptr 
    createFeatureVariableInitializer(solver::Ptr _slvr, Context::Ptr _ctx, const Json::Value& param);


} // namespace etasl

#endif
