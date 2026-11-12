
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_APPLY_HPP_INCLUDED
#define CROSPI_BOOST_MPL_APPLY_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#if !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)
#   include <crospi_boost/mpl/apply_fwd.hpp>
#   include <crospi_boost/mpl/apply_wrap.hpp>
#   include <crospi_boost/mpl/placeholders.hpp>
#   include <crospi_boost/mpl/lambda.hpp>
#   include <crospi_boost/mpl/aux_/na.hpp>
#   include <crospi_boost/mpl/aux_/lambda_support.hpp>
#endif

#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER apply.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/default_params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/partial_spec_params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/enum.hpp>
#   include <crospi_boost/mpl/aux_/config/lambda.hpp>
#   include <crospi_boost/mpl/aux_/config/dtp.hpp>
#   include <crospi_boost/mpl/aux_/nttp_decl.hpp>
#   include <crospi_boost/mpl/aux_/config/eti.hpp>
#   include <crospi_boost/mpl/aux_/config/msvc.hpp>
#   include <crospi_boost/mpl/aux_/config/workaround.hpp>

#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/inc.hpp>
#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

namespace crospi_boost { namespace mpl {

// local macros, #undef-ined at the end of the header
#   define AUX778076_APPLY_PARAMS(param) \
    CROSPI_BOOST_MPL_PP_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        ) \
    /**/

#   define AUX778076_APPLY_DEF_PARAMS(param, value) \
    CROSPI_BOOST_MPL_PP_DEFAULT_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        , value \
        ) \
    /**/

#   define AUX778076_APPLY_N_PARAMS(n, param) \
    CROSPI_BOOST_MPL_PP_PARAMS(n, param) \
    /**/

#   define AUX778076_APPLY_N_COMMA_PARAMS(n, param) \
    CROSPI_BOOST_PP_COMMA_IF(n) \
    CROSPI_BOOST_MPL_PP_PARAMS(n, param) \
    /**/

#   define AUX778076_APPLY_N_PARTIAL_SPEC_PARAMS(n, param, def) \
    CROSPI_BOOST_PP_COMMA_IF(n) \
    CROSPI_BOOST_MPL_PP_PARTIAL_SPEC_PARAMS(n, param, def) \
    /**/
    
#   define AUX778076_APPLY_N_SPEC_PARAMS(n, param) \
    CROSPI_BOOST_MPL_PP_ENUM(CROSPI_BOOST_PP_INC(n), param) \
    /**/


#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(0, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, <crospi_boost/mpl/apply.hpp>))
#include CROSPI_BOOST_PP_ITERATE()

#   if !defined(CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE)
// real C++ version is already taken care of
#   if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)

namespace aux {
// apply_count_args
#define AUX778076_COUNT_ARGS_PREFIX apply
#define AUX778076_COUNT_ARGS_DEFAULT na
#define AUX778076_COUNT_ARGS_ARITY CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY
#include <crospi_boost/mpl/aux_/count_args.hpp>
}


template<
      typename F, AUX778076_APPLY_DEF_PARAMS(typename T, na)
    >
struct apply
    : aux::apply_chooser< 
          aux::apply_count_args< AUX778076_APPLY_PARAMS(T) >::value
        >::template result_< F, AUX778076_APPLY_PARAMS(T) >::type
{
};

#   endif // CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION
#   endif // CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE

#   undef AUX778076_APPLY_N_SPEC_PARAMS
#   undef AUX778076_APPLY_N_PARTIAL_SPEC_PARAMS
#   undef AUX778076_APPLY_N_COMMA_PARAMS
#   undef AUX778076_APPLY_N_PARAMS
#   undef AUX778076_APPLY_DEF_PARAMS
#   undef AUX778076_APPLY_PARAMS

}}

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_APPLY_HPP_INCLUDED

///// iteration, depth == 1

// For gcc 4.4 compatability, we must include the
// CROSPI_BOOST_PP_ITERATION_DEPTH test inside an #else clause.
#else // CROSPI_BOOST_PP_IS_ITERATING
#if CROSPI_BOOST_PP_ITERATION_DEPTH() == 1

#   define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

template<
      typename F AUX778076_APPLY_N_COMMA_PARAMS(i_, typename T)
    >
struct CROSPI_BOOST_PP_CAT(apply,i_)
#if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1300)
    : CROSPI_BOOST_PP_CAT(apply_wrap,i_)< 
          typename lambda<F>::type
        AUX778076_APPLY_N_COMMA_PARAMS(i_, T)
        >
{
#else
{
    typedef typename CROSPI_BOOST_PP_CAT(apply_wrap,i_)< 
          typename lambda<F>::type
        AUX778076_APPLY_N_COMMA_PARAMS(i_, T)
        >::type type;
#endif
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(
          CROSPI_BOOST_PP_INC(i_)
        , CROSPI_BOOST_PP_CAT(apply,i_)
        , (F AUX778076_APPLY_N_COMMA_PARAMS(i_,T))
        )
};


#if defined(CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG)
/// workaround for ETI bug
template<>
struct CROSPI_BOOST_PP_CAT(apply,i_)<AUX778076_APPLY_N_SPEC_PARAMS(i_, int)>
{
    typedef int type;
};
#endif

#   if !defined(CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE)
#   if !defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)

#if i_ == CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY
/// primary template (not a specialization!)
template<
      typename F AUX778076_APPLY_N_COMMA_PARAMS(i_, typename T)
    >
struct apply
    : CROSPI_BOOST_PP_CAT(apply,i_)< F AUX778076_APPLY_N_COMMA_PARAMS(i_, T) >
{
};
#else
template<
      typename F AUX778076_APPLY_N_COMMA_PARAMS(i_, typename T)
    >
struct apply< F AUX778076_APPLY_N_PARTIAL_SPEC_PARAMS(i_, T, na) >
    : CROSPI_BOOST_PP_CAT(apply,i_)< F AUX778076_APPLY_N_COMMA_PARAMS(i_, T) >
{
};
#endif

#   else // CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION

#if !defined(CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE)
namespace aux {

template<>
struct apply_chooser<i_>
{
    template<
          typename F, AUX778076_APPLY_PARAMS(typename T)
        >
    struct result_
    {
        typedef CROSPI_BOOST_PP_CAT(apply,i_)<
              F AUX778076_APPLY_N_COMMA_PARAMS(i_, T)
            > type;
    };
};

} // namespace aux
#endif

#   endif // CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION
#   endif // CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE

#   undef i_

#endif // CROSPI_BOOST_PP_ITERATION_DEPTH()
#endif // CROSPI_BOOST_PP_IS_ITERATING
