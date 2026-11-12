
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_AUX_TEMPLATE_ARITY_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_TEMPLATE_ARITY_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/ttp.hpp>
#include <crospi_boost/mpl/aux_/config/lambda.hpp>

#if !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)
#   include <crospi_boost/mpl/aux_/template_arity_fwd.hpp>
#   include <crospi_boost/mpl/int.hpp>
#   if !defined(CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT)
#   if defined(CROSPI_BOOST_MPL_CFG_EXTENDED_TEMPLATE_PARAMETERS_MATCHING)
#       include <crospi_boost/mpl/aux_/type_wrapper.hpp>
#   endif
#   else
#       include <crospi_boost/mpl/aux_/has_rebind.hpp>
#   endif
#endif

#include <crospi_boost/mpl/aux_/config/static_constant.hpp>
#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER template_arity.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   if !defined(CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT)
#   if defined(CROSPI_BOOST_MPL_CFG_EXTENDED_TEMPLATE_PARAMETERS_MATCHING)

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/range.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/repeat.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/nttp_decl.hpp>

#   include <crospi_boost/preprocessor/seq/fold_left.hpp>
#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/inc.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

#   define AUX778076_ARITY CROSPI_BOOST_PP_INC(CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY)

namespace crospi_boost { namespace mpl { namespace aux {

template< CROSPI_BOOST_MPL_AUX_NTTP_DECL(int, N) > struct arity_tag
{
    typedef char (&type)[(unsigned)N + 1];
};

#   define AUX778076_MAX_ARITY_OP(unused, state, i_) \
    ( CROSPI_BOOST_PP_CAT(C,i_) > 0 ? CROSPI_BOOST_PP_CAT(C,i_) : state ) \
/**/

template<
      CROSPI_BOOST_MPL_PP_PARAMS(AUX778076_ARITY, CROSPI_BOOST_MPL_AUX_NTTP_DECL(int, C))
    >
struct max_arity
{
    CROSPI_BOOST_STATIC_CONSTANT(int, value = 
          CROSPI_BOOST_PP_SEQ_FOLD_LEFT(
              AUX778076_MAX_ARITY_OP
            , -1
            , CROSPI_BOOST_MPL_PP_RANGE(1, AUX778076_ARITY)
            )
        );
};

#   undef AUX778076_MAX_ARITY_OP

arity_tag<0>::type arity_helper(...);

#   define CROSPI_BOOST_PP_ITERATION_LIMITS (1, AUX778076_ARITY)
#   define CROSPI_BOOST_PP_FILENAME_1 <crospi_boost/mpl/aux_/template_arity.hpp>
#   include CROSPI_BOOST_PP_ITERATE()

template< typename F, CROSPI_BOOST_MPL_AUX_NTTP_DECL(int, N) >
struct template_arity_impl
{
    CROSPI_BOOST_STATIC_CONSTANT(int, value = 
          sizeof(::crospi_boost::mpl::aux::arity_helper(type_wrapper<F>(),arity_tag<N>())) - 1
        );
};

#   define AUX778076_TEMPLATE_ARITY_IMPL_INVOCATION(unused, i_, F) \
    CROSPI_BOOST_PP_COMMA_IF(i_) template_arity_impl<F,CROSPI_BOOST_PP_INC(i_)>::value \
/**/

template< typename F >
struct template_arity
{
    CROSPI_BOOST_STATIC_CONSTANT(int, value = (
          max_arity< CROSPI_BOOST_MPL_PP_REPEAT(
              AUX778076_ARITY
            , AUX778076_TEMPLATE_ARITY_IMPL_INVOCATION
            , F
            ) >::value
        ));
        
    typedef mpl::int_<value> type;
};

#   undef AUX778076_TEMPLATE_ARITY_IMPL_INVOCATION

#   undef AUX778076_ARITY

}}}

#   endif // CROSPI_BOOST_MPL_CFG_EXTENDED_TEMPLATE_PARAMETERS_MATCHING
#   else // CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT

#   include <crospi_boost/mpl/aux_/config/eti.hpp>

namespace crospi_boost { namespace mpl { namespace aux {

template< bool >
struct template_arity_impl
{
    template< typename F > struct result_
        : mpl::int_<-1>
    {
    };
};

template<>
struct template_arity_impl<true>
{
    template< typename F > struct result_
        : F::arity
    {
    };
};

template< typename F >
struct template_arity
    : template_arity_impl< ::crospi_boost::mpl::aux::has_rebind<F>::value >
        ::template result_<F>
{
};

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG)
template<>
struct template_arity<int>
    : mpl::int_<-1>
{
};
#endif

}}}

#   endif // CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_AUX_TEMPLATE_ARITY_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

template<
      template< CROSPI_BOOST_MPL_PP_PARAMS(i_, typename P) > class F
    , CROSPI_BOOST_MPL_PP_PARAMS(i_, typename T)
    >
typename arity_tag<i_>::type
arity_helper(type_wrapper< F<CROSPI_BOOST_MPL_PP_PARAMS(i_, T)> >, arity_tag<i_>);

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
