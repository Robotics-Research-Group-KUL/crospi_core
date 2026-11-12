
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_ARG_HPP_INCLUDED
#define CROSPI_BOOST_MPL_ARG_HPP_INCLUDED

// Copyright Peter Dimov 2001-2002
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

#if !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)
#   include <crospi_boost/mpl/arg_fwd.hpp>
#   include <crospi_boost/mpl/aux_/na.hpp>
#   include <crospi_boost/mpl/aux_/na_assert.hpp>
#   include <crospi_boost/mpl/aux_/arity_spec.hpp>
#   include <crospi_boost/mpl/aux_/arg_typedef.hpp>
#endif

#include <crospi_boost/mpl/aux_/config/static_constant.hpp>
#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER arg.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/default_params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/config/lambda.hpp>
#   include <crospi_boost/mpl/aux_/config/dtp.hpp>
#   include <crospi_boost/mpl/aux_/nttp_decl.hpp>

#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/inc.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN

// local macro, #undef-ined at the end of the header
#if !defined(CROSPI_BOOST_MPL_CFG_NO_DEFAULT_PARAMETERS_IN_NESTED_TEMPLATES)
#   define AUX778076_ARG_N_DEFAULT_PARAMS(param,value) \
    CROSPI_BOOST_MPL_PP_DEFAULT_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        , value \
        ) \
    /**/
#else
#   define AUX778076_ARG_N_DEFAULT_PARAMS(param,value) \
    CROSPI_BOOST_MPL_PP_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        ) \
    /**/
#endif

#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(0, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, <crospi_boost/mpl/arg.hpp>))
#include CROSPI_BOOST_PP_ITERATE()


#   undef AUX778076_ARG_N_DEFAULT_PARAMS

CROSPI_BOOST_MPL_AUX_NONTYPE_ARITY_SPEC(1,int,arg)

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_ARG_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

#if i_ > 0

template<> struct arg<i_>
{
    CROSPI_BOOST_STATIC_CONSTANT(int, value = i_);
    typedef arg<CROSPI_BOOST_PP_INC(i_)> next;
    CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(na, tag)
    CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(na, type)

    template<
          AUX778076_ARG_N_DEFAULT_PARAMS(typename U, na)
        >
    struct apply
    {
        typedef CROSPI_BOOST_PP_CAT(U,i_) type;
        CROSPI_BOOST_MPL_AUX_ASSERT_NOT_NA(type);
    };
};

#else

template<> struct arg<-1>
{
    CROSPI_BOOST_STATIC_CONSTANT(int, value = -1);
    CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(na, tag)
    CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(na, type)

    template<
          AUX778076_ARG_N_DEFAULT_PARAMS(typename U, na)
        >
    struct apply
    {
        typedef U1 type;
        CROSPI_BOOST_MPL_AUX_ASSERT_NOT_NA(type);
    };
};

#endif // i_ > 0

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
