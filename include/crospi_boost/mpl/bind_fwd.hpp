
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_BIND_FWD_HPP_INCLUDED
#define CROSPI_BOOST_MPL_BIND_FWD_HPP_INCLUDED

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
#   include <crospi_boost/mpl/aux_/na.hpp>
#endif

#include <crospi_boost/mpl/aux_/config/bind.hpp>
#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER bind_fwd.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/default_params.hpp>
#   include <crospi_boost/mpl/aux_/config/dmc_ambiguous_ctps.hpp>

#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

namespace crospi_boost { namespace mpl {

// local macros, #undef-ined at the end of the header

#   if defined(CROSPI_BOOST_MPL_CFG_DMC_AMBIGUOUS_CTPS)
#       define AUX778076_DMC_PARAM() , int dummy_ = 0
#   else
#       define AUX778076_DMC_PARAM()
#   endif

#   define AUX778076_BIND_DEFAULT_PARAMS(param, value) \
    CROSPI_BOOST_MPL_PP_DEFAULT_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        , value \
        ) \
    AUX778076_DMC_PARAM() \
    /**/

#   define AUX778076_BIND_N_PARAMS(n, param) \
    CROSPI_BOOST_PP_COMMA_IF(n) CROSPI_BOOST_MPL_PP_PARAMS(n, param) \
    AUX778076_DMC_PARAM() \
    /**/

#if !defined(CROSPI_BOOST_MPL_CFG_NO_BIND_TEMPLATE)
template<
      typename F, AUX778076_BIND_DEFAULT_PARAMS(typename T, na)
    >
struct bind;
#endif

#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(0, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, <crospi_boost/mpl/bind_fwd.hpp>))
#include CROSPI_BOOST_PP_ITERATE()

#   undef AUX778076_BIND_N_PARAMS
#   undef AUX778076_BIND_DEFAULT_PARAMS
#   undef AUX778076_DMC_PARAM
}}

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_BIND_FWD_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

template<
      typename F AUX778076_BIND_N_PARAMS(i_, typename T)
    >
struct CROSPI_BOOST_PP_CAT(bind,i_);

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
