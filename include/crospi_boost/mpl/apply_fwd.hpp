
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_APPLY_FWD_HPP_INCLUDED
#define CROSPI_BOOST_MPL_APPLY_FWD_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER apply_fwd.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/default_params.hpp>
#   include <crospi_boost/mpl/aux_/config/ctps.hpp>
#   include <crospi_boost/mpl/aux_/nttp_decl.hpp>

#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

// agurt, 15/jan/02: top-level 'apply' template gives an ICE on MSVC
// (for known reasons)
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1300)
#   define CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE
#endif

namespace crospi_boost { namespace mpl {

// local macro, #undef-ined at the end of the header
#   define AUX778076_APPLY_DEF_PARAMS(param, value) \
    CROSPI_BOOST_MPL_PP_DEFAULT_PARAMS( \
          CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY \
        , param \
        , value \
        ) \
    /**/

#   define AUX778076_APPLY_N_COMMA_PARAMS(n, param) \
    CROSPI_BOOST_PP_COMMA_IF(n) \
    CROSPI_BOOST_MPL_PP_PARAMS(n, param) \
    /**/

#   if !defined(CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE)

#if !defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
// forward declaration
template<
      typename F, AUX778076_APPLY_DEF_PARAMS(typename T, na)
    >
struct apply;
#else
namespace aux {
template< CROSPI_BOOST_AUX_NTTP_DECL(int, arity_) > struct apply_chooser;
}
#endif

#   endif // CROSPI_BOOST_MPL_CFG_NO_APPLY_TEMPLATE

#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(0, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, <crospi_boost/mpl/apply_fwd.hpp>))
#include CROSPI_BOOST_PP_ITERATE()


#   undef AUX778076_APPLY_N_COMMA_PARAMS
#   undef AUX778076_APPLY_DEF_PARAMS

}}

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_APPLY_FWD_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

template<
      typename F AUX778076_APPLY_N_COMMA_PARAMS(i_, typename T)
    >
struct CROSPI_BOOST_PP_CAT(apply,i_);

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
