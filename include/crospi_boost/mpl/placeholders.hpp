
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_PLACEHOLDERS_HPP_INCLUDED
#define CROSPI_BOOST_MPL_PLACEHOLDERS_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2004
// Copyright Peter Dimov 2001-2003
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
#   include <crospi_boost/mpl/arg.hpp>
#   include <crospi_boost/mpl/aux_/adl_barrier.hpp>

#   if !defined(CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE)
#       define CROSPI_BOOST_MPL_AUX_ARG_ADL_BARRIER_DECL(type) \
        using ::CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::type; \
        /**/
#   else
#       define CROSPI_BOOST_MPL_AUX_ARG_ADL_BARRIER_DECL(type) /**/
#   endif

#endif

#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
 && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER placeholders.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/aux_/nttp_decl.hpp>
#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

// watch out for GNU gettext users, who #define _(x)
#if !defined(_) || defined(CROSPI_BOOST_MPL_CFG_NO_UNNAMED_PLACEHOLDER_SUPPORT)
CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN
typedef arg<-1> _;
CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE

namespace crospi_boost { namespace mpl { 

CROSPI_BOOST_MPL_AUX_ARG_ADL_BARRIER_DECL(_)

namespace placeholders {
using CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::_;
}

}}
#endif

/// agurt, 17/mar/02: one more placeholder for the last 'apply#' 
/// specialization
#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(1, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY + 1, <crospi_boost/mpl/placeholders.hpp>))
#include CROSPI_BOOST_PP_ITERATE()

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_PLACEHOLDERS_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN

typedef arg<i_> CROSPI_BOOST_PP_CAT(_,i_);

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE

namespace crospi_boost { namespace mpl { 

CROSPI_BOOST_MPL_AUX_ARG_ADL_BARRIER_DECL(CROSPI_BOOST_PP_CAT(_,i_))

namespace placeholders {
using CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::CROSPI_BOOST_PP_CAT(_,i_);
}

}}

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
