
#ifndef CROSPI_BOOST_MPL_AUX_TRAITS_LAMBDA_SPEC_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_TRAITS_LAMBDA_SPEC_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2008
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/sequence_tag_fwd.hpp>
#include <crospi_boost/mpl/void.hpp>
#include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#include <crospi_boost/mpl/aux_/config/lambda.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT)

#   define CROSPI_BOOST_MPL_ALGORITM_TRAITS_LAMBDA_SPEC_IMPL(i, trait) /**/

#elif !defined(CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG)

#   define CROSPI_BOOST_MPL_ALGORITM_TRAITS_LAMBDA_SPEC_IMPL(i, trait) \
template<> struct trait<void_> \
{ \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i, typename T) > struct apply \
    { \
    }; \
}; \
/**/

#else

#   define CROSPI_BOOST_MPL_ALGORITM_TRAITS_LAMBDA_SPEC_IMPL(i, trait) \
template<> struct trait<void_> \
{ \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i, typename T) > struct apply \
    { \
    }; \
}; \
template<> struct trait<int> \
{ \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i, typename T) > struct apply \
    { \
        typedef int type; \
    }; \
}; \
/**/

#endif // CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT


#define CROSPI_BOOST_MPL_ALGORITM_TRAITS_LAMBDA_SPEC(i, trait) \
    CROSPI_BOOST_MPL_ALGORITM_TRAITS_LAMBDA_SPEC_IMPL(i, trait) \
    template<> struct trait<non_sequence_tag> {}; \
/**/

#endif // CROSPI_BOOST_MPL_AUX_TRAITS_LAMBDA_SPEC_HPP_INCLUDED
