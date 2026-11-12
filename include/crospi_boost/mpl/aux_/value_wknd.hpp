
#ifndef CROSPI_BOOST_MPL_AUX_VALUE_WKND_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_VALUE_WKND_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/static_cast.hpp>
#include <crospi_boost/mpl/aux_/config/integral.hpp>
#include <crospi_boost/mpl/aux_/config/eti.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if defined(CROSPI_BOOST_MPL_CFG_BCC_INTEGRAL_CONSTANTS) \
    || defined(CROSPI_BOOST_MPL_CFG_MSVC_60_ETI_BUG)

#   include <crospi_boost/mpl/int.hpp>

namespace crospi_boost { namespace mpl { namespace aux {
template< typename C_ > struct value_wknd
    : C_
{
};

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_60_ETI_BUG)
template<> struct value_wknd<int>
    : int_<1>
{
    using int_<1>::value;
};
#endif
}}}


#if !defined(CROSPI_BOOST_MPL_CFG_MSVC_60_ETI_BUG)
#   define CROSPI_BOOST_MPL_AUX_VALUE_WKND(C) \
    ::CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::aux::value_wknd< C > \
/**/
#    define CROSPI_BOOST_MPL_AUX_MSVC_VALUE_WKND(C) CROSPI_BOOST_MPL_AUX_VALUE_WKND(C)
#else
#   define CROSPI_BOOST_MPL_AUX_VALUE_WKND(C) C
#   define CROSPI_BOOST_MPL_AUX_MSVC_VALUE_WKND(C) \
    ::crospi_boost::mpl::aux::value_wknd< C > \
/**/
#endif

#else // CROSPI_BOOST_MPL_CFG_BCC_INTEGRAL_CONSTANTS

#   define CROSPI_BOOST_MPL_AUX_VALUE_WKND(C) C
#   define CROSPI_BOOST_MPL_AUX_MSVC_VALUE_WKND(C) C

#endif

#if CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 238)
#   define CROSPI_BOOST_MPL_AUX_NESTED_VALUE_WKND(T, C) \
    CROSPI_BOOST_MPL_AUX_STATIC_CAST(T, C::value) \
/**/
#else
#   define CROSPI_BOOST_MPL_AUX_NESTED_VALUE_WKND(T, C) \
    CROSPI_BOOST_MPL_AUX_VALUE_WKND(C)::value \
/**/
#endif


namespace crospi_boost { namespace mpl { namespace aux {

template< typename T > struct value_type_wknd
{
    typedef typename T::value_type type;
};

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG)
template<> struct value_type_wknd<int>
{
    typedef int type;
};
#endif

}}}

#endif // CROSPI_BOOST_MPL_AUX_VALUE_WKND_HPP_INCLUDED
