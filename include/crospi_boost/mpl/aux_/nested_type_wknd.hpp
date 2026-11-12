
#ifndef CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MPL_CFG_GCC, CROSPI_BOOST_TESTED_AT(0x0302)) \
    || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x561)) \
    || CROSPI_BOOST_WORKAROUND(__SUNPRO_CC, CROSPI_BOOST_TESTED_AT(0x530)) \
    || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))

namespace crospi_boost { namespace mpl { namespace aux {
template< typename T > struct nested_type_wknd
    : T::type
{
};
}}}

#if CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
#   define CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND(T) \
    aux::nested_type_wknd<T> \
/**/
#else
#   define CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND(T) \
    ::crospi_boost::mpl::aux::nested_type_wknd<T> \
/**/
#endif

#else // !CROSPI_BOOST_MPL_CFG_GCC et al.

#   define CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND(T) T::type

#endif 

#endif // CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND_HPP_INCLUDED
