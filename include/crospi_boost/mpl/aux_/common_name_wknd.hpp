
#ifndef CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2002-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, < 0x561)
// agurt, 12/nov/02: to suppress the bogus "Cannot have both a template class 
// and function named 'xxx'" diagnostic
#   define CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND(name) \
namespace name_##wknd { \
template< typename > void name(); \
} \
/**/

#else

#   define CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND(name) /**/

#endif // CROSPI_BOOST_BORLANDC

#endif // CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND_HPP_INCLUDED
