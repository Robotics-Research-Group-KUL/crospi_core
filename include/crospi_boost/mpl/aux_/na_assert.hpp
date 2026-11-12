
#ifndef CROSPI_BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/na.hpp>
#include <crospi_boost/mpl/aux_/config/msvc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if !CROSPI_BOOST_WORKAROUND(_MSC_FULL_VER, <= 140050601)    \
    && !CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 243)
#   include <crospi_boost/mpl/assert.hpp>
#   define CROSPI_BOOST_MPL_AUX_ASSERT_NOT_NA(x) \
    CROSPI_BOOST_MPL_ASSERT_NOT((crospi_boost::mpl::is_na<x>)) \
/**/
#else
#   include <crospi_boost/static_assert.hpp>
#   define CROSPI_BOOST_MPL_AUX_ASSERT_NOT_NA(x) \
    CROSPI_BOOST_STATIC_ASSERT(!crospi_boost::mpl::is_na<x>::value) \
/**/
#endif

#endif // CROSPI_BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED
