
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_OVERLOAD_RESOLUTION_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_OVERLOAD_RESOLUTION_HPP_INCLUDED

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

#if    !defined(CROSPI_BOOST_MPL_CFG_BROKEN_OVERLOAD_RESOLUTION) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE) \
    && (   CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, < 0x590) \
        || CROSPI_BOOST_WORKAROUND(__MWERKS__, < 0x3001) \
        )

#   define CROSPI_BOOST_MPL_CFG_BROKEN_OVERLOAD_RESOLUTION

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_OVERLOAD_RESOLUTION_HPP_INCLUDED
